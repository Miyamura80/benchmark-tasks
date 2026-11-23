"""Re-execute an ordinary recorded action prefix before resuming live control."""
import json
import os
from pathlib import Path

import numpy as np
from yam_arena import ik
from yam_arena.urdf import quat_xyzw_to_matrix

from turnover import CartonTurnoverProbe as LiveTurnover, body_frame


class CartonTurnoverProbe(LiveTurnover):
    """No box or robot state is restored. All physical samples are recomputed."""

    def __init__(self, env, **kwargs):
        super().__init__(env, **kwargs)
        trace = Path(os.environ['ROBOCURVE_TURNOVER_PREFIX'])
        with trace.open() as f:
            self.prefix = [json.loads(line) for line in f]
        self.resume_stage=os.environ.get('ROBOCURVE_TURNOVER_RESUME','first_roll')
        self.resumed = False
        if self.resume_stage in ('bottom_hover','cutter','opening','remaining_flaps','last_flap'):
            return
        if self.resume_stage=='second_roll':
            pre = self.prefix[-2]
            o, r = body_frame(self.as_poses(pre))
            right_q = np.asarray(self.prefix[-1]['action_14d'][7:13])
            p, wrist = ik.tcp_pose(right_q, self.env.rig.bases['right'])
            expected_p, expected_wrist = self.bottom_target(o,r,0.)
            self.saved_bottom_offset = r.T@(p-expected_p)
            self.saved_bottom_rotation = expected_wrist.T@wrist
            return
        first = next(i for i,r in enumerate(self.prefix) if r['phase']=='first_roll')
        pre = self.prefix[first-2]
        contact = self.prefix[first-1]
        box = self.as_poses(pre)
        origin, rotation = body_frame(box)
        self.saved_offsets = []
        self.saved_rotations = []
        q = np.asarray(contact['action_14d'])
        for i,side in enumerate(('left','right')):
            p,r = ik.tcp_pose(q[7*i:7*i+6], self.env.rig.bases[side])
            expected_p,expected_r = self.wall_target(origin,rotation,side,0.,box['south_top'])
            self.saved_offsets.append(rotation.T@(p-expected_p))
            self.saved_rotations.append(expected_r.T@r)
        self.replay_origin = contact['time']-.05

    @staticmethod
    def as_poses(row):
        return {name: (np.asarray(value[:3]), quat_xyzw_to_matrix(value[3:]))
                for name,value in row['poses'].items()}

    def check_prefix(self, prior, measured_poses, measured_arms):
        target = self.as_poses(prior)
        translation_error = max(np.linalg.norm(p-target[n][0])
                                for n,(p,_) in measured_poses.items())
        rotation_error = max(np.linalg.norm(ik._rot_error(r,target[n][1]))
                             for n,(_,r) in measured_poses.items())
        joint_error = max(np.max(np.abs(arm[0]-native['joints']))
                          for arm,native in zip(measured_arms,prior['native_hands']))
        if translation_error>.002 or rotation_error>.034 or joint_error>.028:
            raise ValueError('physical action replay diverged from the recorded prefix')

    def command(self, time):
        step = round(time*20.)
        if not self.resumed:
            try:
                poses=self.poses()
                measured=self.measure()
                if step>0:
                    self.check_prefix(self.prefix[step-1],poses,measured)
                if step<len(self.prefix):
                    row=self.prefix[step]
                    action=np.asarray(row['action_14d'])
                    if not np.isfinite(action).all() or action.shape!=(14,):
                        raise ValueError('invalid recorded robot action')
                    self.previous=action.copy()
                    _,rotation=body_frame(poses)
                    measured_roll=-float(np.degrees(np.arctan2(rotation[2,1],rotation[1,1])))
                    return action,[dict(phase='replay_'+row['phase'],
                                        roll_angle_deg=measured_roll,
                                        failure=None,complete=False)]
                _,rotation=body_frame(poses)
                self.last_raw=-float(np.degrees(np.arctan2(rotation[2,1],rotation[1,1])))
                self.angle=self.best_angle=self.last_raw
                if self.resume_stage=='cutter':
                    from yam_arena.constants import FINGER_OPEN
                    ready=self.env._ready.detach().cpu().numpy().reshape(-1)
                    stable_arms=all(np.max(np.abs(arm[0]-ready[i:i+6]))<.018
                                    and np.max(np.abs(arm[1]-FINGER_OPEN))<.0015
                                    for i,arm in zip((0,7),measured))
                    if abs(abs(self.angle)-180.)>5. or not stable_arms:
                        raise ValueError('the replay did not reach an inverted carton with parked arms')
                    self.done=True
                    self.resumed=True
                    del self.prefix
                    return self.previous.copy(),[dict(phase='return_ready',roll_angle_deg=self.angle,
                                                      failure=None,complete=True)]
                if self.resume_stage in ('opening','remaining_flaps','last_flap'):
                    angles=self.env._base._carton_angle_observer.tracker.angles
                    open_count = {'opening': 0, 'remaining_flaps': 1, 'last_flap': 3}[self.resume_stage]
                    if not all(a>=150. for a in angles[:open_count]) or any(a>=150. for a in angles[open_count:]):
                        raise ValueError('replay did not reach the expected measured flap releases')
                    self.done=True
                    self.resumed=True
                    del self.prefix
                    return self.previous.copy(),[dict(phase='flap_prefix_released',
                                                      failure=None,complete=True)]
                if self.resume_stage=='second_roll':
                    if not 86. <= self.angle <= 97.:
                        raise ValueError('physical replay did not reach the acquired bottom contact')
                    self.change('second_roll',time,measured)
                    self.bottom_preload_offset = self.saved_bottom_offset
                    self.bottom_preload_rotation = self.saved_bottom_rotation
                elif self.resume_stage=='bottom_hover':
                    if not 85. <= self.angle <= 95.:
                        raise ValueError('physical replay did not reach the stable side pose')
                    self.change('bottom_hover',time,measured)
                else:
                    if not 25. < self.angle < 70.:
                        raise ValueError('physical replay did not reach the intended one-arm roll')
                    self.change('first_roll',time,measured)
                    self.started=self.replay_origin
                    self.wall_preload_offsets=self.saved_offsets
                    self.wall_preload_rotations=self.saved_rotations
                    arm=measured[0][0].copy()
                    self.brace_clear=(time-7.,arm,arm)
                self.resumed=True
                del self.prefix
            except ValueError as problem:
                self.failure=str(problem)
                return self.previous.copy(),[dict(phase='replay_failed',failure=self.failure)]
        return super().command(time)
