"""Move clear of the inverted carton before lowering onto the cutter."""
import os
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation

from config import CUTTER_START, TOP
from turnover import solve_target


class ElevatedCutterPickup:
    transit_s = 12.
    aperture = -.032
    yaw = -.3
    handle_offset = -.032

    def __init__(self, env):
        # Calibrated approaches around the inverted carton's near-side flap.
        approaches = {
            'tip': (-.032, .002, -.30, -.032),
            'middle': (-.024, .003, -.50, -.032),
            'center': (-.020, .003, -.80, -.040),
        }
        name = os.environ.get('CARTON_GRASP_SITE', 'tip')
        if name not in approaches:
            raise ValueError(f'unknown cutter grasp site {name!r}')
        self.handle_offset, y_offset, self.yaw, self.aperture = approaches[name]
        ids, _ = env._ids['left_robot']
        q = env._base.scene['left_robot'].data.joint_pos
        q = q.torch if hasattr(q, 'torch') else q
        self.start = q.detach().cpu().numpy()[0, ids].copy()
        self.base = env.rig.bases['left']
        _, starting_rotation = ik.tcp_pose(self.start, self.base)
        pose = env._base.arena_world.get_pose_w('cutter')
        pose = pose.torch if hasattr(pose, 'torch') else pose
        x, y, z = pose.detach().cpu().numpy()[0, :3]
        x += self.handle_offset
        y += y_offset
        self.via, pe, re = solve_target((.20, .43, .98), starting_rotation,
                                        self.start, self.base)
        if pe > .0005 or re > .006:
            raise ValueError('raised cutter transit is unreachable')
        hover = (x, y, TOP+.170)
        self.align, pe, re = solve_target((x, .49, .945), grip_rotation(self.yaw),
                                          self.via, self.base, starts=6)
        if pe > .0005 or re > .006:
            raise ValueError('cutter side approach is unreachable')
        self.hover, pe, re = solve_target(hover, grip_rotation(self.yaw), self.align, self.base, starts=6)
        if pe > .0005 or re > .006:
            raise ValueError('cutter pickup hover is unreachable')
        self.descend = ArmPlan([
            Waypoint(3., (x, y, z+.008), self.aperture, yaw=self.yaw),
            Waypoint(1.5, (x, y, z+.008), -.011, yaw=self.yaw),
            Waypoint(3., hover, -.011, yaw=self.yaw),
        ], self.base, q_start=self.hover, finger_start=self.aperture)

    @property
    def duration(self):
        return self.transit_s + self.descend.duration

    @property
    def worst_ik_mm(self):
        return self.descend.worst_ik_mm

    def transfer(self, time):
        waypoints = (self.start, self.via, self.align, self.hover)
        i = min(int(max(time, 0.) // 4.), 2)
        s = ik.smoothstep(np.clip((time - 4.*i) / 4., 0., 1.))
        return waypoints[i] + s*(waypoints[i+1]-waypoints[i])

    def fingers(self, time):
        s = ik.smoothstep(np.clip((time - 4.) / 4., 0., 1.))
        return FINGER_OPEN + s*(self.aperture-FINGER_OPEN)

    def target(self, time):
        if time >= self.transit_s:
            return self.descend.target(time-self.transit_s)
        p, R = ik.tcp_pose(self.transfer(time), self.base)
        return p, R, self.fingers(time), 'cutter clearance'

    def action7(self, time):
        if time >= self.transit_s:
            return self.descend.action7(time-self.transit_s)
        return np.r_[self.transfer(time), self.fingers(time)/FINGER_OPEN], 'cutter clearance'
