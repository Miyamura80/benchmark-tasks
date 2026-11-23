"""Replay with separate telemetry for passive top flaps during carton turnover."""
import numpy as np
from yam_arena import ik
from replay_turnover import CartonTurnoverProbe as StrictReplay

class CartonTurnoverProbe(StrictReplay):
    def check_prefix(self, prior, measured_poses, measured_arms):
        stage = prior['phase'].replace('replay_', '')
        phases = ('narrow_ready', 'approach', 'contact', 'first_roll', 'clear_first',
                  'bottom_hover', 'bottom_contact', 'second_roll', 'finish_clear', 'return_ready')
        # The right hand presses the folded south top during first roll.
        # The three other top flaps are free hinges and never targeted in these phases.
        passive = {'north_top','east_top','west_top'} if stage in phases else set()
        if not passive:
            self.free_flap_errors = {}
            return super().check_prefix(prior, measured_poses, measured_arms)
        reference = self.as_poses(prior)
        self.free_flap_errors = {}
        for name in passive:
            p, R = measured_poses[name]
            prior_p, prior_R = reference[name]
            dp = float(np.linalg.norm(p-prior_p))
            dr = float(np.linalg.norm(ik._rot_error(R,prior_R)))
            if not np.isfinite([dp,dr]).all():
                raise ValueError('non-finite measured top flap')
            self.free_flap_errors[name] = {'position_m':dp, 'rotation_rad':dr}
        # Preserve the same limits for all walls, bottom flaps, the south top contact,
        # and both robots. The existing scorer still sees every physical panel.
        active = {name: pose for name,pose in measured_poses.items() if name not in passive}
        return super().check_prefix(prior, active, measured_arms)

    def command(self, time):
        target, details = super().command(time)
        details[0]['prefix_mode'] = 'cross_device_top_flap_diagnostics'
        details[0]['passive_top_flap_errors'] = getattr(self, 'free_flap_errors', {})
        return target, details

class BreakoverHandoff(CartonTurnoverProbe):
    """Hand control to the native turnover controller when gravity takes over."""
    def command(self, time):
        if not self.resumed:
            step = round(time * 20.)
            if 0 < step <= len(self.prefix):
                prior = self.prefix[step - 1]
                if prior['phase'].replace('replay_', '') == 'first_roll':
                    poses = self.poses()
                    angle, beyond = self.breakover(poses)
                    if 35. < angle < 75. and beyond > .003:
                        self.resumed = True
                        del self.prefix
                        self.angle = self.last_raw = self.best_angle = angle
                        self.change('clear_first', time, self.measure())
                        self.settle_stable = 0
                        self.handoff_time = time
                        self.handoff_com = beyond
        target, details = super().command(time)
        if hasattr(self, 'handoff_time'):
            details[0]['live_from_breakover_s'] = self.handoff_time
            details[0]['handoff_com_beyond_support_m'] = self.handoff_com
        return target, details

    def breakover(self, poses):
        from collapse_geometry import vertices
        from turnover import horizontal_roll_axis
        positions = {name: pose[0] for name,pose in poses.items()}
        rotations = {name: pose[1] for name,pose in poses.items()}
        cloud = np.concatenate(list(vertices(self.parts, positions, rotations).values()))
        R = rotations['north']
        axis = horizontal_roll_axis(R)
        across = np.array([-axis[1], axis[0], 0.])
        angle = -float(np.degrees(np.arctan2(R[2,1], R[:,1] @ across)))
        contact = cloud[cloud[:,2] <= cloud[:,2].min() + .0005]
        mass = sum(part['mass'] for part in self.parts.values())
        com = sum(self.parts[n]['mass'] * float(p @ across) for n,p in positions.items()) / mass
        return angle, com - float((contact @ across).max())
