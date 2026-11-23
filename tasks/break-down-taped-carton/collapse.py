"""Rest the opened box on a wall, shear the tube, and withdraw the hands."""
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN
from yam_arena.urdf import quat_xyzw_to_matrix

from collapse_geometry import vertices
from config import CUTTER_PAD_START, TOP
from scene import layout
from turnover_safety import Geometry
from turnover import roll_wrist
from turnover_targets import support_target, tip_vertices


class CartonFlattenProbe:
    """Candidate late-stage controller using native poses and ordinary actions."""

    aperture = .012

    def __init__(self, env, previous):
        self.env = env
        self.previous = np.asarray(previous, dtype=float).copy()
        self.geometry = Geometry()
        self.parts = {part['name']: part for part in layout()[0]}
        self.tips = tip_vertices(self.geometry, (self.aperture, self.aperture))
        self.stage = 'lift_to_clearance'
        self.failure = None
        self.done = False
        self.started = None
        self.bias = np.zeros(3)

    @staticmethod
    def array(value):
        value = value.torch if hasattr(value, 'torch') else value
        return value.detach().cpu().numpy()

    def pose(self, name):
        data = self.array(self.env._base.arena_world.get_pose_w(name))[0]
        if data.shape != (7,) or not np.isfinite(data).all():
            raise ValueError('missing finite carton pose')
        return data[:3], quat_xyzw_to_matrix(data[3:])

    def arms(self):
        result = []
        for side in ('left', 'right'):
            name = side+'_robot'
            ids, finger_ids = self.env._ids[name]
            q = self.array(self.env._base.scene[name].data.joint_pos)[0]
            result.append((q[ids], q[finger_ids], *ik.tcp_pose(q[ids], self.env.rig.bases[side])))
        return result

    def change(self, stage, time, arm):
        self.stage, self.started = stage, time
        self.origin, self.rotation = arm[2].copy(), arm[3].copy()
        self.bias[:] = 0.

    def push_wrist(self):
        z = np.array((.65, .35*self.direction, -np.sqrt(1.-.65**2-.35**2)))
        x = np.cross((0., 0., 1.),z)
        x /= np.linalg.norm(x)
        return roll_wrist(np.column_stack((x,np.cross(z,x),z)),90.)

    def roll_target(self, poses, points, press):
        names = (self.contact_wall, self.contact_wall+'_top', self.contact_wall+'_bottom')
        support = np.concatenate([points[name] for name in names])
        wall, frame = poses[self.contact_wall]
        outward = frame[:, 1].copy()
        outward[2] = 0.
        outward /= np.linalg.norm(outward)
        normal = -outward
        tangent = frame[:, 0].copy()
        tangent[2] = 0.
        tangent /= np.linalg.norm(tangent)
        if tangent[0] < 0.:
            tangent = -tangent
        z = .6*normal + .35*tangent - np.array((0.,0.,np.sqrt(1.-.6**2-.35**2)))
        x = tangent-z*(z@tangent)
        x /= np.linalg.norm(x)
        wrist = np.column_stack((x,np.cross(z,x),z))
        contact = wall.copy()
        contact[2] = float(support[:, 2].max())-.025
        contact += (float(np.max(support@outward))-contact@outward)*outward
        target, rotation = support_target(contact, normal, wrist, self.tips, 0. if press is None else press)
        if press is None:
            target -= .055*normal
            target[2] += .025
        return target, rotation, names, normal

    def flatten_target(self, poses, points, pressing):
        lower, upper = sorted(('north', 'south'), key=lambda name: poses[name][0][2])
        contact = points[upper].mean(axis=0)
        contact[2] = points[upper][:, 2].max()
        # Choose a shear towards the centre of the work surface. Apply
        # lateral lead along the collapsing tube, with a downward normal.
        axis = poses[lower][1][:, 0].copy()
        axis[2] = 0.
        axis /= np.linalg.norm(axis)
        if axis[0] > 0.:
            axis *= -1.
        if pressing:
            contact += .008*axis
        rotation = self.push_wrist()
        target, rotation = support_target(contact, (0., 0., -1.), rotation, self.tips,
                                          .0011 if pressing else 0.)
        if not pressing:
            target[2] += .070
        return target, rotation, (upper, upper+'_top', upper+'_bottom')

    def guarded(self, arms, poses, contacts, depth):
        pieces = [self.geometry.parts(side, arm[0], arm[1])
                  for side, arm in zip(('left', 'right'), arms)]
        for i, side in enumerate(('left', 'right')):
            allowed = contacts if side == self.side else ()
            report = self.geometry.guard(side, arms[i][0], arms[i][1], poses, pieces[1-i],
                                         intended=bool(allowed), intended_targets=allowed)
            if not report['safe'] or report['target_gap_m'] < -depth:
                return False
        return True

    def command(self, time):
        if self.failure or self.done:
            return self.previous.copy(), [self.report()]
        try:
            poses = {name: self.pose(name) for name in self.parts}
            points = vertices(self.parts, {n: p for n, (p, _) in poses.items()},
                              {n: r for n, (_, r) in poses.items()})
            measured = self.arms()
            if self.started is None:
                centre_y = np.mean([poses[name][0][1] for name in ('north', 'south')])
                self.direction = -1 if centre_y >= 0. else 1
                self.side = 'left' if self.direction == -1 else 'right'
                self.index = 0 if self.side == 'left' else 1
                self.contact_wall = min(('north', 'south'),
                                        key=lambda name: self.direction*poses[name][0][1])
                self.roll_direction = -poses[self.contact_wall][1][:,1].copy()
                self.roll_direction[2] = 0.
                self.roll_direction /= np.linalg.norm(self.roll_direction)
                self.change('lift_to_clearance', time, measured[self.index])
            arm = measured[self.index]
            age = time-self.started
            up = -poses['north'][1][:, 2]
            lean = float(np.degrees(np.arctan2(self.roll_direction@up, up[2])))
            all_points = np.concatenate(list(points.values()))
            height = float(np.ptp(all_points[:, 2]))
            self.lean, self.height = lean, height
            contacts = ()
            next_stage = None
            if self.stage == 'lift_to_clearance':
                target = self.origin.copy()
                target[2] = max(target[2]+.035, float(all_points[:, 2].max())+.11)
                rotation = self.rotation
                if age >= 5. and np.linalg.norm(arm[2]-target)<.004:
                    next_stage = 'align_above_roll'
            elif self.stage == 'align_above_roll':
                target, rotation, _, _ = self.roll_target(poses, points, None)
                target[2] = self.origin[2]
                if (age >= 5. and np.linalg.norm(arm[2]-target)<.004
                        and np.linalg.norm(ik._rot_error(rotation, arm[3]))<.045):
                    next_stage = 'approach_roll'
            elif self.stage in ('approach_roll', 'push_roll'):
                pushing = self.stage == 'push_roll'
                press = .0036*ik.smoothstep(age/3.) if pushing else None
                target, rotation, wall_names, _ = self.roll_target(poses, points, press)
                if pushing:
                    contacts = wall_names
                    if lean >= 55.:
                        next_stage = 'clear_roll'
                elif age >= 5. and np.linalg.norm(arm[2]-target)<.004:
                    next_stage = 'push_roll'
            elif self.stage == 'clear_roll':
                target = self.origin-.075*self.roll_direction+np.array((0., 0., .035))
                rotation = self.rotation
                contacts = (self.contact_wall, self.contact_wall+'_top', self.contact_wall+'_bottom')
                if age >= 4. and 75. <= lean <= 105. and np.linalg.norm(arm[2]-target)<.006:
                    next_stage = 'approach_flatten'
            elif self.stage in ('approach_flatten', 'press_flatten'):
                pushing = self.stage == 'press_flatten'
                target, rotation, wall_names = self.flatten_target(poses, points, pushing)
                if pushing:
                    contacts = wall_names
                    if height <= .022:
                        next_stage = 'retreat_flat'
                elif age >= 5. and np.linalg.norm(arm[2]-target)<.004:
                    next_stage = 'press_flatten'
            else:
                target, rotation = self.origin+np.array((0., 0., .120)), self.rotation
                if age < 1.:
                    _, _, contacts = self.flatten_target(poses, points, True)
                if age >= 6. and np.linalg.norm(arm[2]-target)<.005:
                    next_stage = 'complete'
            if age>25. or np.min(all_points[:, 2])<TOP-.003:
                raise ValueError('carton flattening reached a physical progress limit')

            duration = 1.6 if self.stage in ('push_roll', 'press_flatten') else 5.
            blend = ik.smoothstep(age/duration)
            waypoint = self.origin+blend*(target-self.origin)
            orientation = ik.slerp_matrix(self.rotation, rotation, blend)
            proposed_bias = np.clip(self.bias+.045*(waypoint-arm[2]), -.003, .003)
            if self.stage == 'push_roll':
                proposed_bias[1] = np.clip(proposed_bias[1], -.00035, .00035)
            if self.stage == 'press_flatten':
                proposed_bias[2] = np.clip(proposed_bias[2], -.00035, .00035)
            q, position_error, rotation_error = ik.solve(
                waypoint+proposed_bias, ik.matrix_to_quat_xyzw(orientation), arm[0],
                base_pos=self.env.rig.bases[self.side], iters=180, tol=1e-6)
            if position_error>.001 or rotation_error>.02:
                raise ValueError('carton flattening target failed inverse kinematics')

            obstacle = {name: (*pose, self.parts[name]['size']) for name, pose in poses.items()}
            tool, tr = self.pose('cutter')
            for key, size, offset in (
                ('handle', (.1,.025,.025), (0,0,0)),
                ('guard', (.02,.03,.02), (.052,0,.002)),
                ('blade', (.014,.0005,.008), (.062,0,-.016)),
            ):
                obstacle['cutter_'+key] = (tool+tr@offset, tr, size)
            obstacle['cutter_pad'] = (np.asarray(CUTTER_PAD_START), np.eye(3), (.17,.07,.002))
            if not self.guarded(measured, obstacle, contacts, .0006):
                raise ValueError('measured carton flattening clearance failed')

            k = 7*self.index
            new = self.previous.copy()
            new[k:k+6], new[k+6] = q, self.aperture/abs(FINGER_OPEN)
            slew = np.tile([.025]*6+[.04], 2)
            new = np.clip(np.clip(new, self.previous-slew, self.previous+slew),
                          self.env._low, self.env._high)
            for fraction in (1., .5, .25, .125, 0.):
                candidate = self.previous+fraction*(new-self.previous)
                commanded = [(candidate[j:j+6], np.repeat(FINGER_OPEN*candidate[j+6],2))
                             for j in (0, 7)]
                if self.guarded(commanded, obstacle, contacts, .0042 if self.stage=='push_roll' else .002):
                    self.previous = candidate
                    if fraction == 1.:
                        self.bias = proposed_bias
                        if next_stage == 'complete':
                            self.done = True
                        elif next_stage is not None:
                            self.change(next_stage, time, arm)
                    break
            else:
                raise ValueError('no guarded carton flattening action')
        except ValueError as error:
            self.failure = str(error)
        return self.previous.copy(), [self.report()]

    def report(self):
        return {'phase': self.stage, 'failure': self.failure, 'complete': self.done,
                'carton_lean_deg': getattr(self, 'lean', None),
                'carton_height_span_m': getattr(self, 'height', None)}
