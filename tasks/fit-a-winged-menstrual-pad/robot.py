"""Stock Dream YAM collision geometry, gravity and six joint-position servos per arm."""
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from dream_examples.yam_mujoco import yam_env
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


SIDES = ('left', 'right')
DT = .001
HZ = 20


def xml(parent, tag, **attributes):
    def fmt(value):
        if isinstance(value, (list, tuple, np.ndarray)):
            return ' '.join(f'{x:.9g}' for x in np.array(value).ravel())
        return str(value)
    return ET.SubElement(parent, tag, {key: fmt(value) for key, value in attributes.items()})


def stock_station():
    root = ET.fromstring(yam_env.build_xml())
    world = root.find('worldbody')
    for item in list(world):
        if item.tag != 'light' and item.get('name') not in ('left_base', 'right_base'):
            world.remove(item)
    for side, x, yaw in [('left', -.33, 0), ('right', .33, np.pi)]:
        base = world.find(f"body[@name='{side}_base']")
        base.set('pos', f'{x} -.14 .025')
        base.set('quat', f'{np.cos(yaw/2)} 0 0 {np.sin(yaw/2)}')
    for geom in root.iter('geom'):
        if geom.get('priority') == '1':
            geom.attrib.update(friction='1.0 .003 .0003', condim='4', solref='.004 1')
    root.find('option').attrib.update(timestep=str(DT), noslip_iterations='0', iterations='70')
    root.find('visual/global').attrib.update(offwidth='960', offheight='720')
    root.find('visual/quality').attrib.update(offsamples='2', shadowsize='2048')
    return root, world


def pose_frame(heading, jaw_normal=(0, 0, 1)):
    axis = np.array(heading, dtype=float)
    axis /= np.linalg.norm(axis)
    normal = np.array(jaw_normal, dtype=float)
    normal -= axis * (axis @ normal)
    normal /= np.linalg.norm(normal)
    return np.column_stack([np.cross(normal, axis), normal, axis])


class Arm:
    def __init__(self, model, side):
        self.model = model
        self.side = side
        self.work = mj.MjData(model)
        joints = [model.joint(f'{side}_joint{i}') for i in range(1, 7)]
        self.positions = np.array([j.qposadr[0] for j in joints])
        self.dofs = np.array([j.dofadr[0] for j in joints])
        self.low, self.high = np.array([j.range for j in joints]).T
        self.actuators = [model.actuator(j.name).id for j in joints]
        self.fingers = [model.actuator(f'{side}_finger_{s}').id for s in (-1, 1)]
        self.finger_q = [model.joint(f'{side}_finger_{s}').qposadr[0] for s in (-1, 1)]
        self.site = model.site(side + '_tcp').id
        self.jacp = np.zeros((3, model.nv))
        self.jacr = np.zeros_like(self.jacp)

    def pose(self, data):
        return data.site_xpos[self.site].copy(), data.site_xmat[self.site].reshape(3, 3).copy()

    def forward(self, q):
        self.work.qpos[self.positions] = q
        mj.mj_kinematics(self.model, self.work)
        return self.pose(self.work)

    def solve(self, target, rotation, seed=None):
        def residual(q):
            pos, current = self.forward(q)
            return np.r_[pos-target, .08*Rotation.from_matrix(rotation @ current.T).as_rotvec()]
        # In a run the previous command supplies a continuous seed. The initial
        # multistart search uses only the fixed reset scene, never task dynamics.
        first = np.asarray([0, 1, 1, 0, 0, 0] if seed is None else seed)
        seeds = [first]
        if seed is None:
            seeds.extend(np.random.default_rng(42).uniform(self.low, self.high, (10, 6)))
        best = None
        for start in seeds:
            result = least_squares(residual, np.clip(start, self.low, self.high),
                                   bounds=(self.low, self.high), max_nfev=85,
                                   ftol=2e-8, gtol=2e-8, xtol=2e-8)
            cost = np.linalg.norm(result.fun)
            if best is None or cost < best[0]:
                best = cost, result.x
            if cost < .00035:
                break
        return best[1], best[0]

    def command(self, data, q, opening):
        data.ctrl[self.actuators] = np.clip(q, self.low, self.high)
        data.ctrl[self.fingers] = np.clip(opening, 0, .0475)

    def reset(self, data, q, opening):
        data.qpos[self.positions] = q
        data.qpos[self.finger_q] = opening
        self.command(data, q, opening)
