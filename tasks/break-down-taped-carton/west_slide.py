"""Plan a fingertip slide under the west flap using its live pose."""
import numpy as np
from yam_arena import ik
from yam_arena.urdf import axis_angle_matrix
from turnover_safety import Geometry
from finger_notch import hook_obstacles
from functools import lru_cache
geometry_model = lru_cache(maxsize=1)(Geometry)

def candidate(hook, edge, tangent, outward, up, yaw, offset, tilt, roll_angle, use_full_tip=False, aperture=.018, gap=.010, depth=.004, overlap=.0025):
    side = 'left'

    fingers = np.full(2, -aperture)
    arm, opening = hook.hand(side)
    base = hook.env.rig.bases[side]
    geometry = geometry_model()
    heading = axis_angle_matrix(up, yaw) @ outward
    roll = axis_angle_matrix(np.array([0., 0., 1.]), roll_angle)
    wrist = hook.wrist(np.radians(tilt), heading, up, 1.) @ roll
    quat = ik.matrix_to_quat_xyzw(wrist)
    tcp, rotation = ik.tcp_pose(arm, base)
    meshes = geometry.parts(side, arm, fingers)
    vertices = np.concatenate([(tri.reshape(-1, 3) - tcp) @ rotation
                               for name, tri in meshes if name == 'tip_left'])
    distal = vertices if use_full_tip else vertices[vertices[:, 2] > .005]
    corner = distal[np.argmax(distal @ wrist.T @ outward)]
    lower = edge + offset * tangent - gap * outward - depth * up
    low_target = lower - wrist @ corner
    high_target = low_target + hook.clearance * up
    insertion = (gap + overlap) * outward
    obstacles, targets = hook_obstacles(hook)
    other = geometry.parts('right', *hook.hand('right'))

    def solve(target, seed):
        q, pe, re = ik.solve(target, quat, seed, base_pos=base, iters=100)
        return q if pe <= .001 and re <= .014 else None

    def allowed(q, finger_positions, contact):
        clearance = geometry.guard(side, q, finger_positions, obstacles, other,
                                   intended=contact, intended_targets=targets)
        return clearance['safe'] and clearance['target_gap_m'] >= -.0015

    end = solve(low_target + insertion, arm)
    if end is None:
        return None
    low = solve(low_target, end)
    if low is None:
        return None
    high = solve(high_target, low)
    if high is None:
        return None
    q = hook.previous[:6].copy()
    for fraction in np.linspace(0., 1., 101):
        nominal = arm + ik.smoothstep(fraction) * (high - arm)
        q = np.clip(nominal, q-.03, q+.03)
        if not allowed(q, fingers, False):
            return None
    q = high
    for start, stop in ((high_target, low_target), (low_target, low_target + insertion)):
        for fraction in np.linspace(0., 1., 41):
            q = solve(start + fraction * (stop - start), q)
            if q is None or not allowed(q, fingers, True):
                return None
    return (side, 1., offset, yaw, (tilt, tilt, gap, depth), aperture, high), corner, roll, overlap


def plan_slide(hook, edge, tangent, outward, up):
    args = hook, edge, tangent, outward, up
    if hook.finger_notch:
        for tilt in (5., 2.):
            result = candidate(*args, 3.1, .007, tilt, 0., True, aperture=.014,
                               gap=.018, depth=.008, overlap=.0015)
            if result is not None:
                return result
        for yaw, roll, tilt in ((3.1, 0., 5.), (-1.55, np.pi, 5.), (2.8, 0., 25.),
                                (.7, -np.pi/2, 20.), (1.4, -np.pi/2, 26.)):
            for offset in (0., -.007, .007):
                result = candidate(*args, yaw, offset, tilt, roll, True, aperture=.014,
                                   gap=.007, depth=.005, overlap=.0015)
                if result is not None:
                    return result
        return None
    for yaw, offset, tilt, roll, full_tip, depth, overlap in (
        (1.4, .040, 26., -np.pi/2, False, .004, .0025),
        (3.1, 0., 5., 0., True, .006, .0015),
        (-1.55, 0., 5., np.pi, True, .003, .0015),
    ):
        result = candidate(*args, yaw, offset, tilt, roll, full_tip, depth=depth,
                           overlap=overlap)
        if result is not None:
            return result
    return None

def guarded_lift(hook, arm, fingers, goal, normal):
    """Back off along the measured underside until the next arm step is clear."""
    geometry = geometry_model()
    obstacles, targets = hook_obstacles(hook, lifting=True)
    other = geometry.parts('right', *hook.hand('right'))
    previous = hook.previous[:6]
    base = hook.env.rig.bases['left']
    position, rotation = ik.tcp_pose(goal, base)
    quat = ik.matrix_to_quat_xyzw(rotation)
    offset = 0.
    q = goal
    for _ in range(15):
        issued = np.clip(q, previous-.03, previous+.03)
        state = geometry.guard('left', issued, fingers, obstacles, other,
                               intended=True, intended_targets=targets)
        if state['safe'] and state['target_gap_m'] >= -.0015:
            return q
        if state['hard_gap_m'] < .0002 and not (state['hard_pair'][1] or '').startswith(hook.flap + '_bottom'):
            return None
        offset += np.clip(max(.0002-state['hard_gap_m'],
                              -.0008-state['target_gap_m']), .0005, .003)
        if offset > .032:
            return None
        q, error, residual = ik.solve(position + normal*offset, quat, arm,
                                       base_pos=base, iters=110)
        if error > .0015 or residual > .025:
            return None
    return None
