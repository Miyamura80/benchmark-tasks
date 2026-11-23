"""Mesh-supported contact targets for an ordinary table turnover.

Planning only: no object pose, force, velocity or success state is written.
Initial candidate is a 180mm-high carton at (250,-100)mm. Physical feasibility
and every approach/withdrawal still require measured native validation.
"""
import math
import numpy as np


def fixed_wrist(side, *, level_right_pad=False):
    """World-fixed 30degree downward tool axis, 90degree wrist polarity."""
    sign = -1. if side == 'left' else 1.
    z = np.array([0., sign*math.cos(math.pi/6), -.5])
    x = np.array([1., 0., 0.])
    base = np.column_stack((x, np.cross(z, x), z))
    if side == 'right' and level_right_pad:
        return base
    return base @ np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])


def tip_vertices(geometry, fingers=(.012, .012)):
    """Use the unchanged actual collision meshes at the supplied jaw state."""
    return np.concatenate([tri.reshape(-1, 3)
                           for _, name, tri in geometry.local_hand(fingers)
                           if name.startswith('tip_')])


def support_target(contact, inward_normal, wrist, vertices, press=.0002):
    """Align an actual tip support point with a specified physical plane.

    Near-coplanar support vertices are averaged within 50um. This operation
    can change selected facets as the carton rotates, so issued commands need
    existing speed/slew/clearance guards and contact tracking.
    """
    n = np.asarray(inward_normal, dtype=float)
    n /= np.linalg.norm(n)
    world = np.asarray(vertices) @ wrist.T
    extent = world @ n
    support = world[extent >= extent.max()-.00005].mean(axis=0)
    return np.asarray(contact)-support+n*press, wrist.copy()


def wall_target(origin, rotation, side, height, width, vertices, press=.0002, *, level_right_pad=False):
    """Low single-wall brace and high folded-double-wall push contact."""
    left = side == 'left'
    sign = -1. if left else 1.
    fraction = .2 if left else .95 if level_right_pad else .9
    local = np.array([0., -sign*(width/2+(.0015 if left else .0045)),
                      fraction*height])
    return support_target(np.asarray(origin)+rotation@local,
                          rotation@np.array([0., sign, 0.]),
                          fixed_wrist(side,level_right_pad=level_right_pad), vertices, press)


def level_wrist_along(hinge):
    """Align the pushing pads with the horizontal projection of a hinge."""
    tangent = np.asarray(hinge,dtype=float).copy()
    tangent[2] = 0.
    tangent /= np.linalg.norm(tangent)
    if tangent[0] < 0.:
        tangent = -tangent
    horizontal = np.array([-tangent[1], tangent[0], 0.])
    tool_axis = math.cos(math.pi / 6) * horizontal - np.array([0., 0., .5])
    return np.column_stack((tangent, np.cross(tool_axis, tangent), tool_axis))


def folded_flap_target(position, rotation, size, vertices, press=.0002):
    """Aim at the measured outer flap instead of extrapolating across the carton."""
    depth, thickness = size[1:]
    inward = rotation[:, 2]
    wrist = level_wrist_along(rotation[:, 0])
    # The folded flap's local +Y points toward the top crease.
    contact = position + rotation[:, 1] * (.4 * depth) - inward * (thickness / 2)
    return support_target(contact, inward, wrist, vertices, press)
