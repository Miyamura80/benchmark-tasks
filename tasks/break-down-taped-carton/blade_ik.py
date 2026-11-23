"""Fit a reachable blade pose inside the permitted cutting angle."""
import numpy as np
from scipy.optimize import least_squares
from yam_arena import ik


def heading_error(rotation, desired, normal):
    wanted = desired[:, 0] - normal * (desired[:, 0] @ normal)
    actual = rotation[:, 0] - normal * (rotation[:, 0] @ normal)
    cross = np.cross(wanted, actual)
    return float(np.arctan2(cross @ normal, wanted @ actual))


def orientation_error(rotation, desired, normal):
    tilt = max(0., .91 - float(rotation[:, 2] @ normal))
    return max(abs(heading_error(rotation, desired, normal)), 3.*tilt)


def solve_blade_pose(goal, desired, normal, tool_in_hand, tip_in_hand, seed, base, *, level_edge=False):
    def residual(joints):
        position, hand = ik.tcp_pose(joints, base)
        tool = hand @ tool_in_hand
        error = position + hand @ tip_in_hand - goal
        yaw = heading_error(tool, desired, normal)
        tilt = max(0., .92 - float(tool[:, 2] @ normal))
        # A pitched 14 mm blade hits the board at one corner before its
        # center reaches the tape. Preserve roll freedom while leveling the edge.
        edge_pitch = level_edge*float(tool[:, 0] @ normal)
        return np.r_[error, .07*yaw, .12*tilt, .09*edge_pitch, .00018*(joints-seed)]

    result = least_squares(residual, np.clip(seed, ik.LOWER, ik.UPPER),
                           bounds=(ik.LOWER, ik.UPPER), ftol=5e-6, xtol=1e-5,
                           gtol=1e-6, max_nfev=45)
    q = result.x
    position, hand = ik.tcp_pose(q, base)
    tool = hand @ tool_in_hand
    tip = position + hand @ tip_in_hand
    return q, float(np.linalg.norm(tip-goal)), orientation_error(tool, desired, normal)
