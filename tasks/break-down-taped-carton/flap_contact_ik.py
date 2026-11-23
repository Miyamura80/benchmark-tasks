"""Follow a flap with a measured fingertip contact."""
import numpy as np
from scipy.optimize import least_squares
from yam_arena import ik


def capture_contact(point, tcp, wrist, flap, flap_frame, hinge, size):
    edge = flap_frame.T @ (point - flap)
    edge[1] = -size[1] / 2
    edge[2] = size[2] / 2
    touch = flap + flap_frame @ edge
    return flap_frame.T @ (touch - hinge), flap_frame.T @ wrist, wrist.T @ (touch - tcp)


def solve_contact(target, target_wrist, tool_point, seed, base):
    def residual(joints):
        tcp, wrist = ik.tcp_pose(joints, base)
        return np.r_[tcp + wrist @ tool_point - target,
                     .005 * ik._rot_error(target_wrist, wrist),
                     .0005 * (joints - seed)]

    goal = least_squares(residual, np.clip(seed, ik.LOWER, ik.UPPER),
                         bounds=(ik.LOWER, ik.UPPER), max_nfev=60).x
    position, wrist = ik.tcp_pose(goal, base)
    return (goal, float(np.linalg.norm(position + wrist @ tool_point - target)),
            float(np.linalg.norm(ik._rot_error(target_wrist, wrist))))
