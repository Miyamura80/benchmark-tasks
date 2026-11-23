"""Maintain fingertip contact on the underside of the west notch shoulder."""
import numpy as np

from finger_notch import NOTCH_WIDTH, hook_obstacles
from flap_contact_ik import solve_contact
from west_slide import geometry_model


def capture_shoulder(tcp, wrist, flap, frame, hinge, size):
    touch = flap + frame @ np.array([-NOTCH_WIDTH / 2, -size[1] / 2, size[2] / 2])
    return frame.T @ (touch - hinge), frame.T @ wrist, wrist.T @ (touch - tcp)


def follow_shoulder(hook, hinge, frame, predicted, arm, fingers):
    geometry = geometry_model()
    obstacles, targets = hook_obstacles(hook, lifting=True)
    other = geometry.parts('right', *hook.hand('right'))
    local, relative_wrist, tip = hook.contact_frame
    previous = hook.previous[:6]
    offset = hook.shoulder_offset
    best_gap = np.inf
    closest = None

    for _ in range(12):
        target = hinge + predicted @ local + offset * frame[:, 2]
        goal, error, rotation_error = solve_contact(target, predicted @ relative_wrist, tip,
                                                    arm, hook.env.rig.bases['left'])
        issued = np.clip(goal, previous - .03, previous + .03)
        state = geometry.guard('left', issued, fingers, obstacles, other, intended=True,
                               intended_targets=targets)
        gap = state['target_gap_m']
        if state['safe'] and -.00135 < gap < .0001 and error <= .0015:
            hook.shoulder_offset = offset
            return goal, error, rotation_error
        if state['safe'] and 0. <= gap < best_gap and error <= .0015:
            best_gap = gap
            closest = (goal, error, rotation_error, offset)
        if not state['safe'] and not (state['hard_pair'][1] or '').startswith('west_bottom'):
            break
        correction = -.0008 - gap
        if not state['safe']:
            correction = max(correction, .0003 - state['hard_gap_m'])
        offset += np.clip(correction, -.0015, .0015)
        if abs(offset) > .025:
            break
    if closest is not None:
        measured = geometry.guard('left', arm, fingers, obstacles, other, intended=True,
                                  intended_targets=targets)
        if best_gap < measured['target_gap_m']:
            goal, error, rotation_error, hook.shoulder_offset = closest
            return goal, error, rotation_error
    return None
