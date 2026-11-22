"""Robot joint targets; all forces on the chain and pedal come from physical contact."""

from math import tau, cos, sin
import numpy as np
from fixtures import AXLES


def reseat(s, partial=False, guide=True, test_before=False, turn_budget=None):
    front = AXLES[0]
    center = front + [-0.118, 0, 0.040]

    def pedal(a):
        return center + np.array([0, -0.058 * cos(a), -0.058 * sin(a)])

    def move(side, p, duration, phase, opening=None, yaw=0):
        s.move_arm(side, p, duration, phase, opening, yaw=yaw)

    a = s.joint("front_angle")
    p = pedal(a)
    move("right", p + [-0.047, 0, 0], 1.1, "Approach along the pedal spindle", 0.08)
    a = s.joint("front_angle")
    p = pedal(a)
    move("right", p, 0.65, "Slide socket over the pedal spindle", 0.08)
    s.wait(0.25)
    if test_before:
        # Test the fault before touching the chain. Return to the same crank phase.
        start_angle = s.joint("front_angle")
        for i in range(1, 102):
            target = pedal(start_angle + tau * i / 101)
            commands = s.targets.copy()
            commands[1, :6], _, _ = s.solve(
                "right", target, s.yaw_choices["right"][1], commands[1, :6]
            )
            s.phase = "Test the dropped chain"
            s.action(commands)
        s.wait(0.45)
    if guide:
        yaw = np.pi / 2
        def jaw(offset, opening, seconds, phase):
            tcp = front + np.asarray(offset) + [-.108, .085, .075]
            s.yaw_choices["left"] = (yaw, yaw)
            s.move_arm("left", tcp, seconds, phase, opening, yaw=yaw)
        rollers = s.data.xpos[s.fixture.ids] - front
        near = rollers[(rollers[:,1] > .05) & (rollers[:,2] > 0)]

        pos = near[np.argmin(abs(near[:,1]-.088))]
        jaw(pos + [0,0,.030], .21, .8, "Approach the upper span from above")
        jaw(pos + [0,0,.001], .21, .65, "Place jaws on either side of the chain")
        jaw(pos + [0,0,.001], .00, .55, "Pinch the side plates")
        jaw([pos[0], pos[1], .068], .00, 1., "Lift the chain over the stay")
        jaw([0, pos[1], .076], .00,.6, "Bring the chain into the tooth plane")
        jaw([0, .037, .067], .00,.85, "Guide the span over the chainring")
        end_y = -0.012
        end_x = 0
        end_z = 0.06
        jaw([end_x, end_y, end_z], .00,.60, "Set rollers on the chainring")
        release_step = 3
    a = s.joint("front_angle")
    count = 155 if partial else (turn_budget or 790)
    for i in range(1, count + 1):
        target = pedal(a + i * 0.066)
        side = "right"
        targets = s.targets.copy()
        joints, error, _ = s.solve(side, target, s.yaw_choices[side][1], targets[1, :6])
        targets[1, :6] = joints
        s.phase = (
            "Turn the crank with the chain dropped"
            if not guide
            else (
                "Feed the first links onto the teeth"
                if i < 25
                else "Turn the crank through two revolutions"
            )
        )
        s.action(targets)
        if guide and i == release_step:
            jaw([end_x, end_y, end_z], .3, .30, "Open the chain jaws")
            jaw([-.030, .070, .104], .3, .75, "Withdraw guiding hand")
            jaw([-.018, .142, .056], .24, .70, "Park beyond the crank")
        # In the fault trial, release the socket in the reachable forward sector.
        ready_to_release = not test_before or 0.4 < s.joint("front_angle") % tau < 0.6
        if (s.fixture.turns >= 2.08 and s.fixture.proven() and ready_to_release) or (
            not guide and s.joint("front_angle") - a >= 2.35 * tau
        ):
            break
    p = s.data.site_xpos[s.sites["right"]].copy()
    move("right", p + [-0.036, 0, 0], 0.65, "Withdraw the pedal socket", 0.08)
    move(
        "right",
        p + [-0.036, -0.01, 0.008],
        0.45,
        "Check both sprockets after release",
        0.95,
    )
    s.wait(0.45)
