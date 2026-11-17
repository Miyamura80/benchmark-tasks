"""Install and field-check a shaft-end magnetic motor encoder."""
from math import atan2
import os

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP

from geom import Geometry

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MAX_STEPS = 1400
INSTRUCTION = (
    "Install the magnetic encoder on the motor shaft. Seat the magnet carrier, "
    "lower the sensor bracket onto its guide posts, and close its retaining catch. "
    "Press the field-test button, then release the assembly."
)
CASES = ("nominal", "skip-catch", "missing-magnet")
CASE = os.environ.get("ROBOCURVE_FIXTURE_CASE", "nominal")
CENTER = (0.306, 0.010)
DECK = TOP + 0.013
MOTOR_TOP = TOP + 0.077
SHAFT_TOP = TOP + 0.092
MAGNET_TOP = SHAFT_TOP + 0.0040
BOARD_Z = MAGNET_TOP + 0.0015
CAP_SEAT = (*CENTER, MAGNET_TOP + 0.003)
HEAD_SEAT = (*CENTER, BOARD_Z + 0.025)
CAP_HOME = (0.179, -0.148, TOP + 0.044)
CAP_GRIP_Z_OFFSET = -.005
HEAD_HOME = (0.269, 0.139, TOP + 0.050)
LATCH_XY = (0.365, -0.054)
LATCH_OPEN = TOP + 0.118
LATCH_TRAVEL = 0.012
TEST_XY = (0.250, -0.124)
TEST_CAP = TOP + 0.052
HOVER = TOP + 0.155

OBJECTS = [
    Box("test_fixture_base", (0.305, 0, TOP + .006), size=(.265, .205, .012),
        color=(.16, .22, .28)),
    Box("carrier_dock", (*CAP_HOME[:2], TOP + .0111), size=(.050, .046, .0222), color=(.18, .25, .29)),
    Box("encoder_dock", (*HEAD_HOME[:2], TOP + .008), size=(.115, .082, .016), color=(.18, .25, .29)),
    Box("encoder_frame", (0.,0.,0.), size=(.01,.01,.01)),
    Box("magnet_carrier", CAP_HOME, mass=.019, size=(.02,.02,.01)),
    Box("sensor_bracket", HEAD_HOME, mass=.056, size=(.03,.03,.01)),
    Box("encoder_catch", (*LATCH_XY,LATCH_OPEN), mass=.019, size=(.02,.02,.01)),
    Box("field_button", (*TEST_XY,TEST_CAP), mass=.008, size=(.02,.02,.01)),
]


def configure_demo(case):
    global CASE
    CASE = case


def demo():
    c, o = FINGER_CLOSED, FINGER_OPEN

    def move(duration, position, grip, name, yaw=0.0):
        return Waypoint(duration, position, grip, name, yaw)

    left_park = (0.275, 0.175, HOVER + .018)
    left = [move(2.7, left_park, o, "clear motor"),
            move(7.0, left_park, o, "wait for magnet")]
    left += [
        move(1.2, (*HEAD_HOME[:2], HOVER), o, "approach sensor bracket"),
        move(1.5, HEAD_HOME, o, "reach sensor grip"),
        move(.85, HEAD_HOME, c, "grasp sensor bracket"),
        move(1.2, (*HEAD_HOME[:2], HOVER + .03), c, "lift bracket"),
        move(1.2, (*CENTER, HOVER + .03), c, "align keyed guide"),
        move(1.5, (*CENTER, HEAD_SEAT[2] - .003), c, "seat bracket"),
        move(.65, (*CENTER, HEAD_SEAT[2] - .003), c, "settle bracket"),
        move(.8, (*CENTER, HEAD_SEAT[2] - .003), o, "release bracket"),
        move(1.3, (*CENTER, HOVER + .04), o, "lift clear of bracket"),
        move(1.3, left_park, o, "retract left"),
    ]
    right_park = (0.230, -0.190, HOVER + .020)
    if CASE == "missing-magnet":
        right = [move(2, right_park, o, "clear dock"), move(7.7, right_park, o, "leave carrier in dock")]
    else:
        pickup = (*CAP_HOME[:2], CAP_HOME[2] + CAP_GRIP_Z_OFFSET)
        seated_grip = (*CENTER, CAP_SEAT[2] + CAP_GRIP_Z_OFFSET - .001)
        right = [
            move(1.8, (*CAP_HOME[:2], HOVER), o, "approach carrier"),
            move(1.4, pickup, o, "reach carrier"),
            move(.85, pickup, c, "grasp carrier"),
            move(1.3, (*CAP_HOME[:2], HOVER), c, "lift carrier"),
            move(1.3, (*CENTER, HOVER), c, "align D-flat with shaft"),
            move(1.4, seated_grip, c, "press carrier onto shaft"),
            move(.75, seated_grip, o, "release carrier"),
            move(.9, (*CENTER, HOVER + .025), o, "clear shaft"),
            move(1.1, right_park, o, "clear motor"),
        ]
    delay = max(.4, sum(p.duration for p in left) - .20 - sum(p.duration for p in right))
    right.append(move(delay, right_park, o, "wait for sensor bracket"))
    if CASE != "skip-catch":
        right += [
            move(1.0, (*LATCH_XY, HOVER + .020), c, "approach bracket catch"),
            move(1.5, (*LATCH_XY, LATCH_OPEN + .013 - LATCH_TRAVEL - .002), c, "press catch closed"),
            move(.60, (*LATCH_XY, LATCH_OPEN + .013 - LATCH_TRAVEL - .002), c, "seat catch"),
            move(.9, (*LATCH_XY, HOVER + .020), c, "lift from catch"),
        ]
    right += [
        move(1.0, (*TEST_XY, HOVER), c, "approach test button"),
        move(1.4, (*TEST_XY, TEST_CAP + .005), c, "press field test"),
        move(.70, (*TEST_XY, TEST_CAP + .005), c, "read field check"),
        move(1.0, (*TEST_XY, HOVER), o, "release test button"),
        move(1.0, right_park, o, "retract right"),
    ]
    return {"left": left, "right": right}


class State:
    def __init__(self, case="nominal"):
        self.latch = False
        self.test_time = 0.
        self.test_result = None

    def advance(self, objects, hands, contacts, dt):
        from yam_arena.urdf import quat_xyzw_to_matrix

        carrier = objects["magnet_carrier"]
        bracket = objects["sensor_bracket"]
        catch = objects["encoder_catch"]
        button = objects["field_button"]
        carrier_basis = quat_xyzw_to_matrix(carrier.quat_xyzw)
        bracket_basis = quat_xyzw_to_matrix(bracket.quat_xyzw)
        yaw = atan2(carrier_basis[1,0], carrier_basis[0,0])
        carrier_on = bool(np.linalg.norm(carrier.pos - CAP_SEAT) <= .003
                          and abs(yaw) < .15 and carrier_basis[2,2] > .995
                          and carrier.resting())
        bracket_on = bool(np.linalg.norm(bracket.pos - HEAD_SEAT) <= .003
                          and bracket_basis[2,2] > .995 and bracket_basis[0,0] > .99
                          and bracket.resting())

        def pressed(shape):
            fingers = {"tip_left", "tip_right", "gripper"}
            return any(shape in pair and pair & fingers for pair, _, force in contacts if force > 0.)

        catch_motion = LATCH_OPEN - catch.pos[2]
        button_motion = TEST_CAP - button.pos[2]
        if not bracket_on or not carrier_on:
            self.test_result = None
        if catch_motion >= .010 and pressed("RedCatch"):
            self.latch = True
        retained = self.latch and catch_motion >= .009
        if button_motion >= .0045 and pressed("BlueTestKey"):
            self.test_time += dt
            if self.test_time >= .30:
                self.test_result = bool(carrier_on and bracket_on and retained)
        else:
            self.test_time = 0.
        clear = all(hand.position[2] > TOP + .133 or abs(hand.position[1]) > .160 for hand in hands.values())
        score = .25 if carrier_on else 0.
        if score and bracket_on:
            score = .65 if retained else .45
        if score == .65 and self.test_result is True and button_motion < .001 and clear:
            score = 1.
        return dict(score=score, carrier=carrier_on, bracket=bracket_on, catch=bool(retained),
                    carrier_position_m=list(map(float, carrier.pos)),
                    bracket_position_m=list(map(float, bracket.pos)),
                    carrier_yaw_rad=float(yaw),
                    catch_mm=round(float(catch_motion)*1000, 3),
                    button_mm=round(float(button_motion)*1000, 3), field_check=self.test_result,
                    caption="Field check passed" if self.test_result else "Install and check the encoder")


class View:
    def __init__(self, state):
        self.g = Geometry()
        self.light = self.g.sphere("FieldLED", (TEST_XY[0]-.019, TEST_XY[1]+.012, TOP+.045),
                                   (.008,.008,.004), (.16,.16,.16))

    def update(self, state):
        self.g.color(self.light, (.05,.75,.22) if state.test_result
                     else (.76,.09,.08) if state.test_result is False else (.16,.16,.16))
