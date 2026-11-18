"""Install and field-check a shaft-end magnetic motor encoder."""
from math import atan2, hypot
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
CASES = ("nominal", "skip-catch", "missing-magnet", "wrong-yaw", "no-detent")
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
CATCH_OPEN = (0.383, -0.082, TOP + .127)
CATCH_SHIFT = .018
CATCH_DROP = .021
CATCH_STROKE = hypot(CATCH_SHIFT, CATCH_DROP)
CATCH_AXIS = (CATCH_SHIFT / CATCH_STROKE, 0., CATCH_DROP / CATCH_STROKE)
CATCH_CLOSED = (CATCH_OPEN[0] - CATCH_SHIFT, CATCH_OPEN[1], CATCH_OPEN[2] - CATCH_DROP)
TAB_POS, TAB_SIZE = (.049, -.050, -.019), (.022, .017, .006)
TONGUE_POS, TONGUE_SIZE = (-.006, .040, .0035), (.010, .014, .005)
TEST_XY = (0.250, -0.124)
TEST_CAP = TOP + 0.052
HOVER = TOP + 0.155
PROOF_FORCE_N = 2.0
PROOF_DURATION_S = .30
PROOF_LIFT_M = .00050
DETENT_HOLD_N = 3.2
DETENT_RELEASE_M = .0014
SEATED_YAW_TOL_RAD = .20

OBJECTS = [
    Box("test_fixture_base", (0.305, 0, TOP + .006), size=(.265, .205, .012),
        color=(.16, .22, .28)),
    Box("carrier_dock", (*CAP_HOME[:2], TOP + .0111), size=(.050, .046, .0222), color=(.18, .25, .29)),
    Box("encoder_dock", (*HEAD_HOME[:2], TOP + .008), size=(.115, .082, .016), color=(.18, .25, .29)),
    Box("encoder_frame", (0.,0.,0.), size=(.01,.01,.01)),
    Box("magnet_carrier", CAP_HOME, mass=.019, size=(.02,.02,.01)),
    Box("sensor_bracket", HEAD_HOME, mass=.056, size=(.03,.03,.01)),
    Box("encoder_catch", CATCH_OPEN, mass=.019, size=(.02,.02,.01)),
    Box("field_button", (*TEST_XY,TEST_CAP), mass=.008, size=(.02,.02,.01)),
]


def configure_demo(case):
    global CASE
    CASE = case


def demo():
    c, o = FINGER_CLOSED, FINGER_OPEN

    def move(duration, position, grip, name, yaw=0.0):
        return Waypoint(duration, position, grip, name, yaw)

    carrier_xy = (CENTER[0] + .0026, CENTER[1] + .0022)
    bracket_xy = (CENTER[0] + .006, CENTER[1] - .004)
    left_park = (0.275, 0.175, HOVER + .018)
    left = [move(2.7, left_park, o, "clear motor"),
            move(7.0, left_park, o, "wait for magnet")]
    left += [
        move(1.2, (*HEAD_HOME[:2], HOVER), o, "approach sensor bracket"),
        move(1.5, HEAD_HOME, o, "reach sensor grip"),
        move(.85, HEAD_HOME, c, "grasp sensor bracket"),
        move(1.2, (*HEAD_HOME[:2], HOVER + .03), c, "lift bracket"),
        move(1.2, (*CENTER, HOVER + .03), c, "align keyed guide"),
        move(1.5, (*bracket_xy, HEAD_SEAT[2] - .003), c, "seat bracket"),
        move(.65, (*bracket_xy, HEAD_SEAT[2] - .003), c, "settle bracket"),
        move(.8, (*bracket_xy, HEAD_SEAT[2] - .003), o, "release bracket"),
        move(1.3, (*CENTER, HOVER + .04), o, "lift clear of bracket"),
        move(1.3, left_park, o, "retract left"),
    ]
    right_park = (0.230, -0.190, HOVER + .020)
    if CASE == "missing-magnet":
        right = [move(2, right_park, o, "clear dock"), move(7.7, right_park, o, "leave carrier in dock")]
    else:
        pickup = (*CAP_HOME[:2], CAP_HOME[2] + CAP_GRIP_Z_OFFSET)
        seated_grip = (*carrier_xy, CAP_SEAT[2] + CAP_GRIP_Z_OFFSET - .001)
        yaw = .85 if CASE == "wrong-yaw" else -.05
        right = [
            move(1.8, (*CAP_HOME[:2], HOVER), o, "approach carrier"),
            move(1.4, pickup, o, "reach carrier"),
            move(.85, pickup, c, "grasp carrier"),
            move(1.3, (*CAP_HOME[:2], HOVER), c, "lift carrier"),
            move(1.5, (*carrier_xy, HOVER), c, "align D-flat with shaft", yaw),
            move(1.6, seated_grip, c, "press carrier onto shaft", yaw),
            move(.50, seated_grip, c, "settle carrier", yaw),
            move(.75, seated_grip, o, "release carrier", yaw),
            move(.9, (*CENTER, HOVER + .025), o, "clear shaft"),
            move(1.1, right_park, o, "clear motor"),
        ]
    delay = max(.4, sum(p.duration for p in left) - .20 - sum(p.duration for p in right))
    right.append(move(delay, right_park, o, "wait for sensor bracket"))
    if CASE != "skip-catch":
        # Track the red cap along the inclined slide while the fingertips
        # supply the closing force. The first target takes up clearance.
        catch_start = (*CATCH_OPEN[:2], CATCH_OPEN[2] + .010)
        catch_end = (*CATCH_CLOSED[:2], CATCH_CLOSED[2] - .006)
        right += [
            move(1.0, (*CATCH_OPEN[:2], HOVER + .020), c, "approach bracket catch"),
            move(1.0, catch_start, c, "touch catch"),
            move(1.5, catch_end, c, "press catch along its guide"),
            move(.60, catch_end, c, "seat catch"),
            move(.9, (*CATCH_CLOSED[:2], HOVER + .020), c, "lift from catch"),
        ]
    right += [
        move(1.0, (*TEST_XY, HOVER), c, "approach test button"),
        move(1.4, (TEST_XY[0] + .003, TEST_XY[1] + .004, TEST_CAP - .004), c, "press field test"),
        move(.70, (TEST_XY[0] + .003, TEST_XY[1] + .004, TEST_CAP - .004), c, "read field check"),
        move(1.0, (*TEST_XY, HOVER), o, "release test button"),
        move(1.0, right_park, o, "retract right"),
    ]
    return {"left": left, "right": right}


class State:
    def __init__(self, case="nominal"):
        self.latch = False
        self.test_time = 0.
        self.test_result = None
        self.detent_enabled = case != "no-detent"
        self.detent_engaged = False
        self.detent_ready = True
        self.proof_elapsed = 0.
        self.proof_origin = None
        self.proof_max_lift = 0.
        self.proof_pass = None
        self.proof_force = 0.
        self.detent_force = 0.

    def advance(self, objects, hands, contacts, dt):
        from yam_arena.urdf import quat_xyzw_to_matrix

        carrier = objects["magnet_carrier"]
        bracket = objects["sensor_bracket"]
        catch = objects["encoder_catch"]
        button = objects["field_button"]
        carrier_basis = quat_xyzw_to_matrix(carrier.quat_xyzw)
        bracket_basis = quat_xyzw_to_matrix(bracket.quat_xyzw)
        yaw = atan2(carrier_basis[1,0], carrier_basis[0,0])
        radial_error = np.linalg.norm(carrier.pos[:2] - CENTER)
        lift = float(carrier.pos[2] - CAP_SEAT[2])
        aligned = radial_error <= .0010 and abs(yaw) <= SEATED_YAW_TOL_RAD and carrier_basis[2,2] > .995
        seated = aligned and abs(lift) <= .00065
        carrier_released = all(np.linalg.norm(hand.position - carrier.pos) > .030 for hand in hands.values())
        carrier_on = bool(seated and carrier_released)
        # The plunger enters its groove as the free, keyed body reaches the
        # stop. A preloaded flank resists extraction up to the ramp travel.
        if self.detent_engaged and (lift > DETENT_RELEASE_M or not aligned):
            self.detent_engaged = False
        if lift > .004 or radial_error > .010:
            self.detent_ready = True
        if self.detent_enabled and self.detent_ready and seated and lift <= .00030:
            self.detent_engaged = True
            self.detent_ready = False
        self.detent_force = -DETENT_HOLD_N if self.detent_engaged else 0.

        if self.proof_force:
            self.proof_elapsed += dt
            self.proof_max_lift = max(self.proof_max_lift, float(carrier.pos[2] - self.proof_origin))
            if self.proof_max_lift > PROOF_LIFT_M or self.proof_elapsed >= PROOF_DURATION_S - 1e-6:
                self.proof_pass = bool(self.proof_max_lift <= PROOF_LIFT_M and aligned)
                self.proof_force = 0.
        if not seated and not self.proof_force and self.proof_pass:
            self.proof_pass = None
            self.proof_origin = None
        bracket_on = bool(np.linalg.norm(bracket.pos - HEAD_SEAT) <= .003
                          and bracket_basis[2,2] > .995 and bracket_basis[0,0] > .99
                          and bracket.resting())

        def pressed(shape):
            fingers = {"tip_left", "tip_right", "gripper"}
            return any(shape in pair and pair & fingers for pair, _, force in contacts if force > 0.)

        catch_motion = float(np.dot(np.asarray(CATCH_OPEN) - catch.pos, CATCH_AXIS))
        button_motion = TEST_CAP - button.pos[2]
        tab_center = bracket.pos + bracket_basis @ TAB_POS
        tab_half = np.abs(bracket_basis) @ (np.array(TAB_SIZE) / 2)
        tongue_center = catch.pos + TONGUE_POS
        tongue_half = np.array(TONGUE_SIZE) / 2
        overlap = np.minimum(tab_center + tab_half, tongue_center + tongue_half) - np.maximum(
            tab_center - tab_half, tongue_center - tongue_half)
        tongue_clearance = (tongue_center[2] - tongue_half[2]) - (tab_center[2] + tab_half[2])
        if not bracket_on or not carrier_on:
            self.test_result = None
        if catch_motion >= CATCH_STROKE * .90 and pressed("RedCatch"):
            self.latch = True
        retained = (self.latch and catch_motion >= CATCH_STROKE * .90
                    and min(overlap[:2]) >= .003 and -.001 <= tongue_clearance <= .003)
        if button_motion >= .0045 and pressed("BlueTestKey"):
            self.test_time += dt
            if self.test_time >= .30:
                self.test_result = bool(carrier_on and bracket_on and retained)
                if self.test_result and self.proof_origin is None:
                    self.proof_origin = float(carrier.pos[2])
                    self.proof_elapsed = self.proof_max_lift = 0.
                    self.proof_force = PROOF_FORCE_N
        else:
            self.test_time = 0.
        clear = all(hand.position[2] > TOP + .133 or abs(hand.position[1]) > .160 for hand in hands.values())
        score = .25 if carrier_on else 0.
        if score and bracket_on:
            score = .65 if retained else .45
        if (score == .65 and self.test_result is True and self.proof_pass is True
                and button_motion < .001 and clear):
            score = 1.
        return dict(score=score, carrier=carrier_on, bracket=bracket_on, catch=bool(retained),
                    carrier_position_m=list(map(float, carrier.pos)),
                    bracket_position_m=list(map(float, bracket.pos)),
                    bracket_local_z=bracket_basis[:,2].tolist(),
                    measured_actuations=[pressed("RedCatch"), pressed("BlueTestKey")],
                    carrier_yaw_rad=float(yaw),
                    carrier_lift_mm=round(lift * 1000, 4),
                    detent_engaged=self.detent_engaged,
                    detent_force_n=self.detent_force,
                    proof_force_n=self.proof_force,
                    proof_max_lift_mm=round(self.proof_max_lift * 1000, 4),
                    proof_pass=self.proof_pass,
                    catch_tab_overlap_mm=round(float(min(overlap[:2])) * 1000, 3),
                    catch_tab_clearance_mm=round(float(tongue_clearance) * 1000, 3),
                    catch_mm=round(float(catch_motion)*1000, 3),
                    button_mm=round(float(button_motion)*1000, 3), field_check=self.test_result,
                    caption="Field and retention checks passed" if self.test_result and self.proof_pass
                    else "Install and check the encoder")


class View:
    def __init__(self, state):
        self.g = Geometry()
        self.light = self.g.sphere("FieldLED", (TEST_XY[0]-.019, TEST_XY[1]+.012, TOP+.045),
                                   (.008,.008,.004), (.16,.16,.16))

    def update(self, state):
        self.g.color(self.light, (.05,.75,.22) if state.test_result
                     else (.76,.09,.08) if state.test_result is False else (.16,.16,.16))
