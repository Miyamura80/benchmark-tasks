"""Contact-driven MuJoCo care-home fixtures and reference dual-YAM policies."""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from bath_fluid import BathFluid
from station import DT, HZ, SIDES, element, geom, station_xml

STEEL = (0.55, 0.61, 0.66, 1)
DARK = (0.14, 0.18, 0.21, 1)
BLACK = (0.055, 0.065, 0.073, 1)
WHITE = (0.88, 0.89, 0.83, 1)
TEAL = (0.09, 0.45, 0.47, 1)
ORANGE = (0.92, 0.37, 0.12, 1)
BLUE = (0.12, 0.28, 0.54, 1)
VIS = {"contype": 0, "conaffinity": 0, "mass": 0}


def box(parent, name, pos, size, color, **kw):
    return geom(parent, name, "box", pos, size, color, **kw)


def cyl(parent, name, pos, radius, half_depth, color, **kw):
    return geom(parent, name, "cylinder", pos, (radius, half_depth), color, **kw)


def capsule(parent, name, start, end, radius, color, **kw):
    return element(
        parent,
        "geom",
        name=name,
        type="capsule",
        fromto=(*start, *end),
        size=radius,
        rgba=color,
        **kw,
    )


def body(parent, name, pos=(0, 0, 0)):
    return element(parent, "body", name=name, pos=pos)


def joint(parent, name, kind, axis, limits, **kw):
    return element(parent, "joint", name=name, type=kind, axis=axis, range=limits, **kw)


def plate(parent, x=0.045, y=0, sx=0.16, sy=0.20):
    box(parent, "mat", (x, y, 0.195), (sx, sy, 0.005), (0.34, 0.40, 0.40, 1))


@dataclass(frozen=True)
class Handle:
    name: str
    joint: str
    side: str
    axis: tuple
    max_force: float = 25.0


class Fixture:
    # A grip retains the measured contact offset along a part's allowed DOF.
    # No object state or score is set by the scripted policy.
    focus = (0.06, 0, 0.267)
    handles = ()
    x_offset = -0.067
    recording_fps = 5
    primary_detail = False
    wide_camera = None
    detail_focus = None

    def __init__(self, variant=0):
        self.variant = variant
        self.focus = (self.focus[0] + self.x_offset, *self.focus[1:])

    def compile(self):
        tree, world = station_xml()
        world.set("pos", f"{self.x_offset} 0 0")
        self.build(world)
        self.constraints(tree)
        return mj.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"))

    def constraints(self, tree):
        pass

    def reset(self, s):
        self.grips = {}
        self.grip_force = {}

    def visual(self, s, scene):
        pass

    def control(self, s):
        s.wait(6, "Leave the fault unchanged")

    def detail_control(self, s):
        self.demo(s, incomplete=True)

    @staticmethod
    def fingertips(s, obj, side):
        ob = s.model.body(obj).id
        fingers = set()
        for c in s.data.contact:
            if c.dist > 0.001:
                continue
            a, b = s.model.geom(c.geom1), s.model.geom(c.geom2)
            for part, hand in ((a, b), (b, a)):
                if part.bodyid[0] == ob and hand.name.startswith(f"{side}_tip_"):
                    fingers.add(int(hand.bodyid[0]))
        return fingers

    def before_step(self, s):
        s.data.qfrc_applied[:] = 0
        self.grip_force = {}
        for h in self.handles:
            command = float(s.data.actuator(f"{h.side}_finger_-1").ctrl[0])
            if command > 0.020:
                self.grips.pop(h.joint, None)
                continue
            q = s.joint(h.joint)
            axis = np.asarray(h.axis, dtype=float)
            tcp = s.data.site(f"{h.side}_tcp").xpos.copy()
            if h.joint not in self.grips:
                if command > 0.007 or len(self.fingertips(s, h.name, h.side)) != 2:
                    continue
                self.grips[h.joint] = (tcp, q)
            start_tcp, start_q = self.grips[h.joint]
            target = start_q + float(axis @ (tcp - start_tcp))
            error = target - q
            if abs(error) > 0.055:
                self.grips.pop(h.joint, None)
                continue
            force = float(np.clip(1350 * error - 18 * s.speed(h.joint), -h.max_force, h.max_force))
            self.grip_force[h.joint] = force
            self.force(s, h.joint, force)
            hand = s.model.site_bodyid[s.sites[h.side]]
            mj.mj_applyFT(s.model, s.data, -force*axis, np.zeros(3), tcp,
                          hand, s.data.qfrc_applied)

    @staticmethod
    def force(s, name, value):
        s.data.qfrc_applied[s.model.joint(name).dofadr[0]] += value

    def after_step(self, s):
        pass

    def go(self, s, side, point, phase, *, opening=None, time=0.85, yaw=0.0):
        x, y, z = point
        s.move_arm(
            side, (x + self.x_offset, y, z), time, phase, opening=opening, yaw=yaw
        )

    def take(self, s, side, point, phase, *, yaw=0.0, lift=0.047):
        x, y, z = point
        self.go(s, side, (x, y, z + lift), phase, opening=0.72, yaw=yaw, time=1.25)
        self.go(s, side, point, phase, yaw=yaw)
        self.go(s, side, point, phase, opening=0.008, yaw=yaw, time=0.5)
        s.wait(0.16)

    def leave(self, s, side, point, phase, *, yaw=0.0, lift=0.035):
        x, y, z = point
        self.go(s, side, point, phase, opening=0.68, yaw=yaw, time=0.25)
        self.go(s, side, (x, y, z + lift), phase, yaw=yaw, time=0.6)

    def park(self, s, side):
        index = SIDES.index(side)
        start = s.targets[index].copy()
        ready = np.array((0, 1, 1, -0.6, 0, 0, 1))
        count = round(1.4*HZ)
        s.phase = "Move the released arm back to standby"
        for step in range(1, count+1):
            t = step/count
            blend = t*t*(3.0-2.0*t)
            targets = s.targets.copy()
            targets[index] = start+blend*(ready-start)
            s.action(targets)
        s.yaw_targets.pop(side, None)
        s.yaw_choices.pop(side, None)

    @staticmethod
    def score(flags):
        # Initial setup is unscored; each stop is a distinct achieved milestone.
        for index, flag in enumerate(flags):
            if not flag:
                return index / len(flags)
        return 1.0


KNIT = (0.07, 0.34, 0.36, 1)
LILAC = (0.42, 0.32, 0.67, 1)
PALE_BLUE = (0.42, 0.70, 0.80, 1)
PINK = (0.78, 0.38, 0.42, 1)
AMBER = (0.89, 0.63, 0.24, 1)
X, Y, Z = (1, 0, 0), (0, 1, 0), (0, 0, 1)


def slide(parent, name, axis, limits, *, damping=1.1, friction=0.12):
    return joint(parent, name, "slide", axis, limits, damping=damping,
                 frictionloss=friction, armature=0.005)


def handle_geom(parent, name, *, at=(0, 0, 0), size=(0.012, 0.015, 0.010),
                color=TEAL, mass=0.10):
    return box(parent, name, at, size, color, mass=mass)


def slide_force(sim, fixture, name, target, *, k=460, damping=6, maximum=12):
    delta = target - sim.joint(name)
    force = float(np.clip(k * delta - damping * sim.speed(name), -maximum, maximum))
    fixture.force(sim, name, force)


def released_for(sim, fixture, condition, seconds):
    fixture.hold_time = fixture.hold_time + DT if condition else 0.0
    return fixture.hold_time >= seconds


class Drawing:
    def __init__(self, scene, offset):
        self.scene = scene
        self.offset = np.array((offset, 0, 0))

    def shape(self, kind, at, size, color, rotation=None):
        index = self.scene.ngeom
        if index >= self.scene.maxgeom:
            return None
        surface = self.scene.geoms[index]
        mj.mjv_initGeom(surface, kind, np.asarray(size, dtype=float),
                        np.asarray(at) + self.offset,
                        np.eye(3).ravel() if rotation is None else rotation.ravel(),
                        np.asarray(color, dtype=np.float32))
        self.scene.ngeom += 1
        return surface

    def sphere(self, at, size, color):
        self.shape(mj.mjtGeom.mjGEOM_ELLIPSOID, at, size, color)

    def box(self, at, size, color, rotation=None):
        self.shape(mj.mjtGeom.mjGEOM_BOX, at, size, color, rotation)

    def disk(self, at, radius, depth, color):
        self.shape(mj.mjtGeom.mjGEOM_CYLINDER, at, (radius, depth, 0), color)

    def cord(self, points, radius, color):
        for a, b in zip(points, points[1:]):
            if np.linalg.norm(np.subtract(a, b)) < 0.0001:
                continue
            surface = self.shape(mj.mjtGeom.mjGEOM_CAPSULE, a, (radius, radius, radius), color)
            if surface is not None:
                mj.mjv_connector(surface, mj.mjtGeom.mjGEOM_CAPSULE, radius,
                                 np.asarray(a) + self.offset, np.asarray(b) + self.offset)




class DentureBath(Fixture):
    title = "Drain and reset a denture bath"
    instruction = "Lift the basket with the denture inside, drain the cup into the catch basin, then put the basket back."
    focus = (0.087, -0.015, 0.270)
    detail_camera = (0.40, 73, -41)
    primary_detail = True
    recording_fps = 10
    x_offset = -0.032
    basket = (0.032, 0.033, 0.316)
    basket_grip = (0.032, 0.033, 0.299)
    pivot = (0.078, -0.090, 0.272)
    lever = (0.123, -0.099, 0.292)
    handles = (
        Handle("basket", "basket_lift", "left", Z, max_force=15),
        Handle("pour_lever", "pour_travel", "right", X, max_force=18),
    )

    def constraints(self, tree):
        equality = tree.find("equality")
        if equality is None:
            equality = element(tree, "equality")
        element(equality, "joint", joint1="cup_angle", joint2="pour_travel",
                polycoef=(0, 26.5, 0, 0, 0), solref=(0.006, 1))

    def build(self, w):
        plate(w, x=0.080, sx=0.19, sy=0.17)
        box(w, "bath_stand", (0.026, -0.019, 0.206), (0.070, 0.070, 0.006), STEEL, **VIS)
        cup = body(w, "cup", self.pivot)
        joint(cup, "cup_angle", "hinge", Y, (-0.035, 1.79), damping=0.075,
              armature=0.009, frictionloss=0.014)
        center = (-0.050, 0.068)
        for dx in (-1, 1):
            box(cup, f"cup_side_x{dx}", (center[0]+dx*0.042, center[1], -0.025),
                (0.004, 0.041, 0.025), (0.16, 0.53, 0.49, 0.88), **VIS)
            box(cup, f"cup_side_y{dx}", (center[0], center[1]+dx*0.045, -0.025),
                (0.046, 0.004, 0.025), (0.16, 0.53, 0.49, 0.88), **VIS)
        box(cup, "cup_floor", (*center, -0.052), (0.046, 0.048, 0.004), TEAL, **VIS)
        box(cup, "pouring_lip", (-0.007, 0.068, -0.005),
            (0.009, BathFluid.lip_half_width, 0.003), (0.23, 0.61, 0.55, 0.82), **VIS)
        for sign in (-1, 1):
            capsule(cup, f"lip_rail_{sign}", (-0.014, 0.068+sign*0.019, 0.001),
                    (0.002, 0.068+sign*0.019, 0.001), 0.0025, TEAL, **VIS)
        element(cup, "inertial", pos=(-0.028, 0.032, -0.021), mass=0.28,
                diaginertia=(0.0006, 0.0010, 0.0009))
        lever = body(w, "pour_lever", self.lever)
        slide(lever, "pour_travel", X, (-0.003, 0.074))
        handle_geom(lever, "pour_handle", color=TEAL)
        box(w, "tipping_rack", (0.154, -0.099, 0.240), (0.048, 0.014, 0.008), DARK, **VIS)
        capsule(lever, "tipping_pushrod", (0, 0, -0.006), (0, 0, -0.041),
                0.004, STEEL, **VIS)
        box(cup, "pivot_axle", (0, 0.025, 0), (0.008, 0.025, 0.008), STEEL, **VIS)
        lid = body(w, "open_lid", (-0.041, -0.020, 0.296))
        box(lid, "lid_face", (0, 0, 0), (0.005, 0.049, 0.045), (0.35, 0.65, 0.59, 0.8),
            euler=(0, -0.36, 0), **VIS)
        basket = body(w, "basket", self.basket)
        slide(basket, "basket_lift", Z, (-0.002, 0.067))
        handle_geom(basket, "basket_handle", at=(0, 0, -0.017),
                    size=(0.013, 0.012, 0.008), color=WHITE)
        capsule(basket, "basket_handle_back", (0, -0.074, -0.021), (0, -0.009, -0.017), 0.003, WHITE, **VIS)
        capsule(basket, "basket_handle_front", (0, -0.002, -0.028), (0, 0, -0.017), 0.003, WHITE, **VIS)
        box(basket, "basket_base", (0, -0.052, -0.074), (0.032, 0.031, 0.003), WHITE, **VIS)
        for row in range(10):
            x = -0.032 + row * 0.007
            for side in (-1, 1):
                capsule(basket, f"basket_spoke_{side}_{row}", (x, -0.052+side*0.034, -0.074),
                        (x, -0.052+side*0.036, -0.040), 0.0018, WHITE, **VIS)
        for side in (-1, 1):
            box(basket, f"basket_rim_{side}", (0, -0.052+side*0.035, -0.042),
                (0.036, 0.0026, 0.003), WHITE, **VIS)
            box(basket, f"basket_edge_{side}", (side*0.035, -0.052, -0.058),
                (0.003, 0.035, 0.017), WHITE, **VIS)
        for index, angle in enumerate(np.linspace(-1.55, 1.55, 14)):
            x = -0.006+0.023*math.cos(angle)
            y = -0.052+0.022*math.sin(angle)
            geom(basket, f"denture_gum_{index}", "ellipsoid", (x, y, -0.065),
                 (0.009, 0.006, 0.005), PINK, **VIS)
            geom(basket, f"denture_tooth_{index}", "ellipsoid", (x, y, -0.060),
                 (0.005, 0.0032, 0.005), WHITE, **VIS)
        cx, cy, sx, sy, floor = BathFluid.catch
        box(w, "catch_base", (cx, cy, floor-0.004), (sx+0.003, sy+0.003, 0.004), DARK, **VIS)
        for side in (-1, 1):
            box(w, f"catch_x{side}", (cx+side*(sx+0.002), cy, floor+0.012),
                (0.003, sy+0.003, 0.012), PALE_BLUE, **VIS)
            box(w, f"catch_y{side}", (cx, cy+side*(sy+0.002), floor+0.012),
                (sx+0.003, 0.003, 0.012), PALE_BLUE, **VIS)

    def reset(self, s):
        super().reset(s)
        self.water = BathFluid(self.pivot)
        self.raised = False
        self.hold_time = 0.0
        self.verified = False

    def before_step(self, s):
        super().before_step(s)
        angle = s.joint("cup_angle")
        rate = s.speed("cup_angle")
        clearance = s.joint("basket_lift")
        interference = min(2.4, 11.0*max(0, angle-0.23)) if clearance < 0.039 else 0.0
        self.force(s, "cup_angle", -0.080*angle-0.032*rate+self.water.weight_torque()-interference)
        slide_force(s, self, "basket_lift", 0.0, k=29, damping=3, maximum=3)

    def after_step(self, s):
        lift = s.joint("basket_lift")
        tilt = s.joint("cup_angle")
        self.water.advance(DT, tilt, s.speed("cup_angle"), lift)
        self.raised |= lift > 0.047
        seated = self.water.cup_ml < 3.0 and self.water.caught_ml >= 105 and self.water.spill_ml < 3.0
        seated &= self.water.in_flight_ml < 0.1 and lift < 0.009 and abs(tilt) < 0.10 and not self.grips
        self.verified = released_for(s, self, seated, 0.45)

    def measure(self, s):
        water = self.water
        return dict(score=self.score((self.raised, water.caught_ml >= 105, self.verified)),
                    caught_ml=round(water.caught_ml, 1), in_cup_ml=round(water.cup_ml, 2),
                    spill_ml=round(water.spill_ml, 2), flow_ml_s=round(water.flow_ml_s, 1),
                    in_flight_ml=round(water.in_flight_ml, 2), lift_mm=round(s.joint("basket_lift")*1000, 1),
                    mass_balance_error_ml=round(water.total_ml-110, 8), reset_verified=self.verified)

    def visual(self, s, scene):
        self.water.draw(Drawing(scene, self.x_offset))

    def demo(self, s, incomplete=False):
        x, y, z = self.basket_grip
        raised = (x, y, z+0.054)
        self.take(s, "left", self.basket_grip, "Grip the denture basket", lift=0.032)
        self.go(s, "left", raised, "Lift the basket clear of the bath")
        if not incomplete:
            self.take(s, "right", self.lever, "Grip the cradle's tipping lever")
            a, b, c = self.lever
            self.go(s, "right", (a+0.068, b, c), "Tip the cup into the catch basin", time=1.3)
            s.wait(4.6, "Drain into the basin")
            self.go(s, "right", self.lever, "Return the empty cup upright", time=1.3)
            self.leave(s, "right", self.lever, "Release the upright cup")
            self.go(s, "left", self.basket_grip, "Lower the basket back into the cup")
            self.leave(s, "left", self.basket_grip, "Release the reseated basket")
        s.wait(0.65, "Check the bath is reassembled")










FIXTURES = {"denture-bath": DentureBath}
