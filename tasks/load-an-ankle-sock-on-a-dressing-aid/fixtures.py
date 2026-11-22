"""Contact-driven MuJoCo care-home fixtures and reference dual-YAM policies."""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

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


class SockAid(Fixture):
    title = "Load a sock on a dressing aid"
    instruction = "Stretch the ankle sock over both edges of the sock aid and leave it square and held by the rim."
    focus = (0.069, 0.0, 0.253)
    detail_camera = (0.46, 30, -66)
    start_x = 0.057
    stroke = 0.057
    sides = {"left": 1, "right": -1}
    handles = tuple(Handle(f"sock_{side}", f"cuff_{side}", side, X, max_force=15)
                    for side in ("left", "right"))

    def build(self, w):
        plate(w, sx=0.155, sy=0.19)
        box(w, "trough_bottom", (0.064, 0, 0.215), (0.076, 0.036, 0.010), BLUE, **VIS)
        for side, sign in self.sides.items():
            box(w, f"trough_wall_{side}", (0.070, sign * 0.041, 0.233),
                (0.070, 0.005, 0.020), BLUE, euler=(sign * -0.29, 0, 0), **VIS)
            box(w, f"lip_{side}", (0.116, sign * 0.047, 0.250),
                (0.011, 0.006, 0.005), PALE_BLUE, **VIS)
            capsule(w, f"toe_curve_{side}", (-0.015, sign * 0.023, 0.222),
                    (0.013, sign * 0.037, 0.226), 0.009, BLUE, **VIS)
            part = body(w, f"sock_{side}", self.point(side))
            slide(part, f"cuff_{side}", X, (-0.004, 0.064))
            handle_geom(part, f"knit_fold_{side}", size=(0.013, 0.012, 0.005), color=KNIT)

    def point(self, side, shift=0):
        return self.start_x + shift, self.sides[side] * 0.058, 0.265

    def reset(self, s):
        super().reset(s)
        self.hold_time = 0.0
        self.verified = False

    def before_step(self, s):
        super().before_step(s)
        for side in self.sides:
            name = f"cuff_{side}"
            # The rolled rim makes a second stable position once the elastic
            # cuff is past the lip. The gripper can still pull it back out.
            target = self.stroke if s.joint(name) > 0.043 else 0.0
            slide_force(s, self, name, target, k=150, damping=3, maximum=6)

    def after_step(self, s):
        vals = [s.joint(f"cuff_{side}") for side in self.sides]
        retained = min(vals) > 0.050 and abs(vals[0] - vals[1]) < 0.007 and not self.grips
        self.verified = released_for(s, self, retained, 0.50)

    def measure(self, s):
        left, right = (s.joint(f"cuff_{side}") for side in self.sides)
        return dict(score=1.0 if self.verified else 0.25 * sum(q > 0.050 for q in (left, right)),
                    left_mm=round(left * 1000, 1), right_mm=round(right * 1000, 1),
                    skew_mm=round(abs(left-right) * 1000, 1), retained=self.verified)

    def visual(self, s, scene):
        d = Drawing(scene, self.x_offset)
        a, b = (self.start_x+s.joint(f"cuff_{side}") for side in self.sides)
        for q in np.linspace(0, 1, 27):
            theta = np.linspace(0, math.pi, 27)
            outline = []
            for t in theta:
                lean = (1+math.cos(t)) / 2
                cuff = lean*a+(1-lean)*b
                width = 0.049*(0.40+0.60*math.sin(min(1.0, 3.3*q)*math.pi/2))
                outline.append((-0.016 + q*(cuff+0.016), width*math.cos(t),
                                0.229 + (0.018+0.012*q)*math.sin(t)))
            tint = (0.12, 0.43, 0.43, 1) if q > 0.92 else KNIT
            d.cord(outline, max(0.0018, (a+b+0.032)*0.008), tint)
        for sign in (-1, 1):
            d.cord([(0.139, sign*0.045, 0.236), (0.163, sign*0.068, 0.206),
                    (0.175, sign*0.106, 0.204), (0.151, sign*0.140, 0.204),
                    (0.128, sign*0.148, 0.207)], 0.0026, WHITE)

    def demo(self, s, incomplete=False):
        for side in ("left",) if incomplete else ("left", "right"):
            start = self.point(side)
            self.take(s, side, start, "Grip the elastic cuff")
            self.go(s, side, self.point(side, self.stroke+0.002), "Stretch the cuff over the rim", time=1.15)
            self.leave(s, side, self.point(side, self.stroke+0.002), "Release the cuff from the gripper")
        s.wait(0.75, "Check the sock holds on the aid")












FIXTURES = {"sock-aid": SockAid}
