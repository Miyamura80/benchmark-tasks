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












class ZipperLoop(Fixture):
    title = "Add an easy-grip zipper loop"
    instruction = "Feed the loop through the slider tab, cinch it, then use it to pull the zipper closed."
    focus = (0.098, 0, 0.253)
    detail_camera = (0.51, 24, -70)
    eye_x = 0.079
    lead = (0.052, 0.053, 0.273)
    tail = (0.126, -0.090, 0.273)
    handles = (
        Handle("cord_lead", "lead_slide", "left", X, max_force=12),
        Handle("cord_tail", "tail_slide", "right", X, max_force=12),
    )

    def build(self, w):
        plate(w, x=0.101, sx=0.19, sy=0.177)
        for sign in (-1, 1):
            box(w, f"garment_swath_{sign}", (0.109, sign*0.067, 0.215),
                (0.120, 0.063, 0.006), (0.12, 0.19, 0.30, 1), **VIS)
            for k in range(36):
                x = -0.006+0.0061*k
                box(w, f"zip_tooth_{sign}_{k}", (x, sign*0.0068, 0.225),
                    (0.0019, 0.0032, 0.0024), STEEL, **VIS)
        slider = body(w, "zipper_slider", (self.eye_x, 0, 0.232))
        slide(slider, "zipper_slide", X, (-0.002, 0.092), damping=1.4, friction=0.42)
        element(slider, "inertial", pos=(0, 0, 0), mass=0.064, diaginertia=(0.0001, 0.0001, 0.0001))
        box(slider, "slider_body", (-0.012, 0, 0), (0.013, 0.013, 0.005), STEEL, **VIS)
        capsule(slider, "slider_tab_stem", (-0.005, 0, 0.004), (0, 0.022, 0.012), 0.004, STEEL, **VIS)
        for k, angle in enumerate(np.linspace(0, math.tau, 13)[:-1]):
            x = 0.009*math.cos(angle)
            y = 0.022+0.009*math.sin(angle)
            geom(slider, f"eye_{k}", "sphere", (x, y, 0.011), (0.0027,), STEEL, **VIS)
        for name, point, limit in (("lead", self.lead, 0.052), ("tail", self.tail, 0.141)):
            part = body(w, f"cord_{name}", point)
            slide(part, f"{name}_slide", X, (-0.003, limit), friction=0.28)
            handle_geom(part, f"woven_cord_end_{name}", size=(0.013, 0.014, 0.007),
                        color=ORANGE, mass=0.055)

    def reset(self, s):
        super().reset(s)
        self.threaded = False
        self.hitched = False
        self.hold_time = 0.0
        self.verified = False

    def before_step(self, s):
        super().before_step(s)
        lead = s.joint("lead_slide")
        tail = s.joint("tail_slide")
        # Draw the cord through a single plane. Tension is transmitted from
        # the cinched loop into the slider and opposed by rail friction.
        if self.hitched and tail > 0.036:
            target = float(np.clip(tail-0.036, 0, 0.090))
            grip = self.grip_force.get("tail_slide", 0)
            error = target - s.joint("zipper_slide")
            force = float(np.clip(330*error-3.5*s.speed("zipper_slide"), 0, 8.0))
            self.force(s, "zipper_slide", force)
            if grip > 0:
                self.force(s, "tail_slide", -force)
        lead_park = 0.039 if self.threaded else 0.0
        slide_force(s, self, "lead_slide", lead_park, k=65, damping=2, maximum=2.7)

    def after_step(self, s):
        self.threaded |= s.joint("lead_slide") > 0.031 and "lead_slide" in self.grips
        self.hitched |= (self.threaded and s.joint("lead_slide") > 0.024
                         and s.joint("tail_slide") > 0.024 and "tail_slide" in self.grips)
        closed = self.hitched and s.joint("zipper_slide") > 0.083
        self.verified = released_for(s, self, closed and not self.grips, 0.55)

    def measure(self, s):
        return dict(score=self.score((self.threaded, self.hitched, self.verified)),
                    threaded=self.threaded, hitch_set=self.hitched,
                    slider_mm=round(s.joint("zipper_slide")*1000, 1),
                    lead_mm=round(s.joint("lead_slide")*1000, 1),
                    tail_mm=round(s.joint("tail_slide")*1000, 1),
                    trial_verified=self.verified)

    def visual(self, s, scene):
        d = Drawing(scene, self.x_offset)
        a = self.lead[0] + s.joint("lead_slide")
        b = self.tail[0] + s.joint("tail_slide")
        eye = self.eye_x + s.joint("zipper_slide")
        if not self.hitched:
            n = min(a+0.009, eye)
            d.cord([(a, self.lead[1], self.lead[2]), (n, 0.039, 0.255),
                    (n, 0.022, 0.244), (n-0.013, 0.011, 0.251),
                    (b, self.tail[1], self.tail[2])], 0.0034, ORANGE)
        else:
            for side in (-1, 1):
                d.cord([(eye, 0.022+side*0.005, 0.246),
                        (0.5*(eye+b), -0.009+side*0.011, 0.262),
                        (b, self.tail[1]+side*0.003, self.tail[2])], 0.0034, ORANGE)
            d.cord([(eye-0.008, 0.021, 0.244), (eye, 0.028, 0.247),
                    (eye+0.009, 0.022, 0.244)], 0.0037, ORANGE)
        for pos in np.arange(self.eye_x, eye, 0.005):
            d.box((pos, 0, 0.225), (0.002, 0.006, 0.002), STEEL)

    def demo(self, s, incomplete=False):
        x, y, z = self.lead
        passed = (x+0.043, y, z)
        self.take(s, "left", self.lead, "Grip the loop's leading side")
        self.go(s, "left", passed, "Feed the cord bight through the metal eye")
        self.leave(s, "left", passed, "Leave a bight through the tab")
        self.park(s, "left")
        if not incomplete:
            x, y, z = self.tail
            self.take(s, "right", self.tail, "Grip the free end of the loop")
            self.go(s, "right", (x+0.032, y, z), "Draw the loop tight through the bight")
            self.go(s, "right", (x+0.131, y, z), "Pull the zipper with the new loop", time=2.1)
            s.wait(0.35)
            self.leave(s, "right", (x+0.131, y, z), "Release the loop and check the slider")
        s.wait(0.78)


FIXTURES = {"zipper-loop": ZipperLoop}
