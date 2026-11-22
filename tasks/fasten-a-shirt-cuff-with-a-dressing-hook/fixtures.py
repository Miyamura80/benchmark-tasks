"""Contact-driven MuJoCo care-home fixtures and reference dual-YAM policies."""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from hand_model import add_cuff, add_hand, add_sleeve
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










class ButtonHook(Fixture):
    title = "Fasten a cuff with a dressing hook"
    instruction = "On the silicone training arm, hold the cuff, feed the dressing hook through the buttonhole, catch the button, and leave the cuff fastened."
    focus = (0.075, -0.017, 0.263)
    detail_focus = (0.125, -0.012, 0.257)
    detail_camera = (0.47, 137, -57)
    primary_detail = True
    cuff = (0.054, 0.050, 0.280)
    handle = (0.122, -0.058, 0.287)
    button = (0.052, -0.058, 0.257)
    hook_reach = 0.053
    handles = (
        Handle("cuff_flap", "cuff_tension", "left", Y, max_force=11),
        Handle("button_hook", "wire_x", "right", X, max_force=11),
        Handle("button_hook", "wire_z", "right", Z, max_force=11),
    )

    def build(self, w):
        plate(w, sx=0.148, sy=0.169)
        body(w, "hand_mount")
        box(w, "forearm_pad", (-0.033, -0.005, 0.211), (0.045, 0.030, 0.011), DARK, **VIS)
        flap = body(w, "cuff_flap", self.cuff)
        slide(flap, "cuff_tension", Y, (-0.004, 0.038))
        handle_geom(flap, "cuff_hem", size=(0.013, 0.014, 0.005), color=BLUE)
        hook = body(w, "button_hook", self.handle)
        slide(hook, "wire_x", X, (-0.038, 0.064))
        slide(hook, "wire_z", Z, (-0.031, 0.030))
        handle_geom(hook, "hook_handle", size=(0.019, 0.010, 0.009), color=TEAL)
        capsule(hook, "hook_shaft", (-0.017, 0, -0.004), (-0.041, 0, -0.009), 0.0021, STEEL, **VIS)
        for sign in (-1, 1):
            capsule(hook, f"wire_curved{sign}", (-0.039, 0, -0.008),
                    (-0.050, sign*0.008, -0.011), 0.0015, STEEL, **VIS)
            capsule(hook, f"wire_tip{sign}", (-0.050, sign*0.008, -0.011),
                    (-0.061, 0, -0.014), 0.0015, STEEL, **VIS)
        button = body(w, "button", self.button)
        slide(button, "button_travel", X, (-0.003, 0.036), damping=0.8)
        element(button, "inertial", pos=(0, 0, 0), mass=0.017, diaginertia=(0.0001, 0.0001, 0.0001))
        cyl(button, "button_face", (0, 0, 0), 0.011, 0.0027, WHITE, **VIS)
        for dx in (-1, 1):
            for dy in (-1, 1):
                cyl(button, f"button_hole_{dx}_{dy}", (dx*0.003, dy*0.003, 0.0029),
                    0.0012, 0.0003, DARK, **VIS)

    def constraints(self, tree):
        hand = tree.find(".//body[@name='hand_mount']")
        add_hand(tree, hand)
        add_sleeve(tree, hand)
        add_cuff(tree, hand)

    def reset(self, s):
        super().reset(s)
        self.caught = False
        self.was_passed = False
        self.hold_time = 0.0
        self.verified = False

    def offset(self, s):
        return s.joint("wire_x"), s.joint("wire_z")

    def before_step(self, s):
        super().before_step(s)
        x, z = self.offset(s)
        tension = s.joint("cuff_tension")
        self.force(s, "wire_z", float(s.model.body("button_hook").mass[0])*9.81)
        slide_force(s, self, "cuff_tension", 0, k=66, damping=2, maximum=4)
        engaged = self.caught and "wire_x" in self.grips and z < -0.011 and tension > 0.017
        held = s.joint("button_travel") > 0.024
        target = float(np.clip(x+0.021, 0.0, 0.032)) if engaged else (0.030 if held else 0.0)
        slide_force(s, self, "button_travel", target, k=305, damping=4, maximum=6)
        if engaged:
            self.force(s, "wire_x", -min(3.0, 95*max(0, target-s.joint("button_travel"))))

    def after_step(self, s):
        x, z = self.offset(s)
        self.caught |= (s.joint("cuff_tension") > 0.017 and x < -0.016 and z < -0.013
                        and "wire_x" in self.grips)
        self.was_passed |= s.joint("button_travel") > 0.025
        stable = self.was_passed and s.joint("button_travel") > 0.025 and z > 0.008
        self.verified = released_for(s, self, stable and not self.grips, 0.48)

    def measure(self, s):
        return dict(score=self.score((self.caught, self.was_passed, self.verified)),
                    hook_engaged=self.caught, button_mm=round(s.joint("button_travel")*1000, 1),
                    cuff_tension_mm=round(s.joint("cuff_tension")*1000, 1),
                    fastened=self.verified)

    def visual(self, s, scene):
        d = Drawing(scene, self.x_offset)
        band = s.joint("cuff_tension")
        # Two fabric lips leave a real visual opening around the scored
        # passage. The cloth edge is a guide model, not a full cloth solve.
        for x in (0.031, 0.095):
            d.cord([(x, -0.005+0.0347*math.sin(a), 0.244+0.0347*math.cos(a))
                    for a in np.linspace(-2.47, 2.52, 24)], 0.00075, (0.51, 0.62, 0.76, 1))
        for sign in (-1, 1):
            opening_y = -0.057 + sign*(0.0038+band*0.10)
            d.cord([(0.044, opening_y, 0.263), (0.067, opening_y, 0.263),
                    (0.081, opening_y+sign*0.003, 0.263)], 0.0018, PALE_BLUE)
        d.cord([(0.054, 0.034, 0.258), (0.054, 0.053+band*0.63, 0.273),
                (0.054, 0.050+band, 0.280)], 0.0050, BLUE)

    def wire(self, dx=0, dz=0):
        x, y, z = self.handle
        return x+dx, y, z+dz

    def demo(self, s, incomplete=False):
        x, y, z = self.cuff
        taut = (x, y+0.027, z)
        self.take(s, "left", self.cuff, "Grip the cuff edge")
        self.go(s, "left", taut, "Tension and open the buttonhole")
        self.take(s, "right", self.handle, "Grip the dressing hook")
        self.go(s, "right", self.wire(dz=-0.018), "Lower the wire loop into the buttonhole")
        self.go(s, "right", self.wire(-0.029, -0.018), "Feed the loop behind the button")
        s.wait(0.26, "Catch the button with the wire loop")
        if not incomplete:
            self.go(s, "right", self.wire(0.016, -0.018), "Draw the button through the cuff", time=1.1)
            s.wait(0.28)
            self.go(s, "right", self.wire(0.027, 0.020), "Withdraw the dressing hook")
            self.leave(s, "right", self.wire(0.027, 0.020), "Release the dressing hook")
        self.leave(s, "left", taut, "Release the cuff and inspect the button")
        s.wait(0.76)




FIXTURES = {"button-hook": ButtonHook}
