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






class WaxGuard(Fixture):
    title = "Replace a hearing-aid wax guard"
    instruction = "Extract the used wax guard, clear the receiver port, and seat a fresh guard flush."
    focus = (0.109, 0.000, 0.277)
    detail_camera = (0.275, 112, -48)
    primary_detail = True
    recording_fps = 10
    tool_x = 0.109
    staged_y = 0.069
    tool_z = 0.324
    stem = 0.048
    port = 0.246
    seat = 0.2506
    tip_gap = 0.0054
    handles = tuple(Handle(f"guard_tool_{side}", f"tool_{side}_{joint_axis}", side, axis, max_force=9)
                    for side in ("left", "right") for joint_axis, axis in (("y", Y), ("z", Z)))

    def build(self, w):
        plate(w, sx=0.137, sy=0.145)
        box(w, "receiver_nest", (0.061, 0, 0.222), (0.051, 0.030, 0.020), STEEL, **VIS)
        cyl(w, "receiver", (self.tool_x, 0, self.port-0.010), 0.011, 0.0136, DARK, **VIS)
        for i, angle in enumerate(np.linspace(0, math.tau, 20, endpoint=False)):
            co, si = math.cos(angle), math.sin(angle)
            box(w, f"receiver_rim_{i}", (self.tool_x+0.0062*co, 0.0062*si, self.seat-0.0005),
                (0.0021, 0.00105, 0.0011), BLACK, euler=(0, 0, angle), **VIS)
        cyl(w, "receiver_bore", (self.tool_x, 0, self.seat-0.0015), 0.0037, 0.00035, DARK, **VIS)
        geom(w, "electronic_body", "ellipsoid", (0.009, 0.046, 0.217),
             (0.023, 0.013, 0.010), AMBER, **VIS)
        receiver_path = [(0.019, 0.039, 0.221), (0.034, 0.022, 0.219), (0.046, 0.005, 0.220),
                         (0.059, -0.006, 0.226), (self.tool_x, -0.005, 0.236)]
        for index, (a, b) in enumerate(zip(receiver_path, receiver_path[1:])):
            capsule(w, f"receiver_wire_{index}", a, b, 0.0017, BLACK, **VIS)
        for side, sign in (("left", 1), ("right", -1)):
            placed_y = sign*self.staged_y
            part = body(w, f"guard_tool_{side}", (self.tool_x, placed_y, self.tool_z))
            limits = (-0.075, 0.006) if sign > 0 else (-0.006, 0.075)
            slide(part, f"tool_{side}_y", Y, limits)
            slide(part, f"tool_{side}_z", Z, (-0.037, 0.047))
            tint = DARK if side == "left" else LILAC
            handle_geom(part, f"applicator_shank_{side}", size=(0.010, 0.013, 0.012), color=tint)
            capsule(part, f"applicator_taper_{side}", (0, 0, -0.010), (0, 0, -self.stem+0.003),
                    0.0043, tint, **VIS)
            cyl(part, f"applicator_end_{side}", (0, 0, -self.stem), 0.0025, 0.004, STEEL, **VIS)
            box(w, f"tool_cradle_{side}", (self.tool_x, placed_y, 0.209),
                (0.018, 0.018, 0.009), PALE_BLUE, **VIS)

        self.guard(w, "spent", 0.0, self.seat, AMBER)
        self.guard(w, "fresh", -self.staged_y, self.tool_z-self.stem-self.tip_gap, WHITE)

    def guard(self, w, label, start_y, start_z, color):
        guard = body(w, label+"_guard", (self.tool_x, start_y, start_z))
        joint(guard, label+"_y", "slide", Y, (-0.089, 0.089),
              damping=0.26, frictionloss=0.006, armature=0.0035)
        joint(guard, label+"_z", "slide", Z, (-0.040, 0.082),
              damping=0.26, frictionloss=0.004, armature=0.0035)
        element(guard, "inertial", pos=(0, 0, 0), mass=0.0025,
                diaginertia=(3e-8, 3e-8, 3e-8))
        flange = 0.0032
        for i, angle in enumerate(np.linspace(0, math.tau, 12, endpoint=False)):
            box(guard, f"{label}_flange_{i}", (flange*math.cos(angle), flange*math.sin(angle), 0.0004),
                (0.0007, 0.0010, 0.00055), color, euler=(0, 0, angle), **VIS)
        cyl(guard, label+"_sleeve", (0, 0, -0.001), 0.0027, 0.0011, color, **VIS)
        cyl(guard, label+"_mesh", (0, 0, 0.0005), 0.0025, 0.00013,
            (0.41, 0.39, 0.36, 1) if label == "spent" else (0.66, 0.70, 0.70, 1), **VIS)
        for i, (u, v) in enumerate(((0, 0), (0.0011, 0), (-0.0011, 0), (0, 0.0011), (0, -0.0011))):
            cyl(guard, f"{label}_pore_{i}", (u, v, 0.00066), 0.00023, 0.00008, DARK, **VIS)

    def tool(self, side, y=None, z=0.0):
        y = (self.staged_y if side == "left" else -self.staged_y) if y is None else y
        return self.tool_x, y, self.tool_z+z

    def tip(self, s, side):
        x, y, z = self.tool(side)
        return np.array((x, y+s.joint(f"tool_{side}_y"),
                         z+s.joint(f"tool_{side}_z")-self.stem))

    def reset(self, s):
        super().reset(s)
        self.spent_offset = None
        self.spent_free = False
        self.fresh_released = False
        self.extraction_peak = 0.0
        self.insertion_peak = 0.0
        self.insertion_hold = 0.0
        self.hold_time = 0.0
        self.verified = False

    def location(self, s, label):
        fresh = label == "fresh"
        y = -self.staged_y if fresh else 0.0
        z = self.tool_z-self.stem-self.tip_gap if fresh else self.seat
        return np.array((self.tool_x, y+s.joint(f"{label}_y"), z+s.joint(f"{label}_z")))

    def link(self, s, label, side, target):
        point = self.location(s, label)
        forces = []
        for axis, index in (("y", 1), ("z", 2)):
            slider = f"{label}_{axis}"
            actuator = f"tool_{side}_{axis}"
            error = target[index]-point[index]
            relative_speed = s.speed(actuator)-s.speed(slider)
            force = float(np.clip(350*error+0.64*relative_speed, -3.1, 3.1))
            self.force(s, slider, force)
            self.force(s, actuator, -force)
            forces.append(force)
        return forces[1]

    def dock(self, s, label, *, attached):
        point = self.location(s, label)
        if abs(point[1]) < 0.005 and point[2] < self.seat+0.006:
            # The tapered bore centres the sleeve and the shoulder limits
            # depth. Static retention relaxes when the extraction barb pulls
            # the spent sleeve beyond the shoulder's break-out distance.
            centering = -410*point[1]-0.55*s.speed(f"{label}_y")
            self.force(s, f"{label}_y", float(np.clip(centering, -2.2, 2.2)))
            depth = self.seat-point[2]
            stiffness = 2550 if depth >= 0 else (520 if attached else 0)
            load = stiffness*depth-0.8*s.speed(f"{label}_z")
            limit = 1.8 if label == "spent" else 4.4
            self.force(s, f"{label}_z", float(np.clip(load, -limit, 4.4)))

    def before_step(self, s):
        super().before_step(s)
        for side in ("left", "right"):
            name = f"tool_{side}_z"
            mass = float(s.model.body(f"guard_tool_{side}").mass[0])
            self.force(s, name, mass*9.81)
            tip = self.tip(s, side)
            if abs(tip[1]) < 0.004 and tip[2] < self.seat+0.0007:
                self.force(s, name, float(np.clip((self.seat+0.0007-tip[2])*760, 0, 4)))

        # Guide carriages counterbalance their own weight. The guard remains
        # an independent jointed part and feels each applied connection force.
        for label in ("spent", "fresh"):
            self.force(s, f"{label}_z", 0.0025*9.81)
        if not self.spent_free:
            self.dock(s, "spent", attached=True)
        if self.spent_offset is not None:
            load = self.link(s, "spent", "left", self.tip(s, "left")+self.spent_offset)
            if not self.spent_free:
                self.extraction_peak = max(self.extraction_peak, load)

        if not self.fresh_released:
            target = self.tip(s, "right")-np.array((0, 0, self.tip_gap))
            pressure = -self.link(s, "fresh", "right", target)
            at_port = abs(target[1]) < 0.0017 and self.spent_free
            new = self.location(s, "fresh")
            seated = at_port and abs(new[2]-self.seat) < 0.00155 and abs(new[1]) < 0.0018
            if at_port:
                self.dock(s, "fresh", attached=False)
                self.insertion_peak = max(self.insertion_peak, pressure)
            pressing = seated and pressure > 0.55 and "tool_right_z" in self.grips
            self.insertion_hold = self.insertion_hold + DT if pressing else 0.0
            if self.insertion_hold >= 0.05:
                self.fresh_released = True
        else:
            self.dock(s, "fresh", attached=True)

    def after_step(self, s):
        old, new = (self.location(s, label) for label in ("spent", "fresh"))
        left, right = (self.tip(s, side) for side in ("left", "right"))
        gap = left-old
        # Contact of the extractor nose with the centre of the old mesh
        # engages its retention barb. An off-centre nose cannot capture it.
        if self.spent_offset is None and "tool_left_z" in self.grips:
            if abs(gap[1]) < 0.0019 and -0.0028 < gap[2] < 0.0032:
                self.spent_offset = old-left
        self.spent_free |= bool(old[2] > self.seat+0.0046 and self.spent_offset is not None)
        old_parked = self.spent_free and old[2] > self.seat+0.013 and old[1] > 0.036
        flush = self.fresh_released and abs(new[2]-self.seat) < 0.0017 and abs(new[1]) < 0.0018
        released = left[1] > 0.035 and right[1] < -0.035 and not self.grips
        self.verified = released_for(s, self, old_parked and flush and released, 0.40)

    def measure(self, s):
        old, new = (self.location(s, label) for label in ("spent", "fresh"))
        old_clear = bool(self.spent_free and old[2] > self.seat+0.013 and old[1] > 0.036)
        flush = bool(self.fresh_released and abs(new[2]-self.seat) < 0.0017 and abs(new[1]) < 0.0018)
        return dict(score=self.score((old_clear, flush, self.verified)),
                    old_clear=old_clear, replacement_flush=flush,
                    extract_n=round(self.extraction_peak, 2), press_n=round(self.insertion_peak, 2),
                    flush_error_mm=round(float(abs(new[2]-self.seat)*1000), 2),
                    tools_parked=self.verified)

    def visit(self, s, side, phase):
        start = self.tool(side)
        self.take(s, side, start, "Grip the wax guard applicator", lift=0.027)
        self.go(s, side, self.tool(side, z=0.015), "Lift the applicator in its guide")
        self.go(s, side, self.tool(side, y=0, z=0.015), "Align above the receiver opening", time=1.1)
        self.go(s, side, self.tool(side, y=0, z=-0.034), phase, time=1.0)
        s.wait(0.38)
        self.go(s, side, self.tool(side, y=0, z=0.016), "Withdraw the applicator")
        self.go(s, side, self.tool(side, z=0.016), "Return the tool to its parking channel")
        self.go(s, side, self.tool(side), "Lower the applicator into its cradle")
        self.leave(s, side, self.tool(side), "Release the parked applicator", lift=0.022)
        self.park(s, side)

    def demo(self, s, incomplete=False):
        self.visit(s, "left", "Extract the used guard")
        if not incomplete:
            self.visit(s, "right", "Seat the new guard")
        s.wait(0.65, "Check the receiver and both tools")








FIXTURES = {"wax-guard": WaxGuard}
