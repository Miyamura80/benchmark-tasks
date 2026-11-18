"""Contact-driven MuJoCo workshop fixtures and reference dual-YAM policies."""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from station import DT, element, geom, station_xml
from realism import decorate, render_materials

STEEL = (0.55, 0.61, 0.66, 1)
DARK = (0.14, 0.18, 0.21, 1)
BLACK = (0.055, 0.065, 0.073, 1)
WHITE = (0.88, 0.89, 0.83, 1)
PAPER = (0.89, 0.76, 0.49, 1)
TEAL = (0.09, 0.45, 0.47, 1)
ORANGE = (0.92, 0.37, 0.12, 1)
BRASS = (0.81, 0.58, 0.22, 1)
COPPER = (0.73, 0.35, 0.20, 1)
BLUE = (0.12, 0.28, 0.54, 1)
RUBBER = (0.21, 0.22, 0.25, 1)
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


def rim(parent, name, radius, z, width, tint, count=18):
    for i in range(count):
        t = math.tau * i / count
        box(
            parent,
            f"{name}_{i}",
            (radius * math.cos(t), radius * math.sin(t), z),
            (width / 2, radius * math.tan(math.pi / count) * 1.04, width / 2),
            tint,
            euler=(0, 0, t),
            **VIS,
        )


@dataclass(frozen=True)
class Handle:
    name: str
    joint: str
    side: str
    axis: tuple
    kind: str = "slide"
    max_force: float = 25.0


class Fixture:
    # A grip retains the measured contact offset along a part's allowed DOF.
    # No object state or score is set by the scripted policy.
    focus = (0.06, 0, 0.267)
    handles = ()
    x_offset = -0.067

    def __init__(self, variant=0):
        self.variant = variant
        self.focus = (self.focus[0] + self.x_offset, *self.focus[1:])

    def compile(self):
        tree, world = station_xml()
        world.set("pos", f"{self.x_offset} 0 0")
        self.build(world)
        self.constraints(tree)
        decorate(tree, world, type(self).__name__)
        return mj.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"))

    def constraints(self, tree):
        pass

    def reset(self, s):
        self.grips = {}
        self.contact_log = set()

    def visual(self, s, scene):
        render_materials(s, scene)

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

    def active_handles(self):
        return self.handles

    def before_step(self, s):
        s.data.qfrc_applied[:] = 0
        for h in self.active_handles():
            command = float(s.data.actuator(f"{h.side}_finger_-1").ctrl[0])
            if command > 0.020:
                self.grips.pop(h.name, None)
                continue
            q = s.joint(h.joint)
            axis = np.asarray(h.axis, dtype=float)
            tcp = s.data.site(f"{h.side}_tcp").xpos.copy()
            origin = s.position(h.name)
            if h.name not in self.grips:
                if command > 0.007 or len(self.fingertips(s, h.name, h.side)) != 2:
                    continue
                self.grips[h.name] = (tcp, origin, q)
                self.contact_log.add(h.name)
            start_tcp, start_origin, start_q = self.grips[h.name]
            if h.kind == "slide":
                target = start_q + float(axis @ (tcp - start_tcp))
                gain, damping = 1350, 18
            else:
                v, u = start_tcp - start_origin, tcp - origin
                v -= axis * (axis @ v)
                u -= axis * (axis @ u)
                angle = math.atan2(axis @ np.cross(v, u), v @ u)
                near = q - start_q
                target = start_q + near + math.remainder(angle - near, math.tau)
                gain, damping = 1.8, 0.065
            error = target - q
            if abs(error) > (0.055 if h.kind == "slide" else 1.35):
                self.grips.pop(h.name, None)
                continue
            self.force(
                s,
                h.joint,
                np.clip(
                    gain * error - damping * s.speed(h.joint), -h.max_force, h.max_force
                ),
            )

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

    def turn(self, s, side, base, radial, first, last, phase, *, vertical=False):
        count = max(1, math.ceil(abs(last - first) / 0.19))
        for angle in np.linspace(first, last, count + 1)[1:]:
            x, y, z = base
            point = (
                (x, y + radial * math.cos(angle), z + radial * math.sin(angle))
                if vertical
                else (x + radial * math.cos(angle), y + radial * math.sin(angle), z)
            )
            self.go(s, side, point, phase, time=0.18)

    @staticmethod
    def score(flags):
        # Initial setup is unscored; each stop is a distinct achieved milestone.
        for index, flag in enumerate(flags):
            if not flag:
                return index / len(flags)
        return 1.0




class Olive(Fixture):
    title = "Pull a seized compression olive off a copper pipe"
    instruction = "Turn the fitted olive puller to free the brass ring, lift it clear, and slide the spare ring onto the pipe to check the fit."
    skill = "Apply coaxial extraction force and prove the tube can be reused."
    focus = (0.036, 0.028, 0.27)
    pivot = (0.023, 0.078, 0.265)
    radius = 0.051
    ring_home = (0.023, -0.068, 0.282)
    handles = (
        Handle("extractor", "screw_turn", "left", (0, 0, 1), "hinge", 0.67),
        Handle("trial_ring", "check_fit", "right", (0, 0, -1)),
    )

    def build(self, w):
        plate(w)
        x, y, z = self.pivot
        box(w, "pipe_support", (x, y, 0.21), (0.037, 0.035, 0.011), DARK)
        cyl(w, "pipe_stub", (x, y, 0.243), 0.0105, 0.035, COPPER)
        cyl(w, "pipe_bore", (x, y, 0.2782), 0.0075, 0.0003, BLACK, **VIS)
        carrier = body(w, "tool_carrier", (x, y, z))
        element(carrier, "inertial", pos=(0,0,0), mass=.045, diaginertia=(.000018, .000018, .000018))
        joint(carrier, "puller_lift", "slide", (0, 0, 1), (0, .048), damping=4, frictionloss=1.7)
        joint(carrier, "puller_park", "slide", (1, 0, 0), (0, .141), damping=4, frictionloss=1.8)
        old = body(carrier, "old_olive", (0, 0, .014))
        joint(
            old,
            "olive_travel",
            "slide",
            (0, 0, 1),
            (0, 0.018),
            damping=2.5,
            frictionloss=0.8,
        )
        element(old, "inertial", pos=(0, 0, 0), mass=0.014, diaginertia=(0.000002,) * 3)
        rim(old, "brass_ferrule", 0.015, 0, 0.007, BRASS)
        e = body(carrier, "extractor")
        joint(
            e,
            "screw_turn",
            "hinge",
            (0, 0, 1),
            (-0.1, 11.0),
            damping=0.035,
            frictionloss=0.021,
        )
        cyl(e, "puller_collar", (0, 0, 0.008), 0.023, 0.015, BRASS, mass=0.049, contype=0, conaffinity=0)
        cyl(e, "lead_screw", (0, 0, 0.033), 0.0080, 0.021, STEEL, mass=0.018, contype=0, conaffinity=0)
        capsule(e, "crossbar", (-0.043, 0, 0.045), (0.043, 0, 0.045), 0.0052, STEEL)
        for sign in (-1, 1):
            geom(
                e,
                f"bar_end{sign}",
                "sphere",
                (sign * self.radius, 0, 0.045),
                (0.011,),
                STEEL,
                mass=0.011,
            )
        bx, by, bz = self.ring_home
        box(w, "spare_ring_tray", (bx, by, .210), (.035, .033, .010), DARK)
        cyl(w, "ring_standoff", (bx, by, .247), .0065, .027, STEEL)
        check = body(w, "trial_ring", self.ring_home)
        joint(check, "ring_transport", "slide", (0, 1, 0), (-.003, .148), damping=4, frictionloss=1.3)
        joint(check, "check_fit", "slide", (0, 0, -1), (-.035, .036), damping=4.0, frictionloss=1.2)
        # Inner clearance matches the original copper stub.
        for i in range(16):
            t = math.tau * i / 16
            box(
                check,
                f"fresh_ring{i}",
                (0.015 * math.cos(t), 0.015 * math.sin(t), 0),
                (0.004, 0.0031, 0.0055),
                BRASS,
                euler=(0, 0, t),
                mass=0.002,
            )

    def constraints(self, tree):
        eq = tree.find("equality")
        if eq is None:
            eq = element(tree, "equality")
        element(
            eq,
            "joint",
            name="screw_lead",
            joint1="olive_travel",
            joint2="screw_turn",
            polycoef=(0, 0.0011, 0, 0, 0),
            solref=".003 1",
            solimp=".98 .999 .0005",
        )

    def reset(self, s):
        super().reset(s)
        self.freed = False
        self.parked = False
        self.transferring = False

    @staticmethod
    def carriage(s, name, target, cap):
        displacement = target - s.joint(name)
        return float(np.clip(1900 * displacement - 26 * s.speed(name), -cap, cap))

    def active_handles(self):
        return self.handles[1:] if self.freed else self.handles

    def before_step(self, s):
        super().before_step(s)
        screw = s.joint("screw_turn")
        if self.freed:
            command = float(s.data.actuator("left_finger_-1").ctrl[0])
            if command > .020:
                self.grips.pop("extractor",None)
                self.transferring = False
            elif not self.parked and "extractor" not in self.grips and command < .007:
                if len(self.fingertips(s, "extractor", "left")) == 2:
                    tcp = s.data.site("left_tcp").xpos.copy()
                    self.grips["extractor"] = (tcp,s.position("extractor"),screw)
        if not self.transferring and not self.parked:
            self.force(s, "puller_park", self.carriage(s, "puller_park", 0, 85))
        if not self.freed:
            # Static breakaway under the puller's axial load, reflected through the screw.
            self.force(s, "screw_turn", -.014 * math.tanh(s.speed("screw_turn") / .04))
            self.force(s, "puller_lift", self.carriage(s, "puller_lift", 0, 65))
        elif self.parked:
            # The cradle keeps the extractor and spent olive out of the ring's path.
            self.force(s, "puller_lift", self.carriage(s, "puller_lift", .024, 70))
            self.force(s, "puller_park", self.carriage(s, "puller_park", .120, 75))
        elif "extractor" in self.grips and self.transferring:
            tcp = s.data.site("left_tcp").xpos
            retreat = tcp - self.tool_grip
            lift = max(0., min(.036, 1.12 * retreat[2]))
            park = max(0., min(.122, 1.12 * retreat[0])) if s.joint("puller_lift") > .014 else 0.
            self.force(s, "puller_lift", self.carriage(s, "puller_lift", lift, 80))
            self.force(s, "puller_park", self.carriage(s, "puller_park", park, 80))
            # Keep screw and olive together while the tool is being carried away.
            self.force(s, "screw_turn", np.clip(2.5*(self.free_angle-screw)-.08*s.speed("screw_turn"), -.8, .8))
        else:
            self.force(s, "puller_lift", self.carriage(s, "puller_lift", 0, 45))

        ring = self.grips.get("trial_ring")
        if ring is not None:
            tcp0, origin, _ = ring
            tcp = s.data.site("right_tcp").xpos
            displacement = tcp[1]-tcp0[1] + origin[1]-self.ring_home[1]
            lifted = s.joint("check_fit") < -.006
            if lifted:
                travel = np.clip(displacement, 0., .146)
                self.force(s, "ring_transport", self.carriage(s, "ring_transport", travel, 24))
        travel = s.joint("ring_transport")
        in_line = travel > .133 and self.parked
        if in_line:
            self.force(s, "ring_transport", self.carriage(s, "ring_transport", .146, 19))
        elif ring is None or s.joint("check_fit") > -.006:
            self.force(s, "ring_transport", self.carriage(s, "ring_transport", 0, 34))
        if not in_line:
            self.force(s, "check_fit", min(0., self.carriage(s, "check_fit", 0, 35)))

    def after_step(self, s):
        if not self.freed and s.joint("olive_travel") > .00255:
            self.freed = True
        if "extractor" not in self.grips and not self.parked:
            self.transferring = False
        if self.freed and not self.parked and "extractor" in self.grips:
            if not self.transferring:
                self.tool_grip = s.data.site("left_tcp").xpos.copy()
                self.free_angle = s.joint("screw_turn")
                self.transferring = True
            if s.joint("puller_park") > .106 and s.joint("puller_lift") > .016:
                self.parked = True

    def measure(self, s):
        travel, fit = s.joint("olive_travel"), s.joint("check_fit")
        ring_offset = abs(.146 - s.joint("ring_transport"))
        return dict(
            score=self.score((self.freed and self.parked, ring_offset < .002 and fit > .022)),
            extraction_mm=round(travel * 1000, 1),
            screw_deg=round(math.degrees(s.joint("screw_turn")), 1),
            parked_mm=round(s.joint("puller_park") * 1000, 1),
            grips=len(self.contact_log),
            puller_clearance_mm=round(s.joint("puller_lift") * 1000, 1),
            ring_offset_mm=round(ring_offset * 1000, 1),
            fit_mm=round(fit * 1000, 1),
        )

    def demo(self, s, incomplete=False):
        x, y, z = self.pivot
        R, h = self.radius, z + 0.047
        for _ in range(9):
            q = s.joint("screw_turn")
            sign = 0 if math.sin(q) >= 0 else math.pi
            start = q + sign
            end = start + 0.86
            g = (x + R * math.cos(start), y + R * math.sin(start), h)
            self.take(s, "left", g, "Grip the puller crossbar", lift=0.022)
            self.turn(s, "left", (x, y, h), R, start, end, "Wind the extractor screw")
            stop = (x + R * math.cos(end), y + R * math.sin(end), h)
            self.leave(s, "left", stop, "Release the crossbar between strokes", lift=.016)
            if self.freed:
                q = s.joint("screw_turn")
                end = q if math.cos(q) > 0 else q + math.pi
                grip = (x + R * math.cos(end), y + R * math.sin(end), h)
                self.take(s,"left",grip,"Regrip the near end of the freed puller",lift=.020)
                if not self.transferring:
                    raise RuntimeError("The freed puller was not retained by both fingertips")
                tx,ty,tz = self.tool_grip - np.array((self.x_offset,0,0))
                high = (tx,ty,tz+.025)
                self.go(s,"left",high,"Lift the freed olive clear of the pipe",time=1.1)
                park = (tx+.119,ty,tz+.025)
                self.go(s,"left",park,"Park the puller and spent olive",time=1.55)
                s.wait(.32)
                self.leave(s,"left",park,"Release the extractor in its cradle",lift=.012)
                break
        if not incomplete:
            bx, by, bz = self.ring_home
            g = (bx, by, bz+.006)
            self.take(s, "right", g, "Grip the replacement olive", lift=.029)
            self.go(s,"right",(bx,by,bz+.031),"Lift the spare olive above the pipe",time=.85)
            self.go(s,"right",(x,y+.004,bz+.031),"Align the spare olive with the original pipe",time=1.7)
            s.wait(.35)
            goal=(x,y,bz-.032)
            self.go(s, "right", goal, "Slide the replacement onto the original pipe",time=1.35)
            s.wait(.24)
            self.leave(s, "right", goal, "Release the seated replacement",lift=.018)
        s.wait(0.65)









