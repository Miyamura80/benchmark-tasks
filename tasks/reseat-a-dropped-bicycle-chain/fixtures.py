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













class Chain(Fixture):
    title = "Reseat a dropped bicycle chain"
    instruction = "Guide the dropped chain sideways onto the chainring, then turn the crank to check that the chain drives the rear sprocket."
    skill = "Align a derailed chain to a moving drivetrain and verify rotation."
    focus = (0.065, -0.005, 0.252)
    center = (0.073, -0.052, 0.269)
    radius = 0.052
    handles = (
        Handle("chain_guide", "chain_align", "left", (1, 0, 0)),
        Handle("crank", "crank_angle", "right", (1, 0, 0), "hinge", 0.70),
    )

    def build(self, w):
        plate(w)
        for y in (-0.106, 0.111):
            box(w, f"service_stand{y}", (0.013, y, 0.215), (0.037, 0.022, 0.016), DARK)
        capsule(
            w, "frame_tube", (0.026, -0.057, 0.277), (0.025, 0.126, 0.287), 0.012, TEAL
        )
        capsule(
            w,
            "triangular_frame",
            (0.026, -0.057, 0.277),
            (0.008, 0.009, 0.382),
            0.010,
            TEAL,
        )
        crank = body(w, "crank", self.center)
        joint(
            crank,
            "crank_angle",
            "hinge",
            (1, 0, 0),
            (-15, 15),
            damping=0.040,
            frictionloss=0.022,
        )
        cyl(
            crank,
            "chainring_disk",
            (0, 0, 0),
            0.051,
            0.005,
            STEEL,
            euler=(0, math.pi / 2, 0),
            mass=0.032,
        )
        for i in range(26):
            a = i * math.tau / 26
            box(
                crank,
                f"ring_tooth{i}",
                (0, 0.055 * math.cos(a), 0.055 * math.sin(a)),
                (0.005, 0.005, 0.0028),
                STEEL,
                euler=(a, 0, 0),
            )
        capsule(
            crank, "crank_arm", (0.027, 0, 0), (0.027, -self.radius, 0), 0.0055, DARK
        )
        box(
            crank,
            "pedal",
            (0.047, -self.radius, 0),
            (0.025, 0.012, 0.009),
            ORANGE,
            mass=0.025,
        )
        back = body(w, "rear_sprocket", (0.073, 0.129, 0.278))
        joint(back, "rear_angle", "hinge", (1, 0, 0), (-32, 32), damping=0.04)
        cyl(
            back,
            "rear_cog",
            (0, 0, 0),
            0.032,
            0.007,
            STEEL,
            euler=(0, math.pi / 2, 0),
            mass=0.013,
        )
        drop = body(w, "chain_guide", (0.101, 0.017, 0.341))
        joint(
            drop,
            "chain_align",
            "slide",
            (1, 0, 0),
            (-0.041, 0.002),
            damping=3.1,
            frictionloss=1.2,
        )
        joint(
            drop,
            "chain_depth",
            "slide",
            (0, 0, -1),
            (0, 0.013),
            damping=3,
            frictionloss=0.1,
        )
        for i in range(15):
            y = -0.066 + i * 0.011
            box(
                drop,
                f"loose_link{i}",
                (0, y, -0.003 + 0.055 * (y**2)),
                (0.006, 0.0055, 0.0035),
                DARK,
                mass=0.003,
            )
        for i in range(15):
            y = -0.065 + i * 0.011
            box(
                w,
                f"return_link{i}",
                (0.073, y, 0.232),
                (0.005, 0.0055, 0.0035),
                DARK,
                **VIS,
            )

    def constraints(self, tree):
        eq = tree.find("equality")
        if eq is None:
            eq = element(tree, "equality")
        element(
            eq,
            "joint",
            name="chain_upper_run_support",
            joint1="chain_depth",
            solref=".003 1",
            solimp=".99 .999 .0005",
        )

    def reset(self, s):
        super().reset(s)
        self.engaged = False
        self.at_engagement = 0.0
        self.rear_at_engagement = 0.0

    def before_step(self, s):
        super().before_step(s)
        support = s.model.equality("chain_upper_run_support").id
        s.data.eq_active[support] = s.joint("chain_align") > -0.021
        if self.engaged:
            ratio = 26 / 16
            crank = s.joint("crank_angle") - self.at_engagement
            lag = ratio * crank - (s.joint("rear_angle") - self.rear_at_engagement)
            deadband = .017
            stretch = math.copysign(max(0., abs(lag) - deadband), lag)
            speed_error = ratio * s.speed("crank_angle") - s.speed("rear_angle")
            torque = np.clip(2.2 * stretch + .045 * speed_error, -1.4, 1.4)
            self.force(s, "rear_angle", torque)
            self.force(s, "crank_angle", -ratio * torque * .34)

    def after_step(self, s):
        if (
            not self.engaged
            and s.joint("chain_align") < -0.022
            and abs(s.speed("crank_angle")) > 0.14
        ):
            self.engaged = True
            self.at_engagement = s.joint("crank_angle")
            self.rear_at_engagement = s.joint("rear_angle")
        if s.joint("chain_align") > -0.018:
            self.engaged = False

    def measure(self, s):
        align = s.joint("chain_align")
        back = s.joint("rear_angle")
        return dict(
            score=self.score(
                (
                    align < -0.021,
                    self.engaged
                    and abs(back) > 1.40
                    and s.joint("chain_depth") > 0.005,
                )
            ),
            lateral_error_mm=round(abs(0.028 + align) * 1000, 1),
            crank_deg=round(math.degrees(s.joint("crank_angle")), 1),
            rear_deg=round(math.degrees(back), 1),
            seat_mm=round(s.joint("chain_depth") * 1000, 1),
            engaged=self.engaged,
        )

    def demo(self, s, incomplete=False):
        chain = (0.101, 0.044, 0.340)
        self.take(s, "left", chain, "Grip the loose upper chain run", yaw=0, lift=0.020)
        placed = (0.073, 0.044, 0.340)
        self.go(s, "left", placed, "Guide the chain to the tooth plane", time=0.95)
        self.leave(s, "left", placed, "Release the aligned chain", lift=0.012)
        if not incomplete:
            x, y, z = self.center
            R = self.radius
            for _ in range(4):
                q = s.joint("crank_angle")
                start = q + math.pi
                g = (x + 0.047, y + R * math.cos(start), z + R * math.sin(start))
                self.take(s, "right", g, "Grip the pedal", lift=0.041)
                end = start - 0.97
                self.turn(
                    s,
                    "right",
                    (x + 0.047, y, z),
                    R,
                    start,
                    end,
                    "Turn the crank to feed the chain",
                    vertical=True,
                )
                stop = (x + 0.047, y + R * math.cos(end), z + R * math.sin(end))
                self.leave(
                    s,
                    "right",
                    stop,
                    "Release and check rear sprocket rotation",
                    lift=0.019,
                )
                if s.reading()["score"] == 1:
                    break
        s.wait(0.65)
