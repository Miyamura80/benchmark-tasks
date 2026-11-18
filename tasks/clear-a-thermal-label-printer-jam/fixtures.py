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


class Printer(Fixture):
    title = "Clear a thermal label printer jam"
    instruction = "Raise the platen, pull out the trapped backing, seat the platen, and press feed to advance a fresh label."
    skill = "Clear a pinched paper path and restore straight label feed."
    focus = (0.055, 0, 0.27)
    handles = (
        Handle("platen", "roller_lift", "left", (0, 0, 1)),
        Handle("jam", "backing_pull", "right", (1, 0, 0)),
    )

    def build(self, w):
        plate(w)
        box(w, "printer_shell", (0.033, 0, 0.221), (0.104, 0.127, 0.020), WHITE)
        for sign in (-1, 1):
            box(
                w,
                f"printer_cheek{sign}",
                (-0.006, sign * 0.117, 0.251),
                (0.061, 0.008, 0.021),
                WHITE,
            )
            cyl(
                w,
                f"roll_end{sign}",
                (-0.014, sign * 0.066, 0.270),
                0.013,
                0.005,
                DARK,
                euler=(math.pi / 2, 0, 0),
                **VIS,
            )
        box(
            w,
            "open_hinged_lid",
            (-0.088, 0, 0.342),
            (0.009, 0.122, 0.083),
            WHITE,
            euler=(0, -0.22, 0),
        )
        box(
            w,
            "lid_printhead",
            (-0.077, 0, 0.321),
            (0.008, 0.081, 0.016),
            DARK,
            euler=(0, -0.22, 0),
        )
        cyl(
            w,
            "label_roll",
            (-0.014, 0, 0.270),
            0.037,
            0.063,
            PAPER,
            euler=(math.pi / 2, 0, 0),
        )
        box(w, "stock_path", (0.068, 0, 0.256), (0.056, 0.062, 0.002), PAPER)
        box(w, "white_label", (0.028, 0, 0.259), (0.018, 0.054, 0.001), WHITE, **VIS)

        p = body(w, "platen", (0.084, 0, 0.277))
        joint(
            p,
            "roller_lift",
            "slide",
            (0, 0, 1),
            (0, 0.047),
            damping=3.0,
            frictionloss=1.4,
        )
        roller = body(p, "platen_surface")
        element(roller, "joint", name="platen_rotation", type="hinge", axis=(0, 1, 0), damping=.004)
        cyl(
            roller,
            "rubber_roller",
            (0, 0, 0),
            0.013,
            0.079,
            RUBBER,
            euler=(math.pi / 2, 0, 0),
            mass=0.044,
        )
        box(p, "platen_bearing_left", (0, 0.096, 0.009), (0.013, 0.011, 0.014), STEEL)
        box(p, "platen_bearing_right", (0, -0.094, 0.009), (0.013, 0.011, 0.013), STEEL)
        j = body(w, "jam", (0.107, -0.074, 0.250))
        joint(
            j,
            "backing_pull",
            "slide",
            (1, 0, 0),
            (0, 0.092),
            damping=2.1,
            frictionloss=0.9,
        )
        for i, (x, z, bend) in enumerate(
            [
                (-0.024, 0.001, 0.18),
                (-0.005, 0.006, -0.39),
                (0.012, 0.002, 0.30),
                (0.026, 0.008, -0.31),
            ]
        ):
            box(
                j,
                f"wrinkled_backing{i}",
                (x, 0.012, z),
                (0.013, 0.035, 0.0023),
                PAPER,
                euler=(0, bend, 0),
            )
        box(
            j,
            "pinch_fold",
            (0.052, -0.023, 0.013),
            (0.015, 0.010, 0.012),
            PAPER,
            mass=0.008,
        )
        b = body(w, "feed_button", (0.125, 0.134, 0.256))
        joint(
            b, "press_feed", "slide", (0, 0, -1), (0, 0.012), stiffness=225, damping=1.7
        )
        cyl(b, "feed_button_cap", (0, 0, 0), 0.011, 0.006, TEAL, mass=0.008)
        lead = body(w, "fresh_stock", (0.075, 0, 0.249))
        joint(
            lead,
            "print_feed",
            "slide",
            (1, 0, 0),
            (0, 0.097),
            damping=4,
            frictionloss=0.12,
        )
        box(
            lead,
            "leading_label",
            (0.003, 0, 0),
            (0.035, 0.057, 0.0013),
            WHITE,
            mass=0.007,
            contype=0,
            conaffinity=0,
        )

    def reset(self, s):
        super().reset(s)
        self.feed = False
        self.peak = 0.0

    def before_step(self, s):
        super().before_step(s)
        if s.joint("roller_lift") < 0.025 and s.joint("backing_pull") < 0.020:
            # The compressed pinch path resists removing folded stock.
            self.force(
                s,
                "backing_pull",
                np.clip(
                    -2400 * s.joint("backing_pull") - 24 * s.speed("backing_pull"),
                    -40,
                    0,
                ),
            )
        if self.feed:
            feed = s.joint("print_feed")
            if feed < 0.081:
                self.force(s, "print_feed", 1.05)
            radius = .013
            self.force(
                s, "platen_rotation",
                np.clip(.065 * (-feed / radius - s.joint("platen_rotation"))
                        + .010 * (-s.speed("print_feed") / radius - s.speed("platen_rotation")),
                        -.13, .13),
            )

    def after_step(self, s):
        self.peak = max(self.peak, s.joint("roller_lift"))
        if (
            s.joint("press_feed") > 0.007
            and s.joint("roller_lift") < 0.012
            and s.joint("backing_pull") > 0.066
        ):
            self.feed = True

    def measure(self, s):
        scrap, lift, feed = (
            s.joint(n) for n in ("backing_pull", "roller_lift", "print_feed")
        )
        return dict(
            score=self.score(
                (scrap > 0.066 and self.peak > 0.027, lift < 0.012 and feed > 0.074
                 and abs(s.joint("platen_rotation")) > 5.5)
            ),
            backing_mm=round(scrap * 1000, 1),
            platen_mm=round(lift * 1000, 1),
            feed_mm=round(feed * 1000, 1),
            platen_deg=round(math.degrees(s.joint("platen_rotation")), 1),
            grips=len(self.contact_log),
        )

    def demo(self, s, incomplete=False):
        p, j = (0.084, 0.096, 0.297), (0.162, -0.097, 0.266)
        self.take(s, "left", p, "Grip the platen bearing")
        self.go(s, "left", (0.084, 0.096, 0.340), "Raise the platen")
        self.take(s, "right", j, "Grip the trapped backing")
        self.go(
            s, "right", (0.247, -0.097, 0.266), "Extract the folded backing", time=1.35
        )
        self.leave(s, "right", (0.247, -0.097, 0.266), "Leave the backing clear")
        self.go(s, "left", p, "Seat the platen in its cradle")
        self.leave(s, "left", p, "Release the seated platen")
        if not incomplete:
            self.go(
                s,
                "left",
                (0.125, 0.134, 0.292),
                "Approach the printer feed button",
                opening=0.008,
            )
            self.go(s, "left", (0.125, 0.134, 0.239), "Press feed")
            self.go(s, "left", (0.125, 0.134, 0.304), "Watch one label advance")
        s.wait(1.6)











