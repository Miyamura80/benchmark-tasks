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









class Bobbin(Fixture):
    title = "Clear tangled thread from a sewing-machine bobbin"
    instruction = "Lift the bobbin, remove the trapped thread, reseat the bobbin, and turn the handwheel to check that the needle moves."
    skill = "Extract a jam below the needle plate and verify the mechanism cycles."
    focus = (0.063, 0.008, 0.282)
    wheel = (0.124, -0.099, 0.273)
    radius = 0.032
    handles = (
        Handle("bobbin", "bobbin_lift", "left", (0, 0, 1)),
        Handle("thread", "thread_pull", "right", (1, 0, 0)),
        Handle("wheel", "handwheel", "right", (0, 0, 1), "hinge", 0.55),
    )

    def build(self, w):
        plate(w)
        box(w, "machine_base", (0.050, 0, 0.222), (0.105, 0.157, 0.020), WHITE)
        box(w, "machine_column", (-0.046, -0.105, 0.297), (0.028, 0.035, 0.055), WHITE)
        box(w, "machine_arm", (-0.047, -0.002, 0.346), (0.024, 0.110, 0.023), WHITE)
        box(w, "needle_plate", (0.067, 0.086, 0.245), (0.067, 0.059, 0.004), STEEL)
        for i in range(9):
            t = i * math.tau / 9
            box(
                w,
                f"wellwall{i}",
                (0.083 + 0.028 * math.cos(t), 0.099 + 0.028 * math.sin(t), 0.253),
                (0.003, 0.011, 0.002),
                DARK,
                euler=(0, 0, t),
            )
        b = body(w, "bobbin", (0.083, 0.099, 0.263))
        joint(
            b,
            "bobbin_lift",
            "slide",
            (0, 0, 1),
            (0, 0.056),
            damping=3.2,
            frictionloss=1.1,
        )
        for flag in (-1, 1):
            cyl(
                b,
                f"bobbin_flange{flag}",
                (0, 0, flag * 0.007),
                0.019,
                0.0024,
                STEEL,
                mass=0.009,
            )
        cyl(b, "thread_on_bobbin", (0, 0, 0), 0.015, 0.0065, BLUE, mass=0.009)
        ring = body(w, "thread", (0.105, 0.005, 0.251))
        joint(
            ring,
            "thread_pull",
            "slide",
            (1, 0, 0),
            (0, 0.104),
            damping=2,
            frictionloss=0.54,
        )
        for i in range(12):
            t = i * math.tau / 12
            u = (i + 1) * math.tau / 12
            capsule(
                ring,
                f"tangle{i}",
                (0.025 * math.cos(t), 0.027 * math.sin(t), 0.003),
                (0.025 * math.cos(u), 0.027 * math.sin(u), 0.003),
                0.0024,
                BLUE,
            )
        capsule(
            ring,
            "trapped_tail",
            (-0.020, 0.090, 0.011),
            (-0.010, 0.020, 0.005),
            0.0019,
            BLUE,
            **VIS,
        )
        box(
            ring,
            "folded_thread_bundle",
            (0.021, -0.014, 0.008),
            (0.006, 0.009, 0.006),
            BLUE,
            mass=0.005,
        )
        v = body(w, "wheel", self.wheel)
        joint(
            v,
            "handwheel",
            "hinge",
            (0, 0, 1),
            (-8.0, 8.0),
            damping=0.04,
            frictionloss=0.012,
        )
        cyl(v, "wheel_face", (0, 0, 0), 0.039, 0.008, DARK, mass=0.042)
        rim(v, "wheel_outer", 0.040, 0.001, 0.005, STEEL)
        capsule(v, "wheel_spoke", (0, 0, 0.012), (0.036, 0, 0.012), 0.005, BLACK)
        geom(
            v,
            "wheel_thumb_grip",
            "sphere",
            (self.radius, 0, 0.012),
            (0.010,),
            ORANGE,
            mass=0.006,
        )
        rod = body(w, "needle_carriage", (0.057, 0.088, 0.310))
        joint(
            rod,
            "needle_travel",
            "slide",
            (0, 0, -1),
            (0, 0.027),
            damping=3,
            armature=0.002,
        )
        capsule(rod, "needle_bar", (0, 0, 0), (0, 0, -0.024), 0.0022, STEEL)
        box(
            w,
            "raised_presser_foot",
            (0.041, 0.084, 0.269),
            (0.007, 0.012, 0.004),
            STEEL,
        )

    def reset(self, s):
        super().reset(s)
        self.lifted = False
        self.stroke = 0.0

    def before_step(self, s):
        super().before_step(s)
        lift, pull = s.joint("bobbin_lift"), s.joint("thread_pull")
        if lift < .037 and pull < .038:
            remaining_pinch = np.clip((.037 - lift) / .017, 0., 1.)
            self.force(
                s,
                "thread_pull",
                np.clip(
                    remaining_pinch * (-1900 * pull - 18 * s.speed("thread_pull")), -32, 0
                ),
            )
        # Cam follower follows handwheel phase, with a compliant spring load.
        a = s.joint("handwheel")
        # Slider-crank with a finite connecting rod and a 12 mm crank radius.
        throw, rod = .012, .040
        target = throw * (1 - math.cos(a)) + rod - math.sqrt(rod**2 - (throw * math.sin(a))**2)
        self.force(
            s,
            "needle_travel",
            np.clip(
                110 * (target - s.joint("needle_travel"))
                - 1.3 * s.speed("needle_travel"),
                -4,
                4,
            ),
        )

    def after_step(self, s):
        self.lifted |= s.joint("bobbin_lift") > 0.033
        if s.joint("bobbin_lift") < 0.010 and s.joint("thread_pull") > 0.033:
            self.stroke = max(self.stroke, s.joint("needle_travel"))

    def measure(self, s):
        thread = s.joint("thread_pull")
        seated = s.joint("bobbin_lift") < 0.011
        return dict(
            score=self.score(
                (self.lifted and thread > 0.033, seated and self.stroke > 0.004)
            ),
            clear_mm=round(thread * 1000, 1),
            bobbin_seated=seated,
            needle_stroke_mm=round(self.stroke * 1000, 1),
            handwheel_deg=round(math.degrees(s.joint("handwheel")), 1),
            grips=len(self.contact_log),
        )

    def demo(self, s, incomplete=False):
        p, r = (0.083, 0.099, 0.279), (0.126, -0.009, 0.262)
        self.take(s, "left", p, "Grip the bobbin flange")
        self.go(s, "left", (0.083, 0.099, 0.329), "Lift the bobbin out of the well")
        self.take(s, "right", r, "Grip the thread tangle")
        self.go(
            s, "right", (0.219, -0.009, 0.262), "Pull the trapped thread out", time=1.25
        )
        self.leave(s, "right", (0.219, -0.009, 0.262), "Leave the thread clear")
        self.go(s, "left", p, "Reseat the bobbin")
        self.leave(s, "left", p, "Release the seated bobbin")
        if not incomplete:
            x, y, z = self.wheel
            self.take(
                s,
                "right",
                (x + self.radius, y, z + 0.012),
                "Grip the handwheel",
                lift=0.040,
            )
            self.turn(
                s,
                "right",
                (x, y, z + 0.012),
                self.radius,
                0,
                -2.12,
                "Turn the handwheel and check the needle",
            )
            stop = (
                x + self.radius * math.cos(-2.12),
                y + self.radius * math.sin(-2.12),
                z + 0.012,
            )
            self.leave(s, "right", stop, "Release the handwheel")
        s.wait(0.65)




