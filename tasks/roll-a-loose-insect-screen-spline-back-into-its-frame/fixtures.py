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











class Screen(Fixture):
    title = "Roll a loose insect-screen spline back into its frame"
    instruction = "Pull the loose mesh square, press the rubber spline into the groove with the roller, and seat the whole loose section."
    skill = "Maintain mesh tension as a roller seats a long rubber retaining strip."
    focus = (0.04, 0.027, 0.237)
    handles = (
        Handle("mesh_edge", "mesh_tension", "left", (0, 1, 0)),
        Handle("roller", "roller_travel", "right", (0, 1, 0)),
    )

    def build(self, w):
        plate(w)
        for y in (-0.134, 0.134):
            box(w, f"frame_end{y}", (0.018, y, 0.221), (0.129, 0.011, 0.010), STEEL)
        for x in (-0.109, 0.147):
            box(w, f"frame_side{x}", (x, 0, 0.213), (0.010, 0.126, 0.006), STEEL)
        for sign in (-1, 1):
            box(
                w,
                f"groove_wall{sign}",
                (0.147 + sign * 0.011, 0, 0.234),
                (0.002, 0.125, 0.004),
                STEEL,
            )
        mesh = body(w, "mesh_edge", (0.016, 0, 0.232))
        joint(
            mesh,
            "mesh_tension",
            "slide",
            (0, 1, 0),
            (0, 0.025),
            stiffness=32,
            damping=3.1,
            frictionloss=0.10,
        )
        for i in range(28):
            y = -0.123 + 0.009 * i
            capsule(
                mesh,
                f"mesh_weft{i}",
                (-0.111, y, 0),
                (0.126, y, 0),
                0.00075,
                RUBBER,
                **VIS,
            )
        for i in range(27):
            x = -0.112 + 0.009 * i
            capsule(
                mesh,
                f"mesh_warp{i}",
                (x, -0.124, 0),
                (x, 0.124, 0),
                0.0007,
                RUBBER,
                **VIS,
            )
        box(
            mesh,
            "folded_mesh_edge",
            (0.022, 0.100, 0.008),
            (0.017, 0.009, 0.008),
            RUBBER,
            mass=0.013,
        )
        for i in range(7):
            y = -0.081 + i * 0.016
            seg = body(w, f"spline_{i}", (0.147, y, 0.246))
            joint(
                seg,
                f"spline_depth_{i}",
                "slide",
                (0, 0, -1),
                (0, 0.016),
                stiffness=7,
                damping=0.7,
                frictionloss=0.20,
            )
            capsule(
                seg,
                f"rubber_segment{i}",
                (0, -0.004, 0),
                (0, 0.004, 0),
                0.005,
                BLACK,
                mass=0.005,
                contype=2,
                conaffinity=1,
            )
        r = body(w, "roller", (0.147, -0.111, 0.268))
        joint(
            r,
            "roller_travel",
            "slide",
            (0, 1, 0),
            (0, 0.206),
            damping=2.7,
            frictionloss=1.0,
        )
        joint(
            r,
            "roller_press",
            "slide",
            (0, 0, -1),
            (0, 0.012),
            stiffness=35,
            damping=2,
            frictionloss=0.1,
            solreflimit=".003 1",
            solimplimit=".98 .999 .0005",
        )
        box(
            r,
            "roller_handle",
            (-0.020, 0, 0.045),
            (0.026, 0.013, 0.011),
            PAPER,
            mass=0.036,
        )
        capsule(r, "roller_neck", (-0.006, 0, 0.042), (0, 0, 0.004), 0.004, STEEL)
        hub = body(r, "roller_wheel")
        joint(hub, "tool_wheel", "hinge", (1, 0, 0), (-30, 30), damping=0.003)
        cyl(
            hub,
            "roller_tire",
            (0, 0, 0),
            0.019,
            0.007,
            RUBBER,
            euler=(0, math.pi / 2, 0),
            mass=0.022,
        )

    def reset(self, s):
        super().reset(s)
        self.captured = set()

    def before_step(self, s):
        super().before_step(s)
        # Tracked roller motion is in the plane of the groove. The wheel is passive.
        if "roller" in self.grips:
            tcp0, _, _ = self.grips["roller"]
            target = max(
                0.0, min(0.012, 1.4 * (tcp0[2] - s.data.site("right_tcp").xpos[2]))
            )
            self.force(
                s,
                "roller_press",
                np.clip(
                    900 * (target - s.joint("roller_press"))
                    - 12 * s.speed("roller_press"),
                    -23,
                    23,
                ),
            )
            roller = s.position("roller")
            for i in range(7):
                key = f"spline_depth_{i}"
                segment = s.position(f"spline_{i}")
                lateral = roller[1] - segment[1]
                radius = .019
                if target > .009 and abs(lateral) < .013:
                    wheel_surface = roller[2] - math.sqrt(radius**2 - lateral**2)
                    compression = segment[2] + .005 - wheel_surface
                    force = 940 * max(0., compression) - 3.1 * s.speed(key)
                    self.force(s, key, np.clip(force, 0, 7.3))
        for i in self.captured:
            key = f"spline_depth_{i}"
            self.force(s, key, 380 * (0.013 - s.joint(key)) - 2.3 * s.speed(key))

    def after_step(self, s):
        tension = s.joint("mesh_tension") > 0.014
        if not tension:
            return
        for i in range(7):
            if s.joint(f"spline_depth_{i}") > 0.009:
                self.captured.add(i)

    def measure(self, s):
        seated = sum(s.joint(f"spline_depth_{i}") > 0.009 for i in self.captured)
        return dict(
            score=round(seated / 7, 4),
            seated_segments=seated,
            mesh_mm=round(s.joint("mesh_tension") * 1000, 1),
            roller_mm=round(s.joint("roller_travel") * 1000, 1),
            wheel_deg=round(math.degrees(s.joint("tool_wheel")), 1),
            press_mm=round(s.joint("roller_press") * 1000, 1),
            depths_mm=[round(1000 * s.joint(f"spline_depth_{i}"), 1) for i in range(7)],
        )

    def demo(self, s, incomplete=False):
        mesh, tool = (0.038, 0.100, 0.242), (0.127, -0.111, 0.315)
        self.take(s, "left", mesh, "Grip the loose screen edge")
        tight = (0.038, 0.122, 0.242)
        self.go(s, "left", tight, "Hold mesh tension")
        self.take(s, "right", tool, "Grip the spline roller")
        self.go(s, "right", (0.127, -0.111, 0.298), "Press the roller into the groove")
        self.go(
            s,
            "right",
            (0.127, -0.078, 0.294),
            "Seat the start of the loose spline",
            time=1.3,
        )
        s.wait(0.5)
        end = -0.040 if incomplete else 0.052
        self.go(
            s, "right", (0.127, end, 0.294), "Roll the spline into its groove", time=3.0
        )
        self.leave(s, "right", (0.127, end, 0.298), "Release the roller")
        self.leave(s, "left", tight, "Check the spline stays seated")
        s.wait(0.7)


