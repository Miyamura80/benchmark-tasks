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







class Sander(Fixture):
    title = "Clamp replacement paper on a rectangular sheet sander"
    instruction = "Pull the new abrasive sheet flat and close the front spring clamp, then tug the sheet to check it stays tight."
    skill = "Maintain abrasive-sheet tension while closing a mechanical clamp."
    handles = (
        Handle("abrasive", "sheet_tension", "left", (0, 1, 0)),
        Handle("clamp", "clamp_travel", "right", (0, 0, -1)),
    )
    focus = (0.05, 0.01, 0.255)

    def build(self, w):
        plate(w)
        box(w, "upturned_motor_shell", (0.047, 0, 0.222), (0.062, 0.102, 0.024), TEAL)
        box(w, "rectangular_pad", (0.047, 0, 0.250), (0.060, 0.098, 0.006), RUBBER)
        for i in range(5):
            box(
                w,
                f"vent{i}",
                (-0.017, -0.055 + i * 0.018, 0.229),
                (0.002, 0.005, 0.004),
                DARK,
                **VIS,
            )
        box(
            w, "closed_rear_clamp", (0.044, -0.091, 0.263), (0.060, 0.008, 0.005), STEEL
        )
        box(
            w,
            "used_abrasive",
            (0.151, -0.102, 0.205),
            (0.037, 0.065, 0.002),
            (0.47, 0.30, 0.19, 1),
            euler=(0.08, 0, -0.18),
        )
        item = body(w, "abrasive", (0.047, 0, 0.258))
        joint(
            item,
            "sheet_tension",
            "slide",
            (0, 1, 0),
            (0, 0.032),
            stiffness=20,
            damping=2.4,
            frictionloss=0.075,
        )
        box(item, "new_abrasive", (0, 0, 0), (0.057, 0.082, 0.0014), PAPER, mass=0.029)
        for i in range(5):
            box(
                item,
                f"grit_line{i}",
                (0, -0.060 + 0.025 * i, 0.0018),
                (0.051, 0.0007, 0.0003),
                (0.68, 0.53, 0.33, 1),
                **VIS,
            )
        box(
            item,
            "folded_paper_edge",
            (-0.073, 0.073, 0.015),
            (0.014, 0.012, 0.010),
            PAPER,
        )
        c = body(w, "clamp", (0.048, 0.102, 0.290))
        joint(
            c,
            "clamp_travel",
            "slide",
            (0, 0, -1),
            (0, 0.025),
            damping=3.5,
            frictionloss=1.6,
        )
        box(c, "front_clamp_bail", (0, 0, -0.005), (0.055, 0.005, 0.003), STEEL)
        capsule(
            c,
            "latch_lever",
            (0.043, -0.005, 0.007),
            (0.043, -0.143, 0.007),
            0.005,
            STEEL,
        )
        box(c, "right_latch_tab", (0.043, -0.143, 0.006), (0.013, 0.015, 0.010), ORANGE)

    def reset(self, s):
        super().reset(s)
        self.locked = False
        self.held = 0.0
        self.test_steps = 0
        self.peak_proof_load = 0.0

    def before_step(self, s):
        super().before_step(s)
        if self.locked:
            # The closed jaw transmits clamp friction along the abrasive feed path.
            jaw_depth = s.joint("clamp_travel") - .014
            capacity = 1.1 * max(0., 3900 * jaw_depth)
            load = np.clip(
                5800 * (self.held - s.joint("sheet_tension"))
                - 33 * s.speed("sheet_tension"), -capacity, capacity
            )
            self.force(s, "sheet_tension", load)
            if "abrasive" in self.grips:
                self.peak_proof_load = max(self.peak_proof_load, abs(load))

    def after_step(self, s):
        ext, clamp = s.joint("sheet_tension"), s.joint("clamp_travel")
        if clamp < 0.014:
            self.locked = False
            self.test_steps = 0
        if (
            clamp > 0.018
            and ext > 0.014
            and "abrasive" in self.grips
            and not self.locked
        ):
            self.held, self.locked = ext, True
        if self.locked and "abrasive" in self.grips:
            target0, _, _ = self.grips["abrasive"]
            squeeze = s.data.site("left_tcp").xpos[1] - target0[1]
            if squeeze > 0.029 and abs(ext - self.held) < 0.006:
                self.test_steps += 1

    def measure(self, s):
        return dict(
            score=self.score((self.locked, self.test_steps >= 75 and self.peak_proof_load > 3.)),
            tension_mm=round(s.joint("sheet_tension") * 1000, 1),
            clamp_mm=round(s.joint("clamp_travel") * 1000, 1),
            pull_test_ms=round(self.test_steps * DT * 1000),
            proof_load_n=round(self.peak_proof_load, 1),
            grips=len(self.contact_log),
        )

    def demo(self, s, incomplete=False):
        sheet, clamp = (-0.026, 0.073, 0.273), (0.091, -0.041, 0.302)
        self.take(s, "left", sheet, "Grip the folded edge of the abrasive")
        taut = (-0.026, 0.099, 0.273)
        self.go(s, "left", taut, "Hold the paper taut")
        self.take(s, "right", clamp, "Grip the front clamp tab")
        self.go(s, "right", (0.091, -0.041, 0.276), "Close the front clamp")
        self.leave(s, "right", (0.091, -0.041, 0.276), "Release the locked clamp")
        if not incomplete:
            self.go(
                s, "left", (-0.026, 0.119, 0.273), "Tug the clamped sheet", time=0.8
            )
            s.wait(0.35)
            self.go(s, "left", taut, "Relax the proof pull", time=0.65)
        self.leave(s, "left", taut, "Leave both ends of the sheet clamped")
        s.wait(0.65)






