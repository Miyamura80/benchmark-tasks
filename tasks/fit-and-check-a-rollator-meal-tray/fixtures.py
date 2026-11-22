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








class RollatorTray(Fixture):
    title = "Fit a rollator meal tray"
    instruction = "Lower the tray onto the rollator fixture, close both clips, and check the tray stays attached."
    focus = (0.075, 0, 0.286)
    detail_camera = (0.50, 119, -42)
    wide_camera = ((0.08, 0, -0.010), 1.61, 123, -9)
    recording_fps = 10
    raised_z = 0.302
    drop = 0.033
    clip_x = 0.116
    clip_y = 0.090
    tray_grip = (0.066, 0.087, 0.334)
    signs = {"left": 1, "right": -1}
    handles = (
        Handle("tray", "tray_lower", "left", (0, 0, -1), max_force=17),
        Handle("clip_left", "clip_left_slide", "left", (0, -1, 0), max_force=13),
        Handle("clip_right", "clip_right_slide", "right", Y, max_force=13),
    )

    def build(self, w):
        floor = -0.460
        box(w, "rollator_floor", (0.030, 0, floor-0.010), (0.85, 0.64, 0.010),
            (0.59, 0.64, 0.66, 1), **VIS)
        for side in (-1, 1):
            y = side*0.098
            back = (-0.102, side*0.171, floor+0.063)
            front = (0.238, side*0.176, floor+0.063)
            pipe = (0.08, 0.23, 0.41, 1)
            capsule(w, f"rear_leg_{side}", (-0.010, y, 0.275), back, 0.012, pipe, **VIS)
            capsule(w, f"forward_leg_{side}", (0.145, y, 0.269), front, 0.012, pipe, **VIS)
            capsule(w, f"rail_brace_{side}", (-0.054, side*0.142, -0.040),
                    (0.196, side*0.152, -0.065), 0.009, pipe, **VIS)
            capsule(w, f"support_rail{side}", (-0.009, y, 0.270), (0.155, y, 0.270),
                    0.0094, STEEL, **VIS)
            handle = (-0.076, side*0.117, 0.405)
            capsule(w, f"handle_riser_{side}", (-0.014, y, 0.253),
                    (-0.027, side*0.112, 0.394), 0.010, pipe, **VIS)
            capsule(w, f"hand_grip{side}", (-0.028, side*0.115, 0.405), handle, 0.012, BLACK, **VIS)
            capsule(w, f"brake_lever_{side}", (-0.027, side*0.130, 0.402),
                    (-0.070, side*0.132, 0.382), 0.004, DARK, **VIS)
            capsule(w, f"cable_upper_{side}", (-0.064, side*0.123, 0.381),
                    (-0.037, side*0.113, 0.250), 0.0021, BLACK, **VIS)
            capsule(w, f"cable_lower_{side}", (-0.037, side*0.113, 0.250),
                    (-0.105, side*0.152, floor+0.084), 0.0021, BLACK, **VIS)
            for label, (x_w, y_w, z_w) in (("front", front), ("rear", back)):
                axle = (x_w, y_w+side*0.017, z_w)
                cyl(w, f"{label}_tyre_{side}", axle, 0.063, 0.012, BLACK,
                    euler=(math.pi/2, 0, 0), **VIS)
                cyl(w, f"{label}_rim_{side}", (x_w, axle[1]+side*0.002, z_w),
                    0.043, 0.012, STEEL, euler=(math.pi/2, 0, 0), **VIS)
                cyl(w, f"{label}_hub_{side}", (x_w, axle[1]+side*0.012, z_w),
                    0.014, 0.008, DARK, euler=(math.pi/2, 0, 0), **VIS)
                capsule(w, f"{label}_fork_{side}", (x_w, y_w, z_w+0.075),
                        (x_w, y_w, z_w), 0.007, pipe, **VIS)
        capsule(w, "rear_crossbar", (-0.073, -0.130, -0.115), (-0.073, 0.130, -0.115),
                0.009, BLUE, **VIS)
        capsule(w, "front_crossbar", (0.207, -0.147, -0.126), (0.207, 0.147, -0.126),
                0.009, BLUE, **VIS)
        box(w, "seat", (0.055, 0, 0.233), (0.095, 0.081, 0.009), BLACK, **VIS)
        box(w, "fabric_basket", (0.059, 0, 0.137), (0.074, 0.072, 0.075),
            (0.16, 0.21, 0.23, 1), **VIS)
        for side in (-1, 1):
            capsule(w, f"basket_trim_{side}", (-0.014, side*0.073, 0.206),
                    (0.135, side*0.073, 0.206), 0.004, DARK, **VIS)
        for side in (-1, 1):
            # Bench-height stands support the robot bases behind the parked walker.
            box(w, f"station_column_{side}", (-0.132, side*0.17995, floor/2),
                (0.050, 0.045, -floor/2), (0.26, 0.32, 0.36, 1), **VIS)
        tray = body(w, "tray", (0, 0, self.raised_z))
        slide(tray, "tray_lower", (0, 0, -1), (-0.003, 0.037), damping=1.6, friction=0.33)
        box(tray, "meal_tray_base", (0.065, 0, 0.017), (0.099, 0.082, 0.005), PALE_BLUE, **VIS)
        for sign in (-1, 1):
            box(tray, f"tray_y{sign}", (0.065, sign*0.081, 0.024),
                (0.101, 0.003, 0.008), PALE_BLUE, **VIS)
            box(tray, f"tray_x{sign}", (0.065+sign*0.098, 0, 0.024),
                (0.003, 0.080, 0.008), PALE_BLUE, **VIS)
        x, y, z = self.tray_grip
        handle_geom(tray, "tray_rim_grip", at=(x, y, z-self.raised_z),
                    size=(0.014, 0.012, 0.009), color=PALE_BLUE, mass=0.38)
        box(tray, "rim_grip_web", (x, 0.080, 0.027), (0.015, 0.012, 0.006),
            PALE_BLUE, **VIS)
        for side, sign in self.signs.items():
            clip = body(tray, f"clip_{side}", (self.clip_x, sign*self.clip_y, 0.026))
            slide(clip, f"clip_{side}_slide", (0, -sign, 0), (-0.003, 0.026))
            handle_geom(clip, f"clip_paddle_{side}", size=(0.012, 0.011, 0.008), color=ORANGE)
            box(clip, f"clip_hook_{side}", (0, sign*0.010, -0.025),
                (0.010, 0.006, 0.021), ORANGE, **VIS)
            box(clip, f"clip_toe_{side}", (0, sign*0.015, -0.044),
                (0.010, 0.007, 0.004), ORANGE, **VIS)

    def reset(self, s):
        super().reset(s)
        self.proof_time = 0.0
        self.tugged = False
        self.proof_peak = 0.0
        self.hold_time = 0.0
        self.verified = False

    def clips(self, s):
        return [s.joint(f"clip_{side}_slide") for side in self.signs]

    def before_step(self, s):
        super().before_step(s)
        down = s.joint("tray_lower")
        for side in self.signs:
            name = f"clip_{side}_slide"
            target = 0.022 if down > 0.016 and s.joint(name) > 0.015 else 0.0
            slide_force(s, self, name, target, k=135, damping=3, maximum=4.0)
        # The fixture counterbalances the tray before placement. Once on the
        # rail, the clips resist lift only if both hooks overlap their stops.
        mass = sum(float(s.model.body(name).mass[0]) for name in ("tray", "clip_left", "clip_right"))
        self.force(s, "tray_lower", -mass*9.81)
        if down > 0.014:
            retained = max(self.clips(s)) > 0.018
            slide_force(s, self, "tray_lower", self.drop, k=6500 if retained else 680,
                        damping=30 if retained else 9, maximum=72 if retained else 9)

    def after_step(self, s):
        locked = min(self.clips(s)) > 0.018 and s.joint("tray_lower") > 0.027
        deflection = max(0, self.drop-s.joint("tray_lower"))
        proof = locked and deflection >= 0.0012 and "tray_lower" in self.grips
        if proof:
            self.proof_peak = max(self.proof_peak, 6500*deflection)
        self.proof_time = self.proof_time + DT if proof else 0.0
        self.tugged |= self.proof_time > 0.25
        keep = locked and self.tugged and not self.grips
        self.verified = released_for(s, self, keep, 0.45)

    def measure(self, s):
        left, right = self.clips(s)
        return dict(score=1.0 if self.verified else 0.25*sum(q > 0.018 for q in (left, right)),
                    left_clip=left > 0.018, right_clip=right > 0.018,
                    seated_mm=round(s.joint("tray_lower")*1000, 1), proof_tug=self.tugged,
                    proof_load_n=round(self.proof_peak, 1))

    def clip(self, side, delta=0):
        return self.clip_x, self.signs[side]*(self.clip_y-delta), self.raised_z+0.026-self.drop

    def demo(self, s, incomplete=False):
        x, y, z = self.tray_grip
        seated = (x, y, z-self.drop)
        self.take(s, "left", self.tray_grip, "Grip the meal tray rim", lift=0.030)
        self.go(s, "left", seated, "Lower the tray over both rails")
        self.leave(s, "left", seated, "Leave the tray resting on the frame")
        for side in ("left",) if incomplete else ("left", "right"):
            self.take(s, side, self.clip(side), "Grip the locking clip")
            self.go(s, side, self.clip(side, 0.025), "Press the clip around the rail")
            self.go(s, side, self.clip(side, 0.025), "Open the gripper clear of the clip",
                    opening=0.50, time=0.48)
            s.wait(0.22)
            x_c, y_c, z_c = self.clip(side, 0.025)
            self.go(s, side, (x_c, y_c, z_c+0.022), "Release the clip and lift away")
        if not incomplete:
            self.take(s, "left", seated, "Grip the tray for a retention check")
            self.go(s, "left", (x, y, seated[2]+0.017), "Tug the tray against the locked clips")
            s.wait(0.60)
            self.go(s, "left", seated, "Relax the upward tug")
            self.leave(s, "left", seated, "Leave the locked tray in place")
        s.wait(0.70)






FIXTURES = {"rollator-tray": RollatorTray}
