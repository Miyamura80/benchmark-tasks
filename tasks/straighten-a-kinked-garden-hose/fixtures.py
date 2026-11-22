"""Garden hose fixture and flow measurements."""

import math
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from station import DT, HZ, element, geom, station_xml

STEEL = (0.59, 0.64, 0.69, 1)
DARK = (0.13, 0.16, 0.19, 1)
BLACK = (0.08, 0.09, 0.11, 1)
BLUE = (0.14, 0.33, 0.52, 1)
TEAL = (0.04, 0.59, 0.57, 1)
ORANGE = (0.98, 0.42, 0.10, 1)
YELLOW = (0.98, 0.74, 0.18, 1)
RUBBER = (0.24, 0.25, 0.27, 1)
WHITE = (0.81, 0.85, 0.86, 1)
BOARD = (0.35, 0.43, 0.45, 1)
WATER = (0.12, 0.70, 0.94, 0.85)
NO_CONTACT = {"contype": 0, "conaffinity": 0, "mass": 0}


def box(parent, name, pos, half, color, **kwargs):
    return geom(parent, name, "box", pos, half, color, **kwargs)


def cylinder(parent, name, pos, size, color, **kwargs):
    return geom(parent, name, "cylinder", pos, size, color, **kwargs)


def body(parent, name, pos):
    return element(parent, "body", name=name, pos=pos)




def hinge(parent, name, axis, limits, **kwargs):
    return element(
        parent, "joint", name=name, type="hinge", axis=axis, range=limits, **kwargs
    )


def ball(parent, name, pos, size, color, **kwargs):
    return geom(parent, name, "ellipsoid", pos, size, color, **kwargs)


def capsule(parent, name, start, end, radius, color, **kwargs):
    return element(
        parent,
        "geom",
        name=name,
        type="capsule",
        fromto=(*start, *end),
        size=radius,
        rgba=color,
        **kwargs,
    )


def stand(parent, xy=(0.015, 0), half=(0.16, 0.19), top=0.200):
    for i, x in enumerate((xy[0] - half[0] + 0.023, xy[0] + half[0] - 0.023)):
        box(
            parent, f"foot{i}", (x, xy[1], 0.182), (0.017, half[1] - 0.015, 0.017), DARK
        )
    return box(parent, "bench_pad", (*xy, top - 0.004), (*half, 0.004), BOARD)


def tray(parent, x, y, prefix="tray", z=0.201, sx=0.046, sy=0.054):
    box(parent, prefix, (x, y, z), (sx, sy, 0.0025), TEAL)
    for i, sign in enumerate((-1, 1)):
        box(
            parent,
            f"{prefix}_x{i}",
            (x + sign * sx, y, z + 0.006),
            (0.002, sy, 0.008),
            TEAL,
        )
        box(
            parent,
            f"{prefix}_y{i}",
            (x, y + sign * sy, z + 0.006),
            (sx, 0.002, 0.008),
            TEAL,
        )


class Fixture:
    def __init__(self, variant=0):
        self.variant = variant

    def compile(self):
        tree, world = station_xml()
        self.asset_meshes(tree.find("asset"))
        self.build(world)
        self.constraints(tree)
        for item in list(world):
            if item.find("freejoint") is not None:
                world.remove(item)
                item.set("childclass", "fixture")
                tree.find("worldbody").append(item)
        return mj.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"))

    def asset_meshes(self, assets):
        pass

    def constraints(self, tree):
        pass

    def reset(self, sim):
        pass

    def before_step(self, sim):
        pass

    def after_step(self, sim):
        pass

    def control(self, sim):
        sim.wait(20, "Unrepaired starting condition")

    def detail_control(self, sim):
        self.demo(sim, incomplete=True)

    def visual(self, sim, scene):
        pass








def line_in_scene(scene, start, end, radius, rgba):
    marker = scene.geoms[scene.ngeom]
    mj.mjv_initGeom(
        marker,
        mj.mjtGeom.mjGEOM_CAPSULE,
        np.zeros(3),
        np.zeros(3),
        np.eye(3).ravel(),
        np.array(rgba, dtype=np.float32),
    )
    mj.mjv_connector(marker, mj.mjtGeom.mjGEOM_CAPSULE, radius, start, end)
    scene.ngeom += 1




class Hose(Fixture):
    title = "Restore water flow through a kinked garden hose"
    instruction = "Straighten the kink in the garden hose until the running nozzle delivers a steady stream. Leave both ends connected."
    skill = "Reshape a pinched flexible conduit and restore flow with its ends still connected."
    focus = (0.020, -0.018, 0.234)
    x, y, z = (0.049, 0.046, 0.237)
    hose_color = (0.18, 0.40, 0.20, 1)

    def build(self, world):
        stand(world, xy=(0.02, -0.018), half=(0.150, 0.184))
        x, y, z = self.x, self.y, self.z
        for i, p in enumerate(((-0.098, 0.048, z), (0.121, -0.085, z))):
            box(
                world,
                f"fixed_coupler_base{i}",
                (p[0], p[1], 0.216),
                (0.021, 0.020, 0.016),
                DARK,
            )
            cylinder(
                world,
                f"coupler{i}",
                p,
                (0.016, 0.020),
                STEEL,
                euler=(0, math.pi / 2, 0),
            )
            for j in range(3):
                cylinder(
                    world,
                    f"knurl{i}_{j}",
                    (p[0] + 0.006 * (j - 1), p[1], p[2]),
                    (0.017, 0.0017),
                    BLACK,
                    euler=(0, math.pi / 2, 0),
                )
        # Feed approaches from +x, so a relaxed outlet continues along -x.
        feed = [(-.078,.048,z),(-.066,.095,z),(.057,.095,z),
                (.086,.082,z),(.093,.061,z),(.081,.048,z),(x,y,z)]
        for i,(a,b) in enumerate(zip(feed,feed[1:])):
            capsule(world,f"feed{i}",a,b,.0118,self.hose_color,**NO_CONTACT)
        kink=body(world,"bend_root",(x,y,z))
        hinge(kink,"kink",(0,0,1),(-.12,1.42),damping=.042,frictionloss=.003)
        for i in range(3):
            if i:
                kink=body(kink,f"hose_link{i}",(-.017,0,0))
                hinge(kink,f"flex{i}",(0,0,1),(-.4,.4),stiffness=.65,damping=.012,armature=.00001)
            capsule(kink,f"rubber_link{i}",(0,0,0),(-.018,0,0),.0117,self.hose_color,mass=.014)
        # The lumped wall has its own inertia and compliance. The controller
        # only drives the YAM joints. Flow is computed from the measured gap.
        wall=body(kink,"lumen_wall",(-.009,0,0))
        element(wall,"joint",name="lumen_gap",type="slide",axis=(0,0,1),
                range=(.00015,.008),damping=.25,armature=.0001)
        element(wall,"inertial",pos=(0,0,0),mass=.003,diaginertia=(1e-6,)*3)
        ball(
            kink,
            "ribbed_grip",
            (-0.034, -0.011, 0.003),
            (0.019, 0.014, 0.014),
            self.hose_color,
            mass=0.018,
        )
        for i in range(5):
            cylinder(
                kink,
                f"hose_rib{i}",
                (-0.044 + i * 0.004, -0.011, 0.003),
                (0.015, 0.0013),
                (0.30, 0.55, 0.26, 1),
                euler=(0, math.pi / 2, 0),
                **NO_CONTACT,
            )
        cylinder(
            world,
            "nozzle",
            (0.121, -0.047, z),
            (0.009, 0.024),
            YELLOW,
            euler=(math.pi / 2, 0, 0),
        )
        tray(world, 0.121, -0.009, prefix="catch_pan", z=0.201, sx=0.027, sy=0.030)
        box(
            world,
            "gauge",
            (0.105, -0.107, 0.209),
            (0.017, 0.028, 0.010),
            DARK,
            **NO_CONTACT,
        )

    def constraints(self, tree):
        contacts=tree.find("contact")
        if contacts is None:
            contacts=element(tree,"contact")
        element(contacts,"exclude",body1="bend_root",body2="hose_link2")

    def reset(self, s):
        self.wet = 0.0
        self.restored = 0.0
        s.data.joint("kink").qpos[0] = 1.32 if self.variant == 0 else 1.15
        s.data.joint("lumen_gap").qpos[0] = .00048
        self.root_dof=s.model.joint("kink").dofadr[0]
        self.wall_dof=s.model.joint("lumen_gap").dofadr[0]

    def bend(self,s):
        return sum(s.joint(j) for j in ("kink","flex1","flex2"))

    def before_step(self,s):
        angle=s.joint("kink")
        s.data.qfrc_applied[self.root_dof]=-.67*angle*(angle-.73)*(angle-1.25)
        curvature=sum(abs(s.joint(j)) for j in ("kink","flex1","flex2"))
        collapse=np.clip((curvature-.20)/1.03,0,1)
        target=.008*(1-.94*collapse)
        q=s.joint("lumen_gap")
        s.data.qfrc_applied[self.wall_dof]=62*(target-q)+.003*9.81

    def flow(self,s):
        # Normalized conductance of an elliptical passage at fixed pressure drop.
        # 205 mL/s is the nominal straight reference for this simulated rig.
        ratio=np.clip(s.joint("lumen_gap")/.008,0,1)
        return .205*(2*ratio**3)/(1+ratio**2)

    def after_step(self, s):
        rate = self.flow(s)
        grip = s.data.geom("ribbed_grip").xpos
        clear = all(np.linalg.norm(s.data.site(f"{side}_tcp").xpos - grip) >= 0.07
                    for side in ("left", "right"))
        self.restored = self.restored + DT if clear and rate >= 0.163 else 0.0
        self.wet = min(0.32, self.wet + rate * DT) if self.restored else 0.0

    def visual(self, s, scene):
        p = s.data.geom("ribbed_grip").xpos.copy()
        q = np.array([0.102, -0.085, self.z])
        c1 = p + [-0.042, -0.035, 0]
        c2 = q + [-0.079, -0.017, 0]

        def on_curve(t):
            return (
                (1 - t) ** 3 * p
                + 3 * (1 - t) ** 2 * t * c1
                + 3 * (1 - t) * t * t * c2
                + t**3 * q
            )

        for i in range(21):
            line_in_scene(
                scene, on_curve(i / 21), on_curve((i + 1) / 21), 0.0118, self.hose_color
            )
        flow = self.flow(s)
        points = [
            np.array([0.121, -0.024 + 0.037 * i / 8, self.z - 0.026 * (i / 8) ** 2])
            for i in range(9)
        ]
        if flow < 0.025:
            points = points[:3]
        for a, b in zip(points, points[1:]):
            line_in_scene(scene, a, b, 0.0011 + 0.013 * flow, WATER)
        line_in_scene(
            scene,
            (0.097, -0.122, 0.220),
            (0.097 + 0.19 * flow, -0.122, 0.220),
            0.0028,
            WATER,
        )

    def measure(self, s):
        rate = self.flow(s)
        return {
            "score": (
                1.0
                if self.restored >= 1.2 and self.wet >= 0.20
                else (0.5 if rate >= 0.105 else 0.0)
            ),
            "bend_deg": round(math.degrees(self.bend(s)), 1),
            "lumen_gap_mm": round(2000*s.joint("lumen_gap"),2),
            "flow_ml_s": round(rate * 1000, 1),
            "hands_clear_flow_seconds": round(self.restored, 2),
            "catch_ml": round(self.wet * 1000, 1),
        }

    def demo(self, s, incomplete=False):
        s.wait(1.05, "Observe the weak stream with the hose kinked")
        x, y, z = s.data.geom("ribbed_grip").xpos
        arm = "right"
        s.move_arm(
            arm,
            (x, y, 0.354),
            1.2,
            "Approach the kink from above",
            opening=0.78,
            yaw=0.0,
        )
        s.move_arm(arm, (x, y, z + 0.014), 0.8, "Reach the kinked hose", yaw=0.0)
        s.move_arm(
            arm,
            (x, y, z + 0.014),
            0.5,
            "Grip the hose at the bend",
            opening=0.04,
            yaw=0.0,
        )
        target_angle = 0.87 if incomplete else 0.04
        for a in np.linspace(s.joint("kink"), target_angle, 6)[1:]:
            tx = self.x - 0.068 * math.cos(a) + 0.011 * math.sin(a)
            ty = self.y - 0.068 * math.sin(a) - 0.011 * math.cos(a)
            s.move_arm(arm, (tx, ty, z + 0.014), 0.41, "Guide the kink open", yaw=0.0)
        s.move_arm(arm, (tx, ty, 0.357), 0.9, "Release the hose", opening=0.85, yaw=0.0)
        s.wait(1.65, "Observe the steady stream after releasing the hose")


