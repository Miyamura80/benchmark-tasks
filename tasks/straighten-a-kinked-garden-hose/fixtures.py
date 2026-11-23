"""Garden hose fixture and flow measurements."""

from collections import deque
import math
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from jug_trial import WINDOW_S, CLEAR_M, score_window
from station import Station, DT, HZ, element, geom, station_xml

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
    focus = (0.067, -0.025, 0.233)
    x, y, z = (0.049, 0.046, 0.237)
    hose_color = (0.43, 0.73, 0.60, 0.85)

    route = np.array([(-.019,.035),(-.053,.028),(-.079,.003),(-.080,-.036),
                          (-.055,-.069),(-.015,-.085),(.027,-.085),(.064,-.085),(.101,-.085)])
    tail_vectors = np.diff(route, axis=0)

    def __init__(self, variant=0, *, reference_run=False):
        super().__init__(variant)
        self.reference_run = reference_run

    def build(self, world):
        stand(world, xy=(0.02, -0.018), half=(0.150, 0.184))
        x, y, z = self.x, self.y, self.z
        radius = .008
        inlet = (.149, y, z)
        outlet = (.101, -.085, z)
        # Mark both fixed barbs and route the outlet into the collection funnel.
        for i, (px, py, pz) in enumerate((inlet, outlet)):
            box(world, f"fixed_coupler_base{i}", (px+.010, py, pz-.018), (.011,.010,.010), DARK)
            cylinder(world, f"coupler{i}", (px+.010,py,pz), (.010,.010), STEEL, euler=(0,math.pi/2,0))
        box(world, "support", (.017,-.028,z-radius-.003), (.150,.172,.003), BOARD)
        anchor = body(world, "inlet_stub", inlet)
        capsule(anchor, "inlet_hose", (0,0,0), (-.030,0,0), radius, self.hose_color)
        parent = anchor
        for i in range(2):
            off = -.030 if i == 0 else -.035
            parent = body(parent, f"feed_body{i}", (off,0,0))
            hinge(parent,f"feed_yaw{i}",(0,0,1),(-.7,.7),stiffness=.15,damping=.012)
            hinge(parent,f"feed_pitch{i}",(0,1,0),(-1.1,1.1),stiffness=.12,damping=.006)
            capsule(parent,f"feed{i}",(0,0,0),(-.035,0,0),radius,self.hose_color,mass=.006)
        kink=body(parent,"bend_root",(-.035,0,0))
        hinge(kink,"kink",(0,0,1),(-.2,1.65),damping=.042,frictionloss=.003)
        hinge(kink,"bend_pitch",(0,1,0),(-1.1,1.1),stiffness=.05,damping=.006)
        for i in range(3):
            if i:
                kink=body(kink,f"hose_link{i}",(-.017,0,0))
                hinge(kink,f"flex{i}",(0,0,1),(-.4,.4),stiffness=.45,damping=.012)
            end = (-.034,-.011,0) if i == 2 else (-.017,0,0)
            capsule(kink,f"rubber_link{i}",(0,0,0),end,radius,self.hose_color,mass=.007)
        # The lumped wall has its own inertia and compliance. The controller
        # only drives the YAM joints. Flow is computed from the measured gap.
        wall=body(kink,"lumen_wall",(-.009,0,0))
        element(wall,"joint",name="lumen_gap",type="slide",axis=(0,0,1),
                range=(.00012,.006),damping=.25,armature=.0001)
        element(wall,"inertial",pos=(0,0,0),mass=.003,diaginertia=(1e-6,)*3)
        # A thin silicone identification sleeve around the hose is the grasp marker.
        ball(kink, "ribbed_grip", (-.030,-.010,0), (.012,.009,.009),
             (.32,.57,.34,1), mass=.002)
        offset = np.array([-.034,-.011,0.])
        parent = kink
        for i, vector in enumerate(self.tail_vectors):
            parent = body(parent, f"tail{i}", offset)
            hinge(parent,f"tail_yaw{i}",(0,0,1),(-2.5,2.5),stiffness=.018,damping=.002)
            hinge(parent,f"tail_pitch{i}",(-vector[1],vector[0],0),(-1.1,1.1),stiffness=.025,damping=.003)
            offset = np.r_[vector, 0.]
            capsule(parent,f"tail_hose{i}",(0,0,0),offset,radius,self.hose_color,mass=.007)
        element(parent,"site",name="tail_tip",pos=offset,size=.001)
        element(world,"site",name="outlet_pin",pos=outlet,size=.001)
        element(parent,"site",name="tail_barb",pos=offset-[.012,0,0],size=.001)
        element(world,"site",name="outlet_barb",pos=np.array(outlet)-[.012,0,0],size=.001)
        # The rigid discharge pipe directs water over the edge into an open receiver.
        capsule(world, "discharge_pipe", (.121,-.085,z),(.203,-.085,z),.007,STEEL)
        capsule(world, "nozzle", (.203,-.085,z),(.203,-.085,z-.017),.007,YELLOW)
        tray(world,.211,-.085,prefix="receiver_base",z=.100,sx=.055,sy=.050)
        for sign in (-1, 1):
            box(world, f"receiver_x{sign}", (.211+sign*.055,-.085,.155),
                (.001,.051,.055), (.35,.70,.75,.30))
            box(world, f"receiver_y{sign}", (.211,-.085+sign*.05,.155),
                (.055,.001,.055), (.35,.70,.75,.30))
    def constraints(self, tree):
        contacts=tree.find("contact")
        if contacts is None:
            contacts=element(tree,"contact")
        eq = tree.find("equality")
        if eq is None:
            eq = element(tree,"equality")
        element(eq,"connect",name="outlet_connection",site1="tail_tip",site2="outlet_pin",
                solref=".004 1",solimp=".99 .999 .0001")
        element(eq,"connect",name="outlet_direction",site1="tail_barb",site2="outlet_barb",
                solref=".004 1",solimp=".99 .999 .0001")
        element(contacts,"exclude",body1="bend_root",body2="hose_link2")

    def reset(self, s):
        # Close the outlet loop at reset while preserving every segment length.
        # This inverse-kinematics step only sets the initial state.
        angle = 0.0 if self.reference_run else (1.25 if self.variant == 0 else 1.19)
        s.data.joint("kink").qpos[0] = angle
        angles = np.zeros(len(self.tail_vectors))
        rot = np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        tip = np.array([self.x,self.y]) + rot @ np.array([-.068,-.011])
        goal = np.array([.101,-.085])
        for _ in range(100):
            absolute = angle + np.cumsum(angles)
            vx, vy = self.tail_vectors.T
            segments = np.c_[vx*np.cos(absolute)-vy*np.sin(absolute), vx*np.sin(absolute)+vy*np.cos(absolute)]
            error = np.r_[goal-tip-segments.sum(axis=0), -.04*absolute[-1]]
            if np.linalg.norm(error)<1e-7:
                break
            levers = np.cumsum(segments[::-1],axis=0)[::-1]
            jac = np.stack((-levers[:,1],levers[:,0],np.full(len(angles),.04)))
            angles += np.clip(jac.T @ np.linalg.solve(jac@jac.T+1e-6*np.eye(3),error),-.2,.2)
        for i, val in enumerate(angles):
            s.data.joint(f"tail_yaw{i}").qpos[0] = val
        s.data.joint("lumen_gap").qpos[0] = .006 if self.reference_run else .00036
        self.window = deque()
        self.collected = 0.0
        self.window_steps = round(WINDOW_S/s.model.opt.timestep)
        self.reference = self.discharge(.006)*WINDOW_S
        self.clear_sites = [s.model.site(f"{side}_tcp").id for side in ("left","right")]
        self.root_dof=s.model.joint("kink").dofadr[0]
        self.wall_dof=s.model.joint("lumen_gap").dofadr[0]

        names = ["inlet_hose", "feed0", "feed1", "rubber_link0", "rubber_link1", "rubber_link2"]
        names.extend(f"tail_hose{i}" for i in range(len(self.tail_vectors)))
        self.hose_ids = np.array([s.model.geom(n).id for n in names])
        self.half_lengths = s.model.geom_size[self.hose_ids,1]
        self.is_hose = np.zeros(s.model.ngeom,dtype=bool)
        self.is_hose[self.hose_ids] = True
        self.is_hose[s.model.geom("ribbed_grip").id] = True
        self.is_arm = np.array([(s.model.body(s.model.geom_bodyid[i]).name or "").startswith(("left_", "right_"))
                                for i in range(s.model.ngeom)])
        if not self.reference_run:
            ref = Station(Hose(reference_run=True),model=s.model)
            ref.wait(.50, "Settle open-hose reference")
            measurements = []
            for _ in range(3):
                ref.wait(WINDOW_S, "Measure open-hose reference")
                if len(ref.fixture.window) != ref.fixture.window_steps:
                    raise RuntimeError("Reference collection interrupted")
                measurements.append(ref.fixture.collected)
            self.reference = float(np.median(measurements))

    def bend(self,s):
        return sum(s.joint(j) for j in ("kink","flex1","flex2"))

    def before_step(self,s):
        angle=s.joint("kink")
        s.data.qfrc_applied[self.root_dof]=-.67*angle*(angle-.73)*(angle-1.25)
        curvature=sum(abs(s.joint(j)) for j in ("kink","flex1","flex2"))
        collapse=np.clip((curvature-.20)/1.03,0,1)
        target=.006*(1-.94*collapse)
        q=s.joint("lumen_gap")
        s.data.qfrc_applied[self.wall_dof]=62*(target-q)+.003*9.81

    @staticmethod
    def discharge(gap):
        ratio = np.clip(gap/.006,0,1)
        return .100*(2*ratio**3)/(1+ratio**2)

    def flow(self,s):
        return self.discharge(s.joint("lumen_gap"))

    def endpoint_error(self, s):
        return max(np.linalg.norm(s.data.site(a).xpos-s.data.site(b).xpos)
                   for a,b in (("tail_tip","outlet_pin"),("tail_barb","outlet_barb")))

    def after_step(self, s):
        centers = s.data.geom_xpos[self.hose_ids]
        axes = s.data.geom_xmat[self.hose_ids, :,].reshape(-1,3,3)[:,:,2]
        delta = s.data.site_xpos[self.clear_sites,None,:] - centers
        projection = np.clip(np.sum(delta*axes,axis=2),-self.half_lengths,self.half_lengths)
        nearest = delta - projection[:,:,None]*axes
        clear = np.min(np.sum(nearest**2,axis=2)) >= CLEAR_M**2
        if clear:
            a,b = s.data.contact.geom1,s.data.contact.geom2
            clear = not np.any((self.is_arm[a]&self.is_hose[b]) | (self.is_arm[b]&self.is_hose[a]))
        if not clear or self.endpoint_error(s) > .002:
            self.window.clear()
            self.collected = 0.0
            return
        if len(self.window) == self.window_steps:
            self.collected -= self.window.popleft()
        volume = self.flow(s)*s.model.opt.timestep
        self.window.append(volume)
        self.collected += volume

    def visual(self, s, scene):
        flow = self.flow(s)
        length = .010 + .082*min(1.0,flow/.100)
        line_in_scene(scene,(.203,-.085,self.z-.018),(.203,-.085,self.z-.018-length),
                      .001+.022*flow,WATER)

    def measure(self, s):
        rate = self.flow(s)
        return {
            "score": score_window(self.collected,self.reference,len(self.window)==self.window_steps),
            "recovered_fraction": round(self.collected/self.reference,4),
            "reference_ml": round(1000*self.reference,2),
            "outlet_error_mm": round(1000*self.endpoint_error(s),3),
            "bend_deg": round(math.degrees(self.bend(s)), 1),
            "lumen_gap_mm": round(2000*s.joint("lumen_gap"),2),
            "flow_ml_s": round(rate * 1000, 1),
            "hands_clear_flow_seconds": round(len(self.window)*s.model.opt.timestep, 2),
            "catch_ml": round(self.collected * 1000, 1),
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
            yaw=1.57,
        )
        s.move_arm(arm, (x, y, z + 0.007), 0.8, "Reach the kinked hose", yaw=1.57)
        s.move_arm(
            arm,
            (x, y, z + 0.007),
            0.5,
            "Grip the hose at the bend",
            opening=0.04,
            yaw=1.57,
        )
        target_angle = 0.87 if incomplete else 0.04
        for a in np.linspace(s.joint("kink"), target_angle, 6)[1:]:
            tx = self.x - 0.068 * math.cos(a) + 0.011 * math.sin(a)
            ty = self.y - 0.068 * math.sin(a) - 0.011 * math.cos(a)
            s.move_arm(arm, (tx, ty, z + 0.007), 0.41, "Guide the kink open", yaw=1.57)
        s.move_arm(arm, (tx, ty, 0.357), 0.9, "Release the hose", opening=0.85, yaw=1.57)
        s.wait(5.5, "Observe the steady stream after releasing the hose")


