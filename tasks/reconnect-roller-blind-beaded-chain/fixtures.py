"""Three contact-driven repair stations for dual-YAM tabletop trials."""

import math

import mujoco as mj
import numpy as np

from station import DT
import realism
from fixture_base import (
    BLACK, BLUE, DARK, ORANGE, RUBBER, STEEL, TEAL, VIS, WHITE,
    Fixture, Handle, body, box, capsule, cyl, element, geom, joint, plate,
)

BRUSH = (0.88, 0.72, 0.52, 1)
FIBER = (0.66, 0.61, 0.50, 1)


def slide(parent, name, axis, limit, **kwargs):
    return joint(parent, name, "slide", axis, limit, **kwargs)


def slide_force(s, name, target, gain, damping, cap):
    return np.clip(gain * (target - s.joint(name)) - damping * s.speed(name), -cap, cap)


def touching(s, first, second):
    a, b = s.model.body(first).id, s.model.body(second).id
    body_ids = s.model.geom_bodyid
    return any(
        c.dist < 0.001 and {int(body_ids[c.geom1]), int(body_ids[c.geom2])} == {a, b}
        for c in s.data.contact
    )






def simple_geom(scene, kind, size, position, color, matrix=None, material=None):
    if scene.ngeom >= scene.maxgeom:
        raise RuntimeError("Fixture render geometry limit reached")
    mj.mjv_initGeom(scene.geoms[scene.ngeom], kind, np.asarray(size), np.asarray(position),
                    np.eye(3).ravel() if matrix is None else np.asarray(matrix).ravel(),
                    np.asarray(color, dtype=np.float32))
    if material is not None:
        scene.geoms[scene.ngeom].matid = material
        scene.geoms[scene.ngeom].texcoord = 1
    scene.ngeom += 1




def line(scene,start,end,radius,color):
    if scene.ngeom >= scene.maxgeom:
        raise RuntimeError("Fixture render geometry limit reached")
    connector=scene.geoms[scene.ngeom]
    mj.mjv_initGeom(connector,mj.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.asarray(color,dtype=np.float32))
    mj.mjv_connector(connector,mj.mjtGeom.mjGEOM_CAPSULE,radius,np.asarray(start,dtype=float),np.asarray(end,dtype=float))
    scene.ngeom+=1


class Blind(Fixture):
    title = "Reconnect a roller blind’s beaded chain"
    instruction = "Reconnect the two ends of the chain through the coupler. Use opposite sides of the closed loop to lower the blind, then raise it back."
    focus = (.066,-.025,.345)
    loop_x, loop_y, roller_z = .175,-.014,.443
    pitch_radius=.032
    bead_pitch=.0084
    up_radius=.063
    down_radius=.084
    mate=(.124,-.127,.241)
    seat=(.124,-.103,.241)
    pull_start_z=.295
    pull_span=.039
    handles=(
        Handle("terminal","terminal_travel","right",(0,1,0),max_force=13),
        Handle("down_chain","down_stroke","right",(0,0,-1),max_force=10),
        Handle("up_chain","up_stroke","left",(0,0,-1),max_force=10),
    )

    def build(self,w):
        plate(w,sx=.19)
        for x in (-.095,.089):
            box(w,f"stand_{x}",(x,.045,.266),(.009,.017,.065),DARK)
            box(w,f"foot_{x}",(x,.028,.207),(.021,.035,.006),DARK)
        roller=body(w,"shade_roller",(.045,self.loop_y,self.roller_z))
        joint(roller,"spool_angle","hinge",(1,0,0),(-.20,2.5),damping=.016,frictionloss=.066)
        cyl(roller,"cloth_roll",(0,0,0),self.pitch_radius,.099,WHITE,euler=(0,math.pi/2,0),mass=.11,contype=0,conaffinity=0)
        sprocket_x = self.loop_x+.020-.045
        cyl(roller,"chain_sprocket",(sprocket_x,0,0),self.pitch_radius-.003,.007,DARK,euler=(0,math.pi/2,0),mass=.025,contype=0,conaffinity=0)
        for i in range(24):
            a=i*math.tau/24
            box(roller,f"pocket_{i}",(sprocket_x,self.pitch_radius*math.cos(a),self.pitch_radius*math.sin(a)),(.005,.0027,.0024),RUBBER,euler=(a,0,0),**VIS)
        for x in (self.loop_x-.010,self.loop_x+.029):
            box(w,f"clutch_cheek_{x}",(x,self.loop_y,self.roller_z+.039),(.004,.034,.006),WHITE,**VIS)
        bar=body(w,"hem",(.045,self.loop_y-.030,self.roller_z-.133))
        slide(bar,"hem_drop",(0,0,-1),(-.007,.070),damping=.22)
        box(bar,"bottom_rail",(0,0,0),(.098,.004,.0048),STEEL,mass=.056,contype=0,conaffinity=0)
        for name,y in (("down",self.loop_y-self.down_radius),("up",self.loop_y+self.up_radius)):
            segment=body(w,name+"_chain",(self.loop_x+.020,y,self.pull_start_z))
            slide(segment,name+"_stroke",(0,0,-1),(0,.045),damping=.65,stiffness=78,springref=-.003)
            for i in (-1,0,1):
                geom(segment,f"{name}_ball_{i}","sphere",(0,0,i*.008),(.0045,),WHITE,mass=.008,friction="1.4 .018 .004")
        receiver=body(w,"coupler",self.seat)
        for sign in (-1,1):
            box(receiver,f"coupler_side{sign}",(sign*.008,0,0),(.0027,.012,.006),WHITE)
        box(receiver,"coupler_floor",(0,0,-.007),(.009,.012,.002),WHITE)
        box(receiver,"coupler_end",(0,.011,0),(.009,.0025,.006),WHITE)
        box(w,"connector_cradle",(.124,-.095,.218),(.019,.021,.008),BLUE)
        terminal=body(w,"terminal",self.mate)
        slide(terminal,"terminal_travel",(0,1,0),(0,.030),damping=.52,frictionloss=.30,stiffness=9)
        for i in range(3):
            geom(terminal,f"terminal_ball{i}","sphere",(0,-i*.0085,0),(.0048,),WHITE,mass=.012,friction="1.4 .018 .004")
        capsule(terminal,"terminal_cord",(0,0,0),(0,-.018,0),.0014,WHITE,mass=.002)
        box(terminal,"bead_end_grip",(0,-.008,0),(.0042,.004,.004),WHITE,mass=.002)

    def decorate(self, tree, world, assets):
        realism.blind(tree, world, assets, self.roller_z)

    def constraints(self,tree):
        eq=element(tree,"equality")
        element(eq,"joint",joint1="hem_drop",joint2="spool_angle",polycoef=f"0 {self.pitch_radius} 0 0 0",solref=".004 1")

    def reset(self,s):
        super().reset(s)
        self.latched=False
        self.starts={}
        self.lowest=0.0
        self.raised_with_chain = False
        self.join_angle = 0.0
        self.released_chain_travel = 0.0

    def before_step(self,s):
        super().before_step(s)
        if not self.latched and s.joint("terminal_travel") > .023:
            self.latched = True
            self.join_angle = s.joint("spool_angle")
        if self.latched:
            self.force(s,"terminal_travel",slide_force(s,"terminal_travel",.025,2300,16,15))
        for direction, sign in (("down",1),("up",-1)):
            grip=direction+"_chain"
            stroke=direction+"_stroke"
            if not self.latched or grip not in self.grips:
                self.starts.pop(direction,None)
                continue
            if direction not in self.starts:
                self.starts[direction]=(s.joint(stroke),s.joint("spool_angle"))
            q0,a0=self.starts[direction]
            # The bead sprocket transmits displacement in either direction with spring compliance.
            stretch=s.joint(stroke)-q0-sign*self.pitch_radius*(s.joint("spool_angle")-a0)
            rel_speed=s.speed(stroke)-sign*self.pitch_radius*s.speed("spool_angle")
            traction=np.clip(650*stretch+2.0*rel_speed,-7,7)
            self.force(s,"spool_angle",sign*self.pitch_radius*traction)
            self.force(s,stroke,-traction)

    def after_step(self,s):
        if self.latched:
            travel = abs(s.joint("spool_angle")-self.join_angle)*self.pitch_radius
            self.released_chain_travel = max(travel, self.released_chain_travel)
        if self.latched and "down_chain" in self.grips:
            self.lowest=max(self.lowest,s.joint("hem_drop"))
        if self.latched and "up_chain" in self.grips and self.lowest - s.joint("hem_drop") > .018:
            self.raised_with_chain = True

    def measure(self,s):
        now=s.joint("hem_drop")
        down=self.lowest>.022
        up=down and self.raised_with_chain and self.lowest-now>.018 and now<.012
        joined=self.latched and s.joint("terminal_travel")>.022
        return dict(score=self.score((joined,down,up and not self.grips)), coupler_closed=joined,
                    shade_drop_mm=round(now*1000,1), peak_drop_mm=round(self.lowest*1000,1),
                    return_mm=round((self.lowest-now)*1000,1), terminal_mm=round(s.joint("terminal_travel")*1000,1))

    def draw_shade(self, s, scene):
        hem = s.position("hem")
        top = self.roller_z + .001
        span = max(.001, top-hem[2])
        simple_geom(scene, mj.mjtGeom.mjGEOM_BOX, (.098,.00044,span/2),
                    (hem[0],hem[1],hem[2]+span/2), (1,1,1,1),
                    material=s.model.mat("shade_linen").id)
        seam = (.74,.69,.59,1)
        for side in (-1,1):
            start = (hem[0]+side*.0978,hem[1]-.00047,hem[2]+.003)
            line(scene, start, (start[0],start[1],top), .00042, seam)
        for y in (hem[1]-.0006, hem[1]+.0006):
            line(scene, (hem[0]-.097,y,hem[2]+.009),
                 (hem[0]+.097,y,hem[2]+.009), .00034, seam)

    @staticmethod
    def bend(a, b, c, d):
        a,b,c,d = map(np.asarray,(a,b,c,d))
        t = np.linspace(0,1,12)[:,None]
        return (1-t)**3*a+3*t*(1-t)**2*b+3*t**2*(1-t)*c+t**3*d

    def chain_middle(self, center):
        x,y,z = center
        right_y = y-self.down_radius
        left_y = y+self.up_radius
        pitch = self.pitch_radius
        return np.vstack((
            (x,left_y,.244),
            self.bend((x,left_y,.330),(x,left_y,z-.057),
                      (x,y+pitch,z-.026),center+(0,pitch,0)),
            [center+(0,pitch*math.cos(a),pitch*math.sin(a))
             for a in np.linspace(0,math.pi,37)[1:-1]],
            self.bend(center-(0,pitch,0),(x,y-pitch,z-.032),
                      (x,right_y,z-.061),(x,right_y,.332)),
            (x,right_y,.244),
        ))

    def chain_path(self, middle, start, end, sway=0.0):
        left,right = middle[0],middle[-1]
        # The ends hang below the working runs, with the bead at each grip
        # staying on the same straight section throughout its pull.
        return np.vstack((
            self.bend(start,start+(sway,.036,0),left-(0,0,.024),left)[:-1],
            middle,
            self.bend(right,right-(0,0,.017),end+(sway,-.017,0),end)[1:],
        ))

    @staticmethod
    def path_distances(path):
        return np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]

    def visual(self, s, scene):
        self.draw_shade(s, scene)
        center = np.array((self.loop_x+.020+self.x_offset,self.loop_y,self.roller_z))
        receiver = s.position("coupler")
        terminal = s.position("terminal")
        middle = self.chain_middle(center)
        pitch = self.bead_pitch
        if self.latched:
            nominal = self.path_distances(self.chain_path(middle,receiver,receiver))[-1]
            count = round(nominal/pitch)
            pitch = nominal/count
            relax = min(1.0, self.released_chain_travel/.031)
            hang = receiver + relax*(np.array((center[0],self.loop_y-.027,.208))-receiver)
            # The low part of the loop lies above the bench. As the coupler
            # leaves the cradle, sag in the return curve takes up the slack.
            low,high = 0.0,.115
            for _ in range(11):
                sway = (low+high)/2
                path = self.chain_path(middle,hang,hang,sway)
                if self.path_distances(path)[-1] > nominal:
                    high = sway
                else:
                    low = sway
            if relax < .001:
                path = self.chain_path(middle,receiver,receiver)
            bead_distances = np.arange(1,count)*pitch
        else:
            path = self.chain_path(middle,receiver,terminal)
            bead_distances = np.arange(pitch,self.path_distances(path)[-1]-.004,pitch)
        length = self.path_distances(path)
        travel = ((s.joint("spool_angle")-self.join_angle)*self.pitch_radius
                  if self.latched else 0.0)

        def point(distance):
            d = distance % length[-1] if self.latched else np.clip(distance, 0, length[-1])
            return np.array([np.interp(d,length,path[:,axis]) for axis in range(3)])

        for offset in bead_distances:
            bead = point(offset+travel)
            simple_geom(scene, mj.mjtGeom.mjGEOM_SPHERE, (.00345,0,0), bead,
                        (.60,.64,.66,1))
        nodes = np.vstack([point(d+travel) for d in np.r_[0,bead_distances,length[-1]]])
        for a,b in zip(nodes,nodes[1:]):
            if np.linalg.norm(b-a) < pitch*1.7:
                line(scene,a,b,.00058,(.44,.45,.43,1))

        if self.latched:
            start, end = point(travel-.0054),point(travel+.0054)
        else:
            start, end = receiver-(0,.007,0),point(.008)
            simple_geom(scene,mj.mjtGeom.mjGEOM_SPHERE,(.0042,0,0),terminal,
                        (.62,.66,.67,1))
        line(scene,start,end,.0047,(.64,.66,.65,1))
        line(scene,(start+end)/2-.23*(end-start)+(0,0,.0039),
             (start+end)/2+.23*(end-start)+(0,0,.0039),.00065,(.29,.31,.29,1))

    def demo(self,s,incomplete=False):
        x,y,z=self.mate
        grasp=(x,y-.015,z)
        self.take(s,"right",grasp,"Grip the loose end of the bead chain")
        finish=(x,y+.028-.015,z)
        self.go(s,"right",finish,"Feed the terminal bead into the coupler",time=1.1)
        self.leave(s,"right",finish,"Release the joined chain")
        if not incomplete:
            self.go(s, "right", (.193,-.156,.343), "Approach the outside of the chain", yaw=math.pi/2, time=1.2)
            for side, sign, phase in (
                ("right",-1,"Lower the shade with the repaired loop"),
                ("left",1,"Raise the shade with the return side of the loop"),
                ("left",1,"Finish raising the repaired shade"),
            ):
                radius = self.down_radius if sign < 0 else self.up_radius
                point=(self.loop_x+.020,self.loop_y+sign*radius,self.pull_start_z)
                self.take(s,side,point,"Grip one side of the working chain",lift=.028,yaw=math.pi/2)
                end=(point[0],point[1],point[2]-self.pull_span)
                self.go(s,side,end,phase,time=1.35,yaw=math.pi/2)
                s.wait(.4)
                self.leave(s,side,end,"Release the chain after the pull",lift=.031,yaw=math.pi/2)
                if side=="right":
                    self.go(s,"right",(.197,-.158,.320),"Clear the working area",yaw=math.pi/2)
        s.wait(.8)
