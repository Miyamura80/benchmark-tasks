"""Pinch the upright carton rim and translate it along the workbench."""
from itertools import product

import numpy as np

from collapse import CartonFlattenProbe
from hook_flap import CartonFlapHook
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN
from yam_arena.urdf import axis_angle_matrix


class RimPull(CartonFlattenProbe):
    aperture = .015

    def __init__(self, env, previous, *, align_for_flattening=False):
        super().__init__(env, previous)
        self.stage = 'hover'
        self.plan = None
        self.shift_m = 0.
        self.up = np.array([0.,0.,1.])
        self.align_for_flattening = align_for_flattening

    def grasp_path(self, poses, contact, wrist, fraction):
        north, wall = poses['north']
        if not self.align_for_flattening:
            return contact + fraction*.100*wall[:,1], wrist
        centre = (north+poses['south'][0])/2.
        outward = wall[:,1]
        initial_yaw=np.arctan2(outward[0],-outward[1])
        turn = axis_angle_matrix(self.up,fraction*(np.radians(30.9)-initial_yaw))
        goal = np.array([.310,.220,centre[2]])
        return centre + fraction*(goal-centre) + turn@(contact-centre), turn@wrist

    def select_rim(self, poses, arms):
        active, other = arms
        seed, opening = active[:2]
        passive_mesh = self.geometry.parts('right', *other[:2])
        base = self.env.rig.bases['left']
        p,wall = poses['north']
        outward = wall[:,1].copy()
        outward[2] = 0.
        outward /= np.linalg.norm(outward)
        size = self.parts['north']['size']
        obstacles = {n:(*pose,self.parts[n]['size']) for n,pose in poses.items() if n in self.parts}
        obstacles['parked_cutter'] = (*poses['cutter'],(.12,.026,.025))
        targets = ('north','north_bottom')
        configurations=product((12.,6.,-8.,17.),(.15,0.,-.15),(1.,-1.),
                               (0.,.020,-.020),(-.007,-.004),(.015,.022))
        for tilt,yaw,sign,offset,zoffset,jaw in configurations:
            heading=axis_angle_matrix(self.up,yaw)@outward
            wrist=CartonFlapHook.wrist(np.radians(tilt),heading,self.up,sign)
            contact=p+offset*wall[:,0]+(.5*size[2]+zoffset)*self.up+.0006*outward
            hover=contact+.052*self.up
            approach=[]
            q=seed
            motion=[(hover,wrist),(contact,wrist)]
            motion.extend(self.grasp_path(poses,contact,wrist,fraction) for fraction in (.5,1.))
            for point, rotation in motion:
                q,pe,re=ik.solve(point,ik.matrix_to_quat_xyzw(rotation),q,base_pos=base,iters=110)
                if pe>.0012 or re>.018:break
                approach.append(q)
            if len(approach)!=len(motion):continue
            high,low=approach[:2]
            opened=np.full(2,-jaw)
            path=[(seed+f*(high-seed),opening+f*(opened-opening),False)
                  for f in np.linspace(0.,1.,10)]
            path.extend((high+f*(low-high),opened,False) for f in (.2,.4,.6,.8,1.))
            path.append((low,np.full(2,-.0008),True))
            if all((report:=self.geometry.guard('left',q,gap,obstacles,passive_mesh,
                                               intended=contact_only,intended_targets=targets))['safe']
                   and report['target_gap_m']>=-.0058 for q,gap,contact_only in path):
                return dict(wrist=wrist,hover=hover,contact=contact,high=high,outward=outward,
                            jaw=jaw,carton_start=p.copy(),poses=poses)
        return None

    def advance(self, stage, time, arm):
        self.stage,self.started=stage,time
        self.start_q,self.start_tcp = arm[0].copy(),arm[2].copy()
        self.bias.fill(0.)

    def command(self, time):
        arms=self.arms()
        arm=arms[0]
        if self.plan is None:
            poses={name:(p.copy(),rotation.copy()) for name in (*self.parts,'cutter')
                   for p,rotation in (self.pose(name),)}
            self.plan=self.select_rim(poses,arms)
            if self.plan is None:
                self.failure='no guarded rim grasp to reposition the carton'
                return self.previous.copy(),[dict(phase='rim_plan',failure=self.failure)]
            self.advance('hover',time,arm)
        p=self.plan
        phase=self.stage
        elapsed=time-self.started
        opening=p['jaw']
        wrist=p['wrist']
        measured_gap=-sum(arm[1])
        self.shift_m=float((self.pose('north')[0]-p['carton_start'])@p['outward'])
        if phase in ('hover','contact'):
            target,duration=p[phase],4.
            following='contact' if phase=='hover' else 'close'
            if elapsed>=duration and np.linalg.norm(arm[2]-target)<.0035:
                self.advance(following,time,arm)
        elif phase=='close':
            target,duration=p['contact'],2.4
            opening=.0008+(opening-.0008)*(1.-ik.smoothstep(elapsed/duration))
            if elapsed>=duration and .003<measured_gap<.020:
                self.advance('pull',time,arm)
        elif phase=='pull':
            duration=7.
            target,wrist=self.grasp_path(p['poses'],p['contact'],p['wrist'],
                                         ik.smoothstep(elapsed/duration))
            opening=.0008
            progress=self.shift_m>=.068
            if self.align_for_flattening:
                north,frame=self.pose('north')
                south=self.pose('south')[0]
                centre=.5*(north+south)
                yaw=np.degrees(np.arctan2(frame[0,1],-frame[1,1]))
                progress=np.linalg.norm(centre[:2]-[.310,.220])<.028 and abs(yaw-30.9)<12.
            if elapsed>=duration and progress:
                self.release_wrist=wrist.copy()
                self.advance('release',time,arm)
            if measured_gap<.0028:
                self.failure='rim slipped out before the measured carton translation'
        elif phase=='release':
            target,duration=self.start_tcp,1.7
            wrist=self.release_wrist
            opening=.0008+(opening-.0008)*ik.smoothstep(elapsed/duration)
            if elapsed>=duration and measured_gap>.022:
                self.advance('withdraw',time,arm)
        else:
            target,duration=self.start_tcp+.043*self.up,4.
            wrist=self.release_wrist
            if elapsed>=duration and np.linalg.norm(arm[2]-target)<.008:
                self.done=True
        blend=ik.smoothstep(elapsed/duration)
        waypoint=target if phase in ('hover','pull') else self.start_tcp+blend*(target-self.start_tcp)
        self.bias=np.clip(self.bias+.045*(waypoint-arm[2]),-.0024,.0024)
        q,pe,re=ik.solve(waypoint+self.bias,ik.matrix_to_quat_xyzw(wrist),arm[0],
                         base_pos=self.env.rig.bases['left'],iters=120)
        if phase=='hover':q=self.start_q+blend*(p['high']-self.start_q)
        if elapsed>duration+6. or phase!='hover' and (pe>.006 or re>.07):
            self.failure='rim grasp stalled during '+phase
        action=self.previous.copy()
        action[:6]=np.clip(q,action[:6]-.026,action[:6]+.026)
        action[6]=opening/abs(FINGER_OPEN)
        self.previous=np.clip(action,self.env._low,self.env._high)
        return self.previous.copy(),[dict(phase='rim_'+phase,failure=self.failure,
            shift_m=self.shift_m,position_error=float(np.linalg.norm(arm[2]-target)),
            ik_error=float(pe),ik_rotation_error=float(re),finger_gap=float(measured_gap))]
