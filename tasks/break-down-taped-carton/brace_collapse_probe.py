"""Brace the far wall while rolling the open carton onto its side."""
import numpy as np

from yam_arena import ik
from yam_arena.constants import FINGER_OPEN

from collapse import CartonFlattenProbe
from collapse_geometry import first_cardboard_surface, vertices
from turnover import roll_wrist
from turnover_targets import support_target


class BracedCollapseProbe(CartonFlattenProbe):
    sides = ('left', 'right')

    def __init__(self, env, previous):
        super().__init__(env, previous)
        self.stage = 'brace_hover'
        self.on_side = False
        self.arm_bias = np.zeros((2,3))
        self.brace_clear = False
        self.high_posture = (.55,.18,0.)
        from scipy.spatial import ConvexHull
        cloud = np.unique(self.tips,axis=0)
        self.support_cloud = cloud[ConvexHull(cloud).vertices]
        self.shear_first = False

    def fingertip_support(self, wrist, inward):
        cloud = self.support_cloud @ wrist.T
        h = cloud @ inward
        weights = np.exp((h-h.max())/.00013)
        return weights @ cloud / weights.sum()

    def flexible_push(self, arm, target, desired, inward, side):
        from scipy.optimize import least_squares
        inward = inward.copy()
        inward[2] = 0.
        inward /= np.linalg.norm(inward)
        aim = target+self.fingertip_support(desired,inward)
        base = self.env.rig.bases[side]

        def residual(q):
            p,R = ik.tcp_pose(q,base)
            error = p+self.fingertip_support(R,inward)-aim
            approach = R[:,2]
            return np.r_[error, .005*(approach-desired[:,2]),
                         .0015*(R[:,0]-desired[:,0]),
                         .025*max(0.,.30-inward@approach),
                         .025*max(0.,.52+approach[2]),
                         .00045*(q-arm[0])]

        result=least_squares(residual,np.clip(arm[0],ik.LOWER,ik.UPPER),
                             bounds=(ik.LOWER,ik.UPPER),max_nfev=38,
                             ftol=1e-6,xtol=1e-5,gtol=1e-6)
        error=return_error=float(np.linalg.norm(result.fun[:3]))
        if np.linalg.norm(result.fun[9:11])>.003: return_error=max(.008,error)
        return result.x,return_error,0.

    def setup_stage(self, phase, time, arms):
        self.stage, self.started = phase, time
        self.starts = [(a[0].copy(), a[2].copy(), a[3].copy()) for a in arms]
        self.arm_bias.fill(0.)

    def support(self, wall, poses, points, high, preload, clearance, posture):
        names = (wall, wall+'_top', wall+'_bottom')
        cloud = np.concatenate([points[n] for n in names])
        p, frame = poses[wall]
        out = frame[:,1].copy()
        out[2] = 0.
        out /= np.linalg.norm(out)
        t = frame[:,0].copy()
        t[2] = 0.
        t /= np.linalg.norm(t)
        if t[0]<0:t = -t
        normal_gain,tangent_gain,roll = posture
        z = -normal_gain*out+tangent_gain*t-np.array((0.,0.,np.sqrt(1.-normal_gain**2-tangent_gain**2)))
        x = t-z*(z@t)
        x /= np.linalg.norm(x)
        R = roll_wrist(np.column_stack((x,np.cross(z,x),z)),roll)
        contact = p.copy()
        contact[2] = cloud[:,2].max()-.024 if high else cloud[:,2].min()+.045
        contact += (np.max(cloud@out)-contact@out)*out
        surface = first_cardboard_surface(contact+.18*out,-out,
                                          [(*poses[n],self.parts[n]['size']) for n in names])
        if surface is not None:
            contact = surface
        goal, R = support_target(contact,-out,R,self.tips,preload)
        goal += clearance*(.028*out+np.array((0.,0.,.026)))
        return goal, R, names

    def command(self,time):
        if self.on_side:return super().command(time)
        poses = {name:self.pose(name) for name in self.parts}
        points = vertices(self.parts,{n:p for n,(p,_) in poses.items()},{n:r for n,(_,r) in poses.items()})
        arms = self.arms()
        if self.started is None:
            positive = np.mean([poses[n][0][1] for n in ('north','south')]) >= 0.
            self.index = 0 if positive else 1
            self.side = self.sides[self.index]
            self.direction = -1 if positive else 1
            self.contact_wall = min(('north','south'),key=lambda n:self.direction*poses[n][0][1])
            self.brace_wall = 'north' if self.contact_wall=='south' else 'south'
            self.roll_direction = -poses[self.contact_wall][1][:,1].copy()
            self.roll_direction[2] = 0.
            self.roll_direction /= np.linalg.norm(self.roll_direction)
            if self.shear_first and not self.configure_diagonal(poses,arms):
                self.failure='opposing carton corners are outside the reachable poses'
                return self.previous.copy(),[self.report()]
            self.setup_stage('brace_hover',time,arms)
        age = time-self.started
        up = -poses['north'][1][:,2]
        self.lean = float(np.degrees(np.arctan2(self.roll_direction@up,up[2])))
        self.height = float(np.ptp(np.concatenate(list(points.values()))[:,2]))
        phase = self.stage
        self.brace_clear |= self.lean>=25. and not self.shear_first
        next_stage = None
        duration = {'brace_hover':6.,'brace_approach':5.,'brace_push':3.,'clear_roll':4.}[phase]
        s = ik.smoothstep(age/duration)
        desired = self.previous.copy()
        contacts = [(),()]
        tracking = []
        error = []
        for i,(side,arm) in enumerate(zip(self.sides,arms)):
            high = i==self.index
            names = self.contact_wall if high else self.brace_wall
            if phase=='clear_roll':
                target = self.starts[i][1]+(.058 if high else -.045)*(-self.roll_direction)+np.array((0,0,.045))
                orientation = self.starts[i][2]
                if age<=1.:
                    contacts[i]=(names,names+'_top',names+'_bottom')
            else:
                preload = 0.
                clearance = 1. if phase=='brace_hover' else .06
                if phase=='brace_push':
                    if high or not self.brace_clear:
                        preload = (.0041 if high else .0038)*s
                    else:
                        clearance = 2.5
                posture = self.high_posture if high else (.55,.18,0.)
                if self.shear_first:
                    target,orientation,expected=self.corner_support(poses,high,preload,clearance)
                else:
                    target, orientation, expected = self.support(names,poses,points,high,preload,clearance,posture)
                if phase!='brace_hover' and clearance<1.:contacts[i]=expected
            origin = self.starts[i]
            use_joint_path = phase=='brace_hover'
            waypoint = target if use_joint_path else origin[1]+s*(target-origin[1])
            wrist = orientation if use_joint_path else ik.slerp_matrix(origin[2],orientation,s)
            tracking.append(float(np.linalg.norm(target-arm[2])))
            if not use_joint_path or s>=1.:
                self.arm_bias[i]=np.clip(self.arm_bias[i]+.035*(waypoint-arm[2]),-.0025,.0025)
                if phase=='brace_push':
                    along=self.roll_direction@self.arm_bias[i]
                    self.arm_bias[i]+=(np.clip(along,-.00045,.00045)-along)*self.roll_direction
            q,pe,re=ik.solve(waypoint+self.arm_bias[i],ik.matrix_to_quat_xyzw(wrist),arm[0],
                             base_pos=self.env.rig.bases[side],iters=155)
            if phase=='brace_push' and (high or self.shear_first) and (pe>.0025 or re>.048):
                if self.shear_first:
                    inward = self.diagonal_inward(poses, high)
                else:
                    inward = -poses[names][1][:,1]
                if self.shear_first or s>=1.:
                    alternate,ap,ar=self.flexible_push(arm,waypoint+self.arm_bias[i],wrist,inward,side)
                    if ap <= .003 and ar <= .048:
                        q,pe,re=alternate,ap,ar
            if use_joint_path:q=origin[0]+s*(q-origin[0])
            error.append((pe,re))
            k=7*i
            desired[k:k+6]=q
            desired[k+6]=self.aperture/abs(FINGER_OPEN)
        if phase=='brace_hover' and age>=duration and max(tracking)<.006:
            next_stage='brace_approach'
        elif phase=='brace_approach' and age>=duration and max(tracking)<.006:
            next_stage='brace_push'
        elif phase=='brace_push' and self.lean>=55.:
            next_stage='clear_roll'
        if self.shear_first:
            dot=float(abs(poses['north'][1][:,0]@poses['east'][1][:,0]))
            self.fold_degrees=float(np.degrees(np.arccos(np.clip(dot,0.,1.))))
            if self.fold_degrees<13.:
                self.done=True
                return self.previous.copy(),[{**self.report(),'corner_angle_deg':self.fold_degrees}]
        if age>24.:self.failure=phase+' did not achieve measured progress'
        if any(pe>.007 or re>.085 for pe,re in error):self.failure='braced roll reached a kinematic limit'
        if self.failure:
            return self.previous.copy(),[{**self.report(),'ik_errors':error,'tracking_errors_m':tracking}]
        bounds = {n:(*pose,self.parts[n]['size']) for n,pose in poses.items()}
        tool,tr=self.pose('cutter')
        bounds['parked_cutter']=(tool,tr,(.12,.026,.025))
        slew = np.tile([.028]*6+[.04],2)
        desired=np.clip(np.clip(desired,self.previous-slew,self.previous+slew),self.env._low,self.env._high)
        if self.shear_first:
            self.previous=desired
            if next_stage:self.setup_stage(next_stage,time,arms)
            return desired,[{**self.report(),'corner_angle_deg':self.fold_degrees,
                              'tracking_errors_m':tracking,'ik_errors':error}]
        hold=self.previous.copy()
        for i,arm in enumerate(arms):hold[7*i:7*i+6]=arm[0]
        candidates=[(self.previous+f*(desired-self.previous), f) for f in (1.,.5,.25,.125,0.)]
        candidates.append((hold,0.))
        for action,f in candidates:
            hands=[(action[i:i+6],np.repeat(action[i+6]*FINGER_OPEN,2)) for i in (0,7)]
            meshes=[self.geometry.parts(side,q,opening) for side,(q,opening) in zip(self.sides,hands)]
            guards=[self.geometry.guard(side,q,g,bounds,meshes[1-i],
                       intended=bool(contacts[i]), intended_targets=contacts[i])
                    for i,(side,(q,g)) in enumerate(zip(self.sides,hands))]
            if all(r['safe'] and r['target_gap_m']>=-.0042 for r in guards):
                self.previous=action
                if next_stage and f==1.:self.setup_stage(next_stage,time,arms)
                break
        else:
            self.failure='no guarded braced roll action'
            self.last_guards=guards
        if phase=='clear_roll' and age>=4. and self.lean>=75. and max(tracking)<.008:
            self.on_side=True
            self.change('approach_flatten',time,arms[self.index])
        report=self.report()
        report.update(tracking_errors_m=tracking, ik_errors=error)
        if self.shear_first:report['corner_angle_deg']=self.fold_degrees
        if self.failure:report['clearance']=getattr(self,'last_guards',None)
        return self.previous.copy(),[report]
