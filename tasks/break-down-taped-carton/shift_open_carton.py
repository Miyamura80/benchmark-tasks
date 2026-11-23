"""Guide the inverted carton back into the bimanual working area."""
from itertools import product

import numpy as np

from collapse import CartonFlattenProbe
from turnover import roll_wrist
from turnover_targets import support_target
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN


class ShiftOpenCarton(CartonFlattenProbe):
    aperture = .010

    def select_wall_pose(self, poses, arms):
        side = 'left'
        q, opening, _, _ = arms[0]
        other = self.geometry.parts('right', *arms[1][:2])
        base = self.env.rig.bases[side]
        wall_name = 'north'
        centre, rotation = poses[wall_name]
        inner = -rotation[:,1]
        inner[2] = 0.
        inner /= np.linalg.norm(inner)
        t = rotation[:,0]
        up = np.array([0.,0.,1.])
        obs = {n: (*poses[n], m['size']) for n,m in self.parts.items()}
        aperture = np.full(2,-self.aperture)
        names = (wall_name,)
        for height, shift, tilt, roll in product(
                (.030,.017,.040,.005),(.0,.026,-.026,.047,-.047),
                (12.,23.,34.,42.),(0.,180.,30.,-30.,90.,-90.)):
            deg = np.radians(tilt)
            z = -inner*np.sin(deg) - up*np.cos(deg)
            R = roll_wrist(np.column_stack((t,np.cross(z,t),z)),roll)
            contact = centre + inner*(self.parts[wall_name]['size'][1]/2.) + up*height + t*shift
            touch, R = support_target(contact,-inner,R,self.tips,.0017)
            lower = touch + inner*.012
            hover = lower + up*.059
            quat = ik.matrix_to_quat_xyzw(R)
            base_q = q
            trajectory=[]
            for point in (hover,lower,touch,touch-.056*inner):
                target,pe,re=ik.solve(point,quat,base_q,base_pos=base,iters=110)
                if pe>.0014 or re>.019:
                    break
                trajectory.append(target)
                base_q=target
            if len(trajectory)!=4:
                continue
            jh, jl, jt, _ = trajectory
            checks = [(q+v*(jh-q), opening+v*(aperture-opening), False, obs)
                      for v in np.linspace(0.,1.,8)]
            checks.extend((jh+v*(jl-jh),aperture,False,obs) for v in (.22,.44,.66,.85,1.))
            checks.extend((jl+v*(jt-jl),aperture,True,obs) for v in (.3,.6,.8,1.))
            if all((clear:=self.geometry.guard(side,a,b,c,other,intended=d,intended_targets=names))['safe']
                   and clear['target_gap_m']>-.0025 for a,b,d,c in checks):
                return dict(rotation=R,direction=-inner,hover=hover,lower=lower,
                            touch=touch,hover_joints=jh,carton_start=centre.copy())
        return None

    def __init__(self,env,previous):
        super().__init__(env,previous)
        self.stage = 'hover'
        self.travel = 0.
        self.plan = None

    def advance(self,stage,time,arm):
        self.stage,self.started=stage,time
        self.initial_q=arm[0].copy()
        self.initial_tcp=arm[2].copy()

    def command(self,time):
        arms=self.arms()
        arm=arms[0]
        if self.plan is None:
            self.plan=self.select_wall_pose({name:self.pose(name) for name in self.parts},arms)
            if self.plan is None:
                self.failure='no guarded inside-wall approach to reposition the carton'
                return self.previous.copy(),[dict(phase='carton_shift',failure=self.failure)]
            self.advance('hover',time,arm)
        plan=self.plan
        direction=plan['direction']
        phase=self.stage
        age=time-self.started
        self.travel=float((self.pose('north')[0]-plan['carton_start'])@direction)
        if phase in ('hover','lower'):
            target=plan[phase]
            duration=4.
        elif phase=='push':
            target=plan['touch']+.100*direction*ik.smoothstep(age/8.)
            duration=2.5
        else:
            target=self.initial_tcp-.027*direction+np.array([0.,0.,.055])
            duration=3.
        blend=ik.smoothstep(age/duration)
        waypoint=self.initial_tcp+blend*(target-self.initial_tcp)
        q,p_error,r_error=ik.solve(waypoint,ik.matrix_to_quat_xyzw(plan['rotation']),arm[0],
                                   base_pos=self.env.rig.bases['left'],iters=120)
        if phase=='hover':
            q=self.initial_q+blend*(plan['hover_joints']-self.initial_q)
        error=float(np.linalg.norm(arm[2]-target))
        if phase=='hover' and age>=duration and error<.008:
            self.advance('lower',time,arm)
        elif phase=='lower' and age>=duration and error<.003:
            self.advance('push',time,arm)
        elif phase=='push' and (self.travel>=.085 or age>=9.):
            if self.travel<.029:
                self.failure='inside-wall pressure did not translate the carton'
            else:
                self.advance('clear',time,arm)
        elif phase=='clear' and age>=duration and error<.008:
            self.done=True
        if age>14.:
            self.failure='carton repositioning stalled during '+phase
        action=self.previous.copy()
        action[:6]=np.clip(q,action[:6]-.03,action[:6]+.03)
        action[6]=self.aperture/abs(FINGER_OPEN)
        self.previous=np.clip(action,self.env._low,self.env._high)
        return self.previous.copy(),[dict(phase='carton_shift_'+phase,failure=self.failure,
            shift_m=self.travel,position_error=error,ik_error=float(p_error))]
