"""Move the inverted sleeve toward the near arm by pushing the inside north face."""
import numpy as np
from yam_arena import ik
from yam_arena.demo import grip_rotation
from yam_arena.constants import FINGER_OPEN
from yam_arena.urdf import axis_angle_matrix
from collapse import CartonFlattenProbe


class InsideShift(CartonFlattenProbe):
    def command(self,time):
        arms=self.arms()
        arm=arms[0]
        boxes={n:(*self.pose(n),p['size']) for n,p in self.parts.items()}
        boxes['parked_cutter']=(*self.pose('cutter'),(.12,.026,.025))
        north,wall,sz=boxes['north']
        centre=(north+boxes['south'][0])/2.
        fingers=np.full(2,-self.aperture)
        other=self.geometry.parts('right',*arms[1][:2])
        if self.started is None:
            west_angle = float(self.env._base._carton_angle_observer.tracker.angles[3])
            if west_angle < 265.:
                if not hasattr(self,'wait_since'): self.wait_since=time
                if time-self.wait_since>25.:self.failure='west flap did not settle clear of approach'
                return self.previous.copy(),[dict(phase='wait_for_west_clearance',west_angle=west_angle,
                                                  failure=self.failure)]
            self.direction=wall[:,1].copy()
            self.direction[2]=0.
            self.direction/=np.linalg.norm(self.direction)
            if self.direction[1]>0:self.direction*=-1.
            self.start_centre=centre.copy()
            self.wrist=(axis_angle_matrix(wall[:,0],np.radians(-20.))
                        @axis_angle_matrix(np.array((0.,0.,1.)),np.pi/2)@grip_rotation())
            local=np.concatenate([(tri.reshape(-1,3)-arm[2])@arm[3]
                                  for n,tri in self.geometry.parts('left',arm[0],fingers)
                                  if n in ('tip_left','tip_right')])
            self.pad=local[np.argmax(local@self.wrist.T@self.direction)]
            self.seed=arm[0].copy()
            self.change('inside_hover',time,arm)
        contact=north.copy()
        contact[2]-=.0085
        tcp=contact-(sz[1]/2+.00025)*self.direction-self.wrist@self.pad
        pre=tcp-.022*self.direction
        hover=pre.copy()
        hover[2]=max(.962,tcp[2]+.044)
        shift=float((centre-self.start_centre)@self.direction)
        age=time-self.started
        stage=self.stage
        g=self.geometry.guard('left',*arm[:2],boxes,other,intended=True,intended_targets=('north',))
        pe=re=0.;nextstage=None;goal=arm[2]
        if stage=='inside_hover':
            goal=hover
            if not hasattr(self,'hover_q'):
                self.hover_q,pe,re=ik.solve(goal,ik.matrix_to_quat_xyzw(self.wrist),self.seed,
                                            base_pos=self.env.rig.bases['left'],iters=120)
            q=self.seed+ik.smoothstep(age/5.)*(self.hover_q-self.seed)
            if age>=5. and np.linalg.norm(arm[2]-goal)<.004:nextstage='inside_descend'
        else:
            if stage=='inside_descend':
                goal=pre;duration=4.;following='inside_contact'
            elif stage=='inside_contact':
                goal=tcp;duration=3.;following='inside_push'
            elif stage=='inside_push':
                goal=tcp+.0040*ik.smoothstep(age/3.)*self.direction
                duration=.20;following=None
                if shift>=.047:nextstage='inside_backoff'
            elif stage=='inside_backoff':
                goal=self.origin-.024*self.direction;duration=2.;following='inside_lift'
            else:
                goal=hover;duration=3.;following='done'
            waypoint=self.origin+ik.smoothstep(age/duration)*(goal-self.origin)
            if stage=='inside_backoff':
                waypoint-=.0015*(1.-ik.smoothstep(age/duration))*self.direction
            q,pe,re=ik.solve(waypoint,ik.matrix_to_quat_xyzw(self.wrist),arm[0],
                             base_pos=self.env.rig.bases['left'],iters=100)
            if following and age>=duration and np.linalg.norm(arm[2]-goal)<.004:
                nextstage=following
        if pe>.004 or re>.05 or age>18.:
            self.failure='inside wall approach lost its measured target'
        if stage=='inside_push' and (not g['safe'] or g['target_gap_m']<-.0015):
            self.failure='measured fingertip or hard-body clearance exceeded during push'
        commanded_limit=.0042 if stage=='inside_push' else .0015
        allowed=stage in ('inside_contact','inside_push','inside_backoff')
        issued=np.clip(q,self.previous[:6]-.024,self.previous[:6]+.024)
        for fraction in (1.,.5,.25,.125,.0625,.03125,0.):
            action=self.previous.copy()
            action[:6]+=fraction*(issued-action[:6])
            action[6]=self.aperture/abs(FINGER_OPEN)
            guard=self.geometry.guard('left',action[:6],fingers,boxes,other,
                                      intended=allowed,intended_targets=('north',))
            parked=self.geometry.guard('right',*arms[1][:2],boxes,
                                      self.geometry.parts('left',action[:6],fingers))
            if parked['safe'] and guard['safe'] and guard['target_gap_m']>=-commanded_limit:
                self.previous=action
                if nextstage and (fraction==1. or stage=='inside_push'):
                    if nextstage=='done':self.done=True
                    else:self.change(nextstage,time,arm)
                break
        else:self.failure='no guarded inside wall step'
        return self.previous.copy(),[dict(phase=stage,shift_m=shift,target=goal.tolist(),
               measured_tcp=arm[2].tolist(),measured_gap_m=g['target_gap_m'],guard=guard,parked_guard=parked,
               failure=self.failure,complete=self.done,ik_errors=[float(pe),float(re)])]
