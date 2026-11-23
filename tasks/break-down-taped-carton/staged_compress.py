"""Stage the two hands outside the carton before applying diagonal pressure."""
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN
from compress_and_tip_probe import CompressAndTipProbe


class StagedCompressAndTip(CompressAndTipProbe):
    def __init__(self,env,previous):
        super().__init__(env,previous)
        self.entry='initialize'

    def command(self,time):
        if self.entry is None:
            return super().command(time)
        probe=self.shear
        if self.entry=='initialize':
            self.previous,report=super().command(time)
            if self.failure:
                return self.previous.copy(),report
            arms=probe.arms()
            poses={n:probe.pose(n) for n in probe.parts}
            high=[]
            for i,(side,arm) in enumerate(zip(self.sides,arms)):
                p,R,_=probe.corner_support(poses,i==probe.index,0.,1.)
                target,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),arm[0],
                                      base_pos=self.env.rig.bases[side],iters=110)
                if pe>.0015 or re>.025:
                    self.failure='diagonal entry hover is outside wrist reach'
                    return self.previous.copy(),[dict(phase='diagonal_entry',failure=self.failure)]
                high.append(target)
            self.high=np.array(high)
            self.source=np.array([a[0] for a in arms])
            self.mid=self.source.copy()
            self.mid[0,[0,2]]=self.high[0,[0,2]]
            self.entry='left_clear_wall'
            self.entry_time=time
        arms=probe.arms()
        bounds={n:(*probe.pose(n),p['size']) for n,p in probe.parts.items()}
        bounds['parked_cutter']=(*probe.pose('cutter'),(.12,.026,.025))
        goal=self.mid if self.entry=='left_clear_wall' else self.high
        age=time-self.entry_time
        destination=self.source+ik.smoothstep(age/4.)*(goal-self.source)
        cmds=self.previous.copy()
        for i in (0,1):
            k=i*7
            cmds[k:k+6]=np.clip(destination[i],self.previous[k:k+6]-.024,self.previous[k:k+6]+.024)
            cmds[k+6]=probe.aperture/abs(FINGER_OPEN)
        for f in (1.,.5,.25,.125,0.):
            action=self.previous+f*(cmds-self.previous)
            states=[(action[k:k+6],np.full(2,action[k+6]*FINGER_OPEN)) for k in (0,7)]
            meshes=[probe.geometry.parts(s,*state) for s,state in zip(self.sides,states)]
            guards=[probe.geometry.guard(s,*state,bounds,meshes[1-i]) for i,(s,state) in enumerate(zip(self.sides,states))]
            if all(g['safe'] for g in guards):
                self.previous=action
                break
        else:self.failure='no guarded diagonal hover step'
        error=float(max(np.max(abs(g-arm[0])) for g,arm in zip(goal,arms)))
        phase=self.entry
        if age>=4. and error<.030 and not self.failure:
            if self.entry=='left_clear_wall':
                self.source=np.array([a[0] for a in arms])
                self.entry_time=time
                self.entry='exterior_hover'
            else:
                self.entry=None
                probe.previous=self.previous.copy()
                probe.setup_stage('brace_approach',time,arms)
        if age>13.:self.failure='guarded diagonal entry did not settle'
        return self.previous.copy(),[dict(phase=phase,joint_error_rad=error,
                                          guards=guards,failure=self.failure)]
