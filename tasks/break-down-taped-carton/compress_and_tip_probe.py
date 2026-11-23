"""Fold the sleeve in plan, then tip the narrow packet onto the bench."""
import numpy as np

from yam_arena import ik

from collapse_geometry import vertices
from config import TOP
from diagonal_shear_probe import DiagonalShearProbe


class CompressAndTipProbe:
    sides = ('left','right')

    def __init__(self, env, previous):
        self.env = env
        self.shear = DiagonalShearProbe(env,previous)
        self.previous = previous.copy()
        self.failure = None
        self.done = False
        self.stage = 'squeeze'
        self.started = 0.

    def begin(self, time, arms):
        self.started = time
        self.origins = [arm[2].copy() for arm in arms]
        self.rotations = [arm[3].copy() for arm in arms]

    def command(self,time):
        if self.stage=='release_support':
            self.previous,report=self.release.command(time)
            self.failure=self.release.failure
            self.done=self.release.done
            return self.previous.copy(),report
        if self.stage=='squeeze':
            self.previous,report=self.shear.command(time)
            self.failure=self.shear.failure
            if self.shear.done:
                self.stage='tip_packet'
                arms=self.shear.arms()
                poses={n:self.shear.pose(n) for n in ('north','south')}
                centre=np.mean([p for p,_ in poses.values()],axis=0)
                outward=poses['north'][1][:,1].copy()
                outward[2]=0.
                outward/=np.linalg.norm(outward)
                self.active=int(arms[1][2][2]>arms[0][2][2])
                sign=np.copysign(1.,(arms[self.active][2]-centre)@outward)
                self.direction=-sign*outward
                self.begin(time,arms)
            return self.previous.copy(),report

        arms=self.shear.arms()
        part=self.shear.parts
        poses={n:self.shear.pose(n) for n in part}
        cloud=vertices(part,{n:p for n,(p,R) in poses.items()},{n:R for n,(p,R) in poses.items()})
        cloud=np.concatenate(list(cloud.values()))
        low,high=float(cloud[:,2].min()),float(cloud[:,2].max())
        up=-poses['north'][1][:,2]
        lean=float(np.degrees(np.arctan2(up@self.direction,up[2])))
        age=time-self.started
        action=self.previous.copy()
        errors=[]
        for i,(arm,side) in enumerate(zip(arms,self.sides)):
            if self.stage=='tip_packet':
                if i==self.active:
                    target=self.origins[i]+.072*self.direction*ik.smoothstep(age/4.)
                else:
                    outward=self.direction
                    target=self.origins[i]+(.073*outward+np.array((0.,0.,.105)))*ik.smoothstep((age-.70)/4.)
            else:
                sign=-1. if i==self.active else 1.
                target=self.origins[i]+(sign*.061*self.direction+np.array((0.,0.,.088)))*ik.smoothstep(age/7.5)
            q,pe,re=ik.solve(target,ik.matrix_to_quat_xyzw(self.rotations[i]),arm[0],
                            base_pos=self.env.rig.bases[side],iters=130)
            errors.append((float(pe),float(re)))
            k=7*i
            action[k:k+6]=np.clip(q,action[k:k+6]-.024,action[k:k+6]+.024)
        if self.stage=='tip_packet' and lean>=22.:
            self.stage='withdraw'
            self.begin(time,arms)
        elif self.stage=='withdraw' and age>=4.:
            clear=all(np.linalg.norm(arm[2]-np.clip(arm[2],cloud.min(0),cloud.max(0)))>.07 for arm in arms)
            self.done=bool(high-low<=.024 and low>=TOP-.002 and high<=TOP+.030 and clear)
        if self.stage=='withdraw' and age>=9. and 25.<=lean<80. and high-low>.024:
            from release_support import ReleaseSupport
            self.release=ReleaseSupport(self.env,self.previous)
            self.stage='release_support'
            return self.command(time)
        if age>12. or low<TOP-.008:
            self.failure='the measured carton did not tip and settle with clear hands'
        self.previous=np.clip(action,self.env._low,self.env._high)
        return self.previous.copy(),[dict(phase=self.stage,failure=self.failure,
            carton_lean_deg=lean,carton_height_span_m=high-low,ik_errors=errors,complete=self.done)]
