"""Fold the sleeve by pressing opposing corners before rolling it flat."""
import numpy as np
from yam_arena import ik

from brace_collapse_probe import BracedCollapseProbe
from collapse_geometry import first_cardboard_surface
from scene import layout
from turnover import roll_wrist
from turnover_targets import support_target


class DiagonalShearProbe(BracedCollapseProbe):
    def __init__(self, env, previous):
        super().__init__(env, previous)
        self.shear_first = True
        self.corners = [h for h in layout()[1] if h['name'].startswith('corner_')]
        self.corner_height = {}

    def corner(self, i, poses, height=.036):
        hinge = self.corners[i]
        p,R=poses[hinge['a']]
        up=-poses['north'][1][:,2]
        return p+R@np.asarray(hinge['anchor_a'])+height*up

    def configure_diagonal(self, poses, arms):
        actual=[self.corner(i,poses) for i in range(4)]
        pairs=sorted(((0,2),(1,3)),key=lambda p:abs(actual[p[0]][0]-actual[p[1]][0]))
        alternatives=(((.48,0.,30.),.036),((.08,0.,0.),.008),
                      ((.48,.56,30.),.008),((.08,.40,30.),.008),
                      ((.48,.0,180.),.036),((.48,.30,0.),.036),
                      ((.35,.24,0.),.036),((.50,.24,30.),.036),
                      ((.42,.25,-45.),.036),((.58,.16,0.),.036),((.48,.0,-45.),.036))
        for pair in pairs:
            self.diagonal=sorted(pair,key=lambda i:actual[i][1],reverse=self.side=='left')
            self.postures={}
            for i,(side,arm) in enumerate(zip(self.sides,arms)):
                high=i==self.index
                default=(.35,0.,30.) if high else (.48,0.,0.)
                base=self.env.rig.bases[side]
                for posture,height in ((default,.036),*alternatives):
                    self.postures[high]=posture
                    self.corner_height[high]=height
                    goal,R,_=self.corner_support(poses,high,0.,1.)
                    quat=ik.matrix_to_quat_xyzw(R)
                    hover,p_err,r_err=ik.solve(goal,quat,arm[0],base_pos=base,iters=110)
                    if p_err>.0015 or r_err>.025:continue
                    touch,_,_=self.corner_support(poses,high,0.,.06)
                    _,p_err,r_err=ik.solve(touch,quat,hover,base_pos=base,iters=110)
                    if p_err<.0015 and r_err<.025:break
                else:
                    break
            else:
                return True
        return False

    def diagonal_inward(self, poses, high):
        first,second=self.diagonal if high else self.diagonal[::-1]
        inward=self.corner(second,poses)-self.corner(first,poses)
        inward[2]=0.
        inward/=np.linalg.norm(inward)
        return inward

    def corner_support(self, poses, high, preload, clearance):
        first=self.diagonal[0 if high else 1]
        origin=self.corner(first,poses,self.corner_height[high])
        inward=self.diagonal_inward(poses,high)
        names=[]
        for side in (self.corners[first]['a'],self.corners[first]['b']):
            names.extend((side,side+'_top',side+'_bottom'))
        up=np.array((0.,0.,1.))
        surface=first_cardboard_surface(origin-.15*inward,inward,
                                       [(*poses[n],self.parts[n]['size']) for n in names])
        if surface is None:surface=origin
        tangent=np.cross(up,inward)
        if tangent[0]<0:tangent=-tangent
        magnitude,tangent_gain,roll=self.postures[high]
        z=magnitude*inward+tangent_gain*tangent-np.sqrt(1.-magnitude**2-tangent_gain**2)*up
        x=tangent-z*(z@tangent)
        x/=np.linalg.norm(x)
        wrist=roll_wrist(np.column_stack((x,np.cross(z,x),z)),roll)
        target,R=support_target(surface,inward,wrist,self.tips,preload)
        target+=clearance*(-.038*inward+.026*up)
        return target,R,tuple(names)
