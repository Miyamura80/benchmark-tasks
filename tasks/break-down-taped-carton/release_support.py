"""Withdraw the right support fingertip and hold both arms clear of the packet."""
import numpy as np

from yam_arena import ik
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import grip_rotation

from collapse_geometry import vertices
from config import CUTTER_START, TOP
from hook_flap import CartonFlapHook
from scene import layout
from turnover_safety import Geometry


class ReleaseSupport:
    def __init__(self, env, action):
        self.env = env
        self.read = CartonFlapHook(env, action)
        self.parts = {p['name']: p for p in layout()[0]}
        self.geometry = Geometry()
        self.previous = action.copy()
        self.starts = None
        self.hold_action = None
        self.hold_since = None
        self.last_positions = None
        self.done = False
        self.failure = None

    def command(self, time):
        sides = ('left', 'right')
        hands = [self.read.hand(s) for s in sides]
        frames = [ik.tcp_pose(q, self.env.rig.bases[s]) for s,(q,g) in zip(sides,hands)]
        if self.starts is None:
            self.starts = frames
            self.started = time
        age = time-self.started
        boxes = {n: (*self.read.pose(n), p['size']) for n,p in self.parts.items()}
        positions = {n:p for n,(p,r,_) in boxes.items()}
        rotations = {n:r for n,(p,r,_) in boxes.items()}
        cloud = np.concatenate(list(vertices(self.parts,positions,rotations).values()))
        cutter, cutter_frame = self.read.pose('cutter')
        boxes['parked_cutter'] = (cutter,cutter_frame,(.12,.026,.025))
        tool_clear = np.linalg.norm(cutter-np.array(CUTTER_START)) < .05
        clear = all(np.linalg.norm(p-np.clip(p,cloud.min(0),cloud.max(0))) > .07
                    and np.linalg.norm(p-cutter) > .065 for p,r in frames)
        low, high = cloud[:,2].min()-TOP, cloud[:,2].max()-TOP
        flat = high-low <= .024 and -.002 <= low <= .008 and high <= .030
        coplanar = all(abs((rotations[wall].T@rotations[wall+'_'+end])[2,1])
                       >= np.cos(np.radians(15.)) for wall in sides_of_carton for end in ('top','bottom'))
        current = np.asarray([*positions.values(),cutter])
        speeds = (np.full(len(current),np.inf) if self.last_positions is None else
                  np.linalg.norm(current-self.last_positions,axis=1)/.05)
        self.last_positions = current
        settled = max(speeds[:-1]) < .01 and speeds[-1] < .02
        if age >= .75 and flat and coplanar and clear and tool_clear and self.hold_action is None:
            self.hold_action = self.previous.copy()

        if self.hold_action is None:
            target = self.previous.copy()
            travel = np.clip((age+.05)/1.,0.,1.)
            goals = ((self.starts[0][0]+np.array((0.,0.,.078))*ik.smoothstep(age/3.),grip_rotation()),
                     (self.starts[1][0]+np.array((.022,-.160,-.043))*travel,self.starts[1][1]))
            for i,(s,(q,g),(p,r)) in enumerate(zip(sides,hands,goals)):
                joints,_,_ = ik.solve(p,ik.matrix_to_quat_xyzw(r),q,base_pos=self.env.rig.bases[s],iters=110)
                k=7*i
                target[k:k+6]=np.clip(joints,self.previous[k:k+6]-.022,self.previous[k:k+6]+.022)
        else:
            target = self.hold_action
        for fraction in (1.,.5,.25,.125,0.):
            command = self.previous+fraction*(target-self.previous)
            states=[(command[k:k+6],np.full(2,command[k+6]*FINGER_OPEN)) for k in (0,7)]
            meshes=[self.geometry.parts(s,*state) for s,state in zip(sides,states)]
            guards=[self.geometry.guard(s,*state,boxes,meshes[1-i],intended=i==1 and age<1.2,
                                        intended_targets=('north_bottom',))
                    for i,(s,state) in enumerate(zip(sides,states))]
            if all(g['safe'] and g['target_gap_m']>=-.0015 for g in guards):
                self.previous=command
                break
        else:
            self.failure='no guarded support-release step'
        if self.hold_action is not None and flat and coplanar and clear and tool_clear and settled:
            if self.hold_since is None:self.hold_since=time
            self.done=time-self.hold_since >= 1.25
        else:
            self.hold_since=None
        if age>8. and not self.done:self.failure='support release did not settle flat with clear hands'
        return self.previous.copy(),[dict(phase='hold_flat_packet' if self.hold_action is not None else 'release_support',
              span_m=float(high-low),coplanar=bool(coplanar),hands_clear=bool(clear),settled=bool(settled),
              guards=guards,complete=self.done,failure=self.failure)]


sides_of_carton=('north','south','east','west')
