"""Run a 20 Hz joint-command demonstration, action replay, or inactivity control."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from run_provenance import capture_run_source, seal_run_evidence
from tuck import finish_near_fold
from material import Materials
from robot import pose_frame,SIDES
from drive import Driver
from free_motion import FreeMotion
from scene import build,PAD_Y,PAD_Z,GUSSET_Y,GUSSET_Z,DX,DY,TAB_ANGLE


MANIPULATION_LIMIT = 1600


def vec(x):return np.asarray(x,float)

def rotation_x(theta):return Rotation.from_rotvec([theta,0,0]).as_matrix()


def poses(side,p,yaw,opening):return {side:(np.asarray(p),yaw,opening)}


def prepare(trial):
    orientations={'left':pose_frame((1,0,0)),'right':pose_frame((-1,0,0))@np.diag([-1,-1,1])}
    trial.reset({'left':(vec([-.126,PAD_Y,PAD_Z+.032]),orientations['left'],.009),'right':(vec([.106,PAD_Y+.03,PAD_Z+.082]),orientations['right'],.009)})
    return orientations


def demo(trial, mat, R, stop=None):
    d=trial.data
    def motion(label,goals,t):
        trial.move(label,goals,t,strict=False)
        if label.startswith(('Touch ', 'Wrap ', 'Withdraw from ')) and 'wing' in label:
            for side,(goal,_,_) in goals.items():
                error=np.linalg.norm(trial.arms[side].pose(d)[0]-goal)
                if error>.018:
                    raise RuntimeError(f'{label}: actual {side} fingertip missed position by {1000*error:.1f} mm')
    def one(label,s,p,r,opening,t):motion(label,poses(s,p,r,opening),t)
    def target_core(side):
        i=0 if side=='left' else 6
        b=d.body(f'core{i}');off=.008 if side=='left' else DX-.005
        return b.xpos+b.xmat.reshape(3,3)@vec([off,0,0])
    def grasped(side,geom):
        fingers=set()
        for c in d.contact:
            if min(c.geom)<0 or c.dist>0:continue
            names=[trial.model.geom(i).name for i in c.geom]
            if geom not in names:continue
            for finger in ('left','right'):
                if any(name.startswith(f'{side}_tip_{finger}_contact_') for name in names):fingers.add(finger)
        return len(fingers)==2
    motion('Settle on the loading shelf',{},.55)
    # This reference demo uses measured simulator body poses for grasps. Saved
    # joint commands can be replayed open-loop with no task-pose observations.
    p=target_core('left');one('Approach pad nose','left',p+[-.018,0,0],R['left'],.007,.65)
    one('Straddle pad nose','left',target_core('left'),R['left'],.007,.6)
    one('Secure pad nose','left',target_core('left'),R['left'],0,.5)
    if stop=='grip':return
    for key in ['main','near','far']:
        name=mat.spec['liners'][key][-1];b=d.body(name);L=.016
        r=b.xmat.reshape(3,3).copy();tgt=b.xpos+r@[L*.80,0,0]; h=-(r@vec([1,0,0]));n=r@vec([0,0,1]);grip_R=pose_frame(h,n)@np.diag([-1,-1,1])
        # Raise away from the previous paper, then slide along the free edge.
        one(f'Approach {key} liner','right',tgt-h*.028+[0,0,.017],grip_R,.009,1.)
        b=d.body(name);r=b.xmat.reshape(3,3).copy();tgt=b.xpos+r@[L*.80,0,0];one(f'Straddle {key} liner','right',tgt,grip_R,.006,.55)
        one(f'Pinch {key} liner','right',tgt,grip_R,0,.45)
        if not grasped('right',f'{key}_leaf{len(mat.spec["liners"][key])-1}'):
            raise RuntimeError(f'The right jaws did not capture the {key} release tab')
        start=d.site('right_tcp').xpos.copy(); length=.19 if key=='main' else .050
        # The lifted margin is outside the adhesive. The underlying sheet stays
        # load-bearing until measurable normal traction opens each local bond.
        for i in range(1,7):
            u=i/6;offset=vec([-length*u,.032*u,.054*np.sin(np.pi*u/2)+.018*u]);one(f'Peel {key} liner {i}','right',start+offset,grip_R,0,.34)
        one(f'Carry {key} paper away','right',start+[-length*.65,-.095,.106],grip_R,0,.72)
        one(f'Discard {key} liner','right',vec([.040,-.330,.26]),R['right'],0,1.0)
        one(f'Release {key} paper','right',vec([.040,-.330,.26]),R['right'],.018,.55)
        if not trial.readings[-1]['papers_clear'][key]:
            raise RuntimeError(f'The {key} liner remained on the adhesive after release')
        if stop==key:return
    # Pick up the second nose in situ. Both stock grippers carry and turn the core.
    p=target_core('right');r=d.body('core6').xmat.reshape(3,3).copy()@R['right'];one('Approach second nose','right',p+.033*(-r[:,2])+[0,0,.008],r,.016,1.05)
    one('Straddle second nose','right',target_core('right'),r,.014,.70)
    for k,gap in enumerate((.006,.002,0.)):
        one(f'Close second grip {k}','right',target_core('right'),r,gap,.33)
    if not grasped('right','absorbent6'):
        target=target_core('right')+r[:,2]*.004;one('Reopen at second nose','right',target,r,.010,.42);one('Regrip second nose','right',target_core('right')+r[:,2]*.004,r,0,.55)
    trial.move('Hold both noses',{},.25,strict=False)
    if not grasped('left','absorbent0') or not grasped('right','absorbent6'):raise RuntimeError('Bilateral stock finger contact missing on a pad nose before transport')
    start={s:trial.arms[s].pose(d) for s in SIDES};centers=mat.core_xyz(d);core=centers[3];major=centers[-1]-centers[0];major/=np.linalg.norm(major);up=d.body('core3').xmat.reshape(3,3)[:,2];minor=np.cross(up,major);minor/=np.linalg.norm(minor);flat=np.column_stack([major,minor,np.cross(major,minor)]).T
    # Keep the pair of contact patches in their measured positions relative to
    # the same object centre. Rotating the wrists independently would shear a
    # pad that starts at a small yaw angle after the separate release peels.
    def transport(tag,center,transform,t):
        goals={s:(center+transform@(start[s][0]-core),transform@start[s][1],0) for s in SIDES};motion(tag,goals,t)
    height=.120;raised=core+vec([0,0,height]);transport('Lift both ends',raised,np.eye(3),1.35)
    clear=raised+vec([0,.055,0]);transport('Clear and square the pad',clear,flat,1.05)
    for i in range(1,9):transport(f'Turn absorbent face up {i}',clear,rotation_x(-np.pi*i/8)@flat,.36)
    end_orientation=rotation_x(-np.pi)@flat
    desired=vec([0,GUSSET_Y,raised[2]]);transport('Carry above the gusset',desired,end_orientation,1.7)
    if stop=='turn':return
    curr={s:d.site(s+'_tcp').xpos.copy() for s in SIDES};i0=(mat.spec['rows']//2)*mat.spec['cols']+mat.spec['cols']//2;mid=d.flexvert_xpos[i0].copy();center=mat.core_xyz(d)[3];aim=mid-center+[0,.002,.007]
    motion('Lower pad into the briefs',{s:(curr[s]+aim,trial.arms[s].pose(d)[1],0) for s in SIDES},1.4)
    for attempt in range(2):
        cls=mat.core_xyz(d)[3];surface=d.flexvert_xpos[i0]
        delta=surface-cls+vec([0,.002,.001])
        if delta[2]>-.002:break
        delta=np.clip(delta,-.025,.025)
        motion('Seat center '+str(attempt),{s:(trial.arms[s].pose(d)[0]+delta,trial.arms[s].pose(d)[1],0) for s in SIDES},.65)
    motion('Open both nose grips',{s:(*trial.arms[s].pose(d),.020) for s in SIDES},.65)
    motion('Withdraw open jaws axially',{s:(trial.arms[s].pose(d)[0]+vec([-.060 if s=='left' else .060,0,.002]),trial.arms[s].pose(d)[1],.020) for s in SIDES},.85)
    motion('Lift both empty hands',{s:(trial.arms[s].pose(d)[0]+vec([0,0,.078]),trial.arms[s].pose(d)[1],.012) for s in SIDES},.85)
    if stop=='place':return
    # The hands turn in free space, then lower closed fingers onto the absorbent top.
    for index in (3,):
        side='left' if index<=3 else 'right';h=1 if side=='left' else -1
        r=pose_frame((h*1.15,0,-1),(0,1,0))@(np.diag([-1,-1,1]) if side=='right' else np.eye(3))
        q=mat.core_xyz(d)[index]
        trial.move('Orient above pad '+str(index),poses(side,q+[-.016*h,0,.073],r,0),1.05,strict=False,joint_travel=True)
        one('Approach top pressure '+str(index),side,q+[-.007*h,0,.020],r,0,.60)
        q=mat.core_xyz(d)[index];one('Smooth pad '+str(index),side,q+[-.004*h,0,-.004],r,0,.65)
        one('Release top pressure '+str(index),side,q+[-.013*h,0,.065],r,0,.65)
    if stop=='press':return
    # The two rollouts use opposite arms, with the approaching fingertip on the
    # outer, uncoated surface. Its approach tilts toward horizontal at the hem.
    def clear_route(side, name, target, seeds):
        arm=trial.arms[side]
        opening=float(trial.last[side][-1])
        initial=d.qpos[arm.positions].copy()
        for attempt in range(4):
            plan=FreeMotion(trial.model,d,arm)
            separated=plan.separate(initial)
            if separated is None:break
            waypoint=arm.forward(separated)
            trial.move(f'Clear residual contact before {name} {attempt+1}',poses(side,*waypoint,opening),.20,
                       joint_targets={side:separated},guard=plan.route_contact)
            initial=d.qpos[arm.positions].copy()
        for seed in [*seeds,initial,None]:
            goal,error=arm.solve(target[0],target[1],seed)
            if error<.004 and plan.clear(goal):
                break
        else:
            raise RuntimeError(f'No collision-free, reachable posture for {name}')
        route=plan.path(initial,goal)
        for j,(begin,finish) in enumerate(zip(route,route[1:])):
            duration=max(.45,float(np.max(abs(finish-begin)))/.90)
            waypoint=arm.forward(finish)
            trial.move(f'{name} {j+1}',poses(side,*waypoint,opening),duration,joint_targets={side:finish},guard=plan.route_contact)
            if np.max(abs(d.qpos[arm.positions]-finish))>.12:
                raise RuntimeError(f'{side} hand could not track its free-motion route')

    for key,side in [('far','right'),('near','left')]:
        support='left' if side=='right' else 'right'
        opp=1 if support=='left' else -1
        srot=pose_frame((opp*1.15,0,-1),(0,1,0))@(np.diag([-1,-1,1]) if support=='right' else np.eye(3))
        q=mat.core_xyz(d)[3 if key=='far' else 4].copy()
        if key=='near':
            core=mat.core_xyz(d)[5].copy()
            center=mat.core_xyz(d)[3].copy()
            hem_y=d.geom('far_glue').xpos[1]
            x=.018
            cloth=mat.textile(d.flexvert_xpos,np.array([x,hem_y+.008]))
            if cloth is None:raise RuntimeError('Cannot locate the far hem for the holding grip')
            z=(cloth[0]+core[2])*.5
            pinch=vec([x,hem_y,z])
            heading=vec([0,np.cos(np.radians(4)),-np.sin(np.radians(4))]);srot=pose_frame(heading)@np.diag([-1,-1,1])
            clear_route(support,'Place stock jaws outside far hem',(pinch-.040*heading,srot,0),[[-.345,1.610,.934,.471,1.219,-1.379]])
            one('Open stock jaws around far hem',support,pinch-.040*heading,srot,.013,.45)
            one('Straddle gusset and pad',support,pinch,srot,.013,.72)
            one('Pinch gusset and pad together',support,pinch,srot,0,.50)
            if stop=='hem':return
        else:
            trial.move('Position opposite hand for '+key,poses(support,q+[-.012*opp,0,.057],srot,0),1.1,strict=False,joint_travel=True)
            one('Support pad during '+key+' fold',support,q+[-.004*opp,0,-.002],srot,0,.75)
        root=d.body(key+'_wing0').xpos.copy();mid=d.flexvert_xpos[i0];sign=1 if root[1]>mid[1] else -1
        initial_normal=d.geom(f'{key}_sheet2').xmat.reshape(3,3)[:,2];theta=np.arctan2(abs(initial_normal[1]),max(.001,-initial_normal[2]));horiz=-1 if side=='right' else 1
        # The limb remains outside the cloth at the beginning and below it for the final press.
        def aim(a,clear):
            n=vec([0,sign*np.sin(a),np.cos(a)]);rad=vec([0,sign*np.cos(a),-np.sin(a)])
            # Keep the palm outside the gusset. The finger axis becomes level
            # before it enters beneath the fabric; the jaws stay closed.
            pitch=np.radians(np.interp(np.degrees(a),[85,120],[-20.,0.]))
            roll=np.radians(np.interp(np.degrees(a),[105,145],[-90.,0.]))
            heading_angle=np.pi/2
            if side=='left':
                heading_angle=np.radians(np.interp(np.degrees(a),[50,88],[50.,70.]))
                pitch=np.radians(np.interp(np.degrees(a),[45,85],[-40.,-20.]))
                roll=np.pi
            head=vec([horiz*np.cos(pitch)*np.cos(heading_angle),-sign*np.cos(pitch)*np.sin(heading_angle),np.sin(pitch)]) if side=='left' else vec([0,-sign*np.cos(pitch),np.sin(pitch)])
            orient=pose_frame(head)@Rotation.from_rotvec([0,0,roll]).as_matrix()
            root=d.body(key+'_wing0').xpos
            p=root+vec([-.004 if side=='left' else .004,0,0])+.028*rad+clear*n-.009*head
            return p,orient,0.
        arm=trial.arms[side]
        one('Close empty '+side+' fingers',side,*arm.pose(d),0,.45)
        entry_seed=[-.36,1.64,1.13,-.29,1.07,.73] if side=='right' else [.21,1.43,1.34,-1.03,.67,-.66]
        clear_route(side,'Clear fixture for '+key,aim(theta,.035),[entry_seed])
        motion('Approach '+key+' wing',{side:aim(theta,.024)},.8);motion('Touch '+key+' wing',{side:aim(theta,.009)},.65)
        for k in range(12 if key=='near' else 22):
            a=np.interp(k,[0,21],[theta,np.pi]);motion(f'Wrap {key} wing {k+1}',{side:aim(a,-.003)},.25)
            if key=='far' and k>=18:
                loads=trial.readings[-1]['loaded_jaws_n']
                distal=sum(items.get('far_sheet2',0.) for jaw,items in loads.items() if jaw.startswith(side+'_tip_'))
                if distal>=.45:break
            if key=='near' and k>=16:
                sensor=trial.readings[-1]['loaded_jaws_n']
                distal=sum(items.get('near_sheet2',0.) for jaw,items in sensor.items() if jaw.startswith(side+'_tip_'))
                film=d.geom('near_glue').xpos
                cloth=mat.textile(d.flexvert_xpos,film[:2])
                if cloth is not None and film[2]-cloth[0]>=-.0043 and distal>=.35:break
        if key=='near':
            finish_near_fold(trial, mat)
            return
        p,r=arm.pose(d)
        t,sr=trial.arms[support].pose(d)
        motion('Unload both faces of '+key,{side:(p+[0,0,-.008],r,0),support:(t+[0,0,.011],sr,0)},.25)
        p,r=arm.pose(d)
        exit_head=vec([0,-sign*np.cos(.21),-np.sin(.21)])
        motion('Withdraw from far wing',poses(side,p+[.010,sign*.009,-.011],pose_frame(exit_head),0),.55)
        q=mat.core_xyz(d)[3]
        one('Remove counterpressure '+key,support,q+[-.014*opp,0,.065],srot,0,.65)
        if stop=='far-wrap':return


def inspect(trial,mat):
    schedule=[('Lift garment 25 mm',.025,0.,1,1.),('Turn the briefs upside down',.025,np.pi,2,2.5),('Hold fully inverted',.025,np.pi,3,2.5),('Return the briefs for judging',.025,0.,4,2.5),('Inspect the released fit',.025,0.,4,.70)]
    for label,lift,angle,phase,duration in schedule:
        begin=len(trial.commands);initial=np.array([float(trial.data.actuator('frame_height').ctrl[0]),float(trial.data.actuator('frame_angle').ctrl[0])]);end=np.array([lift,angle]);count=round(20*duration)
        for k in range(count):
            t=(k+1)/count;w=t*t*(3-2*t);setting=(1-w)*initial+w*end;trial.action([trial.last[s] for s in SIDES],[*setting,phase])
        row={'stage':label,'begin_action':begin,'end_action':len(trial.commands),'state':trial.readings[-1]};trial.stages.append(row)
        with (trial.output/'stages.jsonl').open('a') as stream:stream.write(json.dumps(row)+'\n')



def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True,type=Path);p.add_argument('--mode',choices=['script','idle','replay','dream','dry-dream'],default='script');p.add_argument('--replay',type=Path);p.add_argument('--stop-after');p.add_argument('--restart-dir',type=Path);p.add_argument('--restart-through');p.add_argument('--offset-mm',type=float,default=0);p.add_argument('--no-tack',action='store_true');p.add_argument('--no-wing-tack',action='store_true');args=p.parse_args()
    m,spec=build(args.offset_mm)
    config={'mode':args.mode,'offset_mm':args.offset_mm,'no_tack':args.no_tack,
            'no_wing_tack':args.no_wing_tack,'stop_after':args.stop_after,
            'restored_prefix_requested':args.restart_dir is not None}
    capture_run_source(args.out,m,config)
    (args.out/'stages.jsonl').write_text('')
    mat=Materials(m,spec,args.no_tack,args.no_wing_tack)
    trial=Driver(m,mat,args.out)
    extra={}
    completed=False
    failure_type=None
    try:
        R=prepare(trial)
        if args.restart_dir:trial.reuse_prefix(args.restart_dir,args.restart_through)
        if args.mode=='script':demo(trial,mat,R,args.stop_after)
        elif args.mode=='idle':trial.move('No robot action',{},8.,strict=False)
        elif args.mode=='replay':
            record=np.load(args.replay,allow_pickle=False);rows=record['commands'];fixtures=record['fixture']
            if len(rows)!=len(fixtures) or len(record['qpos'])!=len(rows)+1:raise ValueError('Incomplete replay')
            for row,fixture in zip(rows,fixtures,strict=True):trial.action(row,fixture)
            extra={'max_qpos_replay_error':float(np.max(abs(np.asarray(trial.states)-record['qpos']))),'replayed_all_controls':len(rows)}
        else:
            from policy_adapter import run
            extra=run(trial,'Peel the release papers off the period pad, center it sticky side down in the briefs and wrap both wings underneath.',MANIPULATION_LIMIT,dry_run=args.mode=='dry-dream')
        if not args.stop_after and args.mode not in ('replay','dry-dream'):
            if len(trial.commands)>MANIPULATION_LIMIT:
                raise RuntimeError(f'Manipulation exceeded the common {MANIPULATION_LIMIT}-action limit')
            inspect(trial,mat)
        completed=True
    except BaseException as error:
        failure_type=type(error).__name__
        raise
    finally:
        trial.save()
        result={'mode':args.mode,'completed':completed,'failure_type':failure_type,
                'completed_actions':len(trial.commands),'ended_after':args.stop_after,
                'result':trial.readings[-1] if trial.readings else None,
                'release_events':mat.events,'adhesion_events':mat.adh.events,**extra}
        (args.out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        seal_run_evidence(args.out,completed,failure_type)
        print('FINAL',json.dumps(result),flush=True)
if __name__=='__main__':main()
