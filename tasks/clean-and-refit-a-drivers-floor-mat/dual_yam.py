"""Native dual-YAM proof of concept for the driver's mat cleaning fixture."""
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from demo import BONDED_GRIT, LOOSE_GRIT, TOTAL_GRIT, brush_releases, scene
from yam_station import Station, station_xml

SCENE_OFFSET = np.array([-.265, 0., .145])


def vector(values):
    return ' '.join(f'{value:.8g}' for value in values)


def native_model():
    tree, fixture = station_xml()
    world = tree.find('worldbody')
    for side,y in (('left',.206),('right',-.206)):
        world.find(f'body[@name="{side}_base"]').set('pos',vector((-.17,y,.175)))
    for i,y in ((-1,-.206),(1,.206)):
        world.find(f'geom[@name="pedestal_{i}"]').set('pos',vector((-.17,y,.077)))
    task = ET.fromstring(scene())
    task_world = task.find('worldbody')
    for node in task_world:
        if node.tag=='light' or node.get('name')=='gantry':
            continue
        offset = np.fromstring(node.get('pos','0 0 0'),sep=' ')
        node.set('pos',vector(offset+SCENE_OFFSET))
        if node.tag=='body' and node.find('./freejoint') is not None:
            world.append(node)
        elif node.tag=='body' and node.find('./joint[@type="free"]') is not None:
            world.append(node)
        else:
            fixture.append(node)
    equality=ET.SubElement(tree,'equality')
    for bond in task.find('equality'):
        equality.append(bond)
    ET.SubElement(equality,'weld',name='gripper_hold',body1='right_grasp_site',
                  body2='mat',active='false',solref='.012 1',torquescale='.078')
    wrist = task_world.find('body[@name="gantry"]/body[@name="wrist"]')
    for name, side in (('brush','left'),('vacuum','right')):
        tool=wrist.find(f'body[@name="{name}"]')
        tool.remove(tool.find('joint'))
        tool.set('pos', '0 0 0.006')
        tool.set('quat','0 0 1 0' if name=='brush' else '0 1 0 0')
        mount=world.find(f'.//body[@name="{side}_grasp_site"]')
        mount.append(tool)
        offset_x=-.061 if name=='brush' else -.073
        ET.SubElement(mount,'geom', name=f'{name}_mount',type='box',
                      pos=f'{offset_x/2} 0 -.018',size=f'{abs(offset_x)/2} .010 .010',
                      rgba='.23 .26 .28 1',mass='.01',contype='0',conaffinity='0')
    tree.find('option').set('integrator','implicitfast')
    tree.find('option').set('iterations','45')
    return mj.MjModel.from_xml_string(ET.tostring(tree,encoding='unicode'))


class FloorMat:
    title='Clean and refit the driver’s floor mat'

    def __init__(self):
        self.powered=False
        self.vacuum_control=True

    def compile(self):
        return native_model()

    def reset(self,s):
        m = s.model
        self.mat_id=m.body('mat').id
        self.sockets=[m.site(f'socket{i}').id for i in (-1,1)]
        self.posts=np.array([[.339,-.102,.036],[.339,.102,.036]]) + SCENE_OFFSET
        self.latched=np.ones(2,dtype=bool)
        self.released=np.zeros(2,dtype=bool)
        self.bonds={m.geom(f'crust{i}').id:m.equality(f'bond{i}').id
                    for i in range(LOOSE_GRIT,TOTAL_GRIT)}
        self.grit=np.array([m.body(f'grit{i}').id for i in range(TOTAL_GRIT)])
        self.grit_geoms=[m.geom(f'particle{i}' if i<LOOSE_GRIT else f'crust{i}').id
                         for i in range(TOTAL_GRIT)]
        self.grit_dofs=m.jnt_dofadr[m.body_jntadr[self.grit]]
        self.bristles={m.geom(f'bristle{i}').id for i in range(7)}
        self.releases=[]
        self.motor_indicator=m.geom('vac_switch').id
        self.jaw_sensor=m.joint('right_finger_1').qposadr[0]
        if not self.vacuum_control:
            # Start below the motor window as well as staying below it on every pass.
            s.targets[1,-1]=.55
            s.set_targets(s.targets)
            for name in s.fingers['right']:
                s.data.joint(name).qpos[0]=s.data.actuator(name).ctrl[0]

        self.arm_vel=np.zeros(6)
        self.vac_intake=m.site('vac_intake').id
        self.vac_chamber=m.site('vac_chamber').id
        self.entered=np.zeros(TOTAL_GRIT,dtype=bool)
        self.loaded=np.zeros(TOTAL_GRIT,dtype=bool)
        self.lip=m.geom('pull_lip').id
        self.loaded_sides=set()
        self.hold_active=m.equality('gripper_hold').id
        self.hand_body=m.body('right_grasp_site').id

    def before_step(self,s):
        d,m=s.data,s.model
        d.qfrc_applied[:]=0.
        self.brush(s)
        # Jaw travel closes a magnet-and-reed motor interlock on the tool mount.
        self.powered=bool(.031 < d.qpos[self.jaw_sensor] < .037)
        m.geom_rgba[self.motor_indicator]=(.28,.93,.42,1.) if self.powered else (.92,.42,.16,1.)
        self.vacuum(s)
        for i,(point,post) in enumerate(zip(d.site_xpos[self.sockets],self.posts)):
            error=point-post
            if self.latched[i] and (error[2]>.023 or np.linalg.norm(error[:2])>.021):
                self.latched[i]=False
                self.released[i]=True
            if not self.latched[i] and abs(error[2])<.005 and np.linalg.norm(error[:2])<.0065:
                self.latched[i]=True
            if self.latched[i]:
                force=np.clip(-780*error,-8.,8.)
                mj.mj_applyFT(m,d,force,np.zeros(3),point,self.mat_id,d.qfrc_applied)
        self.grip(s)
        self.last_time=d.time

    def brush(self,s):
        if any(s.data.eq_active[bond] for bond in self.bonds.values()):
            self.releases.extend(brush_releases(s.model, s.data, self.bristles,
                                                self.bonds, self.mat_id))

    def grip(self,s):
        d,m=s.data,s.model
        side='right'
        opening=float(d.actuator(f'{side}_finger_-1').ctrl[0])
        if opening>.020:
            self.loaded_sides.discard(side)
            d.eq_active[self.hold_active]=False
            return
        pads=set()
        for c in d.contact:
            if self.lip not in (c.geom1,c.geom2) or c.dist>.001:
                continue
            finger=m.geom(c.geom1 if c.geom2==self.lip else c.geom2)
            if finger.name.startswith(f'{side}_tip_'):
                pads.add(int(finger.bodyid[0]))
        if len(pads)==2 and opening<.013 and side not in self.loaded_sides:
            self.loaded_sides.add(side)
            basis=d.xmat[self.hand_body].reshape(3,3).T
            relative_pos=basis@(d.xpos[self.mat_id]-d.xpos[self.hand_body])
            relative_quat=np.empty(4)
            mj.mju_mat2Quat(relative_quat,(basis@d.xmat[self.mat_id].reshape(3,3)).ravel())
            m.eq_data[self.hold_active,3:6]=relative_pos
            m.eq_data[self.hold_active,6:10]=relative_quat
            d.eq_active[self.hold_active]=True


    def vacuum(self,s):
        d,m=s.data,s.model
        if not self.powered and not np.any(self.entered):
            return
        intake=d.site_xpos[self.vac_intake]
        chamber=d.site_xpos[self.vac_chamber]
        rotation=d.site_xmat[self.vac_intake].reshape(3,3)
        offset=d.xpos[self.grit]-intake
        local=offset@rotation
        distance=np.linalg.norm(offset,axis=1)
        mj.mj_objectVelocity(m,d,mj.mjtObj.mjOBJ_SITE,self.vac_intake,self.arm_vel,0)
        v=np.stack([d.qvel[x:x+3] for x in self.grit_dofs])-self.arm_vel[3:]
        loose=np.array([not d.eq_active[self.bonds[g]] if g in self.bonds else True
                        for g in self.grit_geoms])
        active=(distance<.044)&(local[:,2]<.013)&(local[:,2]>-.041)&loose&~self.entered&self.powered
        throat=active&(np.linalg.norm(local[:,:2],axis=1)<.013)&(local[:,2]>-.004)
        self.entered|=throat
        for i in np.flatnonzero(throat):
            m.geom_contype[self.grit_geoms[i]]=4
            m.geom_conaffinity[self.grit_geoms[i]]=0
        active&=~throat
        d.xfrc_applied[self.grit,:3]=0
        if np.any(active):
            falloff=(1-distance[active]/.044)**2
            facing=np.maximum(-local[active,2]/np.maximum(distance[active],.001),.18)
            direction=-offset[active]/np.maximum(distance[active,None],.001)
            d.xfrc_applied[self.grit[active],:3]=.125*(falloff*facing)[:,None]*direction-.006*v[active]
        carried=np.flatnonzero(self.entered)
        if len(carried):
            angles=carried*2.399963
            spread=.015*np.sqrt(carried/TOTAL_GRIT)
            packing=np.column_stack((spread*np.cos(angles),spread*np.sin(angles),-.010+.004*(carried%6)))
            error=chamber+packing@rotation.T-d.xpos[self.grit[carried]]
            acceleration=np.clip(950*error-50*v[carried]+[0,0,9.81],-120,120)
            d.xfrc_applied[self.grit[carried],:3]=.0008*acceleration
            self.loaded[carried]|=np.linalg.norm(error,axis=1)<.021

    def after_step(self,s):
        if s.data.time <= self.last_time:
            raise RuntimeError('MuJoCo reset during native replay')

    def measure(self,s):
        d=s.data
        dust=d.xpos[self.grit]-SCENE_OFFSET
        tray=(np.abs(dust[:,0]-.30)<.282)&(np.abs(dust[:,1]+.432)<.192)&(dust[:,2]<.078)
        brushed=sum(not d.eq_active[i] for i in self.bonds.values())
        collected=int(np.sum(tray|self.loaded))
        pitch=math.atan2(-d.xmat[self.mat_id,6],d.xmat[self.mat_id,0])
        flat=abs(pitch)<.085 and abs(d.joint('curl').qpos[0])<.12 and abs(d.xpos[self.mat_id,2]-SCENE_OFFSET[2]-.044)<.012
        parked=bool(np.all(self.latched) and flat and not self.loaded_sides)
        score=(.15*bool(np.all(self.released))+.25*brushed/BONDED_GRIT+.40*min(collected/44,1.)
               +.20*bool(parked and np.all(self.released) and collected>=44 and brushed==BONDED_GRIT))
        return dict(brushed=int(brushed), vacuumed=int(np.sum(self.loaded)), tray=int(np.sum(tray)),
                    vacuum_on=self.powered,
                    bonded_in_vacuum=int(self.loaded[LOOSE_GRIT:].sum()),
                    bonded_in_tray=int((tray & ~self.loaded)[LOOSE_GRIT:].sum()),
                    clips_closed=int(np.sum(self.latched)), clips_released=int(np.sum(self.released)),
                    mat_grasped=bool(self.loaded_sides), flat=bool(flat), score=round(score,3),
                    mat_yaw=round(math.degrees(math.atan2(d.xmat[self.mat_id,3],d.xmat[self.mat_id,0])),1),
                    wrist_yaw=round(math.degrees(math.atan2(-d.site_xmat[s.sites['right'],3],-d.site_xmat[s.sites['right'],0])),1),
                    socket_error_mm=np.round((d.site_xpos[self.sockets]-self.posts)*1000,1).tolist())

    def go(self,s,side,x,y,z,phase,seconds=.75,opening=.72):
        s.yaw_choices[side]=(0.,math.pi)
        if side=='right' and not self.vacuum_control:
            opening=min(opening,.55)
        s.move_arm(side,np.array([x,y,z])+SCENE_OFFSET,seconds,phase,opening=opening,yaw=0.)

    def tilt(self,s,point,start_pitch,end_pitch,seconds,phase):
        side='right'
        start=s.data.site_xpos[s.sites[side]].copy()
        endpoint=np.array(point)+SCENE_OFFSET
        targets=s.targets.copy()
        count=round(20*seconds)
        s.phase=phase
        for i in range(count):
            t=(i+1)/count
            smooth=t*t*(3-2*t)
            pitch=start_pitch+(end_pitch-start_pitch)*smooth
            c,z=math.cos(pitch),math.sin(pitch)
            rotate=np.array([[c,0,z],[0,1,0],[-z,0,c]])@np.diag([-1,1,-1])
            pos=start+smooth*(endpoint-start)
            q,error,angle=s.solve(side,pos,initial=targets[1,:6],rotation=rotate)
            if i+1==count and (error>.016 or angle>.32):
                raise RuntimeError(f'Tray tilt IK residual {error:.4f} m / {angle:.3f} rad')
            targets[1,:6]=q
            s.action(targets)
            if not self.loaded_sides:
                raise RuntimeError('Mat left the right gripper during the tray tilt')

    def park_left(self,s):
        """Fold the brush arm outward above the bench while the vacuum crosses."""
        s.phase='Swing the left brush arm clear of the footwell'
        start=s.targets.copy()
        park=start.copy()
        park[0]=[1.57,1.,1.,-.6,0.,0.,.8]
        for t in np.linspace(0.,1.,23)[1:]:
            s.action(start+t*t*(3-2*t)*(park-start))

    def reset_right_for_grasp(self,s):
        s.phase='Return the right YAM to its known open-gripper approach'
        start=s.targets.copy()
        home=start.copy()
        home[1]=[0.,1.,1.,-.6,0.,0.,.8]
        for t in np.linspace(0.,1.,20)[1:]:
            s.action(start+t*t*(3-2*t)*(home-start))
        s.wait(.14)
        self.go(s,'right',.302,-.150,.140,'Approach the mat from the entry waypoint',.80)

    def vacuum_remaining(self,s):
        """Use measured crumb centers to guide a second, close nozzle pass."""
        d=s.data
        self.attempts=np.zeros(TOTAL_GRIT,dtype=np.int8)
        for _ in range(28):
            points=d.xpos[self.grit]-SCENE_OFFSET
            targetable=(~self.loaded&(self.attempts<2)&(points[:,2]<.080)
                        &(points[:,2]>.028)&(points[:,0]>.194)&(points[:,0]<.682)
                        &(abs(points[:,1])<.155))
            if not np.any(targetable) or np.sum(self.loaded)>=46 or s.steps>=736:
                break
            choices=np.flatnonzero(targetable)
            nearest=choices[np.argmin(np.linalg.norm(
                points[choices]+SCENE_OFFSET-d.site_xpos[self.vac_intake],axis=1))]
            self.attempts[nearest]+=1
            xyz=points[nearest]
            goal=xyz+[-.073,0,.025]
            angle=math.pi
            q,error,_=s.solve('right',goal+SCENE_OFFSET,angle)
            if error>.011:
                continue
            self.go(s,'right',*goal,'Vacuum an individual pocket of trapped grit',.37)
            s.wait(.13)
            if not self.loaded[nearest]:
                delta=xyz+SCENE_OFFSET+[0,0,.010]-d.site_xpos[self.vac_intake]
                next_goal=goal+np.clip(delta,-.019,.019)
                if np.linalg.norm(delta)<.062:
                    self.go(s,'right',*next_goal,'Center the nozzle on the remaining grit',.29)
                    s.wait(.13)

    def final_nozzle_pass(self,s):
        """Collect a visible missed crumb after the mat is locked down."""
        reading=self.measure(s)
        if reading['score']==1.0 or reading['clips_closed']!=2 or s.steps>1167:
            return
        points=s.data.xpos[self.grit]-SCENE_OFFSET
        nearby=(~self.loaded&(points[:,2]>.034)&(points[:,2]<.067)&
                (points[:,0]>.270)&(points[:,0]<.460)&(points[:,1]<.111)&
                (points[:,1]>-.124))
        if not np.any(nearby):
            return
        targets=points[nearby]
        first=targets[np.argmin(np.linalg.norm(targets-[.31,-.09,.05],axis=1))]
        patch=targets[np.linalg.norm(targets-first,axis=1)<.025].mean(axis=0)
        x,y,z=patch+[-.073,0.,.026]
        self.go(s,'right',x,y,max(z+.026,.118),'Return to a missed spot with the vacuum',.49,.75)
        self.go(s,'right',x,y,z,'Vacuum the last reachable patch',.40,.75)
        s.wait(.24)
        self.go(s,'right',x,y,z+.047,'Lift the vacuum from the clipped mat',.34,.75)

    def run(self,s,clean_only=False,base_path=False,refit_only=False,side_entry=False):
        if not refit_only:
            self.go(s,'right',.290,-.300,.120,'Park right YAM while the left arm brushes',1.4)
            self.go(s,'left',.423,.083,.138,'Left YAM approaches the floor mat',1.5)
            for y in (.083, 0., -.083):
                self.go(s,'left',.423,y,.106,'Position brush above the next rib')
                self.go(s,'left',.423,y,.066,'Lower the bristles to the mat',.5)
                self.go(s,'left',.258,y,.066,'Left YAM brushes dry dirt loose',1.25)
                self.go(s,'left',.258,y,.106,'Lift the brush before shifting rows',.5)
            self.go(s,'left',.290,.300,.120,'Park left YAM clear of the vacuum',.9)
            self.park_left(s)
            self.go(s,'right',.245,-.128,.135,'Right YAM approaches with the vacuum',1.5)
            for row,y in enumerate(np.linspace(-.128,.128,8)):
                start,end=(.230,.435) if row%2==0 else (.435,.230)
                self.go(s,'right',start,y,.081,'Move nozzle to the next rib',.45)
                self.go(s,'right',end,y,.081,'Right YAM vacuums the loosened grit',1.10)
            if not base_path and not side_entry:
                self.go(s,'right',.124,.107,.095,'Approach brushed grit at the footwell heel',.46)
                self.go(s,'right',.124,.107,.066,'Lower the nozzle at the heel',.35)
                self.go(s,'right',.124,-.110,.066,'Vacuum loosened grit along the heel',1.40)
                self.go(s,'right',.152,-.110,.068,'Shift across to the mat edge',.32)
                self.go(s,'right',.152,.100,.068,'Vacuum the gap in front of the pull lip',1.35)
                self.vacuum_remaining(s)
            elif not base_path:
                self.go(s,'right',.234,.120,.076,'Line up a slower edge sweep',.55)
                self.go(s,'right',.449,.120,.076,'Vacuum the tunnel-side rim',1.55)
                self.go(s,'right',.449,-.124,.076,'Vacuum along the forward ribs',1.65)
                self.go(s,'right',.234,-.124,.076,'Vacuum the sill-side rim',1.55)
                self.go(s,'right',.234,.099,.076,'Vacuum the pull-lip edge',1.5)
                self.go(s,'right',.358,.099,.076,'Move back over the center ribs',.55)
                self.go(s,'right',.358,-.102,.076,'Collect remaining grit in the center ribs',1.55)
            self.go(s,'right',.302,-.150,.140,'Raise the vacuum',.9)
        else:
            self.go(s,'left',.290,.300,.120,'Park left YAM clear of the mat',1.4)
            self.go(s,'right',.302,-.150,.140,'Approach the mat',.9)
        if clean_only:
            return
        if not refit_only and not base_path:
            self.reset_right_for_grasp(s)
        self.go(s,'right',.285,0,.058,'Approach the rubber pull lip',1.05)
        self.go(s,'right',.285,0,.058,'Close both fingers on the pull lip',.65,0.)
        self.go(s,'right',.215,0,.087,'Unclip and slide the toe out from under the pedals',1.30,0.)
        if not self.loaded_sides:
            raise RuntimeError('Both right gripper pads could not retain the rubber lip')
        self.tilt(s,(.222,0.,.116),0.,-.56,1.15,'Lift the freed toe clear of the footwell')
        self.tilt(s,(.265,-.190,.158),-.56,-.53,1.4,'Bring the angled mat over the door sill')
        self.tilt(s,(.320,-.380,.204),-.53,-.65,1.42,'Carry the tilted mat toward the tray')
        self.tilt(s,(.320,-.380,.204),-.65,-1.65,1.30,'Tip the mat and empty remaining grit')
        s.wait(.68,'Hold the mat over the tray')
        self.tilt(s,(.314,-.380,.184),-1.65,-.58,1.32,'Return to the carry angle')
        self.tilt(s,(.239,-.142,.147),-.58,-.56,1.35,'Return over the door sill')
        self.tilt(s,(.214,0.,.108),-.56,-.41,1.21,'Line up the mat above the inside floor')
        self.tilt(s,(.211,0.,.088),-.41,0.,1.15,'Level the mat just above the pedals')
        x,y,z=.285,0.,.088
        self.go(s,'right',x,y,z,'Advance the mat above both posts',1.0,0.)
        for _ in range(3):
            miss=np.mean(self.posts-s.data.site_xpos[self.sockets],axis=0)
            if np.linalg.norm(miss[:2])<.003:
                break
            x,y=np.array([x,y])+np.clip(miss[:2],-.023,.023)
            self.go(s,'right',x,y,z,'Center both sockets over the posts',.44,0.)
        self.go(s,'right',x,y,.054,'Lower both sockets onto their posts',.95,0.)
        s.wait(.24,'Seat both floor-mat clips')
        for _ in range(2):
            if np.all(self.latched) or not self.loaded_sides:
                break
            miss=np.mean(self.posts-s.data.site_xpos[self.sockets],axis=0)
            x,y=np.array([x,y])+np.clip(miss[:2],-.017,.017)
            self.go(s,'right',x,y,.054,'Square the mat over both posts',.46,0.)
            s.wait(.12)
        self.go(s,'right',x,y,.054,'Open both gripper fingers',.48,.8)
        self.go(s,'right',x,y,.148,'Check the mat sits flat',.85,.8)
        if not refit_only and not base_path and not side_entry:
            self.final_nozzle_pass(s)


class Recorder:
    def __init__(self,path):
        self.path=path
        self.font=ImageFont.truetype('DejaVuSans.ttf',16)
        self.heading=ImageFont.truetype('DejaVuSans-Bold.ttf',18)

    def attach(self,trial):
        self.trial=trial
        self.renderer=mj.Renderer(trial.model,height=440,width=800)
        self.cam=mj.MjvCamera()
        self.cam.lookat[:]=[.022,-.086,.260]
        self.cam.distance,self.cam.azimuth,self.cam.elevation=1.29,142,-50
        self.proc=subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','800x520',
            '-r','10','-i','-','-c:v','libx264','-threads','2','-preset','veryfast','-crf','24','-pix_fmt',
            'yuv420p','-movflags','+faststart',str(self.path)],stdin=subprocess.PIPE)

    def draw(self):
        self.renderer.update_scene(self.trial.data,self.cam)
        self.renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW]=False
        canvas=Image.new('RGB',(800,520),'#152124')
        canvas.paste(Image.fromarray(self.renderer.render()),(0,40))
        ui=ImageDraw.Draw(canvas)
        ui.text((13,9),'Driver’s floor mat | Dual YAM replay',font=self.heading,fill='#eaf1ed')
        ui.text((13,489),self.trial.phase,font=self.font,fill='#eaf1ed')
        state=self.trial.fixture.measure(self.trial)
        motor='ON' if state['vacuum_on'] else 'OFF'
        ui.text((415,489),f"Motor {motor} · Brush {state['brushed']} · Vac {state['vacuumed']} · Clips {state['clips_closed']}/2",
                font=self.font,fill='#ace9d6')
        self.proc.stdin.write(canvas.tobytes())

    def finish(self):
        self.proc.stdin.close()
        if self.proc.wait():
            raise RuntimeError('Video encoding failed')
        self.renderer.close()


def main():
    path=Path(sys.argv[1])
    path.mkdir(parents=True,exist_ok=True)
    fixture=FloorMat()
    fixture.vacuum_control='--vacuum-off' not in sys.argv
    render=Recorder(path/'floor-mat-yam.mp4') if '--video' in sys.argv else None
    trial=Station(fixture,render)
    error=None
    try:
        fixture.run(trial,'--clean-only' in sys.argv,'--base-path' in sys.argv,'--refit-only' in sys.argv,
                    '--side-entry' in sys.argv)
    except RuntimeError as exc:
        error=str(exc)
    finally:
        if render:
            render.finish()
    points=trial.data.xpos[fixture.grit]-SCENE_OFFSET
    remaining=points[~fixture.loaded]
    result={'final':trial.reading(),'error':error, 'bond_releases':fixture.releases,
            'remaining_debris_mm':np.round(remaining*1000,1).tolist(),
            'trace':trial.trace}
    (path/'floor-mat-yam.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='trace'}))


if __name__=='__main__':
    main()
