"""Closed-loop Cartesian gripper demo for the driver's floor-mat fixture."""
from pathlib import Path
import json
import math
import subprocess
import sys

import mujoco as mj
import numpy as np


def box(name, position, size, color, extra=''):
    return f'<geom name="{name}" type="box" pos="{position}" size="{size}" rgba="{color}" {extra}/>'


LOOSE_GRIT = 36
BONDED_GRIT = 12
TOTAL_GRIT = LOOSE_GRIT + BONDED_GRIT


def scene():
    mat = []
    for i in range(2):
        bars = ''.join(box(f'rib_{i}_{j}', f'{-.092+.018*j} 0 .005', '.003 .130 .002',
                           '.118 .130 .136 1', 'mass=".001"')
                       for j in range(11))
        mat.append(box(f'backing{i}', '0 0 0', '.10 .143 .005', '.067 .076 .082 1',
                       'mass=".19"') + bars
                   + ''.join(box(f'raised_edge_{i}_{side}', f'0 {side*.139} .008',
                                 '.10 .003 .005', '.103 .115 .121 1', 'mass=".003"')
                             for side in (-1,1)))

    rng = np.random.default_rng(461)
    grit = ''
    for i in range(36):
        x, y = rng.uniform(.293, .487), rng.uniform(-.117, .117)
        radius = rng.uniform(.0036, .0064)
        color = rng.uniform(.29, .52)
        grit += f'''
        <body name="grit{i}" pos="{x} {y} {.049 + radius}">
          <joint type="free" damping=".000025"/><geom name="particle{i}" type="ellipsoid" size="{radius} {radius*.81} {radius*.7}"
            rgba="{color} {color*.83} {color*.64} 1" mass=".0008" friction=".65 .004 .001"/>
        </body>'''
    for row, y in enumerate((-.083, 0., .083)):
        for column, x in enumerate((.361, .392, .426, .457)):
            i=LOOSE_GRIT + row*4 + column
            color=".68 .46 .30 1"
            grit += f'''<body name="grit{i}" pos="{x} {y} .052">
              <joint type="free" damping=".000025"/><geom name="crust{i}" type="ellipsoid" size=".0048 .0058 .0044"
                rgba="{color}" mass=".0008" friction=".78 .006 .002" contype="3"/>
            </body>'''
    bonds = ''.join(f'<weld name="bond{i}" body1="mat" body2="grit{i}" solref=".003 1"/>'
                    for i in range(LOOSE_GRIT,TOTAL_GRIT))
    bristles = ''.join(f'''<geom name="bristle{i}" type="capsule"
                       fromto=".058 {-.024+i*.008} .018 .070 {-.024+i*.008} -.012"
                       size=".0045" mass=".001" rgba=".36 .22 .14 1"
                       contype="0" conaffinity="2" solref=".03 1" friction=".9 .005 .002"/>'''
                       for i in range(7))
    studs = ''.join(
        f'<geom name="stud{side}" type="cylinder" pos=".339 {side*.102} .037" size=".009 .0045" contype="0" conaffinity="0" rgba=".19 .20 .20 1"/>'
        for side in (-1, 1))
    clips = ''.join(
        f'<geom name="clip{side}" type="cylinder" pos="-.051 {side*.102} .007" size=".015 .003" rgba=".24 .26 .28 1" contype="0" conaffinity="0"/>'
        f'<site name="socket{side}" pos="-.051 {side*.102} -.004" size=".003" rgba=".05 .05 .05 0"/>'
        for side in (-1, 1))
    return f"""
    <mujoco model="Driver floor-mat detailing">
      <compiler angle="radian"/>
      <option timestep=".002" gravity="0 0 -9.81" integrator="implicitfast" iterations="45"/>
      <visual><global offwidth="960" offheight="540"/><headlight ambient=".38 .38 .38" diffuse=".65 .65 .65" specular=".14 .14 .14"/></visual>
      <default>
        <geom friction=".9 .01 .001" solref=".010 1"/>
        <joint damping="2"/>
      </default>
      <worldbody>
        <light pos="-.2 -.3 1" dir=".2 .3 -1"/>
        <light pos=".6 .5 .7" dir="-.2 -.4 -1" castshadow="false"/>
        {box('bench','0 0 -.0375','1 1 .035','.20 .23 .25 1')}
        {box('footwell','.445 0 .016','.260 .177 .019','.123 .140 .155 1')}
        {box('firewall','.724 0 .094','.025 .177 .065','.095 .109 .118 1')}
        {box('tunnel','.439 .176 .076','.265 .018 .059','.113 .125 .137 1')}
        {box('sill','.44 -.185 .028','.267 .013 .031','.22 .24 .26 1')}
        {box('sill_trim','.44 -.190 .060','.25 .013 .003','.31 .35 .38 1')}
        {box('toe_stop','.705 0 .057','.007 .171 .023','.100 .113 .125 1')}
        {box('pedal_arm','.693 .027 .183','.022 .009 .060','.16 .18 .19 1')}
        {box('brake_pad','.668 .027 .148','.019 .040 .012','.067 .075 .083 1','euler="0 -.3 0"')}
        {box('gas_pedal','.680 .114 .128','.034 .021 .009','.070 .079 .085 1','euler="0 -.38 0"')}
        {box('tray_base','.30 -.432 .017','.284 .193 .017','.245 .341 .383 1')}
        {box('tray_east','.587 -.432 .046','.008 .200 .033','.23 .32 .36 1')}
        {box('tray_west','.013 -.432 .046','.008 .200 .033','.23 .32 .36 1')}
        {box('tray_north','.30 -.234 .046','.284 .006 .033','.23 .32 .36 1')}
        {box('tray_south','.30 -.630 .046','.284 .006 .033','.23 .32 .36 1')}
        {studs}
        <body name="mat" pos=".39 0 .040">
          <freejoint name="mat_free"/>
          {mat[0]}{clips}
          {box('pull_lip','-.105 0 .014','.012 .025 .007','.12 .13 .14 1','mass=".009"')}
          <site name="lip" pos="-.105 0 .014" size=".003" rgba=".4 .7 .7 0"/>
          <body name="toe" pos=".1004 0 0">
            <joint name="curl" axis="0 1 0" limited="true" range="-.95 .95" stiffness=".4" damping=".04"/>
            <body name="tip" pos=".1004 0 0">
              {mat[1]}
            </body>
          </body>
        </body>
        {grit}
        <body name="gantry" pos=".286 0 .36">
          <joint name="x" type="slide" axis="1 0 0" range="-.73 .42"/>
          <joint name="y" type="slide" axis="0 1 0" range="-.64 .38"/>
          <joint name="z" type="slide" axis="0 0 1" range="-.32 .08"/>
          <geom type="cylinder" pos="0 0 .066" size=".019 .045" rgba=".29 .32 .35 1" mass=".25" contype="0" conaffinity="0"/>
          <body name="wrist">
            <joint name="pitch" type="hinge" axis="0 1 0" range="-1.7 2.6" damping=".4"/>
            {box('palm','0 0 .027','.019 .052 .012','.20 .27 .31 1','mass=".08"')}
            <site name="pinch" pos="0 0 0" size=".004" rgba=".4 .7 .7 0"/>
            <body name="brush" pos="0 0 .10">
              <joint name="brush_slide" type="slide" axis="0 0 1" range="-.15 .01" damping="1"/>
              {box('brush_carriage','.035 0 .044','.026 .037 .013','.24 .28 .31 1','mass=".03" contype="0" conaffinity="0"')}
              {box('brush_head','.063 0 .020','.019 .034 .007','.79 .52 .25 1','mass=".02" contype="0" conaffinity="0"')}
              {bristles}
            </body>
            <body name="vacuum" pos="0 0 .123">
              <joint name="vacuum_slide" type="slide" axis="0 0 1" range="-.144 .01" damping="1"/>
              <geom name="vac_shell" type="ellipsoid" pos="-.073 0 .078" size=".036 .023 .040" mass=".03" rgba=".12 .15 .16 1" contype="0" conaffinity="0"/>
              <geom name="vac_filter" type="cylinder" pos="-.073 0 .056" size=".012 .008" mass=".007" rgba=".61 .53 .37 1" contype="0" conaffinity="0"/>
              <geom name="vac_bin" type="cylinder" pos="-.073 0 .030" size=".026 .027" mass=".02" rgba=".24 .48 .53 .30" contype="0" conaffinity="0"/>
              <geom name="vac_nozzle" type="cylinder" pos="-.073 0 -.002" size=".019 .013" mass=".009" rgba=".15 .18 .19 1" contype="0" conaffinity="0"/>
              <geom name="vac_intake" type="cylinder" pos="-.073 0 -.015" size=".016 .001" mass=".002" rgba=".031 .035 .037 1" contype="0" conaffinity="0"/>
              <geom name="vac_switch" type="ellipsoid" pos="-.088 -.021 .067" size=".006 .003 .006" mass=".002" rgba=".92 .42 .16 1" contype="0" conaffinity="0"/>
              <site name="vac_intake" pos="-.073 0 -.015" size=".003" rgba="0 0 0 0"/>
              <site name="vac_chamber" pos="-.073 0 .029" size=".003" rgba="0 0 0 0"/>
            </body>
            <body name="left_finger" pos="0 .028 0">
              <joint name="left" type="slide" axis="0 1 0" range="0 .026" damping=".4"/>
              {box('left_jaw','0 0 .008','.017 .006 .024','.29 .33 .35 1','mass=".045"')}
              {box('left_pad','0 -.006 0','.016 .003 .010','.08 .10 .12 1','mass=".008"')}
            </body>
            <body name="right_finger" pos="0 -.028 0">
              <joint name="right" type="slide" axis="0 -1 0" range="0 .026" damping=".4"/>
              {box('right_jaw','0 0 .008','.017 .006 .024','.29 .33 .35 1','mass=".045"')}
              {box('right_pad','0 .006 0','.016 .003 .010','.08 .10 .12 1','mass=".008"')}
            </body>
          </body>
        </body>
      </worldbody>
      <equality>{bonds}</equality>
      <actuator>
        <position name="x" joint="x" kp="1800" kv="65"/>
        <position name="y" joint="y" kp="1800" kv="65"/>
        <position name="z" joint="z" kp="1800" kv="65"/>
        <position name="pitch" joint="pitch" kp="35" kv="1.4"/>
        <position name="left" joint="left" kp="400" kv="3"/>
        <position name="right" joint="right" kp="400" kv="3"/>
        <position name="brush" joint="brush_slide" kp="350" kv="6"/>
        <position name="vacuum" joint="vacuum_slide" kp="510" kv="6"/>
      </actuator>
    </mujoco>"""


class Trial:
    def __init__(self):
        self.model = mj.MjModel.from_xml_string(scene())
        self.data = mj.MjData(self.model)
        self.mat_id = self.model.body('mat').id
        self.wrist_id = self.model.body('wrist').id
        self.socket_ids = [self.model.site(f'socket{i}').id for i in (-1,1)]
        self.posts = np.array([[.339,-.102,.036],[.339,.102,.036]])
        self.latched = np.ones(2, dtype=bool)
        self.released = np.zeros(2, dtype=bool)
        self.was_gripped = self.gripped = False
        self.samples = []
        self.grit = [self.model.body(f'grit{i}').id for i in range(TOTAL_GRIT)]
        self.bonds = {self.model.geom(f'crust{i}').id: self.model.equality(f'bond{i}').id
                      for i in range(LOOSE_GRIT,TOTAL_GRIT)}
        self.bristles = {self.model.geom(f'bristle{i}').id for i in range(7)}
        self.brush_ends = np.zeros(6)
        self.debris_geoms = [self.model.geom(f'particle{i}' if i<LOOSE_GRIT else f'crust{i}').id
                             for i in range(TOTAL_GRIT)]
        self.debris_vel = np.array([int(self.model.body_jntadr[body]) for body in self.grit])
        self.debris_vel = self.model.jnt_dofadr[self.debris_vel]
        self.bin_loaded = np.zeros(TOTAL_GRIT, dtype=bool)
        self.entered_nozzle = np.zeros(TOTAL_GRIT, dtype=bool)
        self.intake = self.model.site('vac_intake').id
        self.chamber = self.model.site('vac_chamber').id
        self.powered = False
        self.pad_ids = {self.model.geom(f'{side}_pad').id for side in ('left', 'right')}
        self.lip_geom = self.model.geom('pull_lip').id
        self.target = np.array([0., 0., 0., 0., .021, .021, 0., 0.])
        self.data.ctrl[:] = self.target
        self.draw = None
        self.step_count = 0
        mj.mj_forward(self.model, self.data)

    def step(self):
        d, m = self.data, self.model
        d.qfrc_applied[:] = 0
        if self.target[4] > .014:
            self.gripped = False
        elif not self.gripped:
            contacts = set()
            for c in d.contact:
                pair = {c.geom1, c.geom2}
                if self.lip_geom in pair and c.dist <= .001:
                    contacts |= self.pad_ids & pair
            self.gripped = contacts == self.pad_ids
            self.was_gripped |= self.gripped
        if self.gripped:
            hand = d.site('pinch').xpos
            lip = d.site('lip').xpos
            if np.linalg.norm(hand - lip) > .10:
                self.gripped = False
            else:
                hand_vel = np.array([d.joint(name).qvel[0] for name in ('x','y','z')])
                mat_joint = d.joint('mat_free')
                linear = np.clip(1850*(hand-lip) + 16*(hand_vel-mat_joint.qvel[:3]), -34, 34)
                rotation = d.xmat[self.wrist_id].reshape(3,3) @ d.xmat[self.mat_id].reshape(3,3).T
                qerr = np.empty(4)
                angle_error = np.empty(3)
                mj.mju_mat2Quat(qerr, rotation.ravel())
                if qerr[0] < 0:
                    qerr *= -1
                mj.mju_quat2Vel(angle_error, qerr, 1.)
                torque = np.clip(5.5*angle_error - .18*mat_joint.qvel[3:], -5, 5)
                mj.mj_applyFT(m,d,linear,torque,lip,self.mat_id,d.qfrc_applied)
        for i,(point,post) in enumerate(zip(d.site_xpos[self.socket_ids], self.posts)):
            error = point-post
            if self.latched[i] and (error[2] > .023 or np.linalg.norm(error[:2]) > .021):
                self.latched[i] = False
                self.released[i] = True
            if not self.latched[i] and abs(error[2]) < .005 and np.linalg.norm(error[:2]) < .0065:
                self.latched[i] = True
            if self.latched[i]:
                force = np.clip(-780*error, -8.0, 8.0)
                mj.mj_applyFT(m,d,force,np.zeros(3),point,self.mat_id,d.qfrc_applied)
        # An advancing bristle peels the contact bond before a rigid impact.
        if d.joint('brush_slide').qpos[0] < -.10 and abs(d.joint('x').qvel[0]) > .008:
            for geom, bond in self.bonds.items():
                if not d.eq_active[bond]:
                    continue
                if any(mj.mj_geomDistance(m,d,bristle,geom,.002,self.brush_ends) < .0007
                       for bristle in self.bristles):
                    d.eq_active[bond] = False
        self.step_vacuum()
        previous_time = d.time
        mj.mj_step(m,d)
        if d.time <= previous_time:
            raise RuntimeError(f'MuJoCo reset at {previous_time:.3f} s')
        self.step_count += 1

    def step_vacuum(self):
        d,m=self.data,self.model
        if not self.powered and not np.any(self.entered_nozzle):
            return
        intake = d.site_xpos[self.intake]
        chamber = d.site_xpos[self.chamber]
        rotation = d.site_xmat[self.intake].reshape((3,3))
        offset = d.xpos[self.grit] - intake
        local = offset @ rotation
        distance = np.linalg.norm(offset,axis=1)
        velocity = np.stack([d.qvel[i:i+3] for i in self.debris_vel])
        tool_velocity = np.array([d.joint(name).qvel[0] for name in ('x','y','z')])
        relative_velocity = velocity-tool_velocity
        radii = np.linalg.norm(local[:,:2],axis=1)
        kind = np.array([not d.eq_active[self.bonds[geom]] if geom in self.bonds else True
                         for geom in self.debris_geoms])
        active = ((distance<.044) & (local[:,2]<.013) & (local[:,2]>-.041) &
                  (kind & ~self.entered_nozzle)) if self.powered else np.zeros(TOTAL_GRIT,dtype=bool)
        entered = active & (radii<.013) & (local[:,2]>-.004)
        self.entered_nozzle |= entered
        active &= ~entered
        for i in np.flatnonzero(entered):
            m.geom_contype[self.debris_geoms[i]] = 4
            m.geom_conaffinity[self.debris_geoms[i]] = 0
        if np.any(active):
            fraction = np.maximum(1-distance[active]/.044,0.)
            forward = np.maximum(-local[active,2]/np.maximum(distance[active],.001),.18)
            toward = -offset[active]/np.maximum(distance[active,None],.001)
            d.xfrc_applied[np.array(self.grit)[active],:3] = (
                .125*fraction[:,None]**2*forward[:,None]*toward -
                .006*relative_velocity[active])
        d.xfrc_applied[np.array(self.grit)[~(active|self.entered_nozzle)],:3] = 0
        carried = np.flatnonzero(self.entered_nozzle)
        if len(carried):
            # Follow through the intake into the filter chamber. A spring retains
            # particles at separate positions behind the inlet's one-way seal.
            angles = carried * 2.399963
            spread = .015 * np.sqrt(carried/TOTAL_GRIT)
            packing = np.column_stack((spread*np.cos(angles),spread*np.sin(angles),
                                       -.010+.004*(carried%6)))
            target = chamber + packing @ rotation.T
            error = target - d.xpos[np.array(self.grit)[carried]]
            accel = 950*error - 50*relative_velocity[carried] + [0,0,9.81]
            d.xfrc_applied[np.array(self.grit)[carried],:3] = .0008*np.clip(accel,-120.,120.)
            self.bin_loaded[carried] |= np.linalg.norm(error,axis=1)<.021

    def metrics(self):
        dust = self.data.xpos[self.grit]
        in_tray = (np.abs(dust[:,0]-.30)<.282) & (np.abs(dust[:,1]+.432)<.192) & (dust[:,2]<.078)
        collected = int(np.sum(in_tray | self.bin_loaded))
        dislodged = sum(not self.data.eq_active[bond] for bond in self.bonds.values())
        pitch = math.atan2(-self.data.xmat[self.mat_id,6],self.data.xmat[self.mat_id,0])
        flat = abs(pitch) < .085 and abs(self.data.joint('curl').qpos[0]) < .12 and abs(self.data.xpos[self.mat_id,2]-.044)<.012
        parked = bool(np.all(self.latched) and flat and not self.gripped)
        score = (.15*bool(np.all(self.released)) + .25*dislodged/BONDED_GRIT
                 + .40*min(collected/44,1.) + .20*bool(parked and np.all(self.released)
                     and collected>=44 and dislodged==BONDED_GRIT))
        return dict(collected=collected, vacuumed=int(np.sum(self.bin_loaded)),
                    in_tray=int(np.sum(in_tray)), dislodged=dislodged, clips_closed=int(np.sum(self.latched)), clips_released=int(np.sum(self.released)),
                    flat=flat, gripped=self.gripped, score=round(score,3))

    def move(self, position, pitch, grip, seconds, phase, brush=0., vacuum=0.):
        start = self.target.copy()
        goal = np.r_[np.asarray(position)-[.286,0,.36], pitch, grip, grip, brush, vacuum]
        steps = round(seconds/.002)
        for i in range(steps):
            t = (i+1)/steps
            self.target = start + t*t*(3-2*t)*(goal-start)
            self.data.ctrl[:] = self.target
            self.step()
            if self.step_count % 25 == 0:
                self.samples.append({'time':round(self.step_count*.002,3),'phase':phase,**self.metrics()})
                if self.draw:
                    self.draw(phase)

    def run(self, control=False, shake_only=False):
        self.move((.287,0,.145), 0,.021,.8,'Approach the driver’s floor mat')
        if control:
            self.move((.287,0,.145), 0,.021,1.4,'Leave the mat untouched')
            return
        if not shake_only:
            for y in (-.083, 0., .083):
                self.move((.266,y,.093), 0,.021,.60,'Position the stiff-bristle brush')
                self.move((.266,y,.093), 0,.021,.24,'Lower the brush onto the ribs', brush=-.141)
                self.move((.409,y,.093), 0,.021,1.5,'Scrub dried dirt out of the rubber ribs', brush=-.141)
                self.move((.409,y,.093), 0,.021,.24,'Lift the brush off the mat')
            self.move((.400,-.114,.100), 0,.021,.60,'Position the car vacuum and switch it on')
            self.powered = True
            self.move((.400,-.114,.083), 0,.021,.32,'Lower the nozzle to the rubber ribs', vacuum=-.136)
            for row,y in enumerate(np.linspace(-.128,.128,8)):
                start,end = (.373,.607) if row%2==0 else (.607,.373)
                self.move((start,y,.083), 0,.021,.22,'Move to the next row of ribs', vacuum=-.136)
                self.move((end,y,.083), 0,.021,1.15,'Vacuum the brushed grit from the mat', vacuum=-.136)
            self.move((.420,.114,.135), 0,.021,.48,'Raise the vacuum and switch it off')
            self.powered = False
            self.move((.287,0,.145), 0,.021,.62,'Set the vacuum clear and approach the pull lip')
        self.move((.285,0,.054), 0,.021,.65,'Lower to the rubber pull lip')
        self.move((.285,0,.054), 0,0,.45,'Grip the rubber pull lip')
        self.move((.245,0,.127),.12,0,1.05,'Pull up to release both retention clips')
        self.move((.182,-.208,.292),0,0,1.1,'Lift the mat over the door sill')
        self.move((.171,-.429,.31),-.10,0,1.5,'Carry the mat to the grit tray')
        self.move((.171,-.429,.218),-1.67,0,1.2,'Tip the mat and empty the grit')
        self.move((.171,-.429,.218),-1.67,0,.95,'Hold the mat over the tray')
        self.move((.171,-.429,.34),0,0,1.05,'Level the cleaned mat')
        self.move((.178,-.064,.250),0,0,1.6,'Bring the mat back over the sill')
        self.move((.221,0,.097),0,0,.9,'Line up both retention clips')
        self.move((.285,0,.054),0,0,1.10,'Press the mat into both retention clips')
        self.move((.285,0,.054),0,.021,.45,'Open the jaws and release the mat')
        self.move((.287,0,.18),0,.021,.75,'Check the mat sits flat and clears the pedals')


def main():
    output = Path(sys.argv[1])
    output.mkdir(parents=True,exist_ok=True)
    trial=Trial()
    video='--no-video' not in sys.argv
    if video:
        from PIL import Image, ImageDraw, ImageFont
        renderer=mj.Renderer(trial.model,height=464,width=800)
        camera=mj.MjvCamera()
        camera.lookat[:]=[.400,-.205,.09]
        camera.distance,camera.azimuth,camera.elevation=1.30,110,-63
        heading=ImageFont.truetype('DejaVuSans-Bold.ttf',18)
        body=ImageFont.truetype('DejaVuSans.ttf',16)
        stream=subprocess.Popen(['ffmpeg','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','800x540','-r','20','-i','-','-c:v','libx264','-threads','2','-preset','fast','-crf','25','-pix_fmt','yuv420p','-movflags','+faststart',str(output/'floor-mat.mp4')], stdin=subprocess.PIPE)
        def draw(phase):
            renderer.update_scene(trial.data,camera)
            renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = False
            canvas=Image.new('RGB',(800,540),'#142225')
            canvas.paste(Image.fromarray(renderer.render()),(0,36))
            ui=ImageDraw.Draw(canvas)
            ui.text((14,8),'Clean and refit the driver’s floor mat',font=heading,fill='#edf4f0')
            ui.text((14,513),phase,font=body,fill='#edf4f0')
            state=trial.metrics()
            ui.text((545,513),f"Bin {state['vacuumed']}  Tray {state['in_tray']}   Clips {state['clips_closed']}/2",font=body,fill='#acf1db')
            stream.stdin.write(canvas.tobytes())
        trial.draw=draw
        for _ in range(16):
            draw('Starting state')
    trial.run()
    if video:
        for _ in range(20):
            draw('Finished')
        stream.stdin.close()
        if stream.wait():
            raise RuntimeError('Video encoding failed')
        renderer.close()
    control=Trial()
    control.run(control=True)
    shake=Trial()
    shake.run(shake_only=True)
    result={'scripted':trial.metrics(),'shake_only_control':shake.metrics(),
            'untouched_control':control.metrics(),'trace':trial.samples}
    (output/'floor-mat.json').write_text(json.dumps(result,indent=2,default=lambda a:a.item())+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='trace'}, default=lambda a:a.item()))


if __name__=='__main__':
    main()
