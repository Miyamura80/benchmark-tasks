"""Joint-actuated MuJoCo YAM cuff probe with native frictional jaw contacts."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from cuff_walk import aperture_samples, edge_lengths, opening_count, prepare_by_contact, rim_sectors
from probe import FACES, LIP_RADIUS, SEGMENTS, STEP, UNIT, WALL_HEIGHT, ease, model
from render_yam_replay import ArmReplay, MOUNTS, scene_with_arms

PICKS = (SEGMENTS - 3, 3)
OPEN = .009
CLOSED = -.0018
BASE_RELEASE = 3.45
BASE_DROP = 4.25
BASE_END = 5.65


def scene_xml(xml, stock_file, stiff_pads=False, mount_x=None):
    """Add colliding YAM arms, paired position servos, and rubber tip patches."""
    root = ET.fromstring(scene_with_arms(xml, Path(stock_file).resolve()))
    stock = ET.parse(stock_file).getroot()
    equality = ET.SubElement(root, 'equality')
    actuator = ET.SubElement(root, 'actuator')
    for prefix, _, y, _ in MOUNTS:
        arm = root.find(f"worldbody/body[@name='{prefix}arm']")
        if mount_x is not None:
            arm.set("pos", f"{mount_x} {y} 0")
            arm.set("euler", f"0 0 {-np.copysign(np.pi / 2, y)}")
        for body in arm.iter('body'):
            body.set('gravcomp', '1')
            for geom in body.findall('geom'):
                if geom.get('class') == 'collision':
                    geom.set('contype', '1')
                    geom.set('conaffinity', '1')
        for finger in ('lf_down', 'rf_down'):
            body = arm.find(f".//body[@name='{prefix}{finger}']")
            # Replace the distal finger collision approximations with a thin
            # backing and a rubber contact patch on the stock fingertip face.
            for geom in body.findall('geom'):
                if geom.get('class') in ('collision', 'sphere_collision'):
                    body.remove(geom)
            ET.SubElement(body, 'geom', name=prefix+finger+'_back', type='box',
                          pos='0 -.005 .044', size='.006 .002 .030', group='3',
                          rgba='.12 .12 .12 1', mass='.003')
            ET.SubElement(body, 'geom', name=prefix+finger+'_pad', type='box',
                          pos='0 -.0022 .078', size='.006 .0024 .005',
                          friction='1.5 .005 .0001', condim='3', mass='.002',
                          rgba='.13 .14 .15 1', solref='.008 1', solimp='.94 .995 .001')
            if stiff_pads:
                body.find(f"geom[@name='{prefix}{finger}_pad']").attrib.update(
                    priority='1', solref='.005 1', solimp='.999 .9999 .0001')
        eq = deepcopy(stock.find('equality/joint'))
        for key in ('joint1', 'joint2'):
            eq.set(key, prefix + eq.get(key))
        equality.append(eq)
        for src in stock.find('actuator'):
            act = deepcopy(src)
            name = act.get('name')
            for key in ('name', 'joint'):
                act.set(key, prefix + act.get(key))
            if name == 'gripper':
                act.attrib.update(kp='650', kv='10', ctrlrange='-.0018 .0375', forcerange='-10 10')
            else:
                big = int(name[-1]) <= 3
                act.attrib.update(kp='290' if big else '165', kv='16' if big else '5')
            actuator.append(act)
    return ET.tostring(root, encoding='unicode')


class Arm(ArmReplay):
    def __init__(self, m, prefix, side, direction):
        super().__init__(m, prefix, side)
        self.pad_ids = np.array([m.geom(prefix+finger+'_pad').id for finger in ('lf_down','rf_down')])
        self.servo_ids = np.array([m.actuator(prefix+f'joint{i}').id for i in range(1,7)])
        self.gripper = m.actuator(prefix+'gripper').id
        self.target_data = mj.MjData(m)
        mj.mj_forward(m, self.d)
        frame = self.d.site_xmat[self.site].reshape(3, 3)
        self.offset = frame.T @ (self.tip_positions(self.d).mean(axis=0) - self.d.site_xpos[self.site])
        self.orient(side, direction)

    def orient(self, side, direction):
        x, y = side * np.array(direction)
        self.rotation = np.array([[-y, x, 0], [x, y, 0], [0, 0, -1.]])

    def tip_positions(self, data):
        return data.geom_xpos[self.pad_ids]

    def initialize(self, data, target, opening):
        # Only the initial robot pose is installed in the live state. Subsequent
        # IK uses a private scratch MjData and writes the resulting joint command.
        self.solve(data, target, opening)
        data.ctrl[self.servo_ids] = self.q
        data.ctrl[self.gripper] = opening

    def command_target(self, data, target, opening):
        self.solve(self.target_data, target, opening)
        data.ctrl[self.servo_ids] = self.q
        data.ctrl[self.gripper] = opening


def prepared_scene(stock_file, film_mass=.026, stiff_pads=False, timestep=STEP, grip_scale=1.):
    # Standardize the partially fitted condition before introducing the robot.
    # Preparation uses the legacy contact-limited point pulls, ends hands-free,
    # and is excluded from episode time and native manipulation evidence.
    source, state, _, puck_v, _ = model(True, film_mass=film_mass, timestep=timestep)
    strain = prepare_by_contact(source, state, source.flex_vertbodyid, PICKS, puck_v, .110, grip_scale=grip_scale)
    m, d, puck, v, puck_geom = model(True, lambda xml: scene_xml(xml, stock_file, stiff_pads), film_mass=film_mass, timestep=timestep)
    rest = d.flexvert_xpos.copy()
    assert np.array_equal(source.jnt_qposadr, m.jnt_qposadr[:source.njnt])
    assert np.array_equal(source.jnt_dofadr, m.jnt_dofadr[:source.njnt])
    d.qpos[:source.nq] = state.qpos
    d.qvel[:source.nv] = state.qvel
    mj.mj_forward(m, d)
    return m, d, puck, v, puck_geom, rest, strain


def route(start, direction, t, spread=.145, lift=.306, lower_radius=.135, withdrawal="lift"):
    raised = np.r_[start[:2], lift]
    outer = np.r_[spread * direction, lift]
    lowered = np.r_[lower_radius * direction, .237]
    if t < .45:
        return start, 'Close jaws'
    if t < .85:
        return ease(start, raised, (t-.45)/.4), 'Lift'
    if t < 1.7:
        return ease(raised, outer, (t-.85)/.85), 'Spread'
    if t < 2.95:
        return ease(outer, lowered, (t-1.7)/1.25), 'Lower'
    if t >= BASE_RELEASE and withdrawal == 'out':
        outer_clear = lowered + np.r_[.038 * direction, -.003]
        up_clear = outer_clear + [0., 0., .120]
        if t < BASE_RELEASE+.45:
            return ease(lowered, outer_clear, (t-BASE_RELEASE-.08)/.37), 'Let go and back away'
        return ease(outer_clear, up_clear, (t-BASE_RELEASE-.45)/.40), 'Withdraw'
    if t < BASE_RELEASE + .18:
        return lowered, 'Release' if t >= BASE_RELEASE else 'Hold'
    clear = lowered + np.r_[.026 * direction, .13]
    return ease(lowered, clear, (t-BASE_RELEASE-.18)/.54), 'Withdraw'


def jaw_direction(vertex, orientation):
    direction = UNIT[vertex]
    if orientation == 'tangent':
        return np.array([-direction[1], direction[0]])
    return np.array([1., 0.]) if orientation == 'x' else direction


def contacts(m, d, pads, robot_geoms, puck_geom):
    forces = np.zeros((2, 2))
    hit = False
    robot_film = False
    puck_film_normal = 0.
    force = np.zeros(6)
    for i,c in enumerate(d.contact):
        if 0 not in c.flex or c.efc_address < 0:
            continue
        geom = max(c.geom)
        hit |= geom == puck_geom
        robot_film |= geom in robot_geoms
        if geom == puck_geom:
            mj.mj_contactForce(m, d, i, force)
            puck_film_normal += max(0.,force[0])
        if geom in pads:
            mj.mj_contactForce(m, d, i, force)
            forces[pads[geom]] += max(0., force[0])
    return forces, bool(hit), bool(robot_film), puck_film_normal


class ContactPinch:
    """Finite compliant hold for film thinner than a simulated fingertip.

    Capture needs positive native film force on both pads of the same jaw,
    opposed physical faces enclosing a nearby material vertex, and a closed
    command. No predetermined material vertex is attached. Loads act equally
    and oppositely on material and jaws. Opening or excessive slip breaks it.
    """
    LIMIT = .45
    BREAK = .018

    def __init__(self, m, arm, grip_scale):
        self.arm = arm
        self.grip_scale = grip_scale
        self.m = m
        self.vertex = None
        self.local = None
        self.force_n = 0.
        self.events = []
        self.prev_tip = None
        self.broken = False

    def apply(self, d, normal, closing, time):
        tips = self.arm.tip_positions(d)
        midpoint = tips.mean(axis=0)
        frame = d.site_xmat[self.arm.site].reshape(3, 3)
        velocity = (midpoint-self.prev_tip)/self.m.opt.timestep if self.prev_tip is not None else np.zeros(3)
        self.prev_tip = midpoint.copy()
        self.force_n = 0.
        if not closing:
            if self.vertex is not None:
                self.events.append(dict(time=time,event='release',vertex=self.vertex))
            self.vertex = None
            return
        if self.vertex is None and not self.broken and min(normal) > .001 and d.ctrl[self.arm.gripper] <= .001:
            positions = d.flexvert_xpos
            offsets = positions - midpoint
            axial = tips[1]-tips[0]
            gap = np.linalg.norm(axial)
            if gap < 1e-6 or gap > .008:
                return
            axis = axial/gap
            cross = offsets @ axis
            local = offsets - cross[:,None]*axis
            candidates = (np.abs(cross) <= gap/2) & (np.linalg.norm(local,axis=1) < .0065)
            surface = []
            for geom in self.arm.pad_ids:
                pad_local = (positions-d.geom_xpos[geom]) @ d.geom_xmat[geom].reshape(3,3)
                distance = np.linalg.norm(np.maximum(abs(pad_local)-self.m.geom_size[geom],0),axis=1)
                surface.append(distance)
            candidates &= np.maximum(*surface) < .0021
            if not candidates.any():
                return
            self.vertex = int(np.argmin(np.where(candidates,np.linalg.norm(offsets,axis=1),np.inf)))
            self.local = frame.T @ offsets[self.vertex]
            self.events.append(dict(time=time,event='bilateral_capture',vertex=self.vertex,
                                    pad_normal_n=normal.tolist(),pad_center_gap_m=float(gap),
                                    vertex_offset_m=self.local.tolist(),
                                    vertex_to_pad_surface_m=[float(a[self.vertex]) for a in surface]))
        if self.vertex is None:
            return
        body = self.m.flex_vertbodyid[self.vertex]
        position = d.flexvert_xpos[self.vertex]
        error = midpoint + frame @ self.local - position
        if np.linalg.norm(error) > self.BREAK or np.linalg.norm(tips[1]-tips[0]) > .016:
            self.events.append(dict(time=time,event='slip',vertex=self.vertex))
            self.vertex = None
            self.broken = True
            return
        dof = self.m.body_dofadr[body]
        force = 40*error + .08*(velocity-d.qvel[dof:dof+3])
        force *= min(1.,self.LIMIT/max(np.linalg.norm(force),1e-12))
        force *= self.grip_scale
        self.force_n = float(np.linalg.norm(force))
        mj.mj_applyFT(self.m,d,force,np.zeros(3),position,body,d.qfrc_applied)
        # Split the reaction across two fingers at the same world point, so
        # the coupling adds no net external wrench to robot and liner.
        for geom in self.arm.pad_ids:
            mj.mj_applyFT(self.m,d,-force/2,np.zeros(3),position,self.m.geom_bodyid[geom],d.qfrc_applied)


def run(stock_file, mode='pinch', spread=.145, lift=.306, lower_radius=.135,
        orientation='radial', film_mass=.026, stiff_pads=False, withdrawal='lift',
        release_opening=OPEN, timestep=STEP, grip_scale=1., *,
        configure_reset=None, pairs=None, contact_schedule=None, require_uncuffed=False,
        joint_policy=None):
    if not np.isfinite(grip_scale) or grip_scale <= 0:
        raise ValueError('Grasp gain scale must be positive and finite.')
    if not 0 < timestep <= .05 or not np.isclose(round(.05/timestep)*timestep,.05):
        raise ValueError('The timestep must divide the 50 ms control period.')
    prepare = prepared_scene if configure_reset is None else configure_reset
    m, d, puck, puck_vel, puck_geom, rest, prep_strain = prepare(
        stock_file, film_mass, stiff_pads, timestep, grip_scale)
    first_pair = PICKS if pairs is None else pairs[0]
    starts = d.flexvert_xpos[list(first_pair)].copy()
    if require_uncuffed and rim_sectors(d.flexvert_xpos).any():
        raise ValueError('The initial liner must have zero seated sectors.')
    arms = []
    for slot, ((prefix,*_), vtx) in enumerate(zip(MOUNTS,first_pair)):
        direction = jaw_direction(vtx, orientation)
        arm = Arm(m, prefix, (1,-1)[slot], direction)
        arm.initialize(d, starts[slot], OPEN)
        arms.append(arm)
    mj.mj_forward(m, d)
    pinches = [ContactPinch(m,arm,grip_scale) for arm in arms]
    samples = aperture_samples()
    lengths = edge_lengths(rest)
    frames = []
    full_rate = []
    last_robot_film = None
    last_assist = None
    max_assist = np.zeros(2)
    pads = {int(geom):(a,j) for a,arm in enumerate(arms) for j,geom in enumerate(arm.pad_ids)}
    prefixes = tuple(mount[0] for mount in MOUNTS)
    robot_geoms = {i for i in range(m.ngeom) if m.body(m.geom_bodyid[i]).name.startswith(prefixes)}
    load_hit = None
    min_released, min_loaded = 12,12
    min_open = 1.
    settled = True
    min_puck_z = np.inf
    terminal_puck_speed = 0.
    terminal_film_contacts = []
    terminal_film_normal = []
    bilateral_ticks = np.zeros(2, int)
    max_forces = np.zeros((2,2))
    max_strain = 0.
    max_tracking = 0.
    window_forces = np.zeros((2,2))
    controller = joint_policy if joint_policy is not None else contact_schedule
    release, drop, end = (BASE_RELEASE, BASE_DROP, BASE_END) if controller is None else controller.evaluation_times
    if joint_policy is not None:
        joint_policy.start(m, d, arms)
    for tick in range(round(end/timestep)+1):
        time = tick*timestep
        released = time >= release
        dropped = time >= drop
        d.qfrc_applied[:] = 0
        d.qfrc_applied[puck_vel+2] = 0. if dropped else .981
        if tick == round(drop/timestep):
            m.geom_contype[puck_geom] = m.geom_conaffinity[puck_geom] = 1
        if tick % round(.05/timestep) == 0:
            if joint_policy is not None:
                commands = joint_policy.command(time, d)
                opening = commands[:, -1]
                closing = opening <= .001
                targets = [arm.tip_positions(d).mean(axis=0) for arm in arms]
                phase = 'Load test' if released else 'Dream-1 policy'
                for arm, cmd in zip(arms, commands, strict=True):
                    d.ctrl[arm.servo_ids] = cmd[:6]
                    d.ctrl[arm.gripper] = cmd[-1]
            elif contact_schedule is not None:
                targets, phase, opening, closing = contact_schedule.command(time, d, arms)
            else:
                motions = [route(pos, UNIT[vtx], time, spread, lift, lower_radius, withdrawal)
                           for pos, vtx in zip(starts, first_pair)]
                targets = [pos for pos, _ in motions]
                phase = motions[0][1]
                opening = release_opening if released else ease(OPEN, CLOSED, time/.18)
                closing = not released
            if joint_policy is None:
                for arm, target, jaw in zip(arms, targets, np.broadcast_to(opening, (2,)), strict=True):
                    arm.command_target(d, target, OPEN if mode == 'open' else jaw)
            closing = np.broadcast_to(closing, (2,))
        forces, hit, robot_film, puck_film_normal = contacts(m,d,pads,robot_geoms,puck_geom)
        if robot_film:
            last_robot_film = time
        if mode == "assisted":
            for a,pinch in enumerate(pinches):
                pinch.apply(d,forces[a],closing[a] and (joint_policy is not None or not released),time)
                if not closing[a]:
                    pinch.broken = False
        applied = [pinch.force_n for pinch in pinches]
        max_assist = np.maximum(max_assist,applied)
        if any(applied):
            last_assist = time
        window_forces = np.maximum(window_forces, forces)
        max_forces = np.maximum(max_forces,forces)
        bilateral_ticks += (forces > .001).all(axis=1)
        if hit and dropped and load_hit is None:
            load_hit = time
        points = d.flexvert_xpos
        sectors = rim_sectors(points)
        count = int(sectors.sum())
        aperture = opening_count(points,samples)/len(samples)
        puck_speed = float(np.linalg.norm(d.qvel[puck_vel:puck_vel+3]))
        full_rate.append([round(time,6),count,aperture,int(robot_film),int(any(applied)),int(hit),
                          float(d.xpos[puck,2]),float(np.linalg.norm(d.xpos[puck,:2])),puck_speed,puck_film_normal])
        if released:
            min_released = min(min_released,count)
            min_open = min(min_open,aperture)
        if dropped:
            min_loaded = min(min_loaded,count)
            min_puck_z = min(min_puck_z,float(d.xpos[puck,2]))
        if time >= end-.35:
            terminal_puck_speed = max(terminal_puck_speed,puck_speed)
            terminal_film_contacts.append(hit)
            terminal_film_normal.append(puck_film_normal)
            settled &= bool(np.linalg.norm(d.xpos[puck,:2]) < .085
                            and .020 <= d.xpos[puck,2] < .090 and puck_speed <= .020)
        if tick % round(.05/timestep) == 0:
            strain = float(np.max(np.abs(edge_lengths(points)/lengths-1)))
            max_strain = max(max_strain,strain)
            tips = np.array([arm.tip_positions(d) for arm in arms])
            max_tracking = max(max_tracking,float(np.max(np.linalg.norm(np.array(targets)-tips.mean(axis=1),axis=1))))
            frames.append(dict(time=round(time,4),phase=phase,points=points.round(6).tolist(),
                               sectors=sectors.tolist(), mouth_ratio=aperture,peak_edge_strain=strain,
                               puck=d.xpos[puck].round(6).tolist() if dropped else None,
                               pad_center_xyz=tips.round(6).tolist(), pinch_vertex=[p.vertex for p in pinches],
                               pinch_force_n=[p.force_n for p in pinches],pad_film_normal_n=forces.tolist(),
                               pad_film_peak_normal_n=window_forces.tolist(),
                               arm_qpos=[d.qpos[np.r_[arm.qpos,arm.fingers]].tolist() for arm in arms],
                               arm_ctrl=[d.ctrl[np.r_[arm.servo_ids,arm.gripper]].tolist() for arm in arms]))
            window_forces.fill(0.)
        if tick < round(end/timestep):
            mj.mj_step(m,d)
    warning = [w.number for w in d.warning]
    clear_at_load = last_robot_film is None or last_robot_film < drop
    clear_assist = last_assist is None or last_assist < release
    valid = (min_released == 12 and min_open >= .85 and settled and load_hit is not None
             and min_puck_z >= .020 and all(terminal_film_contacts)
             and clear_at_load and clear_assist and not any(warning))
    return dict(mode=mode, timestep_s=timestep, grip_scale=grip_scale, spread_m=spread, lift_m=lift, lower_radius_m=lower_radius,
                orientation=orientation, withdrawal=withdrawal, release_opening_m=release_opening, stiff_pads=stiff_pads, film_mass_kg=film_mass, initial_sectors=sum(frames[0]['sectors']),final_sectors=count,
                min_released_sectors=min_released,min_loaded_sectors=min_loaded,min_released_opening=min_open,
                post_load_retained=bool(valid),puck_settled=settled,first_film_contact_s=load_hit,
                minimum_loaded_puck_center_z_m=min_puck_z,
                terminal_puck_max_speed_m_s=terminal_puck_speed,
                terminal_puck_film_contact_fraction=float(np.mean(terminal_film_contacts)),
                terminal_puck_film_mean_normal_n=float(np.mean(terminal_film_normal)),
                final_puck_xyz_m=d.xpos[puck].tolist(),
                last_robot_film_contact_s=last_robot_film, last_assist_force_s=last_assist,
                max_assist_force_n=max_assist.tolist(),
                physics_trace_keys=['time_s','num_sectors','aperture','robot_touches_film','assist_active',
                                    'puck_touches_film','puck_z_m','puck_radius_m','puck_speed_m_s','puck_film_normal_n'], physics_trace=full_rate,
                pinch_events=[p.events for p in pinches],
                bilateral_pad_contact_s=(bilateral_ticks*timestep).tolist(),max_pad_normal_n=max_forces.tolist(),
                preparation_max_sampled_strain=prep_strain,peak_sampled_max_abs_edge_strain=max_strain,
                max_sampled_tracking_error_m=max_tracking, warnings=warning,frames=frames)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--yam-xml', type=Path, required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--mode',choices=('pinch','open','assisted'),default='pinch')
    p.add_argument('--film-mass',type=float,default=.026,help='Total film mass in kilograms')
    p.add_argument('--stiff-pads',action='store_true',help='Use stiffer native fingertip contact')
    p.add_argument('--withdrawal',choices=('lift','out'),default='lift')
    p.add_argument('--release-opening',type=float,default=OPEN)
    p.add_argument('--timestep',type=float,default=STEP)
    p.add_argument('--grip-scale',type=float,default=1.)
    p.add_argument('--spread',type=float,default=.145)
    p.add_argument('--lift',type=float,default=.306)
    p.add_argument('--lower-radius',type=float,default=.135)
    p.add_argument('--orientation',choices=('radial','x','tangent'),default='radial')
    args = p.parse_args()
    r = run(args.yam_xml, args.mode, args.spread, args.lift, args.lower_radius,
            args.orientation, args.film_mass, args.stiff_pads, args.withdrawal,
            args.release_opening, args.timestep, args.grip_scale)
    print(json.dumps({k:v for k,v in r.items() if k not in ('frames','physics_trace')}))
    args.output.write_text(json.dumps(dict(results=[r],faces=FACES.tolist(),wall_height=WALL_HEIGHT,
        lip_radius=LIP_RADIUS,release_s=BASE_RELEASE,drop_s=BASE_DROP),separators=(',',':')))


if __name__=='__main__':
    main()
