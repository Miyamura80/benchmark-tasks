"""Run native carton diagnostics and the ordinary full-motion candidate."""
import argparse,os,json,shutil,hashlib
from pathlib import Path
from config import *
from scene import layout
from model import WIDTH,HEIGHT
from crease import physical_bottom_angle,joint_parameters

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--mode',choices=['dynamics','grip-probe','cut-probe','open-probe','turnover-probe','turn-and-cut-probe','turn-cut-open-probe','scripted-demo-probe','turnover-load-probe','setup-check'],default='dynamics');p.add_argument('--bottom-up-start',action='store_true',help='Legacy bottom-up engineering setup, not the revised full task');p.add_argument('--unsealed',action='store_true',help='Explicit unscored starting condition for folding diagnostics');p.add_argument('--no-video',action='store_true',help='State-only engineering run: omit camera sensors/rendering and record exact native poses, without eligible media')
 p.add_argument('--load-outer-flap',action='store_true',help='Apply the diagnostic high load to the outer flap touched by the robot')
 p.add_argument('--level-right-pad',action='store_true',help='Use the level fingertip pair at the high turnover contact')
 p.add_argument('--tip-lift',action='store_true',help='Add a small upward sweep at the high turnover contact')
 prefix=p.add_mutually_exclusive_group()
 prefix.add_argument('--turnover-action-prefix',type=Path,help='Replay recorded ordinary joint commands from reset before live cutting')
 prefix.add_argument('--cut-action-prefix',type=Path,help='Replay ordinary joint commands through tape cutting and cutter parking')
 prefix.add_argument('--first-flap-action-prefix',type=Path,help='Replay through the measured first-flap release before opening the remaining flaps')
 prefix.add_argument('--third-flap-action-prefix',type=Path,help='Replay through the measured release of three flaps')
 p.add_argument('--reposition-carton',action='store_true',help='Translate the carton by contacting its inside wall before the last flap')
 p.add_argument('--compliant-push',action='store_true',help='Allow more virtual right-pad preload while measuring actual clearance')
 p.add_argument('--elevated-pickup',action='store_true',help='Use the raised cutter transit')
 p.add_argument('--tapered-flap-hook',action='store_true',help='Open the cut flaps with a tapered fingertip')
 p.add_argument('--west-first',action='store_true',help='Open the west flap immediately after parking the cutter')
 p.add_argument('--west-finger-notch',action='store_true',help='Use a die-cut notch in the west inner flap')
 p.add_argument('--west-before-east',action='store_true',help='Probe the west notch immediately after opening the two outer flaps')
 p.add_argument('--blade-tilt',action='store_true',help='Allow blade inclination while keeping the cutting heading')
 p.add_argument('--light-tape-pressure',action='store_true',help='Hold the blade depth correction while the measured tip is inside the film')
 p.add_argument('--device',default='cuda:0',help='Isaac Lab physics device')
 p.add_argument('--cross-device-prefix',action='store_true',help='Record passive top-flap deviations separately during turnover on another GPU')
 p.add_argument('--live-after-breakover',action='store_true',help='Replay through first-roll breakover and continue all remaining stages from native feedback')
 a=p.parse_args()
 opens=a.mode in ('turn-cut-open-probe','scripted-demo-probe')
 full_chain=a.mode=='scripted-demo-probe'
 chained=a.mode=='turn-and-cut-probe' or opens
 load_probe=a.mode=='turnover-load-probe'
 if opens and (a.unsealed or a.bottom_up_start):p.error('chained opening requires an upright, sealed carton')
 if load_probe and (a.unsealed or a.bottom_up_start):p.error('turnover load requires an upright, sealed carton')
 if a.load_outer_flap and not load_probe:p.error('--load-outer-flap requires --mode turnover-load-probe')
 if a.level_right_pad and not (chained or a.mode=='turnover-probe'):p.error('--level-right-pad requires a turnover controller')
 action_prefix=a.third_flap_action_prefix or a.first_flap_action_prefix or a.cut_action_prefix or a.turnover_action_prefix
 if a.live_after_breakover and not (a.cross_device_prefix and a.turnover_action_prefix):
  p.error('--live-after-breakover requires --cross-device-prefix and --turnover-action-prefix')
 if a.cross_device_prefix and not action_prefix:p.error('--cross-device-prefix requires a recorded action prefix')
 if action_prefix and (not chained or a.unsealed or a.bottom_up_start):
  p.error('an action prefix requires a chained upright and sealed run')
 opening_prefix=a.cut_action_prefix or a.first_flap_action_prefix or a.third_flap_action_prefix
 if opening_prefix and not (opens and a.tapered_flap_hook):
  p.error('a cut or flap action prefix requires chained fingertip opening')
 if a.west_first and (not opens or not a.tapered_flap_hook or a.first_flap_action_prefix or a.third_flap_action_prefix):
  p.error('--west-first requires chained fingertip opening with no saved flap prefix')
 if a.west_finger_notch and (not opens or not a.tapered_flap_hook or a.west_first):
  p.error('--west-finger-notch requires chained fingertip opening without --west-first')
 if a.west_before_east and not a.west_finger_notch:
  p.error('--west-before-east requires --west-finger-notch')
 if a.west_before_east and a.third_flap_action_prefix:
  p.error('a third-flap prefix requires west to open last')
 if a.reposition_carton and not a.third_flap_action_prefix:
  p.error('--reposition-carton requires --third-flap-action-prefix')
 if action_prefix:
  os.environ['ROBOCURVE_TURNOVER_PREFIX']=str(action_prefix.resolve(strict=True))
  os.environ['ROBOCURVE_TURNOVER_RESUME']='last_flap' if a.third_flap_action_prefix else 'remaining_flaps' if a.first_flap_action_prefix else 'opening' if a.cut_action_prefix else 'cutter'
 if a.mode=='cut-probe' and not a.bottom_up_start:p.error('legacy cut-probe requires explicit --bottom-up-start; full task starts upright')
 if a.mode=='open-probe' and not a.unsealed:p.error('open-probe requires --unsealed and cannot count as task completion')
 a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False);(a.out/'source').mkdir()
 for f in Path(__file__).parent.glob('*.py'):shutil.copy2(f,a.out/'source'/f.name)
 (a.out/'source-sha256.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in Path(__file__).parent.glob('*.py')},indent=2))
 os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
 from isaaclab.app import AppLauncher
 app=AppLauncher(headless=True,device=a.device,enable_cameras=not a.no_video).app;video=None
 try:
  import numpy as np,torch
  if not a.no_video:import cv2,imageio.v2 as imageio
  from yam_arena import inspect_embodiment as wrapper
  from yam_arena import scene as sc
  from yam_arena.urdf import quat_xyzw_to_matrix
  import isaaclab.sim as sim
  from native import export,install_joints
  from scoring import BreakdownScorer
  from yam_arena import ik
  from yam_arena.constants import FINGER_OPEN
  from pxr import UsdPhysics
  import task
  wrapper._APP=app;parts,hs=layout();byname={s['name']:s for s in parts};assets={s.name:export(a.out/(s.name+'.usda'),s.name,west_finger_notch=a.west_finger_notch) for s in task.OBJECTS if s.name!='cutter_pad'};original=sc.task_object;original_callback=sc.env_cfg_callback
  instrument_contacts=a.mode=='turnover-probe' or chained
  if instrument_contacts:
   def contact_callback(hz=None):
    configure_original=original_callback(hz)
    def configure(cfg):
     cfg=configure_original(cfg)
     if hasattr(cfg.sim,'disable_contact_processing'):cfg.sim.disable_contact_processing=False
     return cfg
    return configure
   sc.env_cfg_callback=contact_callback
  def make(spec):
   if spec.name not in assets:return original(spec)
   cfg=sim.UsdFileCfg(usd_path=assets[spec.name]);pos,q=starting_pose(byname[spec.name],upright=not a.bottom_up_start) if spec.name in byname else (spec.pos,(1,0,0,0))
   if spec.name=='cutter':
    spawn=cfg.func
    def with_joints(path,cfg,translation=None,orientation=None,**kwargs):
     prim=spawn(path,cfg,translation=translation,orientation=orientation,**kwargs);install_joints(prim.GetStage(),str(prim.GetPath()).rsplit('/',1)[0],sealed=not a.unsealed);return prim
    cfg.func=with_joints
   return sc.Object(name=spec.name,object_type=sc.ObjectType.RIGID,spawner_cfg=cfg,initial_pose=sc.Pose(position_xyz=pos,rotation_xyzw=(*q[1:],q[0])))
  from yam_arena.embodiment import BimanualYamEmbodiment
  import review
  import capture
  original_init=BimanualYamEmbodiment.__init__
  def with_reviews(instance,*args,**kwargs):
   capture.configure_embodiment(instance,args,kwargs,original_init,review,no_video=a.no_video)
   if instrument_contacts:
    from turnover_contacts import add_sensors
    add_sensors(instance)
  BimanualYamEmbodiment.__init__=with_reviews
  sc.task_object=make
  try:
   env=wrapper.YamArenaEmbodiment(device=a.device,task=str(Path(__file__).with_name('task.py')),object_jitter=0.,control_hz=20,headless=True,cam_width=320,cam_height=180);env._ensure_sim(None)
  finally:sc.task_object=original;sc.env_cfg_callback=original_callback;BimanualYamEmbodiment.__init__=original_init
  base=env._base
  from turnover_contacts import Reader
  contacts=Reader(base) if instrument_contacts else None
  capture.suppress_state_only_rendering(base,no_video=a.no_video)
  env._env.reset();env._instruction='Native carton mechanics probe';env._t=0.;env._turnover_contact_reader=contacts
  env._carton_elevated_pickup=a.elevated_pickup
  env._carton_blade_tilt=a.blade_tilt
  env._carton_light_tape_pressure=a.light_tape_pressure
  from native_angle_binding import install as install_angle_observer
  angle_observer=install_angle_observer(base)
  from native_angle_binding import PINS
  (a.out/'angle-tracking-reset.json').write_text(json.dumps(dict(version=__import__('angle_tracking').VERSION,source_pins=PINS,caps_deg_s=angle_observer.tracker.caps,initial_state=angle_observer.initial_state),indent=2)+'\n')
  if not a.no_video:review.place(base,(.32,0.,.82))
  from yam_arena.robot import asset_link_paths
  owners=capture.native_body_owners(base.scene,asset_link_paths(),[part['name'] for part in parts])
  for path in owners:
   if not base.scene.stage.GetPrimAtPath(path):raise RuntimeError('missing native capture owner '+path)
  view_cfg=sim.PinholeCameraCfg(focal_length=18.,clipping_range=(.03,20.)) if a.no_video else base.scene[review.NAMES[0]].cfg.spawn
  manifest=capture.capture_manifest(capture.export_scene(base.scene.stage,a.out/'capture-scene.usdc'),owners,
      json.loads((a.out/'source-sha256.json').read_text()),capture.virtual_review_views(view_cfg),no_video=a.no_video)
  (a.out/'capture-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
  (a.out/'mode.json').write_text(json.dumps({'mode':a.mode,'unsealed_start':a.unsealed,'state_only':a.no_video,'camera_observations':not a.no_video,'media_eligible':False,'control_hz':20,'cut_probe_max_control_steps':4800,'cut_probe_limit_s':240.},indent=2)+'\n')
  def arr(v):
   v=v.torch if hasattr(v,'torch') else v
   return v.detach().cpu().numpy()
  def pose(name):return arr(base.arena_world.get_pose_w(name))[0]
  video_name='scripted-demo.mp4' if full_chain else 'diagnostic.mp4'
  if not a.no_video:video=imageio.get_writer(str(a.out/video_name),fps=20,codec='libx264',pixelformat='yuv420p',macro_block_size=1,ffmpeg_params=['-movflags','+faststart'])
  worst=0.;minz=10.;bestflat=10.;robot=None
  if a.mode in ('grip-probe','cut-probe','open-probe','turnover-probe') or chained:
   from robot import CartonGripProbe,CartonCutProbe,CartonOpenProbe
   from hook_flap import CartonFlapHook
   from rim_pull import RimPull as ShiftOpenCarton
   from turnover import CartonTurnoverProbe
   if a.live_after_breakover:
    from cross_device_prefix import BreakoverHandoff as CartonTurnoverProbe
   elif a.cross_device_prefix:
    from cross_device_prefix import CartonTurnoverProbe
   elif action_prefix:
    from replay_turnover import CartonTurnoverProbe
   controller=CartonTurnoverProbe if chained else {'cut-probe':CartonCutProbe,'open-probe':CartonOpenProbe,'turnover-probe':CartonTurnoverProbe}.get(a.mode,CartonGripProbe)
   robot=controller(env,level_right_pad=a.level_right_pad,tip_lift=a.tip_lift,compliant_push=a.compliant_push) if controller is CartonTurnoverProbe else controller(env)
  tracker=BreakdownScorer(require_turnover=chained) if a.mode=='cut-probe' or chained else None;released=False;previous_positions=None;previous_cutter=None
  flap_order=('north','south','east','west')
  if a.west_first:flap_order=('west','north','south','east')
  if a.west_before_east:flap_order=('north','south','west','east')
  phase_started=0.;flattened_at=None
  if tracker is not None:task._tracker=tracker;task._diagnostic=False
  peak_load_roll_deg=0.;released_load=False
  with (a.out/'trajectory.jsonl').open('w') as trace:
   for step in range(240 if load_probe else 2400 if a.mode=='open-probe' else 12000 if full_chain else 6000 if chained else 4800 if a.mode=='cut-probe' else 1600 if a.mode=='turnover-probe' else 440 if robot else 180):
    phase='settle' if step<60 else 'wall_load' if step<120 else 'release'
    f=(0.,-.25,0.) if phase=='wall_load' and not robot and a.mode!='setup-check' else (0.,0.,0.)
    base.scene['east'].permanent_wrench_composer.set_forces_and_torques_index(forces=torch.tensor([[f]],device=base.device),torques=torch.zeros((1,1,3),device=base.device),is_global=True)
    if load_probe:
     # Standalone unscored torque/load diagnostic. Apply a known horizontal
     # push on the high wall and a measured-design low counterforce.
     right_force=float(np.clip((step-40)/60.,0.,1.)*1.8) if step<170 and not released_load else 0.
     left_force=-min(right_force,.30)
     phase='turnover_load' if right_force else 'settle_or_release'
     high_panel='south_top' if a.load_outer_flap else 'south'
     for name,wall,force_y,height in ((high_panel,'south',right_force,.9*HEIGHT),('north','north',left_force,.2*HEIGHT)):
      wall_pose=pose(wall)
      wall_rotation=quat_xyzw_to_matrix(wall_pose[3:])
      force=np.array((0.,force_y,0.))
      point=wall_pose[:3]+wall_rotation[:,2]*(height-HEIGHT/2.)
      lever=point-pose(name)[:3]
      moment=np.cross(lever,force)
      base.scene[name].permanent_wrench_composer.set_forces_and_torques_index(
       forces=torch.tensor(force[None,None,:],device=base.device,dtype=torch.float32),
       torques=torch.tensor(moment[None,None,:],device=base.device,dtype=torch.float32),is_global=True)
    control=None
    if robot:
     target,control=robot.command(step*.05-phase_started);phase=control[0]['phase'];action=torch.tensor(target[None,:],device=base.device,dtype=torch.float32)
    else:action=env._ready
    if robot is not None and getattr(robot,'failure',None):
     (a.out/'rejected-command.json').write_text(json.dumps({'time':env._t,'control':control,'failure':robot.failure},indent=2)+'\n');break
    angle_observer.begin_control()
    env._env.step(action);env._t=(step+1)*.05
    if len(base._reset_env_ids):angle_observer.tracker.invalidate('native episode reset')
    angle_evidence=angle_observer.end_control()
    if not angle_evidence['valid']:raise RuntimeError(angle_evidence['failure'])
    pp={s['name']:pose(s['name']) for s in parts};rr={n:quat_xyzw_to_matrix(v[3:]) for n,v in pp.items()};sep=0.
    angles={}
    for h in hs:
     pa=pp[h['a']][:3]+rr[h['a']]@np.array(h['anchor_a']);pb=pp[h['b']][:3]+rr[h['b']]@np.array(h['anchor_b']);sep=max(sep,float(np.linalg.norm(pa-pb)))
     rel=rr[h['a']].T@rr[h['b']]
     raw=float(np.degrees(np.arctan2(rel[2,1],rel[1,1]) if h['axis']=='X' else np.arctan2(rel[1,0],rel[0,0])))
     angles[h['name']]=float(angle_observer.tracker.angles[['north','south','east','west'].index(h['a'])]) if h['b'].endswith('_bottom') else raw
    vertices=[]
    for s in parts:
     for x in [-1,1]:
      for y in [-1,1]:
       for z in [-1,1]:vertices.append(pp[s['name']][:3]+rr[s['name']]@(.5*np.array(s['size'])*np.array((x,y,z))))
    vertices=np.array(vertices);low=float(vertices[:,2].min()-TOP);high=float(vertices[:,2].max()-TOP);worst=max(worst,sep);minz=min(minz,low);bestflat=min(bestflat,high-low)
    native_hands=[]
    for side in ['left','right']:
     ids,fids=env._ids[side+'_robot'];qnative=arr(base.scene[side+'_robot'].data.joint_pos)[0];tcpnative,rnative=ik.tcp_pose(qnative[ids],env.rig.bases[side]);native_hands.append({'side':side,'joints':qnative[ids].tolist(),'finger_joints':qnative[fids].tolist(),'tcp':tcpnative.tolist(),'tcp_rotation':rnative.tolist(),'opening':float(np.clip(qnative[fids].mean()/FINGER_OPEN,0,1))})
    row={'time':env._t,'phase':phase,'robot_control':control,'action_14d':arr(action)[0].tolist(),'native_hands':native_hands,'cutter_pose':pose('cutter').tolist(),'poses':{n:v.tolist() for n,v in pp.items()},'angles_deg':angles,'max_crease_separation_m':sep,'lowest_vertex_m':low,'highest_vertex_m':high,'force_N':f}
    if load_probe:
     up=rr['north'][:,2]
     lean=float(np.degrees(np.arctan2(up[1],up[2])))
     peak_load_roll_deg=max(peak_load_roll_deg,lean)
     released_load|=lean>55.
     row['turnover_load']={'right_N':right_force,'left_N':left_force,'roll_deg':lean,'high_panel':high_panel}
    if contacts is not None:
     row['fingertip_carton_normal_forces']=contacts.read(env._t)
     from turnover_effort import read_current
     row['current_motor_state']=read_current(base,env,env._t)
    row['angle_evidence']=angle_evidence
    row['angle_substeps']=angle_observer.samples
    row['native_owner_poses']=capture.native_body_world_poses(base.scene,owners,{**pp,'cutter':row['cutter_pose']})
    if tracker is not None:
     cutter=pose('cutter');cr=quat_xyzw_to_matrix(cutter[3:]);tip=cutter[:3]+cr@np.array((.062,0,-.020));reference=rr['north'];origin=pp['north'][:3]-reference@np.array((0,WIDTH/2,HEIGHT/2));localtip=reference.T@(tip-origin);hands=[];held=False
     for side in ['left','right']:
      ids,fids=env._ids[side+'_robot'];q=arr(base.scene[side+'_robot'].data.joint_pos)[0];tcp,rotation=ik.tcp_pose(q[ids],env.rig.bases[side]);opening=float(np.clip(q[fids].mean()/FINGER_OPEN,0,1));hands.append(tcp);held|=np.linalg.norm(tcp-cutter[:3])<.035 and .08<opening<.38
     bottom=[angles[n+'_bottom_hinge'] for n in ['north','south','east','west']];top=[(-angles[n+'_top_hinge'])%360 for n in ['north','south','east','west']];top=[0. if x>350 else x for x in top]
     speed=0. if previous_positions is None else max(float(np.linalg.norm(pp[n][:3]-previous_positions[n])/ .05) for n in pp);cspeed=0. if previous_cutter is None else float(np.linalg.norm(cutter[:3]-previous_cutter)/.05);previous_positions={n:p[:3].copy() for n,p in pp.items()};previous_cutter=cutter[:3].copy();vmin=vertices.min(0);vmax=vertices.max(0)
     sample={'body_up_dot':float(reference[2,2]),'angle_evidence':angle_evidence,'time':env._t,'blade_tip_local':localtip.tolist(),'blade_alignment':float(-np.dot(cr[:,2],reference[:,2])),'cutter_held':bool(held),'bottom_angles':bottom,'top_angles':top,'crease_separation':sep,'min_height':low,'max_height':high,'carton_speed':speed,'cutter_height':float(cutter[2]-TOP),'cutter_parked':bool(np.linalg.norm(cutter[:3]-np.array(CUTTER_START))<.05 and cspeed<.02),'hands_clear':all(np.linalg.norm(h-np.clip(h,vmin,vmax))>.07 and np.linalg.norm(h-cutter[:3])>.065 for h in hands)}
     row['score_sample']=sample;row['metrics']=tracker.update(sample);row['cutter_pose']=cutter.tolist()
     if tracker.coverage.complete and not released:
      for name in ['north','south']:
       joint=UsdPhysics.RevoluteJoint(base.scene.stage.GetPrimAtPath(f'/World/envs/env_0/{name}_bottom/{name}_bottom_hinge'));parameters=joint_parameters(next(h for h in hs if h['name']==name+'_bottom_hinge'));joint.GetLowerLimitAttr().Set(parameters['low']);joint.GetUpperLimitAttr().Set(parameters['high'])
      released=True
    trace.write(json.dumps(row)+'\n')
    if not np.isfinite(vertices).all() or sep>.03:raise RuntimeError('unstable carton/crease failure')
    if not a.no_video:
     panels=review.panels(base);canvas=np.zeros((594,960,3),dtype=np.uint8);canvas[36:306,240:720]=panels[0];canvas[306:576,:480]=panels[1];canvas[306:576,480:]=panels[2]
     cv2.putText(canvas,('CARTON ROBOT TASK | ' if robot else 'NATIVE CARTON | ')+phase+(' | unsealed diagnostic' if a.unsealed else ' | sealed bottom'),(12,25),cv2.FONT_HERSHEY_SIMPLEX,.57,(255,255,255),1,cv2.LINE_AA);cv2.putText(canvas,(('Measured score: '+str(tracker.value) if tracker else 'Ordinary robot commands. Rim grip and lift diagnostic.') if robot else 'Free rigid panels and physical creases. Force diagnostic.'),(12,590),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,205,120),1,cv2.LINE_AA);video.append_data(canvas)
     if full_chain and tracker.value==1.:imageio.imwrite(a.out/'score-1.png',canvas)
     if step in (59,119,179,279,399,599,799,999,1199):imageio.imwrite(a.out/f'step-{step}.png',canvas)
    if opening_prefix and isinstance(robot,CartonTurnoverProbe) and robot.done:
     if not (tracker.turnover_complete and released and row['score_sample']['cutter_parked']):
      raise RuntimeError('opening replay lacks measured turnover, tape release or cutter parking')
     if a.reposition_carton:
      robot=ShiftOpenCarton(env,arr(action)[0])
     else:
      first_live_flap='west' if a.third_flap_action_prefix else flap_order[1 if a.first_flap_action_prefix else 0]
      robot=CartonFlapHook(env,arr(action)[0],flap=first_live_flap,finger_notch=a.west_finger_notch)
     phase_started=env._t
    elif a.reposition_carton and isinstance(robot,ShiftOpenCarton) and robot.done:
     robot=CartonFlapHook(env,arr(action)[0],flap='west');phase_started=env._t
    elif chained and isinstance(robot,CartonTurnoverProbe) and robot.done:
     if not tracker.turnover_complete:raise RuntimeError('controller turnover handoff lacks measured inverted hold')
     robot=CartonCutProbe(env);robot.previous=arr(action)[0].copy();phase_started=env._t
    elif opens and isinstance(robot,CartonCutProbe) and robot.done and robot.failure is None:
     if not released or not row['score_sample']['cutter_parked']:raise RuntimeError('cut-to-open handoff requires measured tape release and parked cutter')
     if a.tapered_flap_hook:
      robot=CartonFlapHook(env,arr(action)[0],flap=flap_order[0],finger_notch=a.west_finger_notch);phase_started=env._t
     else:
      robot=CartonOpenProbe(env);robot.previous=arr(action)[0].copy();phase_started=env._t
    elif opens and isinstance(robot,CartonFlapHook) and robot.done and robot.failure is None:
     next_index=flap_order.index(robot.flap)+1
     if next_index<len(flap_order):
      robot=CartonFlapHook(env,arr(action)[0],flap=flap_order[next_index],finger_notch=a.west_finger_notch);phase_started=env._t
     else:
      if not tracker.opened or not row['score_sample']['cutter_parked']:
       raise RuntimeError('flattening handoff requires measured opened flaps and parked cutter')
      if not full_chain:break
      from translate_and_flatten import TranslateAndFlatten as CartonFlattenProbe
      robot=CartonFlattenProbe(env,arr(action)[0]);phase_started=env._t
    elif full_chain and isinstance(robot,CartonOpenProbe) and robot.index>=len(robot.order):
     if not tracker.opened or not row['score_sample']['cutter_parked']:
      raise RuntimeError('flattening handoff requires measured opened flaps and parked cutter')
     from collapse import CartonFlattenProbe
     robot=CartonFlattenProbe(env,arr(action)[0]);phase_started=env._t
    elif full_chain and getattr(robot,'done',False) and robot.failure is None:
     if flattened_at is None:flattened_at=env._t
     if tracker.invalid or tracker.value==1. or env._t-flattened_at>=3.:break
    elif robot is not None and (getattr(robot,"failure",None) or getattr(robot,"done",False)):break
    if robot is not None and a.mode in ('open-probe','turn-cut-open-probe') and isinstance(robot,CartonOpenProbe) and robot.index>=len(robot.order):break
  result={'diagnostic':True,'task_score':None,'unsealed_start':a.unsealed,'max_crease_separation_m':worst,'lowest_vertex_m':minz,'smallest_height_span_m':bestflat,'checks':{'creases_intact':worst<.004,'above_table':minz>-.004}}
  if a.mode=='turnover-probe':result|={'controller_failure':robot.failure,'turnover_completed':robot.done,'final_roll_angle_deg':robot.angle}
  if load_probe:result|={'peak_load_roll_deg':peak_load_roll_deg,'applied_external_forces':True,'high_load_panel':high_panel}
  if a.mode=='open-probe':result|={'controller_failure':robot.failure,'flaps_completed':robot.index,'final_angles_deg':angles}
  if tracker is not None:
   from last_cell import missing_cells
   result['missing_cut_cells']=missing_cells(tracker.coverage)
  if tracker is not None:result|={'diagnostic':False,'task_score':tracker.value,'privileged_robot_attempt':True,'metrics':tracker.result(),'bottom_up_start':a.bottom_up_start,'controller_failure':robot.failure,'controller_completed_cut_attempt':released,'flaps_completed':sum(v>=150. for n,v in angles.items() if n.endswith('_bottom_hinge'))}
  if video is not None:video.close();video=None
  eligible=bool(full_chain and not a.no_video and tracker.value==1. and tracker.invalid is None
                and robot.done and robot.failure is None and (a.out/video_name).stat().st_size>0
                and (a.out/'score-1.png').is_file())
  result|={'state_only':a.no_video,'media_eligible':eligible,'capture_manifest':'capture-manifest.json'}
  if eligible:result|={'video':video_name,'success_screenshot':'score-1.png'}
  if a.no_video:result|={'diagnostic':True,'diagnostic_reason':'camera-free engineering run; measured scorer values retained, no final media'}
  (a.out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
 except BaseException as error:
  import traceback;traceback.print_exc();(a.out/'error.json').write_text(json.dumps({'error':str(error),'type':type(error).__name__},indent=2));raise
 finally:
  if video is not None:video.close()
  app.close()
if __name__=='__main__':main()
