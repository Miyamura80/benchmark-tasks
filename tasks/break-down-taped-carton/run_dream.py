"""Run Dream-1 with the verified native scene and unchanged measured scorer.

The original run.py stays byte-identical to the verified demonstration. This
driver replaces only policy dispatch, instruction, run termination and output
labels. It never invokes scripted control or an action prefix in a model trial.
"""
from pathlib import Path


def model_source():
    source = Path(__file__).with_name('run.py').read_text()

    def replace(old, new):
        nonlocal source
        if source.count(old) != 1:
            raise RuntimeError('Native runner changed; inspect the model integration: '+old[:80])
        source = source.replace(old, new)

    replace(" a=p.parse_args()", " a=p.parse_args()\n if a.mode!='scripted-demo-probe' or a.no_video or a.unsealed or a.bottom_up_start or any((a.turnover_action_prefix,a.cut_action_prefix,a.first_flap_action_prefix,a.third_flap_action_prefix)):\n  p.error('Dream trials require a fresh sealed, upright, filmed full task with no scripted prefix')")
    replace("cam_width=320,cam_height=180", "cam_width=320,cam_height=176")
    replace("  if a.mode in ('grip-probe','cut-probe','open-probe','turnover-probe') or chained:", "  if False: # no scripted policy is constructed in a Dream trial")
    replace("  tracker=BreakdownScorer", "  from dream_controller import NativeDream\n  robot=NativeDream(env,a.out)\n  tracker=BreakdownScorer")
    replace("    if opening_prefix and isinstance(robot,CartonTurnoverProbe) and robot.done:", "    if tracker.invalid or tracker.value==1.:break\n    if step%100==0:print('Dream native step',step+1,'score',tracker.value,flush=True)\n    continue # model owns every action; never enter scripted handoffs\n    if opening_prefix and isinstance(robot,CartonTurnoverProbe) and robot.done:")
    replace("'privileged_robot_attempt':True", "'privileged_robot_attempt':False,'model':'Dream-1'")
    replace("  app.close()", "  if 'robot' in locals() and hasattr(robot,'close'):robot.close()\n  app.close()")
    replace("  video_name='scripted-demo.mp4' if full_chain else 'diagnostic.mp4'", "  video_name='model-run.mp4'")
    replace("  worst=0.;minz=10.;bestflat=10.;robot=None", "  for _ in range(4):base.sim.render()\n  panels=review.panels(base);setup=np.zeros((594,960,3),dtype=np.uint8);setup[36:306,240:720]=panels[0];setup[306:576,:480]=panels[1];setup[306:576,480:]=panels[2]\n  cv2.putText(setup,'SEALED CARTON START',(12,25),cv2.FONT_HERSHEY_SIMPLEX,.57,(255,255,255),1,cv2.LINE_AA)\n  imageio.imwrite(a.out/'setup.png',setup)\n  worst=0.;minz=10.;bestflat=10.;robot=None;angles={}")
    return source


if __name__=='__main__':
    exec(compile(model_source(),str(Path(__file__).with_name('run.py')),'exec'),
         {'__name__':'__main__','__file__':str(Path(__file__).with_name('run.py'))})
