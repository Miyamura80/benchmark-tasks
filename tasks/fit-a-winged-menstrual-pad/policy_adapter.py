"""Dream's native YAM camera and proprioceptive input; no task-body state is exposed."""
from collections import deque
from pathlib import Path
import importlib
import sys
import time
import numpy as np
import mujoco as mj
from dream_examples.yam_mujoco import yam_env


def sdk():
    # The pinned Dream example imports its sibling modules by their short names.
    path = str(Path(yam_env.__file__).parent)
    if path not in sys.path:
        sys.path.insert(0, path)
    return importlib.import_module('dream_control')


class Observation:
    def __init__(self, trial):
        self.trial = trial
        self.model = trial.model
        self.renderer = mj.Renderer(self.model, height=480, width=640)

    def state(self):
        result = {}
        for side, arm in self.trial.arms.items():
            q = self.trial.data.qpos[arm.positions]
            aperture = self.trial.data.qpos[arm.finger_q].sum()/.095
            result[side] = np.r_[q, np.clip(aperture, 0, 1)]
        return result

    def images(self):
        images = {}
        for side in ('top','left','right'):
            self.renderer.update_scene(self.trial.data, camera=side)
            images[side] = self.renderer.render()[60:420].copy()
        return images

    def close(self):
        self.renderer.close()


def run(trial, instruction, limit, dry_run=False):
    control = sdk()
    env = Observation(trial)
    windows = {name:deque(maxlen=control.FRAMES) for name in ('top','left','right')}
    client = None; calls = []; restarts=[]
    try:
        for step in range((control.FRAMES-1)*control.FRAME_EVERY_STEPS+1):
            if step%control.FRAME_EVERY_STEPS == 0:
                control.capture(env, windows)
            if step < (control.FRAMES-1)*control.FRAME_EVERY_STEPS:
                trial.action([trial.last[s] for s in ('left','right')])
        checkpoint = control.BETA_MODEL
        control.observation(env, windows, instruction, checkpoint, station=True)
        if dry_run:
            return {'model_called':False,'history_frames':control.FRAMES,'state_shapes':{s:list(x.shape) for s,x in env.state().items()}}
        config = control.settings()
        api_key = config.get('DREAM_API_KEY')
        if not api_key:raise RuntimeError('Configure the Dream SDK credential for this environment')
        client = control.open_policy(control.policy_config(checkpoint, config.get('DREAM_GATEWAY_URL','https://api.dreamlabs.ai'), True),api_key)
        steps = len(trial.commands)
        control.feed(client,env,windows,instruction,checkpoint,steps,every=0,station=True)
        expiry_count=0
        while steps < limit:
            started=time.monotonic()
            try:reply=client.get_action()
            except Exception as exc:
                if type(exc).__name__=='DreamSessionExpired' and expiry_count<3:
                    expiry_count+=1;restarts.append({'step':steps,'reason':'DreamSessionExpired'});client.close();client=control.open_policy(control.policy_config(checkpoint,config.get('DREAM_GATEWAY_URL','https://api.dreamlabs.ai'),True),api_key);control.feed(client,env,windows,instruction,checkpoint,steps,every=0,station=True);continue
                raise
            expiry_count=0
            left,right=control.validate_chunk(env,reply,min(8,limit-steps),verbose=False)
            calls.append({'latency_s':time.monotonic()-started,'executed':len(left)})
            for rows in zip(left,right):
                physical=np.array(rows);physical[:,6]*=.0475
                trial.action(physical);steps+=1
                if steps%control.FRAME_EVERY_STEPS==0:
                    control.capture(env,windows)
                    control.feed(client,env,windows,instruction,checkpoint,steps,every=1,station=True)
        if not calls:raise RuntimeError('No Dream inference response was executed')
        return {'model_called':True,'model':checkpoint,'steps':steps,'calls':calls,'session_restarts':restarts}
    finally:
        if client is not None: client.close()
        env.close()
