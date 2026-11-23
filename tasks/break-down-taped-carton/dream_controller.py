"""Dream-1 sees only native policy cameras, measured joints and the instruction."""
import json
import time
from pathlib import Path

INSTRUCTION = ('Break down the empty taped carton for recycling: turn it upside down, '
               'cut the bottom seam and both taped end folds with the guarded cutter, '
               'park the cutter on its pad, open all four bottom flaps, and leave '
               'the intact carton flat on the table with both hands clear.')


class NativeDream:
    def __init__(self, env, out):
        import numpy as np
        from inspect_robots import Scene
        from inspect_robots_dream import DreamPolicy
        self.env, self.out = env, Path(out)
        self.done, self.failure = False, None
        self.started = time.monotonic()
        self.steps = 0
        env._instruction = INSTRUCTION
        self.policy = DreamPolicy(rows_per_request=48)
        try:
            self.policy.bind(env.info)
            self.policy.reset(Scene(id='sealed-carton', instruction=INSTRUCTION))
        except Exception as error:
            self.failure = type(error).__name__ + ': ' + str(error)
        self._save()

    def _save(self):
        transcript = self.policy.transcript()
        transcript.pop('server_url', None)
        for request in transcript.get('requests', []):
            request.pop('session_id', None)
        transcript.update(executed_control_steps=self.steps,
                          elapsed_wall_seconds=time.monotonic()-self.started,
                          failure=self.failure)
        (self.out/'dream-transcript.json').write_text(json.dumps(transcript, indent=2)+'\n')

    def command(self, t):
        import numpy as np
        observation = self.env._observation()
        try:
            if self.failure:
                raise RuntimeError(self.failure)
            chunk = self.policy.act(observation)
            if len(chunk.actions) != 1:
                raise ValueError('Dream adapter must return one action per native control step')
            target = np.asarray(chunk.actions[0].data, dtype=float)
            if target.shape != (14,) or not np.isfinite(target).all():
                raise ValueError('Dream returned an invalid 14D action')
            target = np.clip(target, self.env._low, self.env._high)
            self.steps += 1
        except Exception as error:
            self.failure = type(error).__name__ + ': ' + str(error)
            target = np.asarray(observation.state['joint_pos'], dtype=float)
        self._save()
        return target, [{'phase':'Dream-1', 'model_control_step':self.steps}]

    def close(self):
        self._save()
        self.policy.close()
