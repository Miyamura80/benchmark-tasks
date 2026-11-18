"""Score a real Dream-1 rollout on the same dual-YAM fixture as the scripted demo."""

import argparse
import json
from pathlib import Path

import mujoco as mj
import numpy as np
from dream_examples.yam_mujoco.yam_env import YamEnv
from inspect_robots import (
    ActionSemantics,
    Box,
    CameraSpec,
    EmbodimentInfo,
    Observation,
    ObservationSpace,
    Scene,
    Task,
    eval,
)
from inspect_robots.embodiment import RENDERABLE
from inspect_robots.scorer import Score
from inspect_robots.types import StepResult
from PIL import Image

from recording import add_streams, add_water_surface
from yam_trial import HZ, MAX_STEPS, SIDES, YamToilet, model

CAMERAS = {"top_cam": "top", "left_cam": "left", "right_cam": "right"}
WIDTH, HEIGHT = 320, 176
INSTRUCTIONS = {
    "rim": "Press the blue test button to find the weak toilet rim jet. Pick up the orange "
    "scraper, remove the scale in the weak outlet and take the tool clear. Press the button "
    "again to check that all four jets flow.",
    "trap": "Press the blue test button to test drainage. Pick up the orange-handled plunger, "
    "seat its cup over the toilet drain and pump it until the clog clears. Lift the cup "
    "clear of the bowl and press the blue button again to check drainage.",
}


class ToiletYamEmbodiment:
    def __init__(self, kind, blocked):
        self.kind, self.blocked = kind, blocked
        self.fixture_model = model(kind, blocked)
        joint_ranges = np.array(
            [
                self.fixture_model.joint(f"{side}_joint{i}").range
                for side in SIDES
                for i in range(1, 7)
            ]
        )
        low = np.insert(joint_ranges[:, 0], [6, 12], 0.0)
        high = np.insert(joint_ranges[:, 1], [6, 12], 1.0)
        self.renderer = None
        self.info = EmbodimentInfo(
            name=f"yam-sim-toilet-{kind}",
            action_space=Box(
                (14,),
                low=low,
                high=high,
                semantics=ActionSemantics(
                    control_mode="joint_pos", gripper="continuous"
                ),
            ),
            observation_space=ObservationSpace(
                cameras=tuple(CameraSpec(name, HEIGHT, WIDTH) for name in CAMERAS),
                state_keys=frozenset({"joint_pos"}),
            ),
            control_hz=HZ,
            is_simulated=True,
            capabilities=frozenset({RENDERABLE}),
            morphology="dual-YAM",
            environment_id=f"{kind}-mujoco-3.3.7",
        )

    def reset(self, scene, *, seed=None):
        self.close()
        self.sim = YamToilet(self.kind, self.blocked, fixture_model=self.fixture_model)
        self.renderer = mj.Renderer(self.fixture_model, height=240, width=320)
        self.instruction = scene.instruction
        return self.observe()

    def observe(self):
        images = {}
        for name, camera in CAMERAS.items():
            self.renderer.update_scene(self.sim.data, camera=camera)
            scene = self.renderer.scene
            scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = False
            add_streams(self.sim, scene)
            add_water_surface(self.sim, scene)
            frame = Image.fromarray(self.renderer.render()[30:210])
            images[name] = np.asarray(
                frame.resize((WIDTH, HEIGHT), Image.Resampling.BILINEAR)
            )
        state = YamEnv.state(self.sim)
        return Observation(
            images=images,
            state={"joint_pos": np.concatenate([state[side] for side in SIDES])},
            instruction=self.instruction,
            state_time=float(self.sim.data.time),
        )

    def step(self, action):
        self.sim.action(np.asarray(action.data, dtype=float).reshape(2, 7))
        result = self.sim.summary()
        passed = result["score"] == 1.0
        return StepResult(
            self.observe(),
            terminated=passed,
            termination_reason="verified repair" if passed else None,
            info=result,
        )

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None


class FixtureScorer:
    name = "task_score"

    def __call__(self, record, target):
        if not record.steps:
            return Score(0.0, "No fixture activity recorded")
        result = record.steps[-1].result.info
        return Score(
            result["score"], "Final physical state and completed tests", result
        )


def main():
    from inspect_robots_dream.policy import DreamPolicy

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=("rim", "trap"))
    parser.add_argument("--blocked", type=int, choices=range(4), default=1)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    embodiment = ToiletYamEmbodiment(args.kind, args.blocked)
    task = Task(
        f"toilet-yam-{args.kind}",
        [Scene("nominal", INSTRUCTIONS[args.kind])],
        scorer=FixtureScorer(),
        max_steps=MAX_STEPS,
    )
    try:
        with DreamPolicy(timeout_s=60.0) as policy:
            logs = eval(
                task,
                policy,
                embodiment,
                log_dir=str(args.out),
                seed=0,
                store_frames=True,
                fail_on_error=True,
            )
            (args.out / "dream-transcript.json").write_text(
                json.dumps(policy.transcript(), indent=2) + "\n"
            )
        if not logs or any(log.status != "success" for log in logs):
            raise RuntimeError(
                "Dream-1 trial did not finish. Check the evaluation log."
            )
    finally:
        embodiment.close()


if __name__ == "__main__":
    main()
