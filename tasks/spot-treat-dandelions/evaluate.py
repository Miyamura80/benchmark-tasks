"""Run the Dream-1 policy against the same simulation and native scorer as run.py."""

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

from lawn import Lawn
from film import Film
from demo import save_images
from yam_station import HZ, MAX_STEPS, SIDES, Station

WIDTH, HEIGHT = 320, 176
CAMERAS = {"top_cam": "survey", "left_cam": "left", "right_cam": "right"}


class WorkbenchEmbodiment:
    def __init__(self, fixture, recorder=None):
        self.fixture = fixture
        self.model = fixture.compile()
        self.recorder = recorder
        ranges = np.array(
            [
                self.model.joint(f"{side}_joint{i}").range
                for side in SIDES
                for i in range(1, 7)
            ]
        )
        self.info = EmbodimentInfo(
            name="yam-sim-household-workbench",
            action_space=Box(
                (14,),
                low=np.insert(ranges[:, 0], [6, 12], 0.0),
                high=np.insert(ranges[:, 1], [6, 12], 1.0),
                semantics=ActionSemantics(
                    control_mode="joint_pos", gripper="continuous"
                ),
            ),
            observation_space=ObservationSpace(
                cameras=tuple(CameraSpec(key, HEIGHT, WIDTH) for key in CAMERAS),
                state_keys=frozenset({"joint_pos"}),
            ),
            control_hz=HZ,
            is_simulated=True,
            capabilities=frozenset({RENDERABLE}),
            morphology="dual-YAM",
            environment_id="lawn-mujoco-3.3.7",
        )
        self.renderer = None

    def reset(self, scene, *, seed=None):
        if self.renderer is not None:
            self.renderer.close()
        self.sim = Station(self.fixture, model=self.model, render=self.recorder)
        self.sim.phase = "Dream-1 policy"
        self.renderer = mj.Renderer(self.model, height=240, width=WIDTH, max_geom=3500)
        self.scene_option = mj.MjvOption()
        self.scene_option.geomgroup[5] = 0
        self.instruction = scene.instruction
        return self.observe()

    def observe(self):
        images = {}
        for name, camera in CAMERAS.items():
            self.renderer.update_scene(
                self.sim.data, camera=camera, scene_option=self.scene_option
            )
            self.renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = False
            images[name] = self.renderer.render()[32:208].copy()
        state = YamEnv.state(self.sim)
        return Observation(
            images=images,
            state={"joint_pos": np.concatenate([state[side] for side in SIDES])},
            instruction=self.instruction,
            state_time=float(self.sim.data.time),
        )

    def step(self, action):
        self.sim.action(np.asarray(action.data, dtype=float).reshape(2, 7))
        result = self.sim.reading()
        return StepResult(
            self.observe(),
            terminated=False,
            termination_reason=None,
            info=result,
        )

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None


class NativeScorer:
    name = "task_score"

    def __call__(self, record, target):
        if not record.steps:
            return Score(0.0, "No robot actions recorded")
        reading = record.steps[-1].result.info
        return Score(reading["score"], "Visible spray deposition at trial end", reading)


def main():
    from inspect_robots_dream.policy import DreamPolicy

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--variant", type=int, choices=[0], default=0)
    parser.add_argument(
        "--steps",
        type=int,
        choices=range(1, MAX_STEPS + 1),
        default=MAX_STEPS,
        metavar="1..600",
    )
    parser.add_argument("--video", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    fixture = Lawn(args.variant)
    instruction = json.loads(Path(__file__).with_name("task.json").read_text())[
        "instruction"
    ]
    recorder = None
    if args.video:
        recorder = Film(args.out / "model-run.mp4")
    experiment = WorkbenchEmbodiment(fixture, recorder)
    task = Task(
        "spot-treat-dandelions",
        [Scene("nominal", instruction)],
        scorer=NativeScorer(),
        max_steps=args.steps,
    )
    try:
        with DreamPolicy(timeout_s=60.0) as policy:
            logs = eval(
                task,
                policy,
                experiment,
                log_dir=str(args.out),
                seed=args.variant,
                store_frames=False,
                fail_on_error=True,
            )
            if not logs or any(log.status != "success" for log in logs):
                raise RuntimeError("Dream-1 evaluation did not finish")
            reading = experiment.sim.reading()
            transcript = policy.transcript()
    finally:
        experiment.close()
        if recorder and hasattr(recorder, "proc"):
            recorder.finish()

    # Publish completed evidence only after the policy and encoder succeed.
    (args.out / "commands.json").write_text(json.dumps(experiment.sim.commands) + "\n")
    (args.out / "trace.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in experiment.sim.trace)
    )
    (args.out / "transcript.json").write_text(json.dumps(transcript, indent=2) + "\n")
    save_images(fixture, args.out)
    reading.update(
        title="Spot-treat dandelions", variant=args.variant, run_limit=args.steps
    )
    (args.out / "result.json").write_text(json.dumps(reading, indent=2) + "\n")


if __name__ == "__main__":
    main()
