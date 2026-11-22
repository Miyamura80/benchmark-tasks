"""Evaluate Dream-1 on the same uncuffed-liner reset and load scorer."""
import argparse
from functools import partial
import json
import os
from pathlib import Path
import re

import mujoco as mj
import numpy as np

from native_cuff import CLOSED, OPEN, run
from probe import FACES, LIP_RADIUS, WALL_HEIGHT
from seat_liner import prepare_reset


INSTRUCTION = "Fold the loose bin liner over the entire wastebasket rim. Let go with the opening clear."
HZ = 20
RUN_LIMIT = 360
CAMERAS = {"top_cam": (90, -82, .91), "left_cam": (55, -24, .72), "right_cam": (-55, -24, .72)}


class JointPolicy:
    def __init__(self, policy, paths, steps):
        self.policy = policy
        self.paths = paths
        self.steps = steps
        self.count = 0
        self.last = None
        self.renderer = None
        release = steps / HZ
        self.evaluation_times = release, round(release + .8, 6), round(release + 2.2, 6)

    def start(self, model, state, arms):
        from inspect_robots import ActionSemantics, Box, CameraSpec, EmbodimentInfo, ObservationSpace, Scene

        self.arms = arms
        limits = model.actuator_ctrlrange.copy()
        ids = np.array([arm.servo_ids for arm in arms])
        self.low = limits[ids, 0]
        self.high = limits[ids, 1]
        # The demonstrated reset permits the ordinary 37.5 mm jaw travel.
        self.opening = float(limits[arms[0].gripper, 1])
        low = np.column_stack([self.low, [0., 0.]]).flatten()
        high = np.column_stack([self.high, [1., 1.]]).flatten()
        info = EmbodimentInfo(
            name="seat-a-fresh-bin-liner", morphology="dual-YAM",
            action_space=Box((14,), low=low, high=high, semantics=ActionSemantics(
                control_mode="joint_pos", gripper="continuous")),
            observation_space=ObservationSpace(cameras=tuple(CameraSpec(name, 176, 320) for name in CAMERAS),
                                               state_keys=frozenset({"joint_pos"})),
            control_hz=HZ, is_simulated=True, environment_id="seat-liner-mujoco",
        )
        self.renderer = mj.Renderer(model, width=320, height=176)
        self.option = mj.MjvOption()
        self.option.geomgroup[3:] = 0
        self.option.sitegroup[:] = 0
        self.cameras = {}
        for name, (azimuth, elevation, distance) in CAMERAS.items():
            camera = mj.MjvCamera()
            camera.lookat[:] = (0., 0., .235)
            camera.azimuth, camera.elevation, camera.distance = azimuth, elevation, distance
            self.cameras[name] = camera
        self.policy.bind(info)
        self.policy.reset(Scene("uncuffed", INSTRUCTION))

    def command(self, time, state):
        from inspect_robots import Observation

        if self.count >= self.steps:
            return self.last
        images = {}
        for name, camera in self.cameras.items():
            self.renderer.update_scene(state, camera=camera, scene_option=self.option)
            self.renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = False
            images[name] = self.renderer.render().copy()
        joints = np.array([state.qpos[arm.qpos] for arm in self.arms])
        measured_jaw = np.array([state.qpos[arm.fingers[0]] for arm in self.arms])
        normalized_jaw = ((measured_jaw - CLOSED) / (self.opening - CLOSED)).clip(0., 1.)
        measured = np.column_stack([joints, normalized_jaw]).flatten()
        observation = Observation(images=images, state={"joint_pos": measured},
                                  instruction=INSTRUCTION, state_time=time)
        result = self.policy.act(observation)
        action = np.asarray(result.actions[0].data, dtype=float).reshape(2, 7)
        if not np.isfinite(action).all():
            raise ValueError("Dream-1 produced a non-finite joint command.")
        output = action.copy()
        output[:, :6] = output[:, :6].clip(self.low, self.high)
        output[:, -1] = CLOSED + (self.opening - CLOSED) * output[:, -1].clip(0., 1.)
        with self.paths.joinpath("actions.jsonl").open("a") as stream:
            stream.write(json.dumps({"time_s": time, "action": action.tolist()}) + "\n")
        self.count += 1
        self.last = output
        return output

    def close(self):
        if self.renderer is not None:
            self.renderer.close()


def model_error_message(error):
    message = str(error)
    key = os.environ.get("DREAM_API_KEY")
    if key:
        message = message.replace(key, "[credential redacted]")
    return re.sub(r"dl_live_[A-Za-z0-9_-]+", "[credential redacted]", message)


def main():
    from inspect_robots_dream.policy import DreamPolicy

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yam-xml", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=RUN_LIMIT)
    args = parser.parse_args()
    if args.steps < 80 or args.steps > RUN_LIMIT:
        parser.error("Set steps between 80 and the declared action limit.")
    args.output.mkdir(parents=True)
    with DreamPolicy(timeout_s=40.) as policy:
        controller = JointPolicy(policy, args.output, args.steps)
        try:
            result = run(args.yam_xml, mode="assisted", orientation="tangent", film_mass=.0036,
                         timestep=.00125, grip_scale=.22, pairs=((24, 8),),
                         configure_reset=partial(prepare_reset, mount_x=.02),
                         require_uncuffed=True, joint_policy=controller)
        except Exception as error:
            report = dict(error_type=type(error).__name__, error=model_error_message(error),
                          returned_commands=controller.count, transcript=policy.transcript())
            (args.output / "error.json").write_text(json.dumps(report, indent=2) + "\n")
            raise SystemExit(report["error"])
        finally:
            controller.close()
        transcript = policy.transcript()
    if not transcript.get("requests"):
        raise RuntimeError("No Dream-1 inference request was completed.")
    result.update(mode="dream", scope="seat_uncuffed_liner", mount_x=.02)
    release, drop, _ = controller.evaluation_times
    trace = dict(results=[result], faces=FACES.tolist(), wall_height=WALL_HEIGHT, lip_radius=LIP_RADIUS,
                 release_s=release, drop_s=drop)
    (args.output / "trace.json").write_text(json.dumps(trace, separators=(",", ":")))
    (args.output / "transcript.json").write_text(json.dumps(transcript, indent=2) + "\n")
    print(json.dumps({"score": result["score"], "actions": controller.count}))


if __name__ == "__main__":
    main()
