"""Execute this task with Inspect Robots, YAM and the task_score grader."""

import argparse
import json
import os
import sys
from pathlib import Path

from yam_arena import inspect_embodiment
from yam_arena.urdf import quat_xyzw_to_matrix

import task


from native import Contacts, root_states, substitute_assets
from fixtures import author


def bind(path, directory, setup, case):
    cls = inspect_embodiment.YamArenaEmbodiment
    original_reset, original_step, control_step = cls.reset, cls._task_step, cls.step
    native = substitute_assets({"bowl_fixture", "angled_brush"}, lambda: author(directory))
    native.original = cls._ensure_sim
    cls._ensure_sim = native

    def control(self, action):
        if current(self) and hasattr(self, "_fixture_contacts"):
            self._fixture_contacts.take()
        return control_step(self, action)

    cls.step = control

    def current(instance):
        return instance.task is not None and instance.task.path == path

    def reset(self, *args, **kwargs):
        observation = original_reset(self, *args, **kwargs)
        if not current(self):
            return observation
        if not hasattr(self, "_fixture_contacts"):
            self._fixture_contacts = Contacts({"BrushBristles", "BrushHandle"})
        self._fixture_contacts.take()
        self._fixture_state = task.State(case)
        self._fixture_view = task.View(self._fixture_state)
        self._fixture_passes = 0
        self._fixture_trace = (directory / "mechanism.jsonl").open("w", encoding="utf-8")
        for _ in range(8):
            self._base.sim.render()
        for name in ("top_cam", "left_cam", "right_cam"):
            self._base.scene[name].update(self._base.step_dt, force_recompute=True)
        if setup:
            import cv2

            pixels = self._base.scene["top_cam"].data.output["rgb"]
            pixels = pixels.torch if hasattr(pixels, "torch") else pixels
            cv2.imwrite(str(setup), cv2.cvtColor(pixels[0, ..., :3].cpu().numpy(), cv2.COLOR_RGB2BGR))
        return self._observation()

    def step(self):
        if not current(self):
            return original_step(self)
        from inspect_robots.types import StepResult

        state = self._fixture_state
        brush = root_states(self, ["angled_brush"])["angled_brush"]
        info = state.advance(brush, quat_xyzw_to_matrix(brush.quat_xyzw),
                             self._fixture_contacts.take())
        self._fixture_view.update(state)
        self._fixture_passes = self._fixture_passes + 1 if info["score"] == 1.0 else 0
        success = self._fixture_passes >= 5
        info["success"] = success
        self._fixture_trace.write(json.dumps({"t": round(self._t, 4), **info}, allow_nan=False) + "\n")
        self._fixture_trace.flush()
        return StepResult(observation=self._observation(), terminated=success,
                          termination_reason="success" if success else None, info=info)

    cls.reset, cls._task_step = reset, step


def main():
    from inspect_robots.cli import main as inspect_main

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("policy", choices=("yam_scripted", "dream"))
    parser.add_argument("--log-dir", required=True, type=Path)
    parser.add_argument("--setup", type=Path)
    parser.add_argument("--case", default="nominal", choices=task.CASES)
    args, rest = parser.parse_known_args()
    args.log_dir.mkdir(parents=True, exist_ok=True)
    path = Path(task.__file__).resolve()
    os.environ["ROBOCURVE_FIXTURE_CASE"] = args.case
    task.configure_demo(args.case)
    bind(path, args.log_dir, args.setup, args.case)
    sys.argv[1:] = [
        "run", "--embodiment", "yam_arena", "--policy", args.policy,
        "-E", f"task={path}", "-E", "object_jitter=0", "-E", "control_hz=20",
        "-E", "cam_width=320", "-E", "cam_height=176",
        "--instruction", task.INSTRUCTION, "--scorer", "task_score",
        "--max-steps", str(task.MAX_STEPS), "--store-frames", "--log-dir", str(args.log_dir), *rest,
    ]
    inspect_main()


if __name__ == "__main__":
    main()
