"""Run this task with the pinned YAM embodiment and a task-local paint overlay."""
import json
import os
import sys
from pathlib import Path

import numpy as np

from yam_arena import inspect_embodiment
from yam_arena import ik


from practice_hand import install, update_polish


def stage():
    import omni.usd

    return omni.usd.get_context().get_stage()


def enable_paint(task_path):
    cls = inspect_embodiment.YamArenaEmbodiment
    reset, step = cls.reset, cls._task_step

    def paint_task(self):
        task = self.task
        if task is not None and task.path == task_path:
            return task.module
        return None

    def reset_with_paint(self, *args, **kwargs):
        observation = reset(self, *args, **kwargs)
        module = paint_task(self)
        if module is None:
            return observation
        module.paint = module.Paint()
        install(stage(), module)
        # Flush the USD mesh and visibility changes through RTX before reading the
        # reset observation. Rendering alone leaves the camera's cached frame stale.
        for _ in range(8):
            self._base.sim.render()
        for name in ("top_cam", "left_cam", "right_cam"):
            self._base.scene[name].update(self._base.step_dt, force_recompute=True)
        screenshot = os.environ.get("NAIL_SETUP_PNG")
        if screenshot:
            import cv2

            rgb = self._base.scene["top_cam"].data.output["rgb"]
            rgb = rgb.torch if hasattr(rgb, "torch") else rgb
            cv2.imwrite(screenshot, cv2.cvtColor(rgb[0, ..., :3].cpu().numpy(), cv2.COLOR_RGB2BGR))
        return self._observation()

    def step_with_paint(self):
        module = paint_task(self)
        if module is None:
            return step(self)
        from yam_arena.scene import object_states

        brush = object_states(self._env, [module.TOOL.name])[module.TOOL.name]
        qx, qy, qz, qw = brush.quat_xyzw
        axis = np.array([
            2 * (qx * qz + qw * qy),
            2 * (qy * qz - qw * qx),
            1 - 2 * (qx * qx + qy * qy),
        ])
        tip = brush.pos - axis * module.TOOL_REACH
        upright = axis[2] >= 0.985
        holding = False
        for arm in ("left", "right"):
            arm_ids, fingers = self._ids[arm + "_robot"]
            joints = self._base.scene[arm + "_robot"].data.joint_pos.torch[0]
            base = self.rig.bases[arm]
            tcp, _ = ik.tcp_pose(joints[arm_ids].cpu().numpy(), base)
            if np.linalg.norm(tcp - brush.pos) < 0.028 and abs(joints[fingers].mean().item()) < 0.030:
                holding = True
        contact = upright and holding and abs(tip[2] - module.SURFACE) <= module.CONTACT_TOLERANCE
        if os.environ.get("NAIL_TRACE") == "1":
            dx, dy = (tip[:2] - module.XY) * 1000
            dz = (tip[2] - module.SURFACE) * 1000
            print(f"NAIL_TRACE {self._t:.3f} {dx:.3f} {dy:.3f} {dz:.3f} {int(holding)} {int(upright)} {int(contact)}")
        if module.paint.step(tip[:2], contact):
            update_polish(stage(), module)
        result = step(self)
        covered, spilled = module.paint.metrics()
        result.info.update(nail_coverage=covered, skin_spill=spilled)
        if result.terminated or os.environ.get("NAIL_TRACE") == "1":
            print(f"NAIL_METRICS {self._t:.3f} {covered:.6f} {spilled:.6f}")
        return result

    cls.reset = reset_with_paint
    cls._task_step = step_with_paint


def enable_replay(task_path):
    from inspect_robots.types import Action, ActionChunk
    from yam_arena.scripted import YamScriptedPolicy

    clip = np.asarray(json.loads(task_path.with_name("scripted_actions.json").read_text()), dtype=np.float64)
    if clip.ndim != 2 or clip.shape[1] != 14 or len(clip) == 0 or not np.isfinite(clip).all():
        raise ValueError("Expected a nonempty sequence of finite 14-axis joint targets")
    reset, act = YamScriptedPolicy.reset, YamScriptedPolicy.act

    def is_this_task(policy):
        return policy._task is not None and policy._task.path == task_path

    def start(policy, scene):
        if is_this_task(policy):
            policy._clip_index = 0
        else:
            reset(policy, scene)

    def replay(policy, observation):
        if not is_this_task(policy):
            return act(policy, observation)
        index = policy._clip_index
        policy._clip_index += 1
        action = Action(
            data=clip[min(index, len(clip) - 1)],
            meta={"request_stop": True} if index >= len(clip) + 20 else {},
        )
        return ActionChunk(actions=[action], control_hz=1.0 / policy._dt)

    YamScriptedPolicy.reset = start
    YamScriptedPolicy.act = replay


def main():
    from inspect_robots.cli import main as inspect_main

    task = Path(__file__).with_name("paint_nail.py").resolve()
    enable_paint(task)
    args = sys.argv[1:]
    if not args or args.pop(0) not in ("dream", "yam_scripted"):
        raise SystemExit("Usage: python run_task.py {dream|yam_scripted} [Inspect Robots run flags]")
    policy = sys.argv[1]
    if policy == "yam_scripted":
        enable_replay(task)
    sys.argv[1:] = [
        "run", "--embodiment", "yam_arena", "--policy", policy,
        "-E", f"task={task}", "-E", "object_jitter=0", "-E", "control_hz=20",
        "-E", "cam_width=320", "-E", "cam_height=176",
        "--instruction", "Paint the middle fingernail on the silicone practice hand red. Keep the silicone around that nail clean.",
        "--scorer", "task_score", "--max-steps", "2400", "--store-frames",
        *args,
    ]
    inspect_main()


if __name__ == "__main__":
    main()
