"""Run this task with the pinned YAM embodiment and a task-local paint overlay."""
import json
import os
import sys
from pathlib import Path

import numpy as np

from yam_arena import inspect_embodiment
from yam_arena import ik


from practice_hand import install, update_polish
from native import Contacts, root_states, asset, collider, substitute_assets


def stage():
    import omni.usd

    return omni.usd.get_context().get_stage()


def install_applicator_asset(directory):
    from pxr import Gf, UsdGeom

    path = directory / "polish_applicator.usda"
    path.unlink(missing_ok=True)
    model = asset(path, .023)
    handle = UsdGeom.Cube.Define(model, "/Object/ApplicatorHandle")
    handle.CreateSizeAttr(1.)
    handle.AddScaleOp().Set(Gf.Vec3f(.027, .024, .038))
    handle.CreateDisplayColorAttr([Gf.Vec3f(.78, .04, .13)])
    collider(handle, friction=1.)
    foam = UsdGeom.Cylinder.Define(model, "/Object/PolishTip")
    foam.CreateRadiusAttr(.0028)
    foam.CreateHeightAttr(.004)
    foam.AddTranslateOp().Set(Gf.Vec3d(0., 0., -.021))
    foam.CreateDisplayColorAttr([Gf.Vec3f(.65, .025, .05)])
    collider(foam)
    model.GetRootLayer().Save()
    return {"polish_applicator": path}


def enable_paint(task_path):
    cls = inspect_embodiment.YamArenaEmbodiment
    reset, step, control_step = cls.reset, cls._task_step, cls.step
    import tempfile

    directory = tempfile.TemporaryDirectory(prefix="practice-nail-")
    ensure = substitute_assets({"polish_applicator"}, lambda: install_applicator_asset(Path(directory.name)))
    ensure.original = cls._ensure_sim
    cls._ensure_sim = ensure

    def clear_contacts(self, action):
        if paint_task(self) is not None and hasattr(self, "_paint_contacts"):
            self._paint_contacts.take()
        return control_step(self, action)

    cls.step = clear_contacts

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
        if not hasattr(self, "_paint_contacts"):
            self._paint_contacts = Contacts({"PolishTip"})
        self._paint_contacts.take()
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
        brush = root_states(self, [module.TOOL.name])[module.TOOL.name]
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
        tip_contacts = self._paint_contacts.take()
        # The practice finger proxy is also paintable so a stroke that
        # touches adjacent skin can accrue spill. A hovering tip earns zero.
        target_shapes = {"MiddleNail", "middle_finger_proxy"}
        collision = any(
            pair & target_shapes and impulse >= module.MIN_CONTACT_IMPULSE_NS
            for pair, _, impulse in tip_contacts
        )
        contact = upright and holding and collision
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
    # Raise the pickup for the slotted dock. Lower the painting pass so the
    # native foam collider can make contact with the curved nail.
    from yam_arena import station

    retargeted = clip.copy()
    base = station.BASES["left"]
    for i, action in enumerate(clip):
        pickup = float(np.clip(min((i-42)/15, (104-i)/17), 0., 1.))
        stroke = float(np.clip((i-173)/23, 0., 1.))
        dz = .004*ik.smoothstep(pickup) - .0045*ik.smoothstep(stroke)
        if dz == 0.:
            continue
        start, rotation = ik.tcp_pose(action[:6], base)
        q, position_error, rotation_error = ik.solve(
            start + (0.,0.,dz), ik.matrix_to_quat_xyzw(rotation),
            action[:6], base_pos=base, iters=95,
        )
        if position_error > .001 or rotation_error > .025:
            raise ValueError(f"Applicator pose misses IK tolerance at action {i}")
        retargeted[i,:6] = q
    clip = retargeted
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
