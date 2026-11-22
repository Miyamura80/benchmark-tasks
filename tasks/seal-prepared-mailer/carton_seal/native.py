from contextlib import ExitStack, contextmanager
from unittest.mock import patch
import numpy as np
from .layout import parts, ZONES, RIM_Y, HEIGHT, HINGE_Y
from .score import ReleaseHold


def array(v):
    v = getattr(v, "torch", v)
    return np.asarray(v.detach().cpu().numpy() if hasattr(v, "detach") else v)


@contextmanager
def build_scene(out, task):
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab_arena.utils.configclass import make_configclass
    import isaaclab.sim as sim
    from yam_arena import ik, scene as sc
    from yam_arena.robot import asset_link_paths
    from . import assets

    geometry = parts()
    files = {n: assets.export(out, n) for n in geometry}
    owned_specs = {id(spec) for spec in task.objects}
    saved = sc.task_object
    original_make = sc.make_env

    def object_cfg(spec):
        if id(spec) not in owned_specs or spec.name not in files:
            return saved(spec)
        p, R, _ = geometry[spec.name]
        spawn = sim.UsdFileCfg(usd_path=files[spec.name], activate_contact_sensors=True)
        if spec.name == "lip_4":
            old_spawn = spawn.func

            def final_spawn(path, cfg, translation=None, orientation=None, **kw):
                prim = old_spawn(
                    path, cfg, translation=translation, orientation=orientation, **kw
                )
                assets.install(prim.GetStage(), str(prim.GetPath()).rsplit("/", 1)[0])
                return prim

            spawn.func = final_spawn
        return sc.Object(
            name=spec.name,
            object_type=sc.ObjectType.RIGID,
            spawner_cfg=spawn,
            initial_pose=sc.Pose(
                position_xyz=tuple(map(float, p)),
                rotation_xyzw=tuple(map(float, ik.matrix_to_quat_xyzw(R))),
            ),
        )

    def make_env(instance, *args, **kwargs):
        if kwargs.get("task") is not task:
            return original_make(instance, *args, **kwargs)
        cfg = instance.scene_config
        fields = [
            (n, type(getattr(cfg, n)), getattr(cfg, n))
            for n in cfg.__class__.__annotations__
        ]
        tips = asset_link_paths()["tip_left"].rsplit("/", 1)[0] + "/tip_.*"
        for i in range(5):
            for label, body, target in [
                ("finger", "RightArm" + tips, f"lip_{i}"),
                ("flange", f"lip_{i}", "carton"),
            ]:
                sen = ContactSensorCfg(
                    prim_path="{ENV_REGEX_NS}/" + body,
                    update_period=0.0,
                    track_pose=False,
                    filter_prim_paths_expr=["{ENV_REGEX_NS}/" + target],
                )
                fields.append((label + str(i), ContactSensorCfg, sen))
        instance.scene_config = make_configclass("PreparedMailerSensorConfig", fields)()
        return original_make(instance, *args, **kwargs)

    with ExitStack() as stack:
        stack.enter_context(patch.object(sc, "task_object", object_cfg))
        stack.enter_context(patch.object(sc, "make_env", make_env))
        yield


class Contacts:
    def __init__(self, env, adhesive=True):
        from . import assets
        from yam_arena.urdf import quat_xyzw_to_matrix

        base = env._base
        self.hold = ReleaseHold(float(base.step_dt))
        self.env = env
        stage = base.scene.stage
        parent = "/World/envs/env_0"
        prefix = parent + "/"

        def pose(n):
            b = array(base.arena_world.get_pose_w(n))[0]
            return b[:3].copy(), quat_xyzw_to_matrix(b[3:])

        def get_force(label, i):
            sensor = base.scene[label + str(i)]
            mat = array(sensor.data.force_matrix_w)
            if mat.shape != (1, 2 if label == "finger" else 1, 1, 3) or not np.all(
                np.isfinite(mat)
            ):
                raise RuntimeError("unexpected filtered force shape " + str(mat.shape))
            return mat[0, :, 0, :]

        for i in range(5):
            for label, target in [("finger", "lip_" + str(i)), ("flange", "carton")]:
                get_force(label, i)
                sensor = base.scene[label + str(i)]
                if label == "finger" and set(sensor.body_names) != {
                    "tip_left",
                    "tip_right",
                }:
                    raise RuntimeError("wrong native pad binding")
                paths = sensor.contact_view.filter_paths
                if not all(
                    str(x) == prefix + target
                    for row in paths
                    for x in ([row] if isinstance(row, str) else row)
                ):
                    raise RuntimeError("wrong target contact binding")
        localtab = np.array([0, RIM_Y - HINGE_Y + 0.132, -0.001275])
        times = np.zeros(5)
        bonds = {}
        broken = set()
        pads = np.zeros(5)
        rims = np.zeros(5)
        peak_pad = pads.copy()
        peak_rim = rims.copy()
        events = []
        clock = 0.0
        original_update = base.scene.update

        def sample_contact(dt, *a, **kw):
            nonlocal clock
            result = original_update(dt, *a, **kw)
            clock += dt
            C, U = pose("carton")
            for i, x in enumerate(ZONES):
                lip, R = pose(f"lip_{i}")
                anchor = np.array([x, RIM_Y, HEIGHT])
                point = lip + R @ localtab
                rim = C + U @ anchor
                gap = np.linalg.norm(point - rim)
                pads[i] = abs(get_force("finger", i) @ U[:, 2]).sum()
                rims[i] = abs(get_force("flange", i) @ U[:, 2]).sum()
                peak_pad[i] = max(peak_pad[i], pads[i])
                peak_rim[i] = max(peak_rim[i], rims[i])
                if i in bonds:
                    if gap > 0.0025 or bonds[i].GetJointEnabledAttr().Get() is False:
                        bonds[i].GetJointEnabledAttr().Set(False)
                        broken.add(i)
                    continue
                good = gap < 0.0017 and pads[i] >= 0.55 and rims[i] >= 0.35
                times[i] = times[i] + dt if good else 0.0
                if times[i] >= 0.1 - 1e-8 and adhesive:
                    bonds[i] = assets.bond(stage, parent, i, anchor, localtab)
                    events.append(
                        dict(
                            time=clock,
                            cell=i,
                            pad_N=pads[i],
                            flange_N=rims[i],
                            gap_m=gap,
                        )
                    )
            return result

        self.original_update = original_update
        self.bonds = bonds
        self.broken = broken
        self.pads = pads
        self.pose = pose
        self.events = events
        self.stage = stage
        self.parent = parent
        base.scene.update = sample_contact

    def close(self):
        base = self.env._base
        base.scene.update = self.original_update
        for i in self.bonds:
            self.stage.RemovePrim(self.parent + f"/lip_{i}/activated_adhesive")

    def observe(self):
        from yam_arena import ik

        env = self.env
        pose = self.pose
        C, U = pose("carton")
        _, lid = pose("lid")
        rel = U.T @ lid
        angle = float(np.degrees(np.arctan2(-rel[2, 1], rel[1, 1])))
        clear = True
        for side in ("left", "right"):
            ids, _ = env._ids[side + "_robot"]
            q = array(env._base.scene[side + "_robot"].data.joint_pos)[0, ids]
            tcp, _ = ik.tcp_pose(q, env.rig.bases[side])
            hand = U.T @ (tcp - C)
            clear = clear and (
                hand[2] > HEIGHT + 0.065 or abs(hand[1]) > 0.145 / 2 + 0.09
            )
        inside = True
        for name, size in [
            ("blue_product", [0.064, 0.044, 0.025]),
            ("rust_product", [0.055, 0.044, 0.038]),
        ]:
            p, R = pose(name)
            center = U.T @ (p - C)
            half = np.abs(U.T @ R) @ (np.array(size) / 2)
            inside = (
                inside
                and np.all(
                    np.abs(center[:2]) + half[:2]
                    < [0.220 / 2 - 0.002, 0.145 / 2 - 0.002]
                )
                and center[2] - half[2] >= 0.002
                and center[2] + half[2] < HEIGHT
            )
        self.hold.update(
            env._t, C, angle, set(self.bonds) - self.broken, inside, clear, self.pads
        )
