"""Record a fresh scripted trial using the same reset, action and score entry points as Dream."""

import argparse
import json
import struct
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
from inspect_robots import Scene
from inspect_robots.types import Action
from yam_arena import ik
from yam_arena.constants import READY, FINGER_OPEN
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from carton_seal.layout import ZONES, RIM_Y, HEIGHT, HINGE_Y

ROOT = Path(__file__).resolve().parent


def fingertip_local(R):
    from yam_arena import ik, YAM_URDF
    from yam_arena.constants import READY, ARM_JOINTS
    from yam_arena.urdf import forward_kinematics, rpy_matrix

    fk = forward_kinematics(
        YAM_URDF, {**dict(zip(ARM_JOINTS, READY)), "joint7": -0.003, "joint8": -0.003}
    )
    tcp, frame = ik.tcp_pose(READY, np.zeros(3))
    meshpoints = []
    for link in ET.parse(YAM_URDF).getroot().findall("link"):
        if link.get("name") not in ("tip_left", "tip_right"):
            continue
        F = fk[link.get("name")]
        for c in link.findall("collision"):
            mesh = c.find("geometry/mesh")
            origin = c.find("origin")
            buf = (Path(YAM_URDF).parent / mesh.get("filename")).read_bytes()
            pts = np.ndarray(
                struct.unpack_from("<I", buf, 80)[0],
                dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                buffer=buf,
                offset=84,
            )["v"].reshape(-1, 3)
            scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
            p = (pts * scale) @ rpy_matrix(
                np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
            ).T + np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            meshpoints.append(((p @ F[:3, :3].T + F[:3, 3]) - tcp) @ frame)
    v = np.concatenate(meshpoints)
    h = v @ R[2]
    return np.mean(v[h < h.min() + 0.00025], axis=0)


def run(env, args, out, recorder):
    scene = Scene(
        "prepared-mailer", json.loads((ROOT / "task.json").read_text())["instruction"]
    )
    first = env.reset(scene, seed=0)
    recorder.frame(first)
    monitor = env._mailer_contacts
    pose, pads, bonds, broken = (
        monitor.pose,
        monitor.pads,
        monitor.bonds,
        monitor.broken,
    )
    localtab = np.array([0, RIM_Y - HINGE_Y + 0.132, -0.001275])
    info = {"success": False, "score": 0.0}
    actions = []
    az, tilt = np.radians([args.yaw, args.lean])
    z = np.array([np.cos(az) * np.sin(tilt), np.sin(az) * np.sin(tilt), -np.cos(tilt)])
    x = np.array([np.sin(az), -np.cos(az), 0.0])
    rot = np.column_stack((x, np.cross(z, x), z))
    point_local = fingertip_local(rot)
    goal_quat = ik.matrix_to_quat_xyzw(rot)

    def solve(point, seed):
        for init in [seed, READY]:
            q, pe, re = ik.solve(
                point - rot @ point_local,
                goal_quat,
                np.asarray(init),
                base_pos=env.rig.bases["right"],
                iters=150,
                tol=1e-6,
            )
            if pe < 0.0004 and re < 0.004:
                return q
        raise ValueError("press waypoint unreachable " + str(point))

    action = np.asarray([*READY, 1.0, *READY, 1.0], dtype=float)
    qtarget = action[7:13].copy()
    phase = "settle"
    began = 0.0
    cursor = 0
    order = [2, 1, 0, 3, 4]
    failure = None
    history = []
    if args.control == "one-missing":
        order = order[:-1]

    def change(nextphase, t):
        nonlocal phase, began, q0, depth
        phase = nextphase
        began = t
        q0 = action[7:13].copy()
        depth = 0.0
        history.append([round(t, 3), phase])

    q0 = qtarget.copy()
    depth = 0.0
    nextq = qtarget.copy()
    for k in range(1500):
        t = k * 0.05
        age = t - began
        i = order[min(cursor, len(order) - 1)]
        C, U = pose("carton")
        lp, LR = pose("lid")
        r = U.T @ LR
        angle = float(np.degrees(np.arctan2(-r[2, 1], r[1, 1])))
        lip, T = pose(f"lip_{i}")
        back = lip + T @ (localtab + [0, 0, 0.0024])
        rim = C + U @ [ZONES[i], RIM_Y, HEIGHT]
        up = U[:, 2]
        try:
            if args.control == "no-action":
                if t >= 4.0:
                    break
            elif phase == "settle" and age >= 0.65:
                if angle < 12.0 or bonds:
                    raise ValueError("initial condition failed")
                change("hover", t)
            elif phase == "hover":
                if age <= 0.051:
                    qtarget = solve(back + up * 0.022, q0)
                f = ik.smoothstep(age / 3.0)
                nextq = q0 + (qtarget - q0) * f
                if age > 3.4:
                    change("close", t)
            elif phase == "close":
                point = back.copy()
                point[2] = max(
                    rim[2] + 0.002 - args.depth, back[2] + 0.012 - 0.023 * age
                )
                nextq = solve(point, nextq)
                if abs(angle) <= 4.0:
                    change("press", t)
                elif age > 7.0:
                    raise ValueError("lid did not close under pressure")
            elif phase == "press":
                depth = float(
                    np.clip(depth + 0.00030 * (args.force - pads[i]), 0.0, args.depth)
                )
                point = back.copy()
                point[2] = rim[2] + 0.0022 - depth
                nextq = solve(point, nextq)
                if (i in bonds and i not in broken) or (
                    args.control == "no-adhesive" and age > 1.5
                ):
                    cursor += 1
                    change("release" if cursor == len(order) else "index-up", t)
                elif age > 6.0:
                    raise ValueError(f"no verified adhesive pressure at zone {i}")
            elif phase == "index-up":
                oldp, oldR = pose(f"lip_{order[cursor - 1]}")
                nextq = solve(oldp + oldR @ (localtab + [0, 0, 0.018]), nextq)
                if age > 0.8:
                    change("index-over", t)
            elif phase == "index-over":
                nextq = solve(back + up * 0.018, nextq)
                if age > 1.4:
                    change("press", t)
            elif phase == "release":
                if age < 0.051:
                    qtarget = solve(back + up * 0.055, q0)
                f = ik.smoothstep(age / 1.2)
                nextq = q0 + (qtarget - q0) * f
                if age > 1.45:
                    change("park", t)
            elif phase == "park":
                f = ik.smoothstep(age / 2)
                nextq = q0 + (np.array(READY) - q0) * f
                if age > 6.0:
                    break
        except ValueError as e:
            failure = str(e)
            break
        if args.control != "no-action":
            action[7:13] += np.clip(nextq - action[7:13], -0.038, 0.038)
            action[13] += np.clip(-0.003 / FINGER_OPEN - action[13], -0.048, 0.048)

        result = env.step(Action(action.copy()))
        recorder.frame(result.observation)
        actions.append(action.tolist())
        info = result.info
        if result.terminated:
            break
    (out / "actions.json").write_text(json.dumps(actions))
    report = dict(
        control=args.control,
        info=info,
        steps=len(actions),
        hold_s=monitor.hold.duration,
        bond_events=monitor.events,
        broken=sorted(broken),
        active=sorted(set(bonds) - broken),
        failure=failure,
        stages=history,
    )
    (out / "result.json").write_text(json.dumps(report, indent=2))
    return report


class Recorder:
    def __init__(self, folder):
        self.folder = folder
        self.encoder = None

    def frame(self, observation):
        import subprocess
        from PIL import Image

        image = np.ascontiguousarray(observation.images["top_cam"], dtype=np.uint8)
        if self.encoder is None:
            height, width = image.shape[:2]
            Image.fromarray(image).save(self.folder / "setup.png")
            self.encoder = subprocess.Popen(
                [
                    "ffmpeg",
                    "-loglevel",
                    "error",
                    "-y",
                    "-f",
                    "rawvideo",
                    "-pix_fmt",
                    "rgb24",
                    "-s",
                    f"{width}x{height}",
                    "-r",
                    "20",
                    "-i",
                    "-",
                    "-c:v",
                    "libx264",
                    "-pix_fmt",
                    "yuv420p",
                    "-movflags",
                    "+faststart",
                    str(self.folder / "demo.mp4"),
                ],
                stdin=subprocess.PIPE,
            )
        self.encoder.stdin.write(image.tobytes())

    def close(self):
        if self.encoder is not None:
            self.encoder.stdin.close()
            if self.encoder.wait() != 0:
                raise RuntimeError("Video encoding failed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--control",
        choices=("robot", "no-action", "no-adhesive", "one-missing"),
        default="robot",
    )
    parser.add_argument("--depth", type=float, default=0.008)
    parser.add_argument("--force", type=float, default=3.5)
    parser.add_argument("--yaw", type=float, default=34.0)
    parser.add_argument("--lean", type=float, default=31.0)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    env = YamArenaEmbodiment(
        task=str(ROOT / "task.py"), device=args.device, control_hz=20, object_jitter=0
    )
    env._mailer_adhesive = args.control != "no-adhesive"
    recorder = Recorder(args.out)
    try:
        report = run(env, args, args.out, recorder)
    finally:
        try:
            recorder.close()
        finally:
            env.close()
    print(json.dumps(report, indent=2))
    if bool(report["info"]["score"]) != (args.control == "robot"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
