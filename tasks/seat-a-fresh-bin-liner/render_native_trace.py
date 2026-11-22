"""Render the film and measured YAM joint states from a native cuff trace."""
import argparse
import json
from pathlib import Path
import subprocess

import mujoco
import numpy as np

from native_cuff import scene_xml
from probe import model
from render_yam_replay import HEIGHT, MOUNTS, WIDTH, label_frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("yam_xml", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--poster", type=Path)
    parser.add_argument("--mode", default="pinch")
    args = parser.parse_args()
    trace = json.loads(args.trace.read_text())
    run = next(run for run in trace["results"] if run["mode"] == args.mode)
    frames = run["frames"]
    playback_fps = "12"
    if args.mode == "dream":
        # Play every other 20 Hz observation at real-time speed.
        frames = frames[::2] + frames[-1:] if len(frames) % 2 == 0 else frames[::2]
        playback_fps = "10"
    mount_x = run.get("mount_x")
    m, data, puck, _, puck_geom = model(
        True, lambda xml: scene_xml(xml, args.yam_xml, run.get("stiff_pads", False), mount_x),
    )
    uncuffed = run.get("scope") == "seat_uncuffed_liner"
    vertices = m.flex_vertbodyid.copy()
    rest = data.flexvert_xpos.copy()
    coords = m.jnt_qposadr[m.body_jntadr[vertices]][:, None] + np.arange(3)
    puck_qpos = m.jnt_qposadr[m.body_jntadr[puck]]
    puck_alpha = m.geom_rgba[puck_geom, 3]
    joint_indices = []
    for prefix, *_ in MOUNTS:
        names = [f"joint{i}" for i in range(1, 7)] + ["left_finger", "right_finger"]
        joint_indices.append([m.joint(prefix + name).qposadr[0] for name in names])
    camera = mujoco.MjvCamera()
    camera.lookat[:] = (.04 if uncuffed else .17, 0., .29 if uncuffed else .28)
    camera.azimuth = 148
    camera.elevation = -25
    camera.distance = 1.42
    opt = mujoco.MjvOption()
    opt.geomgroup[3:] = 0
    opt.sitegroup[:] = 0
    command = [
        "ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{WIDTH}x{HEIGHT}", "-r", playback_fps, "-i", "-", "-an", "-c:v", "libx264",
        "-preset", "fast", "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        str(args.output),
    ]
    with mujoco.Renderer(m, height=HEIGHT, width=WIDTH) as renderer, subprocess.Popen(
            command, stdin=subprocess.PIPE) as encoder:
        for i, frame in enumerate(frames):
            points = np.asarray(frame["points"])
            data.qpos[coords] = points - rest
            for joints, qpos in zip(joint_indices, frame["arm_qpos"], strict=True):
                data.qpos[joints] = qpos
            m.geom_rgba[puck_geom, 3] = 0. if frame["puck"] is None else puck_alpha
            if frame["puck"] is not None:
                data.qpos[puck_qpos:puck_qpos + 3] = frame["puck"]
            mujoco.mj_forward(m, data)
            renderer.update_scene(data, camera=camera, scene_option=opt)
            if i == len(frames) - 1:
                phase = "Cuff held through load" if run["post_load_retained"] else "End of load test"
                if "score" in run:
                    phase = f"{sum(run['retained_sectors'])}/12 retained | Score {run['score']:.3f}"
            elif frame["puck"] is not None:
                phase = "100 g puck test"
            else:
                phase = frame["phase"]
            image = label_frame(
                renderer.render(), frame["time"], phase, sum(frame["sectors"]),
                subtitle=("Actuated YAM arms  ·  Contact-assisted grips  ·  Uncuffed reset"
                          if uncuffed else "Native YAM joint and film trace  ·  Partially fitted reset"),
                title="Seat a fresh bin liner around the rim" if uncuffed else "Fit a fresh bin liner",
            )
            if args.poster is not None and i == round(len(frames) * .26):
                image.save(args.poster, quality=88)
            encoder.stdin.write(image.tobytes())
        encoder.stdin.close()
        if encoder.wait():
            raise RuntimeError("FFmpeg could not encode the native replay")


if __name__ == "__main__":
    main()
