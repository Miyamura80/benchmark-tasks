"""Replay measured film shapes with inverse-kinematic I2RT YAM meshes."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from probe import SEGMENTS, model, ease


WIDTH, HEIGHT = 1120, 630
FPS = 24
SLOWDOWN = 2.0
MOUNTS = (("left_", .33, -.39, 130), ("right_", .33, .39, -130))
FONTS = Path("/usr/share/fonts/truetype/dejavu")


def scene_with_arms(xml, stock_file):
    root = ET.fromstring(xml)
    stock = ET.parse(stock_file).getroot()
    stock_dir = stock_file.parent / stock.find("compiler").attrib["meshdir"]
    root.append(deepcopy(stock.find("default")))
    assets = deepcopy(stock.find("asset"))
    for mesh in assets.findall("mesh"):
        source = Path(mesh.attrib["file"])
        mesh.set("name", source.stem)
        mesh.set("file", str(stock_dir / source))
    root.append(assets)
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth=str(WIDTH), offheight=str(HEIGHT))
    ET.SubElement(visual, "headlight", diffuse=".7 .7 .7", ambient=".54 .54 .54")
    ET.SubElement(visual, "rgba", haze=".14 .16 .16 1")
    world = root.find("worldbody")
    ET.SubElement(world, "geom", type="box", size=".70 .70 .035", pos=".14 0 -.037",
                  rgba=".40 .43 .45 1", group="1", contype="0", conaffinity="0")
    ET.SubElement(world, "geom", type="plane", pos="0 0 -.12", size="3 3 .01",
                  rgba=".20 .22 .25 1", group="1", contype="0", conaffinity="0")
    ET.SubElement(world, "light", pos="1 -.3 1.75", dir="-1 .2 -1", diffuse=".55 .55 .55")
    ET.SubElement(world, "light", pos="-.4 .8 1.25", dir=".4 -.8 -1", diffuse=".35 .35 .35")
    world.find("geom[@name='base']").set("rgba", ".20 .25 .27 1")
    for prefix, x, y, yaw in MOUNTS:
        arm = deepcopy(stock.find("worldbody/body"))
        arm.set("pos", f"{x} {y} 0")
        arm.set("euler", f"0 0 {np.deg2rad(yaw)}")
        for item in arm.iter():
            if item.get("name"):
                item.set("name", prefix + item.get("name"))
            if item.tag == "geom" and item.get("class") != "visual":
                item.set("contype", "0")
                item.set("conaffinity", "0")
        world.append(arm)
    return ET.tostring(root, encoding="unicode")


class ArmReplay:
    def __init__(self, m, prefix, sign):
        self.model = m
        self.d = mujoco.MjData(m)
        self.sign = sign
        joints = [m.joint(f"{prefix}joint{i}") for i in range(1, 7)]
        self.qpos = np.array([joint.qposadr[0] for joint in joints])
        self.fingers = [m.joint(prefix + finger).qposadr[0]
                        for finger in ("left_finger", "right_finger")]
        self.low, self.high = np.array([joint.range for joint in joints]).T
        self.site = m.site(prefix + "grasp_site").id
        self.finger_bodies = [m.body(prefix + side + "_down").id for side in ("lf", "rf")]
        # Parallel jaw length axis is local z. Keeping it downward places
        # the fingers next to the hem while the wrist stays above the bin.
        self.rotation = np.array([[0, sign, 0], [sign, 0, 0], [0, 0, -1]])
        self.q = None
        self.offset = self._pad_offset()

    def _pad_offset(self):
        self.d.qpos[self.qpos] = (0., 1.2, 1.2, 0., 0., 0.)
        self.d.qpos[self.fingers] = 0.
        mujoco.mj_kinematics(self.model, self.d)
        pad_local = np.array([0., -.004, .078])
        pads = [self.d.xpos[body] + self.d.xmat[body].reshape(3, 3) @ pad_local
                for body in self.finger_bodies]
        frame = self.d.site_xmat[self.site].reshape(3, 3)
        return frame.T @ (np.mean(pads, axis=0) - self.d.site_xpos[self.site])

    def pose(self, joint_values):
        self.d.qpos[self.qpos] = joint_values
        mujoco.mj_kinematics(self.model, self.d)
        frame = self.d.site_xmat[self.site].reshape(3, 3)
        return self.d.site_xpos[self.site] + frame @ self.offset, frame

    def solve(self, data, target, opening):
        def error(q):
            position, frame = self.pose(q)
            orientation = Rotation.from_matrix(self.rotation @ frame.T).as_rotvec()
            return np.r_[target - position, .035 * orientation]

        if self.q is None:
            rng = np.random.default_rng(81 + self.sign)
            starts = [np.array([0., 1.05, 1.05, 0., 0., 0.])]
            starts.extend(rng.uniform(self.low, self.high) for _ in range(28))
            limit = 95
        else:
            starts = [self.q]
            limit = 30
        best = None
        for seed in starts:
            fit = least_squares(error, np.clip(seed, self.low + 1e-6, self.high - 1e-6),
                                bounds=(self.low, self.high), max_nfev=limit,
                                ftol=1e-7, xtol=1e-7)
            if best is None or np.linalg.norm(fit.fun) < np.linalg.norm(best.fun):
                best = fit
            if np.linalg.norm(best.fun) < .0001:
                break
        self.q = best.x
        data.qpos[self.qpos] = self.q
        data.qpos[self.fingers] = [opening, -opening]
        return float(np.linalg.norm(best.fun[:3]))


def label_frame(pixels, time, phase, sectors,
                subtitle="YAM kinematic replay  ·  Partially fitted reset",
                title="Fit a fresh bin liner"):
    picture = Image.fromarray(pixels)
    draw = ImageDraw.Draw(picture)
    bold = ImageFont.truetype(str(FONTS / "DejaVuSans-Bold.ttf"), 28)
    regular = ImageFont.truetype(str(FONTS / "DejaVuSans.ttf"), 17)
    draw.rectangle((0, 0, WIDTH, 72), fill="#121a1a")
    draw.text((24, 10), title, font=bold, fill="#f5f8f6")
    draw.text((25, 47), subtitle,
              font=regular, fill="#d6e1dd")
    draw.rectangle((0, HEIGHT - 49, WIDTH, HEIGHT), fill="#121a1a")
    draw.text((24, HEIGHT - 35), phase, font=regular, fill="#f5f8f6")
    draw.text((WIDTH - 24, HEIGHT - 35), f"{sectors}/12 rim sectors  ·  {time:.2f} s",
              font=regular, fill="#f5f8f6", anchor="rt")
    return picture


def render(trace_file, stock_file, output, poster=None):
    trace = json.loads(trace_file.read_text())
    run = next(record for record in trace["results"] if record["mode"] == "paired")
    frames = run["frames"]
    m, data, puck, _, puck_geom = model(True, lambda xml: scene_with_arms(xml, stock_file))
    film_bodies = m.flex_vertbodyid.copy()
    ref = data.flexvert_xpos.copy()
    qpos = m.jnt_qposadr[m.body_jntadr[film_bodies]][:, None] + np.arange(3)
    puck_qpos = m.jnt_qposadr[m.body_jntadr[puck]]
    puck_color = m.geom_rgba[puck_geom].copy()
    arms = [ArmReplay(m, prefix, sign) for (prefix, *_), sign in zip(MOUNTS, (1, -1))]
    middle = round(run["centre_deg"] * SEGMENTS / 360.)
    offset = run["grasp_offset_vertices"]
    picks = ((middle - offset) % SEGMENTS, (middle + offset) % SEGMENTS)
    last_active = np.array([frame for frame in frames if len(frame["grasps"]) == 2][-1]["points"])[list(picks)]
    camera = mujoco.MjvCamera()
    camera.lookat[:] = (.17, 0., .29)
    camera.azimuth = 148
    camera.elevation = -25
    camera.distance = 1.40
    opt = mujoco.MjvOption()
    opt.geomgroup[3:] = 0
    opt.sitegroup[:] = 0
    command = [
        "ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{WIDTH}x{HEIGHT}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264",
        "-preset", "fast", "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output),
    ]
    previous = None
    errors = []
    end = frames[-1]["time"]
    release = trace["release_s"]
    with mujoco.Renderer(m, height=HEIGHT, width=WIDTH) as renderer, subprocess.Popen(
            command, stdin=subprocess.PIPE) as encoder:
        for tick in range(round((end * SLOWDOWN + 1.15) * FPS)):
            time = min(end, max(0., (tick / FPS - .4) / SLOWDOWN))
            index = min(round(time * 20), len(frames) - 1)
            frame = frames[index]
            if index != previous:
                points = np.array(frame["points"])
                data.qpos[qpos] = points - ref
                m.geom_rgba[puck_geom, 3] = 0 if frame["puck"] is None else puck_color[3]
                if frame["puck"] is not None:
                    data.qpos[puck_qpos:puck_qpos + 3] = frame["puck"]
                for slot, (arm, vertex) in enumerate(zip(arms, picks)):
                    if frame["time"] < release:
                        target = points[vertex]
                        opening = 0.
                    else:
                        fraction = (frame["time"] - release) / .6
                        target = ease(last_active[slot], last_active[slot] +
                                      [.065, .075 * (2 * slot - 1), .14], fraction)
                        opening = ease(.001, .017, fraction)
                    errors.append(arm.solve(data, target, opening))
                mujoco.mj_forward(m, data)
                renderer.update_scene(data, camera=camera, scene_option=opt)
                stage = frame["phase"] if frame["time"] < end else "Cuff held through 100 g drop"
                picture = label_frame(renderer.render(), frame["time"], stage, sum(frame["sectors"]))
                previous = index
            if poster is not None and tick == round(FPS * 2):
                picture.save(poster, quality=88)
            encoder.stdin.write(picture.tobytes())
        encoder.stdin.close()
        if encoder.wait():
            raise RuntimeError("FFmpeg could not encode the replay")
    print(f"Peak visual IK position residual: {1000 * max(errors):.1f} mm")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("yam_xml", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--poster", type=Path)
    args = parser.parse_args()
    render(args.trace, args.yam_xml, args.output, args.poster)
