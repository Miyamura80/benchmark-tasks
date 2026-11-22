"""Record native simulation pixels and live scorer values in an H.264 MP4."""

from pathlib import Path
import subprocess

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from station import HZ

W, H = 800, 452
BG, INK, MUTED, ACCENT = "#111a20", "#f2f7f8", "#b7c9cc", "#8ce4d2"


def camera(target, distance, azimuth, elevation):
    cam = mj.MjvCamera()
    cam.lookat[:] = target
    cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
    return cam


def font(size, bold=False):
    return ImageFont.truetype("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf", size)


class Recorder:
    def __init__(self, directory):
        self.path = Path(directory)
        self.frames = 0
        self.final_pixels = None
        self.proc = None
        self.renderer = None
        self.heading, self.body, self.small = font(21, True), font(17), font(13)

    def attach(self, trial):
        self.trial = trial
        self.scene_option = mj.MjvOption()
        self.scene_option.geomgroup[5] = 0
        self.renderer = mj.Renderer(trial.model, height=348, width=528, max_geom=3000)
        fixture = trial.fixture
        self.interval = HZ // fixture.recording_fps
        x, y, z = fixture.focus
        wide = ((x - 0.125, y, z + 0.090), 0.93, 307, -35)
        if fixture.wide_camera:
            target, distance, azimuth, elevation = fixture.wide_camera
            wide = ((target[0]+fixture.x_offset, *target[1:]), distance, azimuth, elevation)
        detail = fixture.focus
        if fixture.detail_focus:
            x_d, y_d, z_d = fixture.detail_focus
            detail = (x_d+fixture.x_offset, y_d, z_d)
        views = (("wide", camera(*wide)), ("detail", camera(detail, *fixture.detail_camera)))
        self.cameras = views[::-1] if fixture.primary_detail else views
        argv = [
            "ffmpeg",
            "-nostdin",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
            "-video_size",
            f"{W}x{H}",
            "-framerate",
            str(fixture.recording_fps),
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-threads",
            "2",
            "-preset",
            "veryfast",
            "-crf",
            "25",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(self.path / "demo.mp4"),
        ]
        self.proc = subprocess.Popen(argv, stdin=subprocess.PIPE)

    def draw(self):
        canvas = Image.new("RGB", (W, H), BG)
        trial = self.trial
        for i, (kind, cam) in enumerate(self.cameras):
            self.renderer.update_scene(trial.data, camera=cam, scene_option=self.scene_option)
            scene = self.renderer.scene
            scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = False
            scene.flags[mj.mjtRndFlag.mjRND_REFLECTION] = False
            trial.fixture.visual(trial, scene)
            view = Image.fromarray(self.renderer.render())
            if not self.frames and kind == "detail":
                view.save(self.path / "setup.png")
            if i == 1:
                view = view.crop((81, 0, 447, 348)).resize((256, 244), Image.Resampling.LANCZOS)
            canvas.paste(view, (8, 51) if i == 0 else (540, 51))
        draw = ImageDraw.Draw(canvas)
        draw.text((12, 13), trial.fixture.title, font=self.heading, fill=INK)
        read = trial.reading()
        draw.text(
            (550, 305),
            f"Native score {read['score']:.2f}   |   {read['seconds']:.1f} s",
            font=self.body,
            fill=ACCENT,
        )
        readable = [
            (key, value)
            for key, value in read.items()
            if key not in {"score", "step", "phase", "seconds"} and np.isscalar(value)
        ]
        for i, (key, value) in enumerate(readable[:4]):
            text = key.replace("_", " ").capitalize()
            if isinstance(value, bool):
                value = "yes" if value else "no"
            draw.text(
                (550, 334 + 16 * i), f"{text}  {value}", font=self.small, fill=MUTED
            )
        draw.text((12, 417), trial.phase, font=self.body, fill=INK)
        self.final_pixels = canvas.tobytes()
        self.proc.stdin.write(self.final_pixels)
        self.frames += 1

    def close(self):
        if self.proc:
            self.draw()
            for _ in range(round(0.8*self.trial.fixture.recording_fps)):
                if self.final_pixels:
                    self.proc.stdin.write(self.final_pixels)
            self.proc.stdin.close()
            if self.proc.wait() != 0:
                raise RuntimeError("MP4 encoding failed")
        if self.renderer:
            self.renderer.close()
