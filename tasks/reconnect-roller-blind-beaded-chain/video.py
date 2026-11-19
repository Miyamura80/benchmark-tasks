"""Record native simulation pixels and live scorer values in an H.264 MP4."""

from pathlib import Path
import subprocess

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H, FPS = 1120, 630, 10
VIEWS = {
    'Blind': (((0.028, -0.028, 0.343), 0.635, 103, -21), ((0.107, -0.055, 0.275), 0.34, 165, -39), 'Chain and coupler'),
}
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
        self.renderer = mj.Renderer(trial.model, height=460, width=716, max_geom=3500)
        self.option = mj.MjvOption()
        self.option.geomgroup[5] = 0
        kind = type(trial.fixture).__name__
        primary, secondary, self.secondary_label = VIEWS[kind]
        self.cameras = [camera(*primary), camera(*secondary)]
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
            str(FPS),
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
            "22",
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
        renderer = self.renderer
        for i, cam in enumerate(self.cameras):
            renderer.update_scene(trial.data, camera=cam, scene_option=self.option)
            renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = False
            trial.fixture.visual(trial, renderer.scene)
            view = Image.fromarray(renderer.render())
            if i == 0:
                if not self.frames:
                    view.save(self.path / "setup.png")
                canvas.paste(view, (12, 57))
            else:
                canvas.paste(view.resize((368, 236), Image.Resampling.LANCZOS), (740, 77))
        draw = ImageDraw.Draw(canvas)
        draw.text((12, 13), trial.fixture.title, font=self.heading, fill=INK)
        read = trial.reading()
        draw.text(
            (748, 330),
            f"Native score {read['score']:.2f}   |   {read['seconds']:.1f} s",
            font=self.body,
            fill=ACCENT,
        )
        readable = [
            (key, value)
            for key, value in read.items()
            if key not in {"score", "step", "phase", "seconds"} and np.isscalar(value)
        ]
        draw.text((741,56),self.secondary_label,font=self.small,fill=MUTED)
        for i, (key, value) in enumerate(readable):
            text = key.replace("_", " ").capitalize()
            if isinstance(value, bool):
                value = "yes" if value else "no"
            draw.text(
                (748, 362 + 23 * i), f"{text}  {value}", font=self.small, fill=MUTED
            )
        draw.text((12, 540), trial.phase, font=self.body, fill=INK)
        self.final_pixels = canvas.tobytes()
        self.proc.stdin.write(self.final_pixels)
        self.frames += 1

    def close(self):
        if self.proc:
            for _ in range(10):
                if self.final_pixels:
                    self.proc.stdin.write(self.final_pixels)
            self.proc.stdin.close()
            if self.proc.wait() != 0:
                raise RuntimeError("MP4 encoding failed")
        if self.renderer:
            self.renderer.close()
