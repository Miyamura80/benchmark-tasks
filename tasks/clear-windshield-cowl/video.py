"""Record a continuous exterior view of the complete dual-YAM station."""

from pathlib import Path
import subprocess

import mujoco as mj
from PIL import Image, ImageDraw, ImageFont

WIDTH, HEIGHT, VIEW_HEIGHT, FPS = 960, 540, 484, 10


def camera():
    cam = mj.MjvCamera()
    cam.lookat[:] = (0, -.049, .87)
    cam.distance = 1.70
    cam.azimuth = 109
    cam.elevation = -41
    return cam


class Recorder:
    def __init__(self, directory):
        self.path = Path(directory)
        self.final_pixels = None
        self.title = ImageFont.truetype("DejaVuSans-Bold.ttf", 17)
        self.label = ImageFont.truetype("DejaVuSans.ttf", 14)

    def attach(self, trial):
        self.trial = trial
        self.renderer = mj.Renderer(trial.model, height=VIEW_HEIGHT, width=WIDTH, max_geom=3500)
        self.option = mj.MjvOption()
        self.option.geomgroup[3:6] = 0
        self.camera = camera()
        self.proc = subprocess.Popen(
            [
                "ffmpeg", "-nostdin", "-loglevel", "error", "-y", "-f", "rawvideo",
                "-pixel_format", "rgb24", "-video_size", f"{WIDTH}x{HEIGHT}",
                "-framerate", str(FPS), "-i", "-", "-an", "-c:v", "libx264",
                "-threads", "2", "-preset", "veryfast", "-crf", "21",
                "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                str(self.path / "demo.mp4"),
            ],
            stdin=subprocess.PIPE,
        )

    def draw(self):
        trial = self.trial
        renderer = self.renderer
        renderer.update_scene(trial.data, camera=self.camera, scene_option=self.option)
        renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = False
        trial.fixture.visual(trial, renderer.scene)
        exterior = Image.fromarray(renderer.render())
        if self.final_pixels is None:
            exterior.save(self.path / "setup.png")
        frame = Image.new("RGB", (WIDTH, HEIGHT), "#152029")
        frame.paste(exterior)
        draw = ImageDraw.Draw(frame)
        read = trial.reading()
        draw.text((16, 491), trial.fixture.title, font=self.title, fill="#f5fafb")
        draw.text((16, 516), trial.phase, font=self.label, fill="#c8d4dc")
        draw.text(
            (944, 505),
            f"{read['leaves_in_tray']}/4 leaves   ·   {read['drains_clear']}/2 grilles clear   ·   Score {read['score']:.2f}",
            font=self.label, fill="#8ce4d2", anchor="rm",
        )
        self.final_pixels = frame.tobytes()
        self.proc.stdin.write(self.final_pixels)

    def close(self):
        try:
            if self.final_pixels:
                for _ in range(FPS):
                    self.proc.stdin.write(self.final_pixels)
            self.proc.stdin.close()
            if self.proc.wait():
                raise RuntimeError("MP4 encoding failed")
        finally:
            self.renderer.close()
