"""Record a fixed wide, exterior view of the native dual-YAM trial."""

from pathlib import Path
import subprocess

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H, FPS = 1280, 720, 10
VIEW_HEIGHT = 648
BG, INK, ACCENT = "#111a20", "#f2f7f8", "#8ce4d2"
STAIN_OPACITIES = (.77, .71, .56)
WELL_TONE = np.array((.084, .088, .093))


def camera(target, distance, azimuth, elevation):
    cam = mj.MjvCamera()
    cam.lookat[:] = target
    cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
    return cam


def stain_opacity(trial):
    return {
        trial.model.geom(f"spill_{i}_{layer}").id: opacity
        for i in range(len(trial.fixture.dots))
        for layer, opacity in enumerate(STAIN_OPACITIES)
    }


def paint_opaque_stains(scene, opacities):
    for geom in scene.geoms[:scene.ngeom]:
        if geom.objtype == mj.mjtObj.mjOBJ_GEOM and geom.objid in opacities:
            strength = geom.rgba[3] / opacities[geom.objid]
            geom.rgba[:3] = WELL_TONE + strength * (geom.rgba[:3] - WELL_TONE)
            geom.rgba[3] = 1


class Recorder:
    def __init__(self, directory):
        self.path = Path(directory)
        self.frames = 0
        self.final_pixels = None
        self.proc = None
        self.renderer = None
        self.heading = ImageFont.truetype("DejaVuSans-Bold.ttf", 22)
        self.status_font = ImageFont.truetype("DejaVuSans.ttf", 17)

    def attach(self, trial):
        self.trial = trial
        self.renderer = mj.Renderer(trial.model, height=VIEW_HEIGHT, width=W, max_geom=3500)
        self.option = mj.MjvOption()
        self.option.geomgroup[5] = 0  # Skip collision-only proxies underneath the visible solid skins.
        self.camera = camera((-.076, 0, .275), 1.34, 164, -29)
        self.stains = stain_opacity(trial)
        argv = [
            "ffmpeg", "-nostdin", "-loglevel", "error", "-y", "-f", "rawvideo",
            "-pixel_format", "rgb24", "-video_size", f"{W}x{H}", "-framerate",
            str(FPS), "-i", "-", "-an", "-c:v", "libx264", "-threads", "2",
            "-preset", "veryfast", "-crf", "21", "-pix_fmt", "yuv420p",
            "-movflags", "+faststart", str(self.path / "demo.mp4"),
        ]
        self.proc = subprocess.Popen(argv, stdin=subprocess.PIPE)

    def draw(self):
        trial = self.trial
        renderer = self.renderer
        renderer.update_scene(trial.data, camera=self.camera, scene_option=self.option)
        renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = False
        paint_opaque_stains(renderer.scene, self.stains)
        view = Image.fromarray(renderer.render())
        if not self.frames:
            view.save(self.path / "setup.png")
        canvas = Image.new("RGB", (W, H), BG)
        canvas.paste(view, (0, 40))
        draw = ImageDraw.Draw(canvas)
        draw.text((15, 7), trial.fixture.title, font=self.heading, fill=INK)
        read = trial.reading()
        status = (
            f"SCORE {read['score']:.2f}  /  "
            f"{read['spots_cleaned']}/{len(trial.fixture.dots)} MARKS CLEARED  /  "
            f"{read['seconds']:.1f} s"
        )
        draw.text((W - 15, H - 8), status, anchor="rd", font=self.status_font, fill=ACCENT)
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
