"""Record the continuous robot run alongside the computed spray deposit."""

import subprocess
import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from turf import CX, CY, Z, W, H


class Film:
    def __init__(self, path):
        self.path = path
        font = "DejaVuSans.ttf"
        try:
            self.title = ImageFont.truetype(font, 21)
            self.text = ImageFont.truetype(font, 16)
        except OSError:
            self.title = ImageFont.load_default(size=21)
            self.text = ImageFont.load_default(size=16)

    def attach(self, s):
        self.s = s
        self.renderer = mj.Renderer(s.model, height=400, width=560, max_geom=4200)
        self.cam = mj.MjvCamera()
        self.cam.lookat[:] = [0.032, -0.05, 0.23]
        self.cam.distance = 0.98
        self.cam.elevation = -31
        self.cam.azimuth = 116
        self.opts = mj.MjvOption()
        self.opts.geomgroup[5] = 0
        self.proc = subprocess.Popen(
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
                "960x458",
                "-r",
                "10",
                "-i",
                "-",
                "-c:v",
                "libx264",
                "-crf",
                "20",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(self.path),
            ],
            stdin=subprocess.PIPE,
        )

    def draw(self):
        s = self.s
        f = s.fixture
        self.renderer.update_scene(s.data, camera=self.cam, scene_option=self.opts)
        sc = self.renderer.scene
        sc.flags[mj.mjtRndFlag.mjRND_SHADOW] = False
        if f.pulses and f.pulses[-1].get("time_s", -10) + 0.22 >= s.data.time:
            event = f.pulses[-1]
            origin = np.array(event["origin_m"])
            axis = np.array(event["axis"])
            rng = np.random.default_rng(100 + s.steps)
            for k in range(30):
                t = (k + 1) / 30
                z = origin + t * axis * max(0, origin[2] - Z) / max(0.1, -axis[2])
                z[:2] += rng.normal(0, 0.007 * t, 2)
                if sc.ngeom < sc.maxgeom and z[2] >= Z:
                    g = sc.geoms[sc.ngeom]
                    sc.ngeom += 1
                    mj.mjv_initGeom(
                        g,
                        mj.mjtGeom.mjGEOM_SPHERE,
                        np.full(3, 0.0009),
                        z,
                        np.eye(3).ravel(),
                        np.array([0.61, 0.85, 0.94, 1], np.float32),
                    )
        render = Image.fromarray(self.renderer.render())
        if not s.steps:
            render.save(self.path.parent / "setup.png")
        canvas = Image.new("RGB", (960, 458), "#15241d")
        d = ImageDraw.Draw(canvas)
        d.text((14, 12), "Spot-treat three dandelions", font=self.title, fill="white")
        d.text((575, 15), "Overhead deposition check", font=self.text, fill="white")
        canvas.paste(render, (0, 48))
        rgb = np.asarray(f.texture.resize((W, H))).copy()
        wet = np.asarray(f.wet)
        rgb[wet] = (97, 243, 231)
        canvas.paste(Image.fromarray(rgb).resize((384, 256)), (568, 49))
        d.text(
            (577, 317),
            "Cyan marks simulated spray deposits",
            font=self.text,
            fill="#91e4d6",
        )
        d.text(
            (577, 352),
            "Weed coverage    "
            + "   ".join(f"{x:.0%}" for x in f.result["weed_coverage"]),
            font=self.text,
            fill="white",
        )
        d.text(
            (577, 380),
            f"Grass spill / weed area    {f.result['turf_spill_ratio']:.0%}",
            font=self.text,
            fill="white",
        )
        d.text(
            (577, 417),
            f"{s.data.time:.1f} s    •    {len(f.pulses)} sprays    •    Score {f.result['score']:.2f}",
            font=self.text,
            fill="#e9e4cc",
        )
        self.proc.stdin.write(canvas.tobytes())

    def finish(self):
        self.proc.stdin.close()
        if self.proc.wait():
            raise RuntimeError("FFmpeg failed")
        self.renderer.close()
