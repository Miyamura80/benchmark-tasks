"""Record MuJoCo camera frames and measurements to a single H.264 video."""

import subprocess

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from bench import JETS, JET_HEIGHT, NAMES, jet_axis

BACKGROUND = "#101820"
PANEL = "#192731"
INK = "#edf4f6"
MUTED = "#adc2cb"
ACCENT = "#64d5ea"
GOOD = "#9ee5ba"


def font(points, bold=False):
    style = "-Bold" if bold else ""
    return ImageFont.truetype(f"DejaVuSans{style}.ttf", points)


def camera(lookat, distance, azimuth, elevation):
    result = mj.MjvCamera()
    result.lookat[:] = lookat
    result.distance, result.azimuth, result.elevation = distance, azimuth, elevation
    return result


class Recording:
    def __init__(self, path):
        self.path = path
        self.proc = subprocess.Popen(
            [
                "ffmpeg",
                "-y",
                "-hide_banner",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pixel_format",
                "rgb24",
                "-video_size",
                "960x540",
                "-framerate",
                "25",
                "-i",
                "-",
                "-pix_fmt",
                "yuv420p",
                "-c:v",
                "libx264",
                "-crf",
                "21",
                "-preset",
                "veryfast",
                "-movflags",
                "+faststart",
                str(path),
            ],
            stdin=subprocess.PIPE,
        )
        self.fonts = {n: font(n) for n in (13, 15, 18, 22)}
        self.title_font = font(26, True)

    def attach(self, trial):
        self.t = trial
        self.main = mj.Renderer(trial.model, height=398, width=620)
        self.detail = mj.Renderer(trial.model, height=184, width=284)
        self.main_cam = camera((0.02, -0.02, 0.244), 0.87, 305, -31)
        self.side_cam = (
            camera((*JETS[trial.blocked], JET_HEIGHT - 0.006), 0.19, 132, -12)
            if trial.kind == "rim"
            else camera((0.065, 0, 0.13), 0.38, 98, -20)
        )

    def add_streams(self, scene):
        if self.t.kind != "rim" or self.t.phase not in (
            "Measure each outlet",
            "Retest under the same head",
        ):
            return
        for outlet, flow in zip(JETS, self.t.rim_flow()):
            element = scene.geoms[scene.ngeom]
            mj.mjv_initGeom(
                element,
                mj.mjtGeom.mjGEOM_CAPSULE,
                np.zeros(3),
                np.zeros(3),
                np.eye(3).ravel(),
                np.array([0.14, 0.73, 0.95, 0.68], dtype=np.float32),
            )
            mouth = np.array([*outlet, JET_HEIGHT])
            mj.mjv_connector(
                element,
                mj.mjtGeom.mjGEOM_CAPSULE,
                0.0006 + 0.0015 * flow / 100.0,
                mouth - jet_axis(outlet) * 0.001,
                mouth - jet_axis(outlet) * (0.003 + 0.054 * flow / 100.0),
            )
            scene.ngeom += 1

    def add_water_surface(self, scene):
        if self.t.kind != "trap":
            return
        level = self.t.water_level
        element = scene.geoms[scene.ngeom]
        mj.mjv_initGeom(
            element,
            mj.mjtGeom.mjGEOM_ELLIPSOID,
            np.array([0.018 + 0.060 * level, 0.014 + 0.049 * level, 0.0015]),
            np.array([0.0, 0.0, 0.138 + 0.024 * level]),
            np.eye(3).ravel(),
            np.array([0.13, 0.62, 0.81, 0.50], dtype=np.float32),
        )
        element.transparent = 1
        scene.ngeom += 1

    def shot(self, renderer, cam, cutaway=False):
        renderer.update_scene(self.t.data, camera=cam)
        renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = False
        if cutaway:
            fade = (self.t.model.geom("bowl").id, self.t.model.geom("seat_ring").id)
            for element in renderer.scene.geoms[: renderer.scene.ngeom]:
                if element.objid in fade and element.objtype == mj.mjtObj.mjOBJ_GEOM:
                    element.rgba[3] = 0.12
                    element.transparent = 1
        self.add_streams(renderer.scene)
        self.add_water_surface(renderer.scene)
        return Image.fromarray(renderer.render())

    def bar(self, draw, x, y, w, ratio, label, amount):
        draw.text((x, y), label, fill=INK, font=self.fonts[15])
        draw.text((x + w, y), amount, fill=MUTED, font=self.fonts[15], anchor="ra")
        draw.rounded_rectangle((x, y + 22, x + w, y + 27), radius=2, fill="#324650")
        if ratio > 0:
            draw.rounded_rectangle(
                (x, y + 22, x + w * min(1.0, ratio), y + 27), radius=2, fill=ACCENT
            )

    def draw(self):
        rim = self.t.kind == "rim"
        image = Image.new("RGB", (960, 540), BACKGROUND)
        draw = ImageDraw.Draw(image)
        title = "Restore flow through a rim jet" if rim else "Clear a mixed toilet clog"
        draw.text((16, 15), title, fill=INK, font=self.title_font)
        draw.text((16, 52), self.t.phase, fill=ACCENT, font=self.fonts[18])
        draw.text(
            (944, 21),
            f"{self.t.data.time:04.1f} s",
            fill=MUTED,
            font=self.fonts[15],
            anchor="ra",
        )
        image.paste(self.shot(self.main, self.main_cam), (16, 86))
        draw.rounded_rectangle((648, 86, 944, 484), radius=7, fill=PANEL)
        draw.text(
            (659, 96),
            "Outlet cutaway" if rim else "Trapway cutaway",
            fill=INK,
            font=self.fonts[15],
        )
        image.paste(self.shot(self.detail, self.side_cam, True), (654, 120))
        if rim:
            draw.text(
                (659, 314), "Rinse collection by outlet", fill=INK, font=self.fonts[15]
            )
            for i, (name, value) in enumerate(zip(NAMES, self.t.rinse)):
                self.bar(
                    draw, 659, 339 + i * 32, 274, value / 100, name, f"{value:.0f} mL"
                )
        else:
            draw.text(
                (659, 314), "Paper and simulated waste", fill=INK, font=self.fonts[15]
            )
            self.bar(
                draw,
                659,
                342,
                274,
                self.t.water_level,
                "Water remaining",
                f"{100*self.t.water_level:.0f}%",
            )
            self.bar(
                draw,
                659,
                383,
                274,
                self.t.joint("clog") / 0.105,
                "Clog travel",
                f"{1000*self.t.joint('clog'):.0f} mm",
            )
            self.bar(
                draw,
                659,
                424,
                274,
                self.t.pressure / 8.0,
                "Hydraulic pulse",
                f"{self.t.pressure:.1f} N",
            )
        draw.text(
            (16, 506),
            "MuJoCo contact simulation  ·  Reduced-order water model",
            fill=MUTED,
            font=self.fonts[15],
        )
        self.proc.stdin.write(image.tobytes())

    def close(self):
        self.proc.stdin.close()
        if self.proc.wait():
            raise RuntimeError("Video encoding failed")
        self.main.close()
        self.detail.close()
