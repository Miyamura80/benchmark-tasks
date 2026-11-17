"""Record MuJoCo camera frames and measurements to a single H.264 video."""

import math
import subprocess

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from bench import (
    JETS,
    JET_HEIGHT,
    JET_OPEN_FLOW,
    NAMES,
    RINSE_SECONDS,
    TRAP_PIECES,
    jet_axis,
)

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


def add_streams(trial, scene):
    if trial.kind != "rim" or not trial.rinsing:
        return
    for outlet, flow in zip(JETS, trial.rim_flow()):
        strength = flow / JET_OPEN_FLOW
        if strength <= 0:
            continue
        mouth = np.array([*outlet, JET_HEIGHT])
        axis = jet_axis(outlet)
        start = mouth - axis * 0.001
        end = mouth - axis * (0.003 + 0.061 * strength)
        if strength < 0.99:
            # A restricted, uneven bore deflects the visible trickle.
            sideways = np.array((-axis[1], axis[0], 0.0))
            bend = mouth - axis * (0.003 + 0.006 * strength) + sideways * 0.006
            points = (start, bend, end + sideways * 0.010)
        else:
            points = (start, end)
        for start, end in zip(points, points[1:]):
            element = scene.geoms[scene.ngeom]
            mj.mjv_initGeom(
                element,
                mj.mjtGeom.mjGEOM_CAPSULE,
                np.zeros(3),
                np.zeros(3),
                np.eye(3).ravel(),
                np.array([0.16, 0.75, 0.98, 0.53 + 0.39 * strength], dtype=np.float32),
            )
            mj.mjv_connector(
                element,
                mj.mjtGeom.mjGEOM_CAPSULE,
                0.0007 + 0.0023 * strength,
                start,
                end,
            )
            scene.ngeom += 1


def add_water_surface(trial, scene):
    if trial.kind != "trap":
        return
    level = trial.water_level
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
        if trial.kind == "rim":
            x, y = JETS[trial.blocked]
            azimuth = math.degrees(math.atan2(y, x)) % 360
            self.side_cam = camera((x, y, JET_HEIGHT - 0.001), 0.067, azimuth, -29)
        else:
            self.side_cam = camera((0.081, 0, 0.111), 0.19, 98, -24)
        self.rinse_cam = camera((0, 0, JET_HEIGHT), 0.51, 305, -89)
        self.fade = {trial.model.geom(name).id for name in ("bowl", "seat_ring")}
        self.trap_screen = {
            trial.model.geom(name).id
            for name in (
                "cistern",
                "cistern_cover",
                "rear_pedestal",
                "rear_deck",
                "raised_lid",
                "flush_button",
            )
        }

    def shot(self, renderer, cam, cutaway=False):
        renderer.update_scene(self.t.data, camera=cam)
        renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = False
        if cutaway:
            for element in renderer.scene.geoms[: renderer.scene.ngeom]:
                if element.objtype != mj.mjtObj.mjOBJ_GEOM:
                    continue
                if self.t.kind == "trap" and element.objid in self.trap_screen:
                    element.rgba[3] = 0.0
                    element.transparent = 1
                elif element.objid in self.fade:
                    element.rgba[3] = (
                        0.05 if self.t.kind == "rim" and not self.t.rinsing else 0.12
                    )
                    element.transparent = 1
        add_streams(self.t, renderer.scene)
        add_water_surface(self.t, renderer.scene)
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
        rinse = rim and self.t.rinsing
        if not rim:
            positions = tuple(self.t.trap_positions().values())
            first, last = min(positions), max(positions)
            self.side_cam.lookat[0] = (first + last) / 2
            self.side_cam.distance = max(0.19, 1.65 * (last - first + 0.048))
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
            (
                (
                    "Overhead view · all four outlets"
                    if rinse
                    else "Limescale at the jet"
                )
                if rim
                else "Trapway cutaway"
            ),
            fill=INK,
            font=self.fonts[15],
        )
        image.paste(
            self.shot(self.detail, self.rinse_cam if rinse else self.side_cam, True),
            (654, 120),
        )
        if rim:
            draw.text((659, 314), "Rinse collection", fill=INK, font=self.fonts[15])
            draw.text(
                (933, 314),
                f"Scale left: {len(self.t.scale_remaining())}/4",
                fill=MUTED,
                font=self.fonts[13],
                anchor="ra",
            )
            for i, (name, value) in enumerate(zip(NAMES, self.t.rinse)):
                self.bar(
                    draw,
                    659,
                    339 + i * 32,
                    274,
                    value / (JET_OPEN_FLOW * RINSE_SECONDS),
                    name,
                    f"{value:.0f} mL",
                )
        else:
            cleared = self.t.trap_cleared()
            for i, (material, label, color) in enumerate(
                (
                    ("paper", "Wet paper", "#f3e8c9"),
                    ("feces", "Feces", "#c3845a"),
                )
            ):
                pieces = [p.name for p in TRAP_PIECES if p.material == material]
                count = sum(name in cleared for name in pieces)
                y = 319 + i * 25
                draw.rounded_rectangle((659, y + 2, 672, y + 15), radius=3, fill=color)
                draw.text((680, y), label, fill=INK, font=self.fonts[15])
                draw.text(
                    (933, y),
                    f"{count}/{len(pieces)} cleared",
                    fill=MUTED,
                    font=self.fonts[15],
                    anchor="ra",
                )
            self.bar(
                draw,
                659,
                374,
                274,
                self.t.water_level,
                "Water remaining",
                f"{100*self.t.water_level:.0f}%",
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
