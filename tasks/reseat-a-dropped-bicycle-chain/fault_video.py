"""Show the dropped front chain edge-on, then show the same view through the repair."""

from PIL import Image, ImageDraw
import mujoco as mj
from video import Recorder, BG, INK, MUTED, W, H, camera
from fixtures import AXLES

GOLD = "#ffbb50"
BLUE = "#76b9ff"


class FaultRecorder(Recorder):
    def attach(self, trial):
        if trial.fixture.lift_pick:
            trial.model.vis.quality.offsamples = 0
        super().attach(trial)
        # Only change render colors. Contacts, masses and joint positions stay identical.
        m = trial.model
        for i in range(m.ngeom):
            name = m.geom(i).name or ""
            if name.startswith(("roller", "cheek")):
                m.geom_rgba[i] = [0.94, 0.48, 0.035, 1]
            elif name.startswith(("front_tooth", "front_rim")):
                m.geom_rgba[i] = [0.08, 0.38, 0.72, 1]
        for i in range(m.ntendon):
            if m.tendon_width[i] > 0.001:
                m.tendon_rgba[i] = [0.94, 0.48, 0.035, 1]
        self.cameras[0] = camera(AXLES[0] + [0, .015, .012], .30, 325, -20)
        self.cameras[1] = camera(AXLES[0] + [0, .105, .055], 1.15, 290, -27)

    def draw(self):
        canvas = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(canvas)
        s = self.trial
        read = s.reading()
        for renderer, cam, xy in zip(
            self.renderers, self.cameras, [(8, 50), (540, 50)]
        ):
            renderer.update_scene(s.data, camera=cam, scene_option=self.scene_option)
            renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = not s.fixture.lift_pick
            canvas.paste(Image.fromarray(renderer.render()), xy)
        d.text((12, 11), "Dropped bicycle chain", font=self.heading, fill=INK)
        d.text((335, 15), "Chain", font=self.body, fill=GOLD)
        d.text((435, 15), "Front teeth", font=self.body, fill=BLUE)
        d.text((550, 302), "Turns since start", font=self.body, fill=INK)
        for y, title, value in [
            (333, "Crank", read["front_from_reset"]),
            (362, "Rear sprocket", read["rear_from_reset"]),
        ]:
            d.text((550, y), title, font=self.body, fill=MUTED)
            d.text((750, y), f"{abs(value):.2f}", font=self.body, fill=INK)
        caption = s.phase
        if read["score"] == 1:
            caption = "Repair verified. Both tools released."
        d.text((550, 394), f'Score {read["score"]:.2f}   |   {read["seconds"]:.1f} s', font=self.small, fill=INK)
        d.text((12, 414), caption, font=self.body, fill=INK)
        self.final_pixels = canvas.tobytes()
        self.proc.stdin.write(self.final_pixels)
        if not self.frames:
            canvas.save(self.path / "setup.png")
        self.frames += 1
