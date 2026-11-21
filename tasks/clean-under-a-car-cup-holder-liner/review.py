"""Capture reviewer evidence from a replay of the native reference actions."""

import argparse
import json
from pathlib import Path

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from fixtures import CupHolder
from station import Station
from video import camera, paint_opaque_stains, stain_opacity

# Select phases where the liner and applicator have moved out of the target well's view.
CAPTURES = {
    0: ("setup", "Starting setup"),
    150: ("dirty-well", "Before scrubbing"),
    404: ("clean-well", "After scrubbing"),
    567: ("seated-liner", "Liner refitted"),
}
CAPTURE_WIDTH, CAPTURE_HEIGHT = 960, 540


def capture_reference(commands, output):
    trial = Station(CupHolder())
    option = mj.MjvOption()
    option.geomgroup[5] = 0
    cameras = {
        "setup": camera((-.076, 0, .275), 1.34, 164, -29),
        "detail": camera((.035, .028, .232), .295, 38, -60),
    }
    geom_opacity = stain_opacity(trial)

    def capture(stage, renderer):
        renderer.update_scene(
            trial.data, camera=cameras["setup" if stage == "setup" else "detail"],
            scene_option=option,
        )
        renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = False
        paint_opaque_stains(renderer.scene, geom_opacity)
        Image.fromarray(renderer.render()).save(output / f"{stage}.png")
        liner = trial.data.body("liner")
        offset = liner.xpos - (trial.fixture.cx + trial.fixture.x_offset,
                               trial.fixture.cy, trial.fixture.seat_z)
        rotation = liner.xmat.reshape(3, 3)
        reading = trial.reading()
        reading.pop("phase")
        return {
            **reading,
            "continuous_seat_hold_s": round(trial.fixture.reseat_hold, 3),
            "liner_x_y_error_mm": round(float(np.linalg.norm(offset[:2])) * 1000, 2),
            "liner_height_error_mm": round(float(abs(offset[2])) * 1000, 2),
            "liner_tilt_deg": round(float(np.degrees(np.arccos(np.clip(rotation[2, 2], -1, 1)))), 2),
        }

    records = {}
    with mj.Renderer(trial.model, height=CAPTURE_HEIGHT, width=CAPTURE_WIDTH) as renderer:
        stage, title = CAPTURES[0]
        records[stage] = {"title": title, **capture(stage, renderer)}
        for command in commands:
            trial.action(command.reshape(2, 7))
            if item := CAPTURES.get(trial.steps):
                stage, title = item
                records[stage] = {"title": title, **capture(stage, renderer)}
    if trial.reading()["score"] != 1.0:
        raise RuntimeError(f"Reference replay scored {trial.reading()['score']:.2f}")
    (output / "review.json").write_text(json.dumps(records, indent=2) + "\n")
    return records


def make_sheet(output, records):
    """Combine the unaltered renders with captions drawn from the replay's scorer."""
    width, height = 1320, 840
    sheet = Image.new("RGB", (width, height), "#e7e9e7")
    draw = ImageDraw.Draw(sheet)
    header = ImageFont.truetype("DejaVuSans-Bold.ttf", 33)
    body = ImageFont.truetype("DejaVuSans.ttf", 21)
    stage_title = ImageFont.truetype("DejaVuSans-Bold.ttf", 22)
    draw.text((30, 26), "Clean under a car cup-holder liner", font=header, fill="#172222")
    draw.text((30, 75), "One simulated run · Native dual-YAM replay", font=body, fill="#425155")
    box_w, box_h = 620, 342
    for index, (key, record) in enumerate(records.items()):
        x, y = 30 + index % 2 * 640, 126 + index // 2 * 352
        draw.rounded_rectangle((x, y, x + box_w, y + box_h), radius=12, fill="#fcfdfb")
        draw.text((x + 13, y + 9), record["title"], font=stage_title, fill="#172222")
        with Image.open(output / f"{key}.png") as photo:
            panel = photo.resize((466, 262), Image.Resampling.LANCZOS)
            sheet.paste(panel, (x + (box_w - 466) // 2, y + 42))
        marks = record["spots_cleaned"]
        if key == "setup":
            status = "Liner installed over the well · Applicator in its pocket"
        elif key == "seated-liner":
            status = f"Score {record['score']:.2f} · Liner seated · {record['seconds']:.2f} s"
        else:
            status = f"{marks}/9 marks cleared · {record['seconds']:.2f} s"
        draw.text((x + 13, y + 310), status, font=body, fill="#425155")
    sheet.save(output / "cup-holder-review.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--commands", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    commands = np.asarray(json.loads(args.commands.read_text()), dtype=float)
    if commands.shape != (max(CAPTURES), 14) or not np.isfinite(commands).all():
        raise ValueError("Expected the 567-command, 14-joint reference replay.")
    args.out.mkdir(parents=True, exist_ok=True)
    records = capture_reference(commands, args.out)
    make_sheet(args.out, records)


if __name__ == "__main__":
    main()
