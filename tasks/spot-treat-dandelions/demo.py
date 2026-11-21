import argparse, json
from pathlib import Path
import numpy as np
from PIL import Image
from lawn import Lawn
from yam_station import Station
from turf import world, uv_pair


def save_images(f, out):
    f.texture.save(out / "daylight.jpg", quality=92)
    Image.fromarray(f.labels).save(out / "labels.png")
    for name, im in zip(("before", "after"), uv_pair(np.asarray(f.wet))):
        Image.fromarray(im).save(out / f"{name}.png")


def run(case, seed, out, video=False):
    f = Lawn(seed)
    recorder = None
    if video:
        from film import Film

        recorder = Film(out / "demo.mp4")
    s = Station(f, render=recorder)
    points = (
        f.centers
        if case != "blanket"
        else [(x, y) for y in (40, 100, 160) for x in (55, 115, 175, 235)]
    )
    if case == "miss":
        points = points[:-1]
    if case == "idle":
        points = []
    for i, (x, y) in enumerate(points):
        h = 0.065 if case != "blanket" else 0.095
        goal = world(x, y, h + 0.036)
        s.move_arm("right", goal, 1.2 if i == 0 else 0.70, "Aim at the lawn", opening=1)
        s.wait(0.35, "Settle")
        target = s.targets.copy()
        target[1, -1] = 0.16
        for _ in range(8):
            s.action(target)
        target[1, -1] = 1.0
        for _ in range(7):
            s.action(target)
    s.wait(0.4)
    if case != "idle":
        s.move_arm("right", world(175, 178, 0.15), 0.8, "Withdraw sprayer", opening=1)
    s.wait(1.2, "Inspect turf")
    if recorder:
        recorder.finish()
    if out is not None:
        save_images(f, out)
        (out / "result.json").write_text(
            json.dumps(dict(**s.reading(), pulse_geometry=f.pulses), indent=2)
        )
        (out / "commands.json").write_text(json.dumps(s.commands))
    return s.reading()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument(
        "--case", choices=["careful", "miss", "blanket", "idle"], default="careful"
    )
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--video", action="store_true")
    a = p.parse_args()
    a.out.mkdir(exist_ok=True, parents=True)
    print(run(a.case, a.seed, a.out, a.video))
