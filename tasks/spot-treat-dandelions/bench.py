"""Register real bench photos to a 340 × 230 mm frame and check exposure controls."""

import argparse, json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageOps
from verify import score

FRAME = (680, 460)  # 2 pixels per mm
TRAY = (40, 30, 640, 430)  # 300 x 200 mm
CLEAN = (90, 434, 106, 450)  # 8 mm square centres at (49,221) mm
POSITIVE = (574, 434, 590, 450)  # (291,221) mm


def rectify(im, points):
    dst = np.array([(0, 0), (680, 0), (680, 460), (0, 460)], float)
    pts = np.asarray(points, float)
    if (
        pts.shape != (4, 2)
        or not np.isfinite(pts).all()
        or (pts < 0).any()
        or (pts >= im.size).any()
    ):
        raise ValueError("Mark the four frame corners inside the photo.")
    edges = np.roll(pts, -1, axis=0) - pts
    turn = edges * np.roll(np.roll(edges, -1, axis=0), 1, axis=1)
    if np.any(turn[:, 0] - turn[:, 1] <= 0) or np.any(
        np.linalg.norm(edges, axis=1) < [680, 460, 680, 460]
    ):
        raise ValueError("Use at least 2 px/mm and click TL, TR, BR, BL.")
    rows = []
    for (u, v), (x, y) in zip(dst, pts):
        rows.extend(
            [[u, v, 1, 0, 0, 0, -u * x, -v * x], [0, 0, 0, u, v, 1, -u * y, -v * y]]
        )
    coeff = np.linalg.solve(rows, pts.ravel())
    return im.transform(
        FRAME, Image.Transform.PERSPECTIVE, coeff, Image.Resampling.BILINEAR
    )


def registered(path, corners):
    im = ImageOps.exif_transpose(Image.open(path)).convert("RGB")
    if corners.exists():
        pts = json.loads(corners.read_text())
    else:
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots()
        ax.imshow(im)
        ax.set_title("Frame crosses: top left, top right, bottom right, bottom left")
        pts = plt.ginput(4, timeout=0)
        plt.close(fig)
        result = rectify(im, pts)
        corners.write_text(json.dumps(pts))
        return result
    return rectify(im, pts)


def exgreen(im):
    a = np.array(im, dtype=float)
    return a[:, :, 1] - (a[:, :, 0] + a[:, :, 2]) / 2


def alignment_overlay(before, after, labels):
    """Compare held-out daylight foliage against the original mask, at native scale."""
    if before.size != after.size or np.asarray(before).shape[:2] != labels.shape:
        raise ValueError("Daylight crops and weed labels must share dimensions.")
    if set(np.unique(labels)) != {0, 1, 2, 3}:
        raise ValueError("Mark all three weed rosettes before checking alignment.")
    bordered = np.pad(labels, 1, constant_values=0)
    outline = (labels > 0) & (
        (bordered[1:-1, :-2] != labels)
        | (bordered[1:-1, 2:] != labels)
        | (bordered[:-2, 1:-1] != labels)
        | (bordered[2:, 1:-1] != labels)
    )
    width, height = before.size
    panel = Image.new("RGB", (2 * width, height + 28), (36, 36, 36))
    draw = ImageDraw.Draw(panel)
    for col, (crop, name) in enumerate(
        (
            (before, "Before spray / original outlines"),
            (after, "After spray / same outlines"),
        )
    ):
        rgb = np.array(crop.convert("RGB"))
        rgb[outline] = (255, 66, 201)
        panel.paste(Image.fromarray(rgb), (col * width, 28))
        draw.text((col * width + 8, 8), name, fill="white")
    return panel


def grade_frames(before, after, labels, aligned):
    results = {}
    for name, box in [("dry", CLEAN), ("positive", POSITIVE)]:
        a, b = before.crop(box), after.crop(box)
        change = exgreen(b) - exgreen(a)
        results[name + "_new_fraction"] = float(np.mean(change >= 35))
        if name == "dry":
            results["dry_changed_fraction"] = float(np.mean(np.abs(change) >= 35))
        results[name + "_clipped_fraction"] = float(
            np.mean(np.maximum(np.asarray(a)[:, :, 1], np.asarray(b)[:, :, 1]) >= 253)
        )
    reasons = []
    if not aligned:
        reasons.append("Alignment unconfirmed")
    if results["dry_changed_fraction"] > 0.02:
        reasons.append("Dry reference changed")
    if results["positive_new_fraction"] < 0.70:
        reasons.append("Weak positive reference")
    if (
        max(results["positive_clipped_fraction"], results["dry_clipped_fraction"])
        >= 0.10
    ):
        reasons.append("Reference exposure clipped")
    if reasons:
        return dict(status="unreadable", score=None, reasons=reasons, **results)
    metrics = score(np.asarray(before.crop(TRAY)), np.asarray(after.crop(TRAY)), labels)
    status = "pass" if metrics["score"] == 1 else "partial" if metrics["score"] > 0 else "fail"
    return dict(status=status, **results, **metrics)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("workdir", type=Path)
    p.add_argument("--daylight", type=Path)
    p.add_argument(
        "--check-daylight",
        type=Path,
        help="Prepare a before/after mask overlay from the finish daylight photo.",
    )
    p.add_argument("--before", type=Path)
    p.add_argument("--after", type=Path)
    p.add_argument("--labels", type=Path)
    p.add_argument(
        "--alignment-checked",
        action="store_true",
        help="Confirm leaf edges stayed aligned and exposure was locked.",
    )
    a = p.parse_args()
    a.workdir.mkdir(exist_ok=True, parents=True)
    if a.daylight:
        daylight = registered(a.daylight, a.workdir / "daylight-corners.json")
        daylight.crop(TRAY).save(a.workdir / "daylight-crop.png")
    elif a.check_daylight:
        if not a.labels:
            p.error("Supply the pre-run weed labels with the finish daylight photo.")
        before = Image.open(a.workdir / "daylight-crop.png").convert("RGB")
        after = registered(a.check_daylight, a.workdir / "finish-daylight-corners.json")
        labels = np.asarray(Image.open(a.labels).convert("L"))
        panel = alignment_overlay(before, after.crop(TRAY), labels)
        panel.save(a.workdir / "alignment-overlay.png")
    else:
        if not all((a.before, a.after, a.labels)):
            p.error("Provide daylight, or before, after and labels.")
        frames = [
            registered(path, a.workdir / (name + "-corners.json"))
            for name, path in [("before", a.before), ("after", a.after)]
        ]
        for name, im in zip(("before", "after"), frames):
            im.save(a.workdir / (name + "-registered.png"))
        metrics = grade_frames(
            *frames, np.asarray(Image.open(a.labels).convert("L")), a.alignment_checked
        )
        (a.workdir / "bench-result.json").write_text(json.dumps(metrics, indent=2))
        print(json.dumps(metrics, indent=2))
