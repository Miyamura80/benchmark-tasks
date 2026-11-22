"""Exercise the physical photo workflow on synthetic inputs with known outcomes."""

from pathlib import Path
import json
import numpy as np
from PIL import Image, ImageDraw
from bench import FRAME, TRAY, POSITIVE, CLEAN, alignment_overlay, rectify, grade_frames


def checks(root):
    # Three equal-area weed regions and the grass-spill boundary are exact in pixels.
    from verify import score
    labels = np.repeat(np.array([0, 0, 0, 1, 2, 3], dtype=np.uint8), 100)[None, :]
    clean = np.zeros((1, labels.size, 3), dtype=np.uint8)
    records = {}
    for count in range(4):
        wet = clean.copy()
        for weed in range(1, count + 1):
            ix = np.flatnonzero(labels[0] == weed)[:30]
            wet[0, ix, 1] = 35
        wet[0, :135, 1] = 35
        assert score(clean, wet, labels)["score"] == count / 3
        wet[0, 135, 1] = 35
        assert score(clean, wet, labels)["score"] == 0

    for name in ("careful", "miss", "blanket", "idle"):
        case = root / name
        labels = np.asarray(
            Image.open(case / "labels.png").resize((600, 400), Image.Resampling.NEAREST)
        )
        frames = []
        for tag in ("before", "after"):
            frame = Image.new("RGB", FRAME, (25, 21, 45))
            tex = Image.open(case / (tag + ".png")).resize(
                (600, 400), Image.Resampling.NEAREST
            )
            frame.paste(tex, TRAY)
            if tag == "after":
                ImageDraw.Draw(frame).rectangle(POSITIVE, fill=(49, 180, 35))
            photo = Image.new("RGB", (720, 500))
            photo.paste(frame, (20, 20))
            frames.append(rectify(photo, [(20, 20), (700, 20), (700, 480), (20, 480)]))
        result = grade_frames(*frames, labels, True)
        assert result["score"] == {"careful": 1.0, "miss": 2 / 3, "blanket": 0.0, "idle": 0.0}[name], (name, result)
        records[name] = result
    for name in (
        "bad_positive",
        "unstable_dry",
        "dimmed_dry",
        "unconfirmed_alignment",
        "saturated_positive",
    ):
        a, b = [im.copy() for im in frames]
        if name == "bad_positive":
            b.paste(a.crop(POSITIVE), POSITIVE)
        if name == "unstable_dry":
            ImageDraw.Draw(b).rectangle(CLEAN, fill=(20, 180, 40))
        if name == "dimmed_dry":
            ImageDraw.Draw(a).rectangle(CLEAN, fill=(20, 180, 40))
        if name == "saturated_positive":
            ImageDraw.Draw(b).rectangle(POSITIVE, fill=(20, 255, 40))
        result = grade_frames(a, b, labels, name != "unconfirmed_alignment")
        assert result["score"] is None and result["status"] == "unreadable"
        records[name] = result
    with Image.open(root / "careful" / "daylight.jpg") as day:
        clean = day.convert("RGB").resize((600, 400))
    panel = alignment_overlay(clean, clean, labels)
    left = np.asarray(panel.crop((0, 28, 600, 428)))
    right = np.asarray(panel.crop((600, 28, 1200, 428)))
    assert np.array_equal(left, right)
    outline = np.all(left == [255, 66, 201], axis=2)
    assert all(np.any(outline & (labels == k)) for k in (1, 2, 3))
    assert not np.any(outline & (labels == 0))
    return records


if __name__ == "__main__":
    import sys

    print(json.dumps(checks(Path(sys.argv[1])), indent=2))
