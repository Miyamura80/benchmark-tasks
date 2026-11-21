"""Score registered, cropped UV photographs and a hand-labelled daylight mask."""

import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image

MIN_WEED = 0.30
MAX_SPILL = 0.45  # marked non-weed area divided by total visible weed area


def score(before, after, labels, delta=35):
    if before.shape != after.shape or before.shape != (*labels.shape, 3):
        raise ValueError("Register and crop all three images to the same tray edges.")
    if set(np.unique(labels)) != {0, 1, 2, 3}:
        raise ValueError(
            "The mask must use 0 for turf and 1, 2, 3 for the three weeds."
        )

    def green(im):
        a = im.astype(float)
        return a[:, :, 1] - (a[:, :, 0] + a[:, :, 2]) / 2

    wet = green(after) - green(before) >= delta
    coverage = [float(np.mean(wet[labels == i])) for i in (1, 2, 3)]
    spill = float(np.sum(wet & (labels == 0)) / np.sum(labels > 0))
    return dict(
        weed_coverage=coverage,
        turf_spill_ratio=spill,
        score=float(min(coverage) >= MIN_WEED and spill <= MAX_SPILL),
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("before", "after", "labels"):
        p.add_argument(name, type=Path)
    p.add_argument("--delta", type=float, default=35)
    args = p.parse_args()
    if not 0 < args.delta <= 255:
        p.error("delta must be in (0,255]")

    def read(path, mode):
        return np.asarray(Image.open(path).convert(mode))

    print(
        json.dumps(
            score(
                read(args.before, "RGB"),
                read(args.after, "RGB"),
                read(args.labels, "L"),
                args.delta,
            ),
            indent=2,
        )
    )
