"""Measure visible foundation coverage from registered, locked-exposure photos."""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image

REGIONS = {1: "left_cheek", 2: "right_cheek", 3: "nose", 4: "protected"}
MIN_REFERENCE_DELTA = 12.0


def rgb(path):
    with Image.open(path) as image:
        return np.asarray(image.convert("RGB"), dtype=float)


def measure(clean, reference, trial, mask):
    if clean.shape != reference.shape or clean.shape != trial.shape or clean.shape[:2] != mask.shape:
        raise ValueError("The three photos and the region mask must have the same dimensions")
    if set(np.unique(mask)) != {0, *REGIONS}:
        raise ValueError("Use mask values 0 through 4 and include each labeled region")

    selected = mask != 0
    reference_delta = reference[selected] - clean[selected]
    squared_contrast = np.sum(reference_delta**2, axis=1)
    if np.any(squared_contrast < MIN_REFERENCE_DELTA**2):
        raise ValueError("Reference contrast is too low in a labeled pixel. Recalibrate the photos.")

    trial_delta = trial[selected] - clean[selected]
    fraction = np.clip(np.sum(trial_delta * reference_delta, axis=1) / squared_contrast, 0, 1)
    labels = mask[selected]
    covered = fraction >= 0.82
    checks = {}
    for label, name in REGIONS.items():
        region = labels == label
        checks[name] = float(np.max(fraction[region]) if label == 4 else np.mean(covered[region]))
    checks["combined_coverage"] = float(np.mean(covered[labels != 4]))
    checks["meets_photo_thresholds"] = (
        min(checks["left_cheek"], checks["right_cheek"], checks["nose"]) >= 0.93
        and checks["protected"] <= 0.20
    )
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("clean", "reference", "trial", "mask"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    with Image.open(args.mask) as image:
        mask = np.asarray(image.convert("L"))
    print(json.dumps(measure(rgb(args.clean), rgb(args.reference), rgb(args.trial), mask), indent=2))


if __name__ == "__main__":
    main()
