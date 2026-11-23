"""Score a five-second hands-clear collection, using the same rule on both rigs."""
import argparse
import csv
import json
import math
from statistics import median

WINDOW_S = 5.0
CLEAR_M = 0.070


def score_window(collected, reference, eligible):
    if not eligible:
        return 0.0
    if not (math.isfinite(reference) and reference > 0 and math.isfinite(collected) and collected >= 0):
        raise ValueError("Expected measured nonnegative collection and a positive reference")
    fraction = collected / reference
    return 1.0 if fraction >= 0.80 else 0.5 if fraction >= 0.50 else 0.0


def assess(rows):
    data = {key: [] for key in ("reference", "kinked", "trial")}
    for row in rows:
        stage = row["stage"]
        mass, duration = float(row["net_water_g"]), float(row["seconds"])
        if row["hands_clear"] not in ("yes", "no") or row["ends_connected"] not in ("yes", "no"):
            raise ValueError("Use yes/no for each checklist field")
        eligible = row["hands_clear"] == row["ends_connected"] == "yes"
        if stage not in data or not math.isfinite(mass) or mass < 0 or duration != WINDOW_S:
            raise ValueError("Each recorded collection must be a five-second window")
        if stage != "trial" and not eligible:
            raise ValueError("Calibration requires clear hands and connected ends")
        data[stage].append((mass, eligible))
    if len(data["reference"]) != 3 or len(data["kinked"]) != 1 or len(data["trial"]) != 1:
        raise ValueError("Expected three reference collections, one kinked check and one trial")
    ref = median(mass for mass, _ in data["reference"])
    if ref <= 0 or data["kinked"][0][0] > .20 * ref:
        raise ValueError("Invalid reset or reference")
    mass, eligible = data["trial"][0]
    return dict(reference_g=ref, recovered_fraction=mass/ref, score=score_window(mass, ref, eligible))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file")
    with open(parser.parse_args().csv_file, newline="") as file:
        print(json.dumps(assess(csv.DictReader(file)), indent=2))
