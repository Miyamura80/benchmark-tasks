"""Check one before-and-after rim-jet trial against an open-flow reference."""

import argparse
import csv
import json
import math
from pathlib import Path


OUTLETS = ("front_left", "front_right", "rear_left", "rear_right")
MIN_FLOW = 0.80
MAX_FLOW = 1.20
BLOCKED_FLOW = 0.40
MIN_BASELINE_ML = 50.0


def read_measurements(path):
    """Read one volume collection per outlet in each of the three rinses."""
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    names = [row["outlet"] for row in rows]
    if len(rows) != len(OUTLETS) or set(names) != set(OUTLETS):
        raise ValueError("Supply each of the four outlet names exactly once")
    reference, before, after = {}, {}, {}
    for row in rows:
        key = row["outlet"]
        for column, run in (
            ("reference_ml", reference), ("before_ml", before), ("after_ml", after),
        ):
            volume = float(row[column])
            if not math.isfinite(volume) or volume < 0:
                raise ValueError(f"{key} {column} must be finite and non-negative")
            run[key] = volume
        if reference[key] < MIN_BASELINE_ML:
            raise ValueError(f"{key} reference collection must be at least {MIN_BASELINE_ML:g} mL")
    return reference, before, after


def evaluate(reference, before, after, diagnosed, intact, recovered):
    initial = {key: before[key] / reference[key] for key in OUTLETS}
    final = {key: after[key] / reference[key] for key in OUTLETS}
    restricted = [key for key in OUTLETS if initial[key] <= BLOCKED_FLOW]
    if len(restricted) != 1:
        raise ValueError("The reset must have exactly one restricted outlet")
    fault = restricted[0]
    if any(key != fault and not MIN_FLOW <= initial[key] <= MAX_FLOW for key in OUTLETS):
        raise ValueError("The three healthy outlets must start within the reference band")

    restored = all(MIN_FLOW <= ratio <= MAX_FLOW for ratio in final.values())
    score = 0.0
    if intact and diagnosed == fault:
        score = 1.0 if restored and recovered else 0.25
    return {"score": score, "restricted_outlet": fault, "diagnosis_correct": diagnosed == fault,
            "flow_restored": restored, "obstruction_recovered": recovered, "fixture_intact": intact,
            "before_percent": {key: round(100 * ratio, 1) for key, ratio in initial.items()},
            "after_percent": {key: round(100 * ratio, 1) for key, ratio in final.items()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("measurements", type=Path)
    parser.add_argument("--diagnosed", choices=OUTLETS, required=True)
    parser.add_argument("--fixture-intact", action="store_true")
    parser.add_argument("--obstruction-recovered", action="store_true")
    args = parser.parse_args()
    try:
        measurements = read_measurements(args.measurements)
        result = evaluate(*measurements, args.diagnosed, args.fixture_intact,
                          args.obstruction_recovered)
    except (OSError, KeyError, ValueError) as exc:
        parser.error(str(exc))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
