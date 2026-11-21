"""Replay a saved joint-command sequence from the same clean reset and report its score."""

import argparse, json
from pathlib import Path
import numpy as np
from lawn import Lawn
from yam_station import Station, MAX_STEPS


def replay(path, seed):
    rows = np.asarray(json.loads(path.read_text()), dtype=float)
    if rows.ndim not in (2, 3) or len(rows) > MAX_STEPS or not np.isfinite(rows).all():
        raise ValueError("Invalid command sequence")
    s = Station(Lawn(seed))
    for row in rows:
        s.action(row.reshape(2, 7))
    return s.reading()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("commands", type=Path)
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()
    print(json.dumps(replay(args.commands, args.seed), indent=2))
