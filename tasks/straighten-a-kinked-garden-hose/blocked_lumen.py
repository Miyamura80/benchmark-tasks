"""Replay the same arm commands with a restrictive sleeve on the crease."""
import argparse
import json
import numpy as np
from pathlib import Path
from run import Hose, Station

if __name__ == "__main__":
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("commands", type=Path)
    cli.add_argument("output", type=Path)
    cli.add_argument("--half-gap-mm", type=float, default=1.0)
    args = cli.parse_args()
    sim = Station(Hose())
    # Fit the sleeve after measuring the unoccluded reference.
    joint = sim.model.joint("lumen_gap").id
    sim.model.jnt_range[joint,1] = args.half_gap_mm/1000
    for action in json.loads(args.commands.read_text()):
        sim.action(np.asarray(action).reshape(2,7))
    result = sim.reading()
    args.output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result))
