"""Run the physical fixture with a dual-YAM scripted policy or inactivity control."""

import argparse
import json
from pathlib import Path

import numpy as np

from sweeper import Sweeper
from station import HZ, MAX_STEPS, Station

FIXTURES = {"sweeper": Sweeper}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fixture", choices=FIXTURES)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--variant", type=int, choices=[0, 1], default=0)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--control", dest="mode", action="store_const", const="inactivity control")
    mode.add_argument("--replay", type=Path)
    mode.add_argument("--detail-control", dest="mode", action="store_const", const="specific failure control")
    mode.add_argument("--short-return", dest="mode", action="store_const", const="incomplete drawer return",
                      help="Release the drawer before it meets the back stop")
    parser.set_defaults(mode="scripted")
    parser.add_argument("--video", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    render = None
    if args.video:
        from video import Recorder

        render = Recorder(args.out)
    fixture = FIXTURES[args.fixture](args.variant)
    trial = Station(fixture, render=render)
    status = None
    try:
        trial.trace.append(trial.reading())
        if args.replay:
            commands = json.loads(args.replay.read_text())
            array = np.asarray(commands, dtype=float)
            if (
                array.ndim != 2
                or array.shape[1] != 14
                or not 1 <= len(array) <= MAX_STEPS
                or not np.isfinite(array).all()
            ):
                raise ValueError(
                    "Expected 1 to 420 finite 14-dimensional dual-YAM commands"
                )
            trial.phase = "Replaying recorded robot commands"
            for row in array:
                trial.action(row.reshape(2, 7))
        else:
            trial.wait(0.45)
            if args.mode == "inactivity control":
                fixture.control(trial)
            elif args.mode == "specific failure control":
                fixture.detail_control(trial)
            else:
                fixture.demo(trial, short_return=args.mode == "incomplete drawer return")
    except Exception as error:
        status = str(error)
        raise
    finally:
        if render:
            render.close()
        result = {
            **trial.reading(),
            "title": fixture.title,
            "variant": args.variant,
            "mode": "replay" if args.replay else args.mode,
            "simulator": "MuJoCo dual YAM",
            "control_hz": HZ,
            "error": status,
        }
        (args.out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        (args.out / "trace.jsonl").write_text(
            "".join(json.dumps(r) + "\n" for r in trial.trace)
        )
        (args.out / "commands.json").write_text(json.dumps(trial.commands) + "\n")
        print(json.dumps(result))


if __name__ == "__main__":
    main()
