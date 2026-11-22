"""Run the physical fixture with a dual-YAM scripted policy or inactivity control."""

import argparse
import json
from pathlib import Path

import numpy as np

from foundation import Foundation
from station import HZ, MAX_STEPS, Station

FIXTURES = {"foundation": Foundation}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fixture", choices=FIXTURES)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--variant", type=int, choices=[0], default=0)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--control", action="store_true")
    mode.add_argument("--replay", type=Path)
    mode.add_argument("--detail-control", action="store_true")
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
                    "Expected 1 to 1200 finite 14-dimensional dual-YAM commands"
                )
            trial.phase = "Replaying recorded robot commands"
            for row in array:
                trial.action(row.reshape(2, 7))
        else:
            trial.wait(0.45)
            if args.control:
                fixture.control(trial)
            elif args.detail_control:
                fixture.detail_control(trial)
            else:
                fixture.demo(trial)
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
            "mode": (
                "inactivity control"
                if args.control
                else (
                    "replay"
                    if args.replay
                    else (
                        "specific failure control"
                        if args.detail_control
                        else "scripted"
                    )
                )
            ),
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
