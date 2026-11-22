"""Replay the task with one or both grippers forced to stay open."""
import argparse
import json
from unittest.mock import patch

from dual_yam import Arm
from task import run


def run_held_open(prefixes, *, bench=False):
    original = Arm.command

    def command(self, data, opening):
        if self.prefix in prefixes:
            opening = .035
        original(self, data, opening)

    with patch.object(Arm, "command", command):
        return run(None, bench=bench)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bench", action="store_true")
    args = parser.parse_args()
    cases = {
        "left jaw held open": ("L_",),
        "right jaw held open": ("R_",),
        "both jaws held open": ("L_", "R_"),
    }
    outcomes = {case: run_held_open(arms, bench=args.bench) for case, arms in cases.items()}
    print(json.dumps(outcomes, indent=2))
    expected = {
        "left jaw held open": .5,
        "right jaw held open": .5,
        "both jaws held open": 0,
    }
    assert all(outcomes[case]["final"]["score"] == score for case, score in expected.items())
    if args.bench:
        assert all(case["moving_arm_support_contacts"] == 0 for case in outcomes.values())
