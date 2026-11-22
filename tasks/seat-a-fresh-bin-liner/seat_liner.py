"""Seat an open, uncuffed liner with two sequential pairs of YAM grasps."""
import argparse
from functools import partial
import json
from pathlib import Path

import mujoco as mj
import numpy as np

from loose_reset import four_point_preparation
from native_cuff import CLOSED, OPEN, jaw_direction, run, scene_xml
from probe import FACES, LIP_RADIUS, SEGMENTS, UNIT, WALL_HEIGHT, ease, model


def prepare_reset(stock, film_mass, stiff_pads, timestep, _grip_scale, *, offset=0, mount_x=None):
    source, data, _, puck_vel, _ = model(True, film_mass=film_mass, timestep=timestep)
    reference = data.flexvert_xpos.copy()
    peak_strain = four_point_preparation(source, data, puck_vel, reference, .108, .20, offset)
    if any(warning.number for warning in data.warning):
        raise RuntimeError("Simulation warning while preparing the uncuffed reset.")
    transform = None if stock is None else lambda xml: scene_xml(xml, stock, stiff_pads, mount_x)
    scene, state, puck, velocity, geom = model(
        True, transform, film_mass=film_mass, timestep=timestep,
    )
    assert np.array_equal(source.jnt_qposadr, scene.jnt_qposadr[:source.njnt])
    assert np.array_equal(source.jnt_dofadr, scene.jnt_dofadr[:source.njnt])
    state.qpos[:source.nq] = data.qpos
    state.qvel[:source.nv] = data.qvel
    mj.mj_forward(scene, state)
    return scene, state, puck, velocity, geom, reference, peak_strain


class SeatingMotion:
    def __init__(self, pairs, spread, lift, lower, height, settle, orientation, transfer):
        self.pairs = pairs
        self.spread = spread
        self.lift = lift
        self.lower = lower
        self.height = height
        self.orientation = orientation
        self.transfer = transfer
        self.starts = None
        self.retract_from = None
        self.anchor = ([a == b for a, b in zip(*pairs, strict=True)]
                       if len(pairs) == 2 else [False, False])
        self.first_release = 2.20
        self.second_grasp = round(self.first_release + transfer, 6)
        self.second_starts = None
        release = round((self.first_release if len(pairs) == 1 else self.second_grasp + 2.20) + settle, 6)
        self.evaluation_times = release, round(release + .80, 6), round(release + 2.20, 6)

    def pull(self, starts, pair, t):
        down = np.c_[self.lower * UNIT[list(pair)], np.full(2, self.height)]
        if t < .35:
            return starts, "Close jaws"
        raised = starts.copy()
        raised[:, 2] = np.maximum(starts[:, 2], self.lift)
        if t < .60:
            return ease(starts, raised, (t - .35) / .25), "Lift"
        outward = np.c_[self.spread * UNIT[list(pair)], np.full(2, self.lift)]
        if t < 1.05:
            return ease(raised, outward, (t - .60) / .45), "Spread"
        return ease(outward, down, (t - 1.05) / .60), "Fold down"

    def withdraw(self, pair, t):
        down = np.c_[self.lower * UNIT[list(pair)], np.full(2, self.height)]
        clear = down + np.c_[.022 * UNIT[list(pair)], np.full(2, .13)]
        return ease(down, clear, (t - .16) / .50), "Release", OPEN, False

    def command(self, t, state, arms):
        t = round(t, 6)
        first, *remaining = self.pairs
        release = self.evaluation_times[0]
        if t >= release:
            return self.withdraw(self.pairs[-1], t - release)
        if self.starts is None:
            self.starts = state.flexvert_xpos[list(first)].copy()
        if not remaining or t < self.first_release:
            targets, phase = self.pull(self.starts, first, t)
            return targets, phase, ease(OPEN, CLOSED, t/.15), True
        second = remaining[0]
        if t < self.second_grasp:
            if self.retract_from is None:
                self.retract_from = np.array([arm.tip_positions(state).mean(axis=0) for arm in arms])
            fraction = (t - self.first_release) / self.transfer
            end = state.flexvert_xpos[list(second)].copy()
            crest = max(.341, float(np.max(end[:, 2])) + .045)
            raised = self.retract_from.copy()
            raised[:, 2] = crest
            over = end.copy()
            over[:, 2] = crest
            anchored = np.asarray(self.anchor)
            open_values = np.where(anchored, CLOSED, OPEN)
            if fraction < .34:
                targets, phase = ease(self.retract_from, raised, fraction/.34), "Release near side"
            else:
                for slot, (arm, vertex) in enumerate(zip(arms, second, strict=True)):
                    arm.orient(1 if slot == 0 else -1, jaw_direction(vertex, self.orientation))
                if fraction < .72:
                    targets, phase = ease(raised, over, (fraction-.34)/.38), "Move to far side"
                else:
                    targets, phase = ease(over, end, (fraction-.72)/.28), "Approach far hem"
            targets[anchored] = self.retract_from[anchored]
            return targets, phase, open_values, self.anchor
        if self.second_starts is None:
            self.second_starts = state.flexvert_xpos[list(second)].copy()
        local = t - self.second_grasp
        targets, phase = self.pull(self.second_starts, second, local)
        anchored = np.asarray(self.anchor)
        targets = targets.copy()
        targets[anchored] = self.retract_from[anchored]
        opening = np.where(anchored, CLOSED, ease(OPEN, CLOSED, local/.15))
        return targets, phase + " far side", opening, True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yam-xml", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", choices=("assisted", "open", "pinch"), default="assisted")
    parser.add_argument("--first", nargs=2, type=int, choices=range(SEGMENTS), default=(24, 8))
    parser.add_argument("--second", nargs=2, type=int, choices=range(SEGMENTS), default=(16, 8))
    parser.add_argument("--spread", type=float, default=.146)
    parser.add_argument("--lift", type=float, default=.310)
    parser.add_argument("--lower", type=float, default=.137)
    parser.add_argument("--height", type=float, default=.241)
    parser.add_argument("--settle", type=float, default=.80)
    parser.add_argument("--transfer", type=float, default=2.15)
    parser.add_argument("--one-pair", action="store_true")
    parser.add_argument("--mount-x", type=float, default=.02)
    parser.add_argument("--offset", type=int, default=0)
    parser.add_argument("--orientation", choices=("tangent", "radial", "x"), default="tangent")
    args = parser.parse_args()

    pairs = (args.first,) if args.one_pair else (args.first, args.second)
    motion = SeatingMotion(pairs, args.spread, args.lift, args.lower, args.height, args.settle,
                           args.orientation, args.transfer)
    trial = run(args.yam_xml, args.mode, args.spread, args.lift, args.lower, args.orientation,
                film_mass=.0036, timestep=.00125, grip_scale=.22,
                configure_reset=partial(prepare_reset, offset=args.offset, mount_x=args.mount_x),
                pairs=pairs, contact_schedule=motion, require_uncuffed=True)
    release, drop, _ = motion.evaluation_times
    trial.update(scope="seat_uncuffed_liner", grasp_pairs=pairs, transfer_s=args.transfer, settle_s=args.settle,
                 mount_x=args.mount_x, reset_offset=args.offset, lower_height_m=args.height)
    trace = dict(results=[trial], faces=FACES.tolist(), wall_height=WALL_HEIGHT,
                 lip_radius=LIP_RADIUS, release_s=release, drop_s=drop)
    args.output.write_text(json.dumps(trace, separators=(",", ":")))
    keys = ("mode", "initial_sectors", "final_sectors", "min_released_sectors", "min_released_opening",
            "post_load_retained", "preparation_max_sampled_strain", "peak_sampled_max_abs_edge_strain",
            "pinch_events", "warnings")
    print(json.dumps({key: trial[key] for key in keys}))


if __name__ == "__main__":
    main()
