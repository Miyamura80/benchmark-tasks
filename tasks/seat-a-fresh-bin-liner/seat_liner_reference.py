"""Run the 0/12 reset and cuff proof with local force-grasp proxies."""
import argparse
import json
from pathlib import Path

import mujoco as mj
import numpy as np

from cuff_walk import Pinches, aperture_samples, edge_lengths, opening_count, rim_sectors
from native_cuff import contacts
from probe import FACES, LIP_RADIUS, UNIT, WALL_HEIGHT, ease
from seat_liner import prepare_reset


STEP = .00125
RELEASE = 3.
DROP = 3.4
END = 4.8
PAIRS = ((28, 4), (20, 12))


def trial(mode):
    m, d, puck, puck_vel, puck_geom, rest, prep_strain = prepare_reset(None, .0036, False, STEP, .22)
    hands = Pinches(m, m.flex_vertbodyid, .22)
    lengths = edge_lengths(rest)
    samples = aperture_samples()
    initial = int(rim_sectors(d.flexvert_xpos).sum())
    if initial:
        raise ValueError(f"Expected zero seated sectors at reset, found {initial}.")
    frames = []
    history = []
    pull_starts = None
    first_contact = None
    peak_strain = 0.
    release_minimum = 12
    release_opening = 1.
    load_clearance = np.inf
    terminal_normal = []
    terminal_speed = []
    terminal_inside = True
    for tick in range(round(END / STEP) + 1):
        time = tick * STEP
        released = tick >= round(RELEASE / STEP)
        dropped = tick >= round(DROP / STEP)
        if tick == round(DROP / STEP):
            m.geom_contype[puck_geom] = m.geom_conaffinity[puck_geom] = 1
        d.xfrc_applied[:] = 0.
        d.qfrc_applied[puck_vel + 2] = 0. if dropped else .981
        hands.active.clear()
        phase = "Load" if dropped else "Release"
        if not released:
            stage = int(tick >= round(1.2 / STEP))
            local = (time - stage * 1.2) / 1.2
            pair = PAIRS[stage]
            if pull_starts is None or tick == round(1.2 / STEP):
                pull_starts = d.flexvert_xpos[list(pair)].copy()
                hands.release(0)
                hands.release(1)
            phase = ("Near side", "Far side")[stage]
            if mode == "proxy":
                for slot, vertex in enumerate(pair):
                    raised = np.r_[pull_starts[slot, :2], .310]
                    outward = np.r_[.146 * UNIT[vertex], .310]
                    finished = np.r_[.137 * UNIT[vertex], .241]
                    if local < .22:
                        target = ease(pull_starts[slot], raised, local / .22)
                    elif local < .52:
                        target = ease(raised, outward, (local - .22) / .30)
                    else:
                        target = ease(outward, finished, (local - .52) / .38)
                    hands.hold(d, slot, vertex, target)
        points = d.flexvert_xpos
        sectors = rim_sectors(points)
        count = int(sectors.sum())
        opening = opening_count(points, samples) / len(samples)
        _, hit, _, normal = contacts(m, d, {}, set(), puck_geom)
        height = float(d.xpos[puck, 2])
        radius = float(np.linalg.norm(d.xpos[puck, :2]))
        speed = float(np.linalg.norm(d.qvel[puck_vel:puck_vel + 3]))
        if released:
            release_minimum = min(release_minimum, count)
            release_opening = min(release_opening, opening)
        if dropped:
            load_clearance = min(load_clearance, height)
            if hit and first_contact is None:
                first_contact = time
        if time >= END - .35:
            terminal_normal.append(normal)
            terminal_speed.append(speed)
            terminal_inside &= radius < .085 and .025 <= height <= .090
        history.append([round(time, 6), count, opening, 0, int(bool(hands.active)), int(hit),
                        height, radius, speed, normal])
        if tick % round(.05 / STEP) == 0:
            strain = float(np.max(np.abs(edge_lengths(points) / lengths - 1.)))
            peak_strain = max(peak_strain, strain)
            frames.append(dict(time=round(time, 4), phase=phase, points=points.round(6).tolist(),
                               sectors=sectors.tolist(), grasps=list(hands.active.values()),
                               mouth_ratio=opening, peak_edge_strain=strain,
                               puck=d.xpos[puck].round(6).tolist() if dropped else None))
        if time < END:
            mj.mj_step(m, d)
    warnings = [item.number for item in d.warning]
    passed = (release_minimum == 12 and release_opening >= .85 and load_clearance >= .020
              and terminal_inside and max(terminal_speed) <= .020 and min(terminal_normal) > 0
              and first_contact is not None and not any(warnings))
    return dict(mode=mode, timestep_s=STEP, initial_sectors=initial,
                min_released_sectors=release_minimum, min_released_opening=release_opening,
                final_sectors=count, post_load_retained=bool(passed), first_film_contact_s=first_contact,
                minimum_loaded_puck_center_z_m=load_clearance, peak_sampled_max_abs_edge_strain=peak_strain,
                preparation_max_sampled_strain=prep_strain, warnings=warnings, frames=frames,
                physics_trace_keys=["time_s", "num_sectors", "aperture", "robot_touches_film",
                                    "assist_active", "puck_touches_film", "puck_z_m", "puck_radius_m",
                                    "puck_speed_m_s", "puck_film_normal_n"], physics_trace=history)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    results = [trial("proxy"), trial("passive")]
    trace = dict(results=results, release_s=RELEASE, drop_s=DROP, faces=FACES.tolist(),
                 wall_height=WALL_HEIGHT, lip_radius=LIP_RADIUS)
    args.output.write_text(json.dumps(trace, separators=(",", ":")))
    for result in results:
        print(json.dumps({key: value for key, value in result.items()
                          if key not in ("frames", "physics_trace")}))
