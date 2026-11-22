"""Independently re-evaluate the liner's visible terminal state from a sampled trace."""
import argparse
import json
from pathlib import Path

import numpy as np

from cuff_walk import aperture_samples, opening_count, rim_sectors
from probe import SEGMENTS, WALL_HEIGHT


def inspect_frame(frame, samples):
    points = np.asarray(frame["points"], dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("The trace contains missing or non-finite film coordinates.")
    sectors = int(rim_sectors(points).sum())
    opening = opening_count(points, samples) / len(samples)
    lowest = float(np.min(points[:, 2]))
    puck = None if frame.get("puck") is None else np.asarray(frame["puck"], dtype=float)
    return sectors, opening, lowest, puck


def audit_physics_steps(run, release, drop):
    rows = run.get("physics_trace")
    if not rows:
        return None
    cols = dict(zip(run["physics_trace_keys"], np.asarray(rows, dtype=float).T, strict=True))
    time = cols["time_s"]
    if not all(np.isfinite(col).all() for col in cols.values()):
        raise ValueError("Non-finite value in physics trace.")
    if not np.allclose(np.diff(time), run["timestep_s"], atol=1e-7, rtol=0):
        raise ValueError("Physics samples must be contiguous at the reported timestep.")
    after_release = time >= release - 1e-8
    after_drop = time >= drop - 1e-8
    terminal = time >= time[-1] - .35 - 1e-8
    if not after_release.any() or not after_drop.any() or time[-1] < drop + 1.35:
        raise ValueError("The physics trace is missing a release or full load observation.")
    rim = int(cols["num_sectors"][after_release].min())
    aperture = float(cols["aperture"][after_release].min())
    min_z = float(cols["puck_z_m"][after_drop].min())
    terminal_z = cols["puck_z_m"][terminal]
    max_speed = float(cols["puck_speed_m_s"][terminal].max())
    had_contact = bool(cols["puck_touches_film"][after_drop].any())
    normal = cols["puck_film_normal_n"][terminal]
    contact_fraction = float(np.mean(normal > 0.))
    hand_clear = (not cols["assist_active"][after_release].any()
                  and not cols["robot_touches_film"][after_drop].any())
    terminal_inside = (terminal_z.min() >= .025 and terminal_z.max() <= .090
                       and cols["puck_radius_m"][terminal].max() < .085)
    passed = (rim == 12 and aperture >= .85 and min_z >= .020 and max_speed <= .02
              and terminal_inside and had_contact and contact_fraction == 1. and hand_clear
              and not any(run.get("warnings", [])))
    return {"physics_samples": len(rows), "min_released_rim": rim,
            "min_released_opening": aperture, "min_loaded_puck_height_m": min_z,
            "max_terminal_puck_speed_m_s": max_speed, "terminal_puck_film_contact_fraction": contact_fraction,
            "hands_clear_during_load": bool(hand_clear), "retention_pass": bool(passed)}


def audit_run(run, release, drop):
    frames = run.get("frames", [])
    if not frames:
        return None
    samples = aperture_samples()
    data = [inspect_frame(frame, samples) for frame in frames]
    time = np.array([frame["time"] for frame in frames])
    if not np.isfinite(time).all() or not np.all(np.diff(time) > 0):
        raise ValueError("Sample timestamps must be finite and strictly increasing.")
    filmed_release = time >= release
    filmed_load = time >= drop
    if not filmed_release.any() or not filmed_load.any():
        raise ValueError("The recording does not include both release and the puck drop.")
    series = np.asarray([(s, o, z) for s, o, z, _ in data])
    released = series[filmed_release]
    loaded = series[filmed_load]
    if any(frame.get("puck") is not None for frame, after_drop in zip(frames, filmed_load)
           if not after_drop):
        raise ValueError("The video shows the puck active before the documented release.")
    puck_motion = [p for t, (_, _, _, p) in zip(time, data)
                   if p is not None and t >= max(drop, time[-1] - .35)]
    settled_inside = bool(puck_motion) and all(
        np.isfinite(p).all() and np.linalg.norm(p[:2]) < .085 and .025 <= p[2] <= .090
        for p in puck_motion
    ) and np.linalg.norm(np.ptp(np.asarray(puck_motion), axis=0)) <= .005
    stayed_above_floor = all(p is None or p[2] >= .020 for _, _, _, p in data)
    contact_time = run.get("first_film_contact_s")
    recorded_contact = bool(run.get("puck_touched_film", contact_time is not None) and contact_time is not None
                            and drop <= contact_time <= time[-1])
    grasps_clear = all(not frame.get("grasps", []) and
                       all(vertex is None for vertex in frame.get("pinch_vertex", []))
                       for frame, after_release
                       in zip(frames, filmed_release) if after_release)
    pads_clear = all(np.max(np.abs(frame.get("pad_film_normal_n", [0.])), initial=0.) < .001
                     for frame, after_drop in zip(frames, filmed_load) if after_drop)
    rim_open = bool(released[:, 1].min() >= .85)
    reaches_into_bin = bool(loaded[:, 2].max() < WALL_HEIGHT - .100)
    holds_cuff = bool(released[:, 0].min() == 12)
    passed = (rim_open and reaches_into_bin and holds_cuff and settled_inside and
              recorded_contact and grasps_clear and pads_clear and stayed_above_floor
              and not any(run.get("warnings", [])))
    return {
        "mode": run["mode"],
        "samples": len(frames),
        "maximum_sample_gap_s": round(float(np.diff(time).max()), 6),
        "initial_rim_sectors": int(series[0, 0]),
        "initial_hem_height_range_mm": [
            round(1000 * float(np.min(np.asarray(frames[0]["points"])[:SEGMENTS, 2])), 2),
            round(1000 * float(np.max(np.asarray(frames[0]["points"])[:SEGMENTS, 2])), 2),
        ],
        "minimum_released_rim_sectors": int(released[:, 0].min()),
        "minimum_loaded_rim_sectors": int(loaded[:, 0].min()),
        "minimum_released_opening_fraction": float(released[:, 1].min()),
        "sampled_grasps_clear": grasps_clear,
        "sampled_pads_clear_during_load": pads_clear,
        "reported_native_puck_contact": recorded_contact,
        "sampled_puck_settled_inside": bool(settled_inside),
        "sampled_puck_above_floor": bool(stayed_above_floor),
        "sampled_liner_reaches_into_bin": reaches_into_bin,
        "sampled_retention_pass": bool(passed),
        "physics_step_audit": audit_physics_steps(run, release, drop),
    }


def audit_rigid_contacts(run, stock):
    import mujoco as mj

    from native_cuff import MOUNTS, scene_xml
    from probe import model

    mount_x = run.get("mount_x")
    scene, state, puck, _, _ = model(
        True, lambda xml: scene_xml(xml, stock, run.get("stiff_pads", False), mount_x),
    )
    rest = state.flexvert_xpos.copy()
    vertices = scene.flex_vertbodyid
    film_joints = scene.jnt_qposadr[scene.body_jntadr[vertices]][:, None] + np.arange(3)
    prefixes = tuple(entry[0] for entry in MOUNTS)
    names = [f"joint{i}" for i in range(1, 7)] + ["left_finger", "right_finger"]
    robot_joints = [[scene.joint(prefix + name).qposadr[0] for name in names] for prefix in prefixes]
    robot_geoms = {i for i in range(scene.ngeom) if scene.body(scene.geom_bodyid[i]).name.startswith(prefixes)}
    puck_xyz = scene.jnt_qposadr[scene.body_jntadr[puck]]
    overlaps = []
    for frame in run["frames"]:
        state.qpos[film_joints] = np.asarray(frame["points"]) - rest
        for joints, values in zip(robot_joints, frame["arm_qpos"], strict=True):
            state.qpos[joints] = values
        if frame.get("puck") is not None:
            state.qpos[puck_xyz:puck_xyz + 3] = frame["puck"]
        mj.mj_forward(scene, state)
        for contact in state.contact:
            if min(contact.geom) >= 0 and robot_geoms.intersection(contact.geom) and contact.dist < 0:
                overlaps.append((frame["time"], float(-contact.dist)))
    return {
        "samples": len(run["frames"]),
        "maximum_rigid_overlap_m": max((overlap for _, overlap in overlaps), default=0.),
        "contacts_over_0p5_mm": sum(overlap > .0005 for _, overlap in overlaps),
        "initial_contacts_over_0p5_mm": sum(time == 0 and overlap > .0005 for time, overlap in overlaps),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("--yam-xml", type=Path, help="Also replay sampled rigid arm contacts.")
    args = parser.parse_args()
    trace = json.loads(args.trace.read_text())
    reports = []
    for run in trace["results"]:
        if run.get("frames"):
            report = audit_run(run, trace["release_s"], trace["drop_s"])
            if args.yam_xml is not None and "arm_qpos" in run["frames"][0]:
                report["sampled_rigid_contacts"] = audit_rigid_contacts(run, args.yam_xml)
            reports.append(report)
    print(json.dumps({"runs": reports}, indent=2))
