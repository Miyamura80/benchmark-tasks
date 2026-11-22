"""CPU contact probe for the opening and cuffing stages of bin relining."""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np


SEGMENTS = 32
WALL_HEIGHT = 0.270
WALL_RADIUS = 0.116
LIP_RADIUS = 0.124
FILM_RADIUS = 0.00065
STEP = 0.0025
FRAME_STEPS = 20
GRASP_LIMIT = 0.7
WALL_LAYERS = 7
ANGLE = np.arange(SEGMENTS) * (2 * np.pi / SEGMENTS)
UNIT = np.column_stack((np.cos(ANGLE), np.sin(ANGLE)))
FACES = np.array([
    face
    for layer in range(WALL_LAYERS - 1)
    for i in range(SEGMENTS)
    for j in [(i + 1) % SEGMENTS]
    for face in (
        (layer * SEGMENTS + i, layer * SEGMENTS + j, (layer + 1) * SEGMENTS + j),
        (layer * SEGMENTS + i, (layer + 1) * SEGMENTS + j, (layer + 1) * SEGMENTS + i),
    )
] + [
    ((WALL_LAYERS - 1) * SEGMENTS + i,
     (WALL_LAYERS - 1) * SEGMENTS + (i + 1) % SEGMENTS,
     WALL_LAYERS * SEGMENTS)
    for i in range(SEGMENTS)
])


def liner(seated):
    if seated:
        # Independently seeded reference used only to challenge the retention assay.
        sections = [(0.132, 0.132, .236), (.132, .132, .277), (.105, .105, .282),
                    (.105, .105, .217), (.100, .100, .154), (.097, .097, .085),
                    (.089, .089, .027)]
        rings = [np.column_stack((UNIT * (rx, ry), np.full(SEGMENTS, z)))
                 for rx, ry, z in sections]
    else:
        # A bench diagnostic with the liner already introduced into the can.
        # The two lips initially have 8 mm separation across the centre.
        sections = [(.191, .004, .319), (.163, .006, .304), (.106, .012, .279),
                    (.094, .018, .216), (.085, .020, .144), (.082, .023, .075),
                    (.074, .021, .024)]
        rings = []
        for rx, ry, z in sections:
            # Evenly spaced samples along the flattened closed ring. This
            # avoids tiny edges at the sides and long edges at the lip centre.
            fine = np.linspace(0, 2 * np.pi, 4097)
            xy = np.column_stack((rx * np.cos(fine), ry * np.sin(fine)))
            distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
            samples = distance[-1] * np.arange(SEGMENTS) / SEGMENTS
            rings.append(np.column_stack(
                (np.interp(samples, distance, xy[:, 0]),
                 np.interp(samples, distance, xy[:, 1]), np.full(SEGMENTS, z))))
    return np.vstack([*rings, [[0, 0, .024]]])


def model(seated, transform_scene=None, *, film_mass=.026, timestep=STEP):
    if not np.isfinite(film_mass) or film_mass <= 0:
        raise ValueError("Film mass must be positive and finite.")
    if not np.isfinite(timestep) or timestep <= 0:
        raise ValueError("Physics timestep must be positive and finite.")
    def numbers(values):
        return " ".join(f"{x:.7g}" for x in np.asarray(values).ravel())

    walls = []
    for i, (x, y) in enumerate(UNIT):
        for name, radius, z, size in (
            ("wall", WALL_RADIUS, WALL_HEIGHT / 2, (.0025, .0119, WALL_HEIGHT / 2)),
            ("lip", .120, WALL_HEIGHT, (.004, .0124, .005)),
        ):
            walls.append(f'<geom name="{name}{i}" type="box" pos="{radius*x} {radius*y} {z}" '
                         f'size="{numbers(size)}" friction="0.55 0.01 0.001" '
                         f'euler="0 0 {ANGLE[i]}" rgba=".27 .31 .36 1"/>')
    points = liner(seated)
    ring_xml = "\n".join(walls)
    xml = f"""
    <mujoco model="thin liner diagnostic">
      <compiler angle="radian"/>
      <option timestep="{timestep}" solver="CG" iterations="45" integrator="implicitfast"/>
      <size memory="128M"/>
      <worldbody>
        <geom name="base" type="cylinder" pos="0 0 .01" size=".119 .01"/>
        {ring_xml}
        <flexcomp name="film" type="direct" dim="2" point="{numbers(points)}"
                  element="{numbers(FACES)}" radius="{FILM_RADIUS}" mass="{film_mass}"
                  rgba=".34 .83 .77 1">
          <edge equality="true" damping=".03"/>
          <contact selfcollide="auto" internal="false" friction=".45 .01 .001"/>
        </flexcomp>
        <body name="proof_puck" pos="0 0 .38">
          <freejoint/>
          <geom name="proof_puck_geom" type="cylinder" size=".028 .010" mass=".1" rgba=".9 .64 .25 1"/>
        </body>
        <geom name="proof_floor" type="plane" pos="0 0 .020" size=".115 .115 .001"
              contype="0" conaffinity="0" rgba="0 0 0 0"/>
      </worldbody>
      <contact>
        <pair geom1="proof_floor" geom2="proof_puck_geom" solref=".006 1"
              solimp=".95 .99 .001"/>
      </contact>
    </mujoco>
    """
    if transform_scene is not None:
        xml = transform_scene(xml)
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    puck_id = m.body("proof_puck").id
    puck_joint = m.body_jntadr[puck_id]
    puck_velocity = m.jnt_dofadr[puck_joint]
    puck_geom = m.body_geomadr[puck_id]
    # The puck starts parked in every case. Reset its position only before the
    # trial. Gravity is countered until the independent evaluator releases it.
    m.geom_contype[puck_geom] = 0
    m.geom_conaffinity[puck_geom] = 0
    mujoco.mj_forward(m, d)
    return m, d, puck_id, puck_velocity, puck_geom


def mouth_clearance(points):
    xy = points[:SEGMENTS, :2]
    following = np.roll(xy, -1, axis=0)
    area = abs(np.sum(xy[:, 0] * following[:, 1] - following[:, 0] * xy[:, 1])) / 2
    return float(area / (np.pi * WALL_RADIUS**2))


def rim_fraction(points):
    # Verify the film crosses the top of the rim in every covered angular
    # sector: a lip outside/below plus contiguous material on the inside.
    hem = points[:SEGMENTS]
    sectors = set()
    for i, (x, y, z) in enumerate(hem):
        if np.hypot(x, y) < LIP_RADIUS + .001 or z > WALL_HEIGHT - .009:
            continue
        column = points[i::SEGMENTS][:WALL_LAYERS]
        radial = np.linalg.norm(column[:, :2], axis=1)
        crossing = (radial[:-1] >= LIP_RADIUS) & (radial[1:] <= LIP_RADIUS)
        fraction = np.divide(radial[:-1] - LIP_RADIUS, radial[:-1] - radial[1:],
                             out=np.zeros(len(radial) - 1), where=crossing)
        height = column[:-1, 2] + fraction * np.diff(column[:, 2])
        if not np.any(crossing & (height >= WALL_HEIGHT + .003)):
            continue
        angle = (np.arctan2(y, x) + 2 * np.pi) % (2 * np.pi)
        sectors.add(int(np.floor(angle * 12 / (2 * np.pi) + .5)) % 12)
    return len(sectors) / 12


def ease(start, end, t):
    t = np.clip(t, 0.0, 1.0)
    return start + (end - start) * (t * t * (3 - 2 * t))


def apply_grasp(m, d, film_body, targets, attachments):
    # The two grasps are compliant bounded pulls on a localized patch.
    # Their capture is a geometric proxy, with no YAM geometry or jaw force.
    for i, target in targets.items():
        body = film_body[i]
        position = d.flexvert_xpos[i]
        if i not in attachments and np.linalg.norm(position - target) <= .003:
            attachments.add(i)
        if i in attachments:
            force = 25 * (target - position) - .15 * d.qvel[m.body_dofadr[body]:
                                                               m.body_dofadr[body] + 3]
            length = np.linalg.norm(force)
            if length > GRASP_LIMIT:
                force *= GRASP_LIMIT / length
            d.xfrc_applied[body, :3] = force


def trial(mode):
    m, d, puck, puck_velocity, puck_geom = model(mode == "seated_reference")
    film_body = m.flex_vertbodyid[:len(liner(False))]
    m.geom_friction[puck_geom, 0] = .5
    start = d.flexvert_xpos.copy()
    pairs = ((8, 24), (0, 16))
    lift_first = mode == "lift_first"
    lowering = .96 if lift_first else .46
    regrasp = lowering + .42
    final_release = regrasp + .25
    puck_release = final_release + .29
    end = puck_release + 1.68
    frames = []
    attached = set()
    second_origin = None
    max_warning = np.zeros(len(d.warning), dtype=int)
    puck_film_contact = False
    puck_settled = True
    stayed_above_base = True
    for step in range(round(end / STEP)):
        time = step * STEP
        d.xfrc_applied[:] = 0
        if time < puck_release:
            d.qfrc_applied[puck_velocity + 2] = .981
        else:
            d.qfrc_applied[puck_velocity + 2] = 0
            m.geom_contype[puck_geom] = 1
            m.geom_conaffinity[puck_geom] = 1

        stage = "Release"
        targets = {}
        if mode in ("open", "lift_first"):
            if time < regrasp:
                stage = "Open the mouth" if time < lowering else "Lower first pair"
                if lift_first and time < .44:
                    stage = "Lift both lips"
                for i in pairs[0]:
                    _, sign = UNIT[i]
                    high = np.array([0, sign * .145, .308])
                    low = np.array([0, sign * .137, .239])
                    if lift_first:
                        lifted = np.array([0, sign * .013, .474])
                        spread = np.array([0, sign * .132, .427])
                        if time < .44:
                            targets[i] = ease(start[i], lifted, time / .44)
                        elif time < .74:
                            targets[i] = ease(lifted, spread, (time - .44) / .30)
                        elif time < lowering:
                            targets[i] = ease(spread, high, (time - .74) / (lowering - .74))
                        else:
                            targets[i] = ease(high, low, (time - lowering) / (regrasp - lowering))
                    elif time < lowering:
                        targets[i] = ease(start[i], high, time / lowering)
                    else:
                        targets[i] = ease(high, low, (time - lowering) / (regrasp - lowering))
            elif time < final_release:
                stage = "Try opposite sides"
                if second_origin is None:
                    attached.clear()
                    second_origin = d.flexvert_xpos[list(pairs[1])].copy()
                for i, origin in zip(pairs[1], second_origin):
                    sign, _ = UNIT[i]
                    targets[i] = ease(origin, np.array([sign * .140, 0, .239]),
                                      (time - regrasp) / (final_release - regrasp))
            apply_grasp(m, d, film_body, targets, attached)

        if not targets:
            attached.clear()
        if time >= puck_release:
            stage = "100 g puck test"
        if step % FRAME_STEPS == 0 or step == round(end / STEP) - 1:
            points = d.flexvert_xpos.copy()
            frames.append({
                "time": round(time, 4),
                "phase": stage,
                "points": points.round(6).tolist(),
                "grasps": [targets[i].round(6).tolist() for i in targets if i in attached],
                "puck": d.xpos[puck].round(6).tolist() if time >= puck_release else None,
                "mouth_ratio": round(mouth_clearance(points), 4),
                "rim_fraction": round(rim_fraction(points), 4),
            })
        mujoco.mj_step(m, d)
        max_warning = np.maximum(max_warning, [w.number for w in d.warning])
        if time >= puck_release:
            stayed_above_base &= bool(d.xpos[puck, 2] >= .018)
        if time >= end - .35:
            puck_settled &= bool(.025 <= d.xpos[puck, 2] <= .090
                                 and np.linalg.norm(d.xpos[puck, :2]) < .085
                                 and np.linalg.norm(d.qvel[puck_velocity:puck_velocity + 3]) <= .02)
        if time >= puck_release and not puck_film_contact:
            puck_film_contact = any(
                puck_geom in contact.geom and 0 in contact.flex
                for contact in d.contact
            )
    last = [f for f in frames if f["time"] >= end - .5]
    puck_inside = puck_settled and stayed_above_base
    retained = (puck_film_contact and puck_inside
                and all(f["rim_fraction"] >= .999 and f["mouth_ratio"] >= .85
                        and not f["grasps"] for f in last))
    return {
        "mode": mode,
        "peak_mouth_ratio": max(f["mouth_ratio"] for f in frames),
        "peak_rim_fraction": max(f["rim_fraction"] for f in frames),
        "last_half_second_min_rim_fraction": min(f["rim_fraction"] for f in last),
        "puck_touched_film": puck_film_contact,
        "puck_settled_inside": puck_inside,
        "post_load_retained": retained,
        "warnings": max_warning.tolist(),
        "frames": frames,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("probe.json"))
    parser.add_argument("--mode", choices=("all", "open", "lift_first", "no_grasp", "seated_reference"),
                        default="all")
    args = parser.parse_args()
    modes = ("open", "lift_first", "no_grasp", "seated_reference") if args.mode == "all" else (args.mode,)
    results = [trial(mode) for mode in modes]
    args.output.write_text(json.dumps({"results": results, "faces": FACES.tolist(),
                                       "wall_height": WALL_HEIGHT,
                                       "lip_radius": LIP_RADIUS}, separators=(",", ":")))
    for result in results:
        print(json.dumps({k: v for k, v in result.items() if k != "frames"}))


if __name__ == "__main__":
    main()
