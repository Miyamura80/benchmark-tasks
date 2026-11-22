"""MuJoCo experiment for pulling an uncuffed section over a bin rim."""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from probe import (
    FACES, FILM_RADIUS, LIP_RADIUS, SEGMENTS, STEP, UNIT, WALL_HEIGHT,
    WALL_LAYERS, ease, model,
)

HEM_SECTORS = 12
DROP_MASS = .100
RELEASE = 2.85
DROP = 3.25
END = 4.65
ACTION_PERIOD = .050
GRASP_FORCE = .7
MIN_OVERHANG = .009
MIN_OPENING = .85
TEST_COUNT = 80
EDGES = np.unique(np.sort(
    np.column_stack((FACES[:, [0, 1, 2]].ravel(), FACES[:, [1, 2, 0]].ravel())), axis=1
), axis=0)


def make_reset(m, d, film_bodies, centre_deg, span_deg):
    """Unroll the chosen cuff arc before time zero, keeping the rest mesh fixed."""
    ref = d.flexvert_xpos.copy()
    angles = np.arctan2(UNIT[:, 1], UNIT[:, 0])
    difference = (angles - np.deg2rad(centre_deg) + np.pi) % (2 * np.pi) - np.pi
    inner = np.deg2rad(span_deg / 2)
    ramp = np.clip((inner + np.deg2rad(20) - np.abs(difference)) / np.deg2rad(20), 0, 1)
    angle = np.deg2rad(137) * (ramp**2 * (3 - 2 * ramp))
    reset = ref.copy()
    seam = ref[SEGMENTS:2 * SEGMENTS]
    height = seam[:, 2] - ref[:SEGMENTS, 2]
    reset[:SEGMENTS, :2] -= (height * np.sin(angle))[:, None] * UNIT
    reset[:SEGMENTS, 2] += height * (1 - np.cos(angle))
    for i, body in enumerate(film_bodies[:SEGMENTS]):
        joint = m.body_jntadr[body]
        coordinate = m.jnt_qposadr[joint]
        axes = m.jnt_axis[joint:joint + 3]
        d.qpos[coordinate:coordinate + 3] += axes @ (reset[i] - ref[i])
    mujoco.mj_forward(m, d)
    assert np.max(np.abs(d.flexvert_xpos - reset)) < 1e-8
    return ref, difference


def rim_sectors(points):
    """A sector needs a local hem-to-interior path across the lip crown."""
    rings = points[:SEGMENTS * WALL_LAYERS].reshape(WALL_LAYERS, SEGMENTS, 3)
    hem = rings[0]
    radial = np.linalg.norm(rings[:, :, :2], axis=2)
    crossing = (radial[:-1] >= LIP_RADIUS) & (radial[1:] < LIP_RADIUS)
    denominator = radial[:-1] - radial[1:]
    fraction = np.divide(radial[:-1] - LIP_RADIUS, denominator,
                         out=np.zeros_like(denominator), where=crossing)
    crossing_z = rings[:-1, :, 2] + fraction * np.diff(rings[:, :, 2], axis=0)
    over_lip = np.any(crossing & (crossing_z > WALL_HEIGHT + .003), axis=0)
    exterior = (radial[0] >= LIP_RADIUS + .001) & (hem[:, 2] <= WALL_HEIGHT - MIN_OVERHANG)
    interior = np.any((radial < LIP_RADIUS - .011) & (rings[:, :, 2] < WALL_HEIGHT - .025), axis=0)
    sectors = np.zeros(HEM_SECTORS, dtype=bool)
    angle = np.arctan2(hem[:, 1], hem[:, 0])
    ids = np.round(angle * HEM_SECTORS / (2 * np.pi)).astype(int) % HEM_SECTORS
    sectors[ids[over_lip & exterior & interior]] = True
    return sectors


def opening_count(points, samples):
    """Even-odd fill of the projected mouth, evaluated inside the bin."""
    hem = points[:SEGMENTS, :2]
    following = np.roll(hem, -1, axis=0)
    xi, yi = hem.T[:, None, :]
    xj, yj = following.T[:, None, :]
    x, y = samples.T[:, :, None]
    # The divisor is ignored on edges which cannot cross the horizontal ray.
    delta = np.where(np.abs(yj - yi) > 1e-12, yj - yi, 1.)
    crossings = ((yi > y) != (yj > y)) & (x < (xj - xi) * (y - yi) / delta + xi)
    return np.count_nonzero(np.count_nonzero(crossings, axis=1) % 2)


def edge_lengths(points):
    return np.linalg.norm(points[EDGES[:, 0]] - points[EDGES[:, 1]], axis=1)


def aperture_samples():
    radius = np.sqrt((np.arange(TEST_COUNT) + .5) / TEST_COUNT) * (LIP_RADIUS - .011)
    angle = np.arange(TEST_COUNT) * np.pi * (3 - np.sqrt(5))
    return radius[:, None] * np.column_stack((np.cos(angle), np.sin(angle)))


class Pinches:
    """Two force-limited local grasp proxies with bounded approach and release."""

    def __init__(self, m, bodies, grip_scale=1.):
        self.bodies = bodies
        self.velocities = m.body_dofadr[bodies]
        self.grip_scale = grip_scale
        self.captures = {}
        self.active = {}

    def hold(self, d, slot, vertex, target):
        key = (slot, vertex)
        offset = target - d.flexvert_xpos[vertex]
        distance = np.linalg.norm(offset)
        if self.captures.get(slot) != key:
            if distance > .003:
                return False
            self.captures[slot] = key
        elif distance > .032:
            self.release(slot)
            return False
        v = self.velocities[vertex]
        force = 25 * offset - .15 * d.qvel[v:v + 3]
        force *= min(1., GRASP_FORCE / max(np.linalg.norm(force), 1e-12))
        d.xfrc_applied[self.bodies[vertex], :3] += self.grip_scale * force
        self.active[slot] = target.tolist()
        return True

    def release(self, slot):
        self.captures.pop(slot, None)
        self.active.pop(slot, None)



def prepare_by_contact(m, d, film_bodies, vertices, puck_vel, opening_radius, *, grip_scale=1.):
    """Reproduce a loose cuff segment by lifting and releasing two pinches."""
    hands = Pinches(m, film_bodies, grip_scale)
    rest = d.flexvert_xpos.copy()
    rest_lengths = edge_lengths(rest)
    peak_strain = 0.
    timestep = m.opt.timestep
    for tick in range(round(1.65 / timestep)):
        t = tick * timestep
        d.xfrc_applied[:] = 0
        d.qfrc_applied[puck_vel + 2] = DROP_MASS * 9.81
        hands.active.clear()
        if t < 1.45:
            for slot, vertex in enumerate(vertices):
                direction = UNIT[vertex]
                start = rest[vertex]
                raised = np.r_[.143 * direction, .307]
                inside = np.r_[opening_radius * direction, .300]
                target = (ease(start, raised, t / .60) if t < .60 else
                          ease(raised, inside, (t - .60) / .55))
                hands.hold(d, slot, vertex, target)
        mujoco.mj_step(m, d)
        if tick % 10 == 0:
            strain = np.max(np.abs(edge_lengths(d.flexvert_xpos) / rest_lengths - 1.))
            peak_strain = max(peak_strain, float(strain))
    d.time = 0.
    return peak_strain

def crown_route(start, direction, t):
    up = np.r_[.118 * direction, .318]
    over = np.r_[.153 * direction, .310]
    down = np.r_[.141 * direction, .232]
    return (
        ease(start, up, t / .22) if t < .22 else
        ease(up, over, (t - .22) / .25) if t < .47 else
        ease(over, down, (t - .47) / .30)
    )


def trial(mode, centre=0., span=25., lift=.301, spread=.141, offset=3,
          reset_method="contact", opening_radius=.110, film_mass=.026,
          timestep=STEP, grip_scale=1.):
    m, d, puck, puck_vel, puck_geom = model(True, film_mass=film_mass, timestep=timestep)
    film_bodies = m.flex_vertbodyid.copy()
    hand = Pinches(m, film_bodies, grip_scale)
    rest = d.flexvert_xpos.copy()
    if mode == "below_rim_control":
        interior = rest.copy()
        interior[:, :2] *= .80
        interior[:, 2] = .04 + .71 * interior[:, 2]
        count = int(rim_sectors(interior).sum())
        return {"mode": mode, "initial_sectors": count, "post_load_retained": False}

    angles = np.arctan2(UNIT[:, 1], UNIT[:, 0])
    difference = (angles - np.deg2rad(centre) + np.pi) % (2 * np.pi) - np.pi
    middle = int(np.argmin(abs(difference)))
    paired = ((middle - offset) % SEGMENTS, (middle + offset) % SEGMENTS)
    ref = rest
    preparation_peak_strain = None
    if mode != "seated_reference":
        if reset_method == "contact":
            preparation_peak_strain = prepare_by_contact(m, d, film_bodies, paired,
                                                         puck_vel, opening_radius,
                                                         grip_scale=grip_scale)
        else:
            ref, difference = make_reset(m, d, film_bodies, centre, span)
    base = d.flexvert_xpos.copy()
    samples = aperture_samples()
    working_vertices = np.flatnonzero(np.abs(difference) <= np.deg2rad(span / 2 + 10))
    selected = sorted(working_vertices, key=lambda i: abs(difference[i]))
    workers = [selected[0]]
    if len(selected) > 3:
        workers += [max(selected, key=lambda i: difference[i]),
                    min(selected, key=lambda i: difference[i])]
    anchor = int((workers[0] + SEGMENTS // 2) % SEGMENTS)
    starts = {}
    proof_pass = True
    proof_ticks = 0
    contacted_film = False
    first_contact = None
    peak_sectors = 0
    minimum_released_opening = 1.0
    rest_lengths = edge_lengths(rest)
    reset_strain = float(np.max(np.abs(edge_lengths(base) / rest_lengths - 1.)))
    release_strain = None
    max_sampled_strain = reset_strain
    min_after_release = HEM_SECTORS
    min_during_proof = HEM_SECTORS
    retained_before_drop = None
    frames = []
    warnings = np.array([warning.number for warning in d.warning], dtype=int)
    endpoint = round(END / timestep)
    release = round(RELEASE / timestep)
    drop = round(DROP / timestep)

    for tick in range(endpoint + 1):
        time = tick * timestep
        is_released = tick >= release
        dropped = tick >= drop
        d.xfrc_applied[:] = 0
        hand.active.clear()
        d.qfrc_applied[puck_vel + 2] = 0 if dropped else DROP_MASS * 9.81
        if tick == drop:
            m.geom_contype[puck_geom] = m.geom_conaffinity[puck_geom] = 1
        phase = "Settle"
        if mode == "walk" and not is_released:
            # Hold the already-seated opposite edge. The other hand regrips
            # successive hem sections while the anchor prevents withdrawal.
            hand.hold(d, 0, anchor, ref[anchor])
            for pass_number, vertex in enumerate(workers):
                t = time - .24 - pass_number * .82
                if not 0 <= t < .81:
                    continue
                if vertex not in starts:
                    hand.release(1)
                    starts[vertex] = d.flexvert_xpos[vertex].copy()
                hand.hold(d, 1, vertex, crown_route(starts[vertex], UNIT[vertex], t))
                phase = f"Seat section {pass_number + 1}"
                break
        elif mode == "paired" and not is_released:
            for slot, vertex in enumerate(paired):
                direction = UNIT[vertex]
                start = base[vertex]
                raised = np.r_[start[:2], lift]
                outer = np.r_[spread * direction, lift]
                lowered = np.r_[.132 * direction, .236]
                if time <= .30:
                    phase = "Lift both sides"
                    target = ease(start, raised, time / .30)
                elif time <= 1.10:
                    phase = "Spread over the rim"
                    target = ease(raised, outer, (time - .30) / .80)
                elif time <= 2.30:
                    phase = "Lower both sides"
                    target = ease(outer, lowered, (time - 1.10) / 1.20)
                else:
                    phase = "Hold cuff"
                    target = lowered
                hand.hold(d, slot, vertex, target)
        elif is_released:
            hand.release(0)
            hand.release(1)
            phase = "100 g drop" if dropped else "Hands released"
        live_sectors = rim_sectors(d.flexvert_xpos)
        count = int(live_sectors.sum())
        peak_sectors = max(peak_sectors, count)
        opening = opening_count(d.flexvert_xpos, samples) / len(samples)
        if tick == drop:
            retained_before_drop = count
        if tick == release:
            release_strain = float(np.max(np.abs(edge_lengths(d.flexvert_xpos) / rest_lengths - 1.)))
        if is_released:
            min_after_release = min(min_after_release, count)
            minimum_released_opening = min(minimum_released_opening, opening)
            proof_pass &= count == HEM_SECTORS and opening >= MIN_OPENING and not hand.active
        if dropped:
            min_during_proof = min(min_during_proof, count)
            if (not contacted_film and any(contact.dist <= FILM_RADIUS
                                          and puck_geom in contact.geom
                                          and 0 in contact.flex for contact in d.contact)):
                contacted_film = True
                first_contact = round(time, 4)
            position = d.xpos[puck]
            speed = np.linalg.norm(d.qvel[puck_vel:puck_vel + 3])
            proof_pass &= position[2] >= .018
            settled = (np.linalg.norm(position[:2]) < .085
                       and .025 <= position[2] <= .090 and speed <= .02)
            if time >= END - .35:
                proof_pass &= settled
            proof_ticks += 1
        if tick % round(ACTION_PERIOD / timestep) == 0 or tick == endpoint:
            strain = float(np.max(np.abs(edge_lengths(d.flexvert_xpos) / rest_lengths - 1.)))
            max_sampled_strain = max(max_sampled_strain, strain)
            frames.append({
                "time": round(time, 4), "phase": phase,
                "points": d.flexvert_xpos.round(6).tolist(),
                "grasps": list(hand.active.values()),
                "puck": d.xpos[puck].round(6).tolist() if dropped else None,
                "mouth_ratio": opening,
                "rim_fraction": count / HEM_SECTORS, "sectors": live_sectors.tolist(),
                "peak_edge_strain": strain,
            })
        if tick < endpoint:
            mujoco.mj_step(m, d)
            warnings = np.maximum(warnings, [warning.number for warning in d.warning])

    return {
        "mode": mode, "centre_deg": centre, "span_deg": span,
        "lift_m": lift, "spread_m": spread, "grasp_offset_vertices": offset,
        "reset_method": reset_method, "opening_radius_m": opening_radius, "film_mass_kg": film_mass,
        "timestep_s": timestep, "grip_scale": grip_scale,
        "preparation_max_sampled_strain": preparation_peak_strain,
        "initial_sectors": int(rim_sectors(base).sum()), "peak_sectors": peak_sectors,
        "sectors_at_drop": retained_before_drop,
        "min_released_sectors": min_after_release, "min_loaded_sectors": min_during_proof,
        "final_sectors": count, "post_load_observation_s": proof_ticks * timestep,
        "post_load_retained": bool(proof_ticks and proof_pass and contacted_film and not warnings.any()),
        "puck_touched_film": contacted_film, "first_film_contact_s": first_contact,
        "min_released_opening": minimum_released_opening,
        "reset_max_abs_edge_strain": reset_strain,
        "release_max_abs_edge_strain": release_strain,
        "peak_sampled_max_abs_edge_strain": max_sampled_strain,
        "jaw_approximation": "force-limited vertex grasp",
        "warnings": warnings.tolist(), "frames": frames,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", choices=("all", "walk", "paired", "gravity", "seated_reference", "below_rim_control"), default="all")
    parser.add_argument("--span", type=float, default=25.)
    parser.add_argument("--centre", type=float, default=0.)
    parser.add_argument("--lift", type=float, default=.301)
    parser.add_argument("--spread", type=float, default=.141)
    parser.add_argument("--offset", type=int, default=3)
    parser.add_argument("--reset-method", choices=("contact", "geometric"), default="contact")
    parser.add_argument("--opening-radius", type=float, default=.110)
    parser.add_argument("--film-mass", type=float, default=.026, help="total film mass in kg")
    parser.add_argument("--timestep", type=float, default=STEP, help="physics timestep in seconds")
    parser.add_argument("--grip-scale", type=float, default=1., help="proxy grip gain and force scale")
    args = parser.parse_args()
    if not 1 <= args.offset < SEGMENTS // 4:
        parser.error("--offset must be between 1 and 7 vertices")
    if not 0 < args.timestep <= ACTION_PERIOD or not np.isfinite(args.grip_scale) or args.grip_scale <= 0:
        parser.error("use a timestep in (0, 0.050] seconds and a positive finite grip scale")
    if args.mode == "all":
        modes = ("paired", "gravity", "seated_reference", "below_rim_control")
    else:
        modes = (args.mode,)
    results = []
    for mode in modes:
        result = trial(mode, args.centre, args.span, args.lift, args.spread, args.offset,
                       args.reset_method, args.opening_radius, args.film_mass,
                       args.timestep, args.grip_scale)
        print(json.dumps({key: value for key, value in result.items() if key != "frames"}), flush=True)
        results.append(result)
    args.output.write_text(json.dumps({
        "faces": FACES.tolist(), "wall_height": WALL_HEIGHT, "lip_radius": LIP_RADIUS,
        "release_s": RELEASE, "drop_s": DROP, "results": results,
    }, separators=(",", ":")))


if __name__ == "__main__":
    main()
