"""Mount the existing toilet fixture in Dream's dual-YAM MuJoCo reference."""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from dream_examples.yam_mujoco import yam_env

from bench import DOCK, DT, JET_HEIGHT, JET_OPEN_FLOW, JETS, RINSE_SECONDS, TRAP_PIECES
from bench import Trial, model_xml

HZ = 20
SIDES = ("left", "right")
SPACING = 0.61
BASE_X = -0.25
BASE_Z = 0.175
SWITCH = (-0.16, 0.212, 0.231)
SWITCH_YAW = math.pi / 4
PLUNGER_YAW = math.pi
FINGER_OPEN = 0.0475
MAX_STEPS = 1200
RIM_DOCK = (DOCK[0], 0.0, DOCK[2])


def model(kind, blocked):
    reference = ET.fromstring(yam_env.build_xml())
    fixture = ET.fromstring(model_xml(kind, blocked))
    world = reference.find("worldbody")
    # The original reference faces -Y. The toilet is authored facing +X.
    # Keep the two bases and their cameras in one rigid transform.
    turn = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    shift = np.array([BASE_X + 0.24, 0.0, BASE_Z - 0.772])
    q_turn = np.array([2**-0.5, 0.0, 0.0, 2**-0.5])
    for element in list(world):
        name = element.get("name")
        if name not in ("top", "left_base", "right_base") and element.tag != "light":
            world.remove(element)
            continue
        if element.tag == "light":
            continue
        position = np.fromstring(element.get("pos"), sep=" ")
        quat = np.fromstring(element.get("quat"), sep=" ")
        rotated_quat = np.empty(4)
        mj.mju_mulQuat(rotated_quat, q_turn, quat)
        element.set("pos", yam_env.numbers(turn @ position + shift))
        element.set("quat", yam_env.numbers(rotated_quat))

    # Use the contact settings of the original toilet model on its geometry.
    fixture_defaults = fixture.find("default")
    fixture_defaults.set("class", "fixture")
    ET.SubElement(fixture_defaults, "joint", damping="0", armature="0")
    reference.find("default").append(fixture_defaults)
    fixture_assets = fixture.find("asset")
    reference.find("asset").extend(fixture_assets)
    for element in fixture.find("worldbody"):
        if element.tag == "light" or element.get("name") == "carriage":
            continue
        if kind == "rim" and element.get("name") in ("tool", "dock"):
            xyz = np.fromstring(element.get("pos"), sep=" ")
            xyz[1] = RIM_DOCK[1]
            element.set("pos", yam_env.numbers(xyz))
        element.set("class" if element.tag == "geom" else "childclass", "fixture")
        world.append(element)
    reference.append(fixture.find("equality"))
    for side in (-1, 1):
        ET.SubElement(
            world,
            "geom",
            name=f"pedestal_{side}",
            type="box",
            pos=yam_env.numbers((BASE_X, side * SPACING / 2, 0.08)),
            size=".078 .065 .080",
            rgba=".19 .23 .27 1",
        )
    x, y, z = SWITCH
    ET.SubElement(
        world,
        "geom",
        name="test_switch_stand",
        type="box",
        pos=yam_env.numbers((x, y, 0.098)),
        size=".029 .026 .098",
        rgba=".19 .23 .27 1",
    )
    switch = ET.SubElement(
        world,
        "body",
        name="test_switch",
        pos=yam_env.numbers(SWITCH),
        childclass="fixture",
    )
    ET.SubElement(
        switch,
        "joint",
        name="test_switch",
        type="slide",
        axis="0 0 -1",
        range="0 .018",
        stiffness="300",
        damping="2",
    )
    ET.SubElement(
        switch,
        "geom",
        name="test_cap",
        type="cylinder",
        size=".021 .005",
        mass=".023",
        rgba=".12 .59 .78 1",
    )
    return mj.MjModel.from_xml_string(ET.tostring(reference, encoding="unicode"))


class YamToilet(Trial):
    def __init__(self, kind, blocked=1, recording=None, fixture_model=None):
        self.kind = kind
        self.blocked = blocked
        self.model = (
            fixture_model if fixture_model is not None else model(kind, blocked)
        )
        self.data = mj.MjData(self.model)
        self.ik_data = mj.MjData(self.model)
        self.renderer = None
        self.tests = []
        self.test_steps = 0
        self.test_contact_at_start = 0
        self.valid_test = False
        self.button_down = False
        self.yaw_targets = {}
        self.targets = np.array([[0.0, 1.0, 1.0, -0.6, 0.0, 0.0, 1.0]] * 2)
        self.joints = {
            side: [self.model.joint(f"{side}_joint{i}") for i in range(1, 7)]
            for side in SIDES
        }
        self.fingers = {
            side: [f"{side}_finger_{sign}" for sign in (-1, 1)] for side in SIDES
        }
        self.sites = {side: self.model.site(f"{side}_tcp").id for side in SIDES}
        self.set_targets(self.targets)
        for index, side in enumerate(SIDES):
            for joint, value in zip(self.joints[side], self.targets[index, :6]):
                self.data.qpos[joint.qposadr[0]] = value
            for name in self.fingers[side]:
                self.data.joint(name).qpos[0] = FINGER_OPEN
        self.reset_fixture(recording)
        self.finger_geoms = {
            self.model.geom(i).name
            for i in range(self.model.ngeom)
            if "_tip_" in self.model.geom(i).name
            and "_contact_" in self.model.geom(i).name
        }
        if recording:
            from recording import camera

            recording.main_cam = camera((-0.13, 0.0, 0.24), 1.23, 302, -33)

    def set_targets(self, targets):
        targets = np.asarray(targets, dtype=float)
        if targets.shape != (2, 7) or not np.isfinite(targets).all():
            raise ValueError(
                "Expected two sets of six YAM joint targets and gripper opening"
            )
        for i, side in enumerate(SIDES):
            for joint, angle in zip(self.joints[side], targets[i, :6]):
                self.data.actuator(joint.name).ctrl[0] = np.clip(angle, *joint.range)
            for name in self.fingers[side]:
                self.data.actuator(name).ctrl[0] = FINGER_OPEN * np.clip(
                    targets[i, -1], 0, 1
                )

    def solve(self, side, goal, yaw, initial=None):
        data, model = self.ik_data, self.model
        data.qpos[:] = self.data.qpos
        pos_indices = [j.qposadr[0] for j in self.joints[side]]
        vel_indices = [j.dofadr[0] for j in self.joints[side]]
        site = self.sites[side]
        limits = np.array([j.range for j in self.joints[side]])
        if initial is not None:
            data.qpos[pos_indices] = initial
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])
        jac_pos, jac_rot = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        relative = np.empty(4)
        rot_error = np.empty(3)
        for _ in range(100):
            mj.mj_kinematics(model, data)
            mj.mj_comPos(model, data)
            position_error = goal - data.site_xpos[site]
            mj.mju_mat2Quat(
                relative, (rotation @ data.site_xmat[site].reshape(3, 3).T).ravel()
            )
            if relative[0] < 0:
                relative *= -1
            mj.mju_quat2Vel(rot_error, relative, 1)
            if (
                np.linalg.norm(position_error) < 0.0006
                and np.linalg.norm(rot_error) < 0.012
            ):
                break
            mj.mj_jacSite(model, data, jac_pos, jac_rot, site)
            jac = np.vstack((jac_pos[:, vel_indices], 0.27 * jac_rot[:, vel_indices]))
            error = np.r_[position_error, 0.27 * rot_error]
            delta = jac.T @ np.linalg.solve(jac @ jac.T + 0.002 * np.eye(6), error)
            current = data.qpos[pos_indices]
            data.qpos[pos_indices] = np.clip(
                current + np.clip(delta, -0.18, 0.18), *limits.T
            )
        return (
            data.qpos[pos_indices].copy(),
            float(np.linalg.norm(position_error)),
            float(np.linalg.norm(rot_error)),
        )

    def tool_clear(self):
        x, y, z = self.data.body("tool").xpos
        return x < -0.19 or abs(y) > 0.19 or z > 0.275

    def update_test(self):
        down = self.joint("test_switch") >= 0.010 and self.contact(
            self.finger_geoms, {"test_cap"}
        )
        if down and not self.button_down and not self.test_steps and self.tool_clear():
            seconds = RINSE_SECONDS if self.kind == "rim" else 1.25
            self.test_steps = round(seconds / DT)
            self.valid_test = True
            self.test_contact_at_start = self.tool_contacts
            self.rinsing = self.kind == "rim"
            self.rinse[:] = 0
            self.water_level = 1.0
        self.button_down = down
        if not self.test_steps:
            return
        self.valid_test &= self.tool_clear()
        if self.kind == "rim":
            self.rinse += DT * self.rim_flow()
        else:
            opened = (len(self.trap_cleared()) / len(TRAP_PIECES)) ** 2
            self.water_level = max(0.0, self.water_level - DT * (0.04 + 0.75 * opened))
        self.test_steps -= 1
        if not self.test_steps:
            self.rinsing = False
            if self.valid_test:
                readings = (
                    self.rinse.tolist() if self.kind == "rim" else [self.water_level]
                )
                self.tests.append((readings, self.test_contact_at_start))

    def action(self, targets):
        if round(self.data.time * HZ) >= MAX_STEPS:
            raise RuntimeError("The 1,200-step trial limit was reached")
        self.set_targets(targets)
        for _ in range(round(1 / (HZ * DT))):
            self.update()
            mj.mj_step(self.model, self.data)
            self.update_test()
            if round(self.data.time / DT) % 20 == 0:
                self.trace.append(self.reading())
                if self.frames:
                    self.frames.draw()
        mj.mj_forward(self.model, self.data)
        self.targets[:] = targets

    def move_arm(self, side, goal, seconds, phase, opening=None, yaw=0.0):
        index = SIDES.index(side)
        start_pose = self.data.site_xpos[self.sites[side]].copy()
        start = self.targets.copy()
        target = start.copy()
        count = max(1, round(seconds * HZ))
        start_yaw = self.yaw_targets.get(side, yaw)
        turn = math.remainder(yaw - start_yaw, math.tau)
        self.phase = phase
        for i in range(count):
            t = (i + 1) / count
            weight = t * t * (3 - 2 * t)
            pos = start_pose + weight * (np.asarray(goal) - start_pose)
            q, error, rot = self.solve(
                side, pos, start_yaw + weight * turn, target[index, :6]
            )
            if i + 1 == count and (error > 0.005 or rot > 0.11):
                raise RuntimeError(
                    f"{side} {phase} IK error {error:.4f} m / {rot:.3f} rad"
                )
            target[index, :6] = q
            if opening is not None:
                target[index, -1] = start[index, -1] + weight * (
                    opening - start[index, -1]
                )
            self.action(target)
        self.yaw_targets[side] = yaw

    def collect_test(self, phase):
        x, y, z = SWITCH
        self.move_arm(
            "left", (x, y, z + 0.075), 1.5, phase, opening=0.0, yaw=SWITCH_YAW
        )
        self.move_arm(
            "left", (x, y, z - 0.016), 1.1, "Press blue test switch", yaw=SWITCH_YAW
        )
        for _ in range(9):
            self.action(self.targets)
        self.move_arm(
            "left", (x, y, z + 0.075), 1.1, "Release test switch", yaw=SWITCH_YAW
        )
        for _ in range(round(RINSE_SECONDS * HZ)):
            self.action(self.targets)

    def score(self):
        if len(self.tests) < 2:
            return 0.0
        (first, start), (last, end) = self.tests[0], self.tests[-1]
        if not self.tool_clear() or end <= start:
            return 0.0
        if self.kind == "trap":
            return float(
                first[0] >= 0.70
                and last[0] <= 0.15
                and len(self.trap_cleared()) == len(TRAP_PIECES)
            )
        reference = JET_OPEN_FLOW * RINSE_SECONDS
        first, last = np.array(first), np.array(last)
        restricted = first <= 0.40 * reference
        healthy = first[~restricted]
        if (
            restricted.sum() != 1
            or not ((healthy >= 0.8 * reference) & (healthy <= 1.2 * reference)).all()
        ):
            return 0.0
        restored = (
            not self.scale_remaining()
            and ((last >= 0.8 * reference) & (last <= 1.2 * reference)).all()
        )
        return 1.0 if restored else 0.25

    def scripted_trap(self, control=False):
        def move(goal, seconds, phase, opening=None):
            self.move_arm("right", goal, seconds, phase, opening, PLUNGER_YAW)

        x, y, z = DOCK
        self.collect_test("Test drainage before repair")
        move((x, y, z + 0.17), 1.6, "Move above plunger")
        move((x, y, z), 1.4, "Approach plunger handle")
        move((x, y, z), 0.8, "Grip plunger", opening=0.12)
        move((x, y, 0.327), 1.8, "Lift plunger above the seat")
        x = -0.050 if control else 0.0
        move((x, 0, 0.327), 1.7, "Centre plunger over drain")
        move((x, 0, 0.206), 1.0, "Seat the cup")
        for i in range(8):
            move((x, 0, 0.174), 0.25, f"Compression stroke {i + 1} of 8")
            move((x, 0, 0.204), 0.35, "Release stroke")
        move((x, 0, 0.327), 1.2, "Lift the cup clear")
        self.collect_test("Retest drainage")

    def scripted_rim(self, control=False):
        self.collect_test("Rinse to find the weak jet")
        if not self.tests:
            raise RuntimeError("The first rinse did not complete")
        outlet = np.array(JETS[int(np.argmin(self.tests[0][0]))])
        side = "left" if outlet[1] > 0 else "right"
        radial = outlet / np.linalg.norm(outlet)
        across = np.array([-radial[1], radial[0]])
        yaw = math.atan2(radial[1], radial[0])
        inside = outlet - 0.070 * radial
        engaged = outlet - 0.038 * radial + (0.024 * across if control else 0.0)
        approach_z = JET_HEIGHT + 0.022

        def move(goal, seconds, phase, opening=None, rotation=yaw):
            self.move_arm(side, goal, seconds, phase, opening, rotation)

        x, y, z = RIM_DOCK
        move((x, y, 0.267), 2.0, "Move above scraper", 1.0, 0.0)
        move((x, y, z), 1.2, "Approach scraper handle", rotation=0.0)
        move((x, y, z), 0.7, "Grip scraper", 0.12, 0.0)
        move((x, y, 0.267), 1.6, "Lift scraper clear", rotation=0.0)
        move((*inside, 0.282), 1.7, "Carry scraper above the weak outlet")
        move((*inside, approach_z), 0.9, "Lower inside the bowl")
        move((*(engaged - 0.008 * across), approach_z), 0.9, "Set edge under the crust")
        passes = (
            (0.008, 0.0, 0.0),
            (-0.008, 0.006, 0.0),
            (0.008, 0.009, 0.0),
            (-0.008, 0.010, -0.002),
            (0.008, 0.008, -0.002),
            (-0.008, 0.005, 0.001),
        )
        if side == "left":
            passes += passes
        elif outlet[0] > 0:
            passes += tuple(
                (sideways * 1.25, height + 0.002, depth + 0.003)
                for sideways, height, depth in passes
            )
        for offset, height, depth in passes:
            move(
                (*(engaged + offset * across + depth * radial), approach_z + height),
                0.75,
                "Scrape limescale",
            )
        move((*inside, approach_z), 0.7, "Withdraw scraper")
        move((*inside, 0.286), 0.8, "Lift over the lip")
        move((x, y, 0.267), 1.7, "Park scraper clear of the rinse", rotation=0.0)
        if side == "left":
            move((x, y, z + 0.002), 1.2, "Return scraper to the dock", rotation=0.0)
            move((x, y, z + 0.002), 0.55, "Release scraper", 1.0, 0.0)
            move((x, y, 0.267), 1.4, "Lift the empty gripper", rotation=0.0)
        self.collect_test("Rinse again to verify flow")

    def summary(self):
        return {
            "score": self.score(),
            "test_readings": [readings for readings, _ in self.tests],
            "finger_contact_steps": self.finger_contacts,
            "tool_contact_steps": self.tool_contacts,
            "remaining": (
                len(self.scale_remaining())
                if self.kind == "rim"
                else len(TRAP_PIECES) - len(self.trap_cleared())
            ),
            "seconds": float(self.data.time),
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=("rim", "trap"))
    parser.add_argument("--blocked", type=int, choices=range(4), default=1)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--reach", action="store_true")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--control", action="store_true")
    args = parser.parse_args()
    if args.reach:
        sim = YamToilet(args.kind, args.blocked)
        checks = [
            ("left", (*SWITCH[:2], SWITCH[2] - 0.016), SWITCH_YAW, "Test switch"),
            ("right", DOCK, PLUNGER_YAW, "Tool dock"),
            ("right", (0, 0, 0.327), PLUNGER_YAW, "Bowl approach"),
            ("right", (0, 0, 0.206), PLUNGER_YAW, "Drain contact"),
        ]
        for i, (x, y) in enumerate(JETS):
            side = "left" if y > 0 else "right"
            yaw = math.atan2(y, x)
            radial = np.array((x, y)) / math.hypot(x, y)
            point = np.array((x, y)) - 0.038 * radial
            checks.append((side, (*point, JET_HEIGHT + 0.022), yaw, f"Rim outlet {i}"))
        for side, pos, yaw, label in checks:
            _, position, angle = sim.solve(
                side, np.array(pos), yaw, sim.targets[SIDES.index(side), :6]
            )
            print(f"{label}: {side}, {1000 * position:.2f} mm, {angle:.3f} rad")
        return
    if args.out is None:
        parser.error("--out is required for a scripted trial")
    args.out.mkdir(parents=True, exist_ok=True)
    recording = None
    if args.video:
        from recording import Recording

        recording = Recording(args.out / "demo.mp4")
    sim = YamToilet(args.kind, args.blocked, recording)
    try:
        script = sim.scripted_rim if args.kind == "rim" else sim.scripted_trap
        script(args.control)
    finally:
        summary = sim.summary()
        (args.out / "result.json").write_text(json.dumps(summary, indent=2) + "\n")
        (args.out / "trace.jsonl").write_text(
            "".join(json.dumps(item) + "\n" for item in sim.trace)
        )
        if recording:
            recording.close()
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
