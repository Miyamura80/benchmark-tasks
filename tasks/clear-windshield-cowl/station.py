"""Expose the native windshield-cowl scene to a 20 Hz joint-position policy."""

import mujoco as mj
import numpy as np

from task import ARM_PREFIXES, Arm, DT, HEIGHT, make_scene, score

HZ = 20
MAX_STEPS = 600
SIDES = ARM_PREFIXES


class Cowl:
    title = "Clear leaves from the base of the windshield"
    instruction = "Pick up the leaves below the windshield and put them in the tray. Clear the leaves off both drain grilles."

    def __init__(self, variant):
        if variant != 0:
            raise ValueError("Only the documented fixed reset is supported")

    def compile(self):
        return mj.MjModel.from_xml_string(make_scene())

    def visual(self, sim, scene):
        pass


class Station:
    def __init__(self, fixture, render=None, model=None):
        self.fixture = fixture
        self.model = fixture.compile() if model is None else model
        self.data = mj.MjData(self.model)
        self.arms = [Arm(self.model, prefix, side) for prefix, side in zip(SIDES, (-1, 1))]
        self.grip_limits = [
            min(self.model.actuator_ctrlrange[arm.gripper, 1],
                self.model.joint(arm.prefix + "left_finger").range[1])
            for arm in self.arms
        ]
        for arm, x in zip(self.arms, (-.333, .333)):
            error = arm.initialize(self.data, np.array([x, -.114, .268]) + HEIGHT, .030)
            if error > .003:
                raise RuntimeError(f"{arm.prefix} arm start pose error: {error:.4f} m")
        mj.mj_step(self.model, self.data, nstep=round(1.32 / DT))
        mj.mj_forward(self.model, self.data)
        self.reset_time = self.data.time
        self.phase = "Starting state"
        self.steps = 0
        self.commands = []
        self.trace = []
        self.render = render
        self.checkpoint = score(self.model, self.data)
        if render:
            render.attach(self)
            render.draw()

    def joint_state(self):
        return np.concatenate([
            np.r_[self.data.qpos[arm.qids], np.clip(
                (self.data.qpos[arm.fingers[0]] - self.data.qpos[arm.fingers[1]]) / (2 * limit),
                0, 1)]
            for arm, limit in zip(self.arms, self.grip_limits)
        ])

    def action(self, targets):
        if self.steps >= MAX_STEPS:
            raise RuntimeError("Trial action limit reached")
        if targets.shape != (2, 7) or not np.isfinite(targets).all():
            raise ValueError("Expected finite joint and gripper targets for both arms")
        for arm, target, limit in zip(self.arms, targets, self.grip_limits):
            arm.q = np.clip(target[:6], arm.low, arm.high)
            arm.command(self.data, np.clip(target[6], 0, 1) * limit)
        mj.mj_step(self.model, self.data, nstep=round(1 / (HZ * DT)))
        mj.mj_forward(self.model, self.data)
        self.steps += 1
        self.commands.append(targets.flatten().tolist())
        self.checkpoint = score(self.model, self.data)
        if self.steps % 2 == 0:
            self.trace.append(self.reading())
            if self.render:
                self.render.draw()

    def reading(self):
        return {"step": self.steps, "seconds": self.steps / HZ,
                "phase": self.phase, **self.checkpoint}
