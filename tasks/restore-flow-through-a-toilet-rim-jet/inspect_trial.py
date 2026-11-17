"""Run a toilet fixture through Inspect Robots using raw gripper controls."""

import argparse
from pathlib import Path

import mujoco as mj
import numpy as np
from inspect_robots import eval
from inspect_robots.embodiment import EmbodimentInfo, RENDERABLE
from inspect_robots.policy import PolicyConfig, PolicyInfo
from inspect_robots.scene import Scene
from inspect_robots.scorer import Score
from inspect_robots.spaces import ActionSemantics, Box, CameraSpec, ObservationSpace
from inspect_robots.task import Task
from inspect_robots.types import Action, ActionChunk, Observation, StepResult

from bench import DT, JET_OPEN_FLOW, RINSE_SECONDS, TRAP_PIECES, Trial
from recording import add_streams, add_water_surface, camera

HZ = 20
MAX_STEPS = 1200
ACTION = Box(
    (7,),
    low=np.array([-0.285, -0.23, 0.087, -np.pi, 0, 0, 0]),
    high=np.array([0.23, 0.23, 0.37, np.pi, 0.019, 0.019, 1]),
    semantics=ActionSemantics(
        control_mode="joint_pos",
        frame="world",
        dim_labels=("x", "y", "z", "yaw", "left_finger", "right_finger", "test_switch"),
        max_step=(0.013, 0.013, 0.011, 0.18, 0.005, 0.005, 1),
    ),
)
INSTRUCTIONS = {
    "rim": "Run a rinse with the tool clear of the bowl. Find the weak jet using the four "
    "collection volumes, scrape the limescale away, and verify the repair with a second rinse.",
    "trap": "Test drainage, pick up the plunger, seal its cup on the drain and clear the mixed "
    "obstruction with plunging strokes. Lift the cup clear and verify drainage.",
}
NOTES = (
    "The first three joints position a Cartesian gripper in metres. Positive X points from "
    "the probe or plunger dock toward the cistern. The bowl is centred at (0, 0). "
    "Yaw rotates the gripper about vertical, in radians. Keep both fingers at 0 to open "
    "or 0.0175 to close. The tool handle starts at (-0.245, -0.144, 0.123). "
    "Lift the tool above the seat before crossing the rim. The test switch controls the bench "
    "rinse or drain measurement. Raise it from 0 to 1 with the tool clear, hold still until "
    "fixture_status reports completion, then return the switch to 0 to rearm it. "
    "Fixture readings update after a completed test. No tools automate a repair."
)


class ToiletFixture:
    def __init__(self, kind, blocked):
        self.kind, self.blocked = kind, blocked
        kind_notes = (
            "Rinse readings are millilitres in order front left, front right, rear left, rear right. "
            "Jet positions have X and Y coordinates (±0.1056, ±0.0792) at Z=0.211."
            if kind == "rim"
            else "The drain is centred at (0, 0). The reading is the fraction of bowl water "
            "remaining after the timed drain test."
        )
        self.info = EmbodimentInfo(
            name=f"toilet-{kind}-fixture",
            action_space=ACTION,
            observation_space=ObservationSpace(
                cameras=(CameraSpec("workcell", 300, 400),),
                state_keys=frozenset(
                    {"joint_pos", "fixture_status", "fixture_readings"}
                ),
            ),
            control_hz=HZ,
            is_simulated=True,
            capabilities=frozenset({RENDERABLE}),
            docs=f"{NOTES}\n{kind_notes}",
            morphology="single-cartesian-gripper",
            environment_id="mujoco-3.3.7-toilet-maintenance",
        )
        self.renderer = None

    def reset(self, scene, *, seed=None):
        self.close()
        self.sim = Trial(self.kind, self.blocked)
        self.renderer = mj.Renderer(self.sim.model, height=300, width=400)
        self.camera = camera((-0.025, -0.028, 0.224), 0.72, 305, -41)
        self.instruction = scene.instruction
        self.test_steps = 0
        self.test_switch = 0.0
        self.tests = []
        self.measurement = np.zeros(4 if self.kind == "rim" else 1)
        self.valid_test = False
        return self.observe()

    def observe(self):
        sim = self.sim
        self.renderer.update_scene(sim.data, camera=self.camera)
        self.renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW] = False
        add_streams(sim, self.renderer.scene)
        add_water_surface(sim, self.renderer.scene)
        pose = [sim.joint(name) for name in ("x", "y", "z", "yaw", "left", "right")]
        return Observation(
            images={"workcell": self.renderer.render().copy()},
            state={
                "joint_pos": np.array([*pose, self.test_switch]),
                "fixture_status": np.array([len(self.tests), int(self.test_steps > 0)]),
                "fixture_readings": self.measurement.copy(),
            },
            instruction=self.instruction,
            state_time=float(sim.data.time),
        )

    def tool_clear(self):
        x, y, z = self.sim.data.body("tool").xpos
        return x < -0.19 or abs(y) > 0.19 or z > 0.275

    def update_measurement(self):
        sim = self.sim
        if self.kind == "rim":
            sim.rinse += DT * sim.rim_flow()
        else:
            cleared = len(sim.trap_cleared()) / len(TRAP_PIECES)
            sim.water_level = max(0, sim.water_level - DT * (0.04 + 0.75 * cleared**2))

    def step(self, action):
        target = np.asarray(action.data, dtype=float)
        if target.shape != (7,) or not np.all(np.isfinite(target)):
            raise ValueError("The fixture expects seven finite control targets")
        target = np.clip(target, ACTION.low, ACTION.high)
        delta = np.array(ACTION.semantics.max_step[:6])
        sim = self.sim
        target[:6] = sim.target + np.clip(target[:6] - sim.target, -delta, delta)
        trigger = self.test_switch < 0.5 <= target[-1]
        self.test_switch = float(target[-1])
        if trigger and not self.test_steps and self.tool_clear():
            duration = RINSE_SECONDS if self.kind == "rim" else 1.25
            self.test_steps = round(duration * HZ)
            self.valid_test = True
            sim.rinsing = self.kind == "rim"
            sim.rinse[:] = 0
            sim.water_level = 1.0
        sim.step(
            target[:6],
            1 / HZ,
            "Fixture test" if self.test_steps else "Manipulation",
            self.update_measurement if self.test_steps else None,
        )
        if self.test_steps:
            self.valid_test = self.valid_test and self.tool_clear()
            self.test_steps -= 1
            if not self.test_steps:
                sim.rinsing = False
                if self.valid_test:
                    self.measurement = (
                        sim.rinse.copy()
                        if self.kind == "rim"
                        else np.array([sim.water_level])
                    )
                    self.tests.append(self.measurement.tolist())
        info = {
            "tests": self.tests.copy(),
            "tool_clear": bool(self.tool_clear()),
            "tool_contact_steps": sim.tool_contacts,
            "remaining": (
                len(sim.scale_remaining())
                if self.kind == "rim"
                else len(TRAP_PIECES) - len(sim.trap_cleared())
            ),
        }
        return StepResult(self.observe(), info=info)

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None


class FixtureScore:
    name = "task_score"

    def __init__(self, kind):
        self.kind = kind

    def __call__(self, record, target):
        if not record.steps:
            return Score(0.0, "No movement or measurements recorded")
        state = record.steps[-1].result.info
        tests = state["tests"]
        if len(tests) < 2:
            return Score(0.0, "Two complete tests are required", state)
        before, after = np.array(tests[0]), np.array(tests[-1])
        physical_repair = bool(
            state["tool_clear"]
            and state["tool_contact_steps"] > 0
            and state["remaining"] == 0
        )
        if self.kind == "rim":
            normal = JET_OPEN_FLOW * RINSE_SECONDS
            restricted = before <= 0.4 * normal
            started_blocked = restricted.sum() == 1 and np.all(
                (before[~restricted] >= 0.8 * normal)
                & (before[~restricted] <= 1.2 * normal)
            )
            measurement_pass = np.all((after >= 0.8 * normal) & (after <= 1.2 * normal))
            partial = (
                0.25
                if started_blocked
                and state["tool_contact_steps"] > 0
                and state["tool_clear"]
                else 0.0
            )
        else:
            started_blocked = before[0] >= 0.70
            measurement_pass = after[0] <= 0.15
            partial = 0.0
        return Score(
            (
                1.0
                if started_blocked and measurement_pass and physical_repair
                else partial
            ),
            "Final fixture readings and physical state",
            state,
        )


class RinseOnly:
    """A harness smoke control that tests twice without attempting a repair."""

    info = PolicyInfo("no-contact smoke control", ACTION)
    config = PolicyConfig()

    def reset(self, scene):
        self.steps = 0

    def act(self, observation):
        action = np.asarray(observation.state["joint_pos"]).copy()
        action[-1] = 0.0 if self.steps == 43 else 1.0
        self.steps += 1
        return ActionChunk([Action(action)])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=("rim", "trap"))
    parser.add_argument("--blocked", type=int, choices=range(4), default=1)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--model")
    parser.add_argument("--base-url")
    parser.add_argument("--api-key-env")
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args()
    if args.smoke == bool(args.model):
        parser.error("Choose --model or --smoke")
    fixture = ToiletFixture(args.kind, args.blocked)
    if args.smoke:
        policy = RinseOnly()
    else:
        from inspect_robots_agent.policy import LLMAgentPolicy

        policy = LLMAgentPolicy(
            model=args.model,
            base_url=args.base_url,
            api_key_env=args.api_key_env,
            max_llm_calls=100,
            wire_capture=False,
        )
    task = Task(
        f"toilet-{args.kind}",
        [Scene("nominal", INSTRUCTIONS[args.kind])],
        scorer=FixtureScore(args.kind),
        max_steps=90 if args.smoke else MAX_STEPS,
    )
    try:
        logs = eval(
            task,
            policy,
            fixture,
            log_dir=str(args.out),
            seed=0,
            store_frames=not args.smoke,
            fail_on_error=True,
        )
        if any(log.status != "success" for log in logs):
            raise RuntimeError(
                "Evaluation did not complete. See the Inspect Robots log."
            )
    finally:
        fixture.close()


if __name__ == "__main__":
    main()
