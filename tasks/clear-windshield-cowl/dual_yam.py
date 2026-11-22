"""Articulated I2RT YAMs with contact pads at the linear gripper fingertips."""

from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


BASE_Z = .872
BASE_X = .465
BASE_Y = -.12
GRASP_SHIM = np.array([.044, 0, -.002])
ASSET_DIR = Path(__file__).resolve().parent / "vendor/i2rt_yam"
ARM_PREFIXES = ("L_", "R_")


def append_yams(scene):
    origin = ET.parse(ASSET_DIR / "yam.xml").getroot()
    defaults = scene.find("default")
    if defaults is None:
        defaults = ET.SubElement(scene, "default")
    for default in origin.find("default"):
        defaults.append(deepcopy(default))

    assets = scene.find("asset")
    for element in origin.find("asset"):
        element = deepcopy(element)
        if element.tag == "mesh":
            element.set("name", Path(element.get("file")).stem)
            element.set("file", str(ASSET_DIR / "assets" / element.get("file")))
        assets.append(element)
    world = scene.find("worldbody")
    equality = ET.SubElement(scene, "equality")
    actuators = scene.find("actuator")
    if actuators is None:
        actuators = ET.SubElement(scene, "actuator")

    for prefix, side, yaw in zip(ARM_PREFIXES, (-1, 1), (40, 140)):
        arm = deepcopy(origin.find("worldbody/body"))
        arm.set("pos", f"{side*BASE_X} {BASE_Y} {BASE_Z}")
        angle = math.radians(yaw)/2
        arm.set("quat", f"{math.cos(angle)} 0 0 {math.sin(angle)}")
        for item in arm.iter():
            if item.tag == "joint" and item.get("type", "hinge") != "slide" and item.get("class") != "finger":
                # The scene's compiler uses degrees. The original limits are radians.
                low, high = map(float, item.get("range").split())
                item.set("range", f"{math.degrees(low)} {math.degrees(high)}")
            if item.tag == "body":
                item.set("gravcomp", "1")
            if item.tag == "geom" and item.get("class") in ("collision", "sphere_collision"):
                # Preserve the official collision proxies on the arm links. The
                # linear fingers use snug replaceable pads for thin curled leaves.
                if item.get("class") == "sphere_collision":
                    item.set("contype", "0")
                    item.set("conaffinity", "0")
                item.set("group", "5")
            if item.get("name"):
                item.set("name", prefix + item.get("name"))
        for down in ("lf_down", "rf_down"):
            finger = next(b for b in arm.iter("body") if b.get("name") == prefix+down)
            for collision in finger.findall("geom"):
                collision.set("contype", "0")
                collision.set("conaffinity", "0")
            ET.SubElement(finger, "geom", name=prefix+down+"_pad", type="box",
                          pos="0 -.0058 .0722", size=".008 .0058 .0045",
                          rgba=".12 .13 .134 1", mass=".004",
                          friction="2.5 .03 .0005", solref=".009 1", solimp=".97 .995 .001")
        world.append(arm)
        finger_equality = deepcopy(origin.find("equality/joint"))
        for attribute in ("joint1","joint2"):
            finger_equality.set(attribute, prefix+finger_equality.get(attribute))
        equality.append(finger_equality)

        for act in origin.find("actuator"):
            act = deepcopy(act)
            name = act.get("name")
            act.set("name", prefix+name)
            act.set("joint", prefix+act.get("joint"))
            if name == "gripper":
                act.set("kp", "460")
                act.set("kv", "15")
                act.set("forcerange", "-10 10")
            else:
                arm_servo = int(name[-1]) <= 3
                act.set("kp", "290" if arm_servo else "165")
                act.set("kv", "16" if arm_servo else "5")
            actuators.append(act)


class Arm:
    def __init__(self, model, prefix, side):
        self.model = model
        self.prefix = prefix
        self.side = side
        self.spec = mj.MjData(model)
        self.joints = [model.joint(prefix+f"joint{i}") for i in range(1,7)]
        self.qids = np.array([joint.qposadr[0] for joint in self.joints])
        self.vids = np.array([joint.dofadr[0] for joint in self.joints])
        self.ctrl_ids = np.array([model.actuator(prefix+f"joint{i}").id for i in range(1,7)])
        self.gripper = model.actuator(prefix+"gripper").id
        self.fingers = [model.joint(prefix+finger).qposadr[0] for finger in ("left_finger", "right_finger")]
        self.low, self.high = np.array([joint.range for joint in self.joints]).T
        self.site = model.site(prefix+"grasp_site").id
        self.rotation = np.array([[0,side,0],[side,0,0],[0,0,-1.]])
        self.q = np.array([0., 1.047, 1.047, 0., 0., 0.])

    def site_target(self, target):
        return np.asarray(target) - self.rotation @ GRASP_SHIM

    def forward(self, q):
        d = self.spec
        d.qpos[self.qids] = q
        mj.mj_kinematics(self.model, d)
        return d.site_xpos[self.site], d.site_xmat[self.site].reshape(3,3)

    def error(self, q, position):
        point, rot = self.forward(q)
        error = Rotation.from_matrix(self.rotation @ rot.T).as_rotvec()
        return np.r_[position-point, .15*error]

    def solve_start(self, target):
        position = self.site_target(target)
        best = None
        initial_q = self.q.copy()
        rng = np.random.default_rng(348 + self.side)
        starts = [initial_q]
        for shoulder in (.4, 1.2, 2.6):
            for elbow in (.7, 1.5, 2.6):
                starts.append(np.array([.2,shoulder,elbow,0.,.2,0.]))
        starts.extend(rng.uniform(self.low, self.high) for _ in range(28))
        for start in starts:
            trial = least_squares(self.error, np.clip(start, self.low+1e-5, self.high-1e-5),
                                  args=(position,), bounds=(self.low+1e-6,self.high-1e-6),
                                  xtol=1e-8, max_nfev=110)
            status = np.linalg.norm(trial.fun)
            if best is None or status < best[0]:
                best = (status, trial.x)
            if status < .00015:
                break
        self.q = best[1].copy()
        return best[0]

    def solve(self, target):
        position = self.site_target(target)
        d = self.spec
        q = self.q.copy()
        jac_p = np.zeros((3,self.model.nv))
        jac_r = np.zeros_like(jac_p)
        for _ in range(8):
            residual = self.error(q, position)
            if np.linalg.norm(residual) < .00004:
                break
            mj.mj_comPos(self.model, d)
            mj.mj_jacSite(self.model, d, jac_p, jac_r, self.site)
            jac = np.concatenate((jac_p[:,self.vids], .15*jac_r[:,self.vids]))
            damp = .001
            correction = jac.T @ np.linalg.solve(jac @ jac.T + damp*np.eye(6), residual)
            q = np.clip(q+np.clip(correction,-.08,.08), self.low+1e-5, self.high-1e-5)
        self.q = q
        return float(np.linalg.norm(self.error(q, position)))

    def initialize(self, data, target, opening):
        error = self.solve_start(target)
        data.qpos[self.qids] = self.q
        data.qpos[self.fingers] = [opening, -opening]
        self.command(data, opening)
        return error

    def command(self, data, opening):
        data.ctrl[self.ctrl_ids] = self.q
        data.ctrl[self.gripper] = opening
