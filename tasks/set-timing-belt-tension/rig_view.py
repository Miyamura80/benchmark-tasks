"""Visible USD representation of the measurable joint fixture."""
from math import cos, pi, sin

import numpy as np
from pxr import Gf, UsdGeom, Vt
import omni.usd

import belt_task as task
from mechanics import BELT_WIDTH, BOLT_PITCH, PITCH_RADIUS, START_SPAN, TARGET_DEFLECTION, midspan_deflection

ROOT = "/World/BeltJoint"
METAL = (0.48, 0.60, 0.68)
DARK = (0.10, 0.14, 0.18)
MOTOR = (0.15, 0.22, 0.27)
GOLD = (0.70, 0.48, 0.18)
RUBBER = (0.11, 0.12, 0.14)
HANDLE = (0.88, 0.25, 0.045)
BLUE = (0.05, 0.47, 0.87)
GREEN = (0.11, 0.73, 0.20)
WHITE = (0.90, 0.91, 0.86)
RED = (0.85, 0.12, 0.10)
GAUGE_START = 0.209
GAUGE_RATIO = 10.0


class FixtureView:
    def __init__(self, contacts):
        self.stage = omni.usd.get_context().get_stage()
        self.stage.RemovePrim(ROOT)
        UsdGeom.Xform.Define(self.stage, ROOT)
        self.xforms = {}
        self.base_shapes()
        self.moving_shapes()
        self.update(contacts)

    def group(self, name, parent=ROOT):
        path = f"{parent}/{name}"
        xform = UsdGeom.Xform.Define(self.stage, path)
        common = UsdGeom.XformCommonAPI(xform)
        self.xforms[path] = common
        return path

    def move(self, path, position, yaw=0.0):
        xform = self.xforms[path]
        xform.SetTranslate(Gf.Vec3d(*map(float, position)))
        if yaw is not None:
            xform.SetRotate(Gf.Vec3f(0, 0, float(np.degrees(yaw))))

    def shape(self, name, shape, pos, color, parent=ROOT):
        path = f"{parent}/{name}"
        model = shape.Define(self.stage, path)
        model.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdGeom.XformCommonAPI(model).SetTranslate(Gf.Vec3d(*map(float, pos)))
        return model

    def box(self, name, pos, size, color, parent=ROOT):
        model = self.shape(name, UsdGeom.Cube, pos, color, parent)
        model.CreateSizeAttr(1.0)
        UsdGeom.XformCommonAPI(model).SetScale(Gf.Vec3f(*size))
        return model

    def cyl(self, name, pos, radius, height, color, parent=ROOT, axis="Z"):
        model = self.shape(name, UsdGeom.Cylinder, pos, color, parent)
        model.CreateRadiusAttr(radius)
        model.CreateHeightAttr(height)
        model.CreateAxisAttr(axis)
        return model

    def base_shapes(self):
        t = task.TOP
        for i, by in enumerate((-0.073, 0.074)):
            self.box(f"rail_{i}", (0.313, by, t + 0.019), (0.111, 0.010, 0.010), DARK)
            self.box(f"rail_face_{i}", (0.313, by, t + 0.024), (0.111, 0.005, 0.002), METAL)
        for i, bx in enumerate(task.BOLT_X):
            self.box(f"slot_{i}", (bx, task.BOLT_Y, t + 0.0264), (0.015, 0.006, 0.002), DARK)
        self.box("joint_casting", (task.DRIVEN_X, 0.030, t + 0.032), (0.051, 0.048, 0.038), METAL)
        self.cyl("driven_axle", (task.DRIVEN_X, 0.012, task.AXIS_Z), 0.005, 0.074, DARK, axis="Y")
        group = self.group("driven_pulley")
        self.pulley(group)
        self.move(group, (task.DRIVEN_X, 0, task.AXIS_Z))
        self.box("bellcrank_pivot_support", (0.298, 0.079, t + 0.028), (0.025, 0.027, 0.026), DARK)
        self.cyl("bellcrank_pivot", (0.298, 0.079, t + 0.043), 0.0065, 0.016, METAL)
        self.box("gauge_stand", (0.235, 0.046, t + 0.045), (0.015, 0.016, 0.064), BLUE)
        gauge_z = t + 0.073
        self.box("gauge_scale", (GAUGE_START + 0.038, 0.055, gauge_z), (0.080, 0.020, 0.004), WHITE)
        low, high = TARGET_DEFLECTION
        self.box("gauge_band", (GAUGE_START + GAUGE_RATIO * (low + high) / 2, 0.052, gauge_z + 0.0022),
                 (GAUGE_RATIO * (high - low), 0.014, 0.001), GREEN)
        for mm in range(8):
            self.box(f"tick_{mm}", (GAUGE_START + mm * 0.001 * GAUGE_RATIO, 0.059, gauge_z + 0.003),
                     (0.0009, 0.005 if mm % 2 else 0.009, 0.001), DARK)
        self.box("gauge_bridge", (task.GAUGE_XY[0], 0.016, t + 0.068), (0.013, 0.047, 0.007), BLUE)
        self.cyl("probe_guide", (*task.GAUGE_XY, t + 0.075), 0.006, 0.021, METAL)

    def pulley(self, parent):
        self.cyl("timing_crown", (0, 0, 0), PITCH_RADIUS - 0.0002, BELT_WIDTH, GOLD, parent, "Y")
        for i, y in enumerate((-0.0058, 0.0058)):
            self.cyl(f"flange_{i}", (0, y, 0), PITCH_RADIUS + 0.002, 0.0022, METAL, parent, "Y")
        for i in range(16):
            a = i * 2 * pi / 16
            self.box(f"flute_{i}", (PITCH_RADIUS * cos(a), 0, PITCH_RADIUS * sin(a)),
                     (0.0017, BELT_WIDTH, 0.0017), GOLD, parent)

    def moving_shapes(self):
        self.motor = self.group("motor_mount")
        self.box("mount", (0, -0.027, 0.019), (0.103, 0.106, 0.011), METAL, self.motor)
        self.box("carriage_left", (0, 0.030, 0.021), (0.076, 0.020, 0.006), DARK, self.motor)
        self.cyl("motor_can", (0, 0.033, 0.041), 0.023, 0.035, MOTOR, self.motor, "Y")
        self.cyl("motor_endcap", (0, 0.051, 0.041), 0.020, 0.004, METAL, self.motor, "Y")
        self.cyl("motor_shaft", (0, 0.013, 0.041), 0.005, 0.035, DARK, self.motor, "Y")
        pulley = self.group("motor_pulley", self.motor)
        self.pulley(pulley)
        self.move(pulley, (0, 0, 0.041))
        self.bolts = []
        for i in range(2):
            path = self.group(f"motor_bolt_{i}")
            self.cyl("washer", (0, 0, -0.0038), 0.0050, 0.0013, DARK, path)
            self.cyl("head", (0, 0, -0.0013), 0.0046, 0.004, METAL, path)
            self.box("angle_mark", (0, 0.0029, 0.00085), (0.0017, 0.0030, 0.00065), WHITE, path)
            self.bolts.append(path)
        self.tab = self.group("pull_tab")
        self.box("link", (0.020, -0.010, -0.005), (0.050, 0.012, 0.012), DARK, self.tab)
        self.box("grip", (0, 0, 0), (0.023, 0.022, 0.025), HANDLE, self.tab)
        self.driver = self.group("socket_driver")
        self.box("grip", (0, 0, 0), (0.029, 0.023, 0.024), HANDLE, self.driver)
        self.box("torque_crossbar", (0, 0, 0.014), (0.052, 0.012, 0.010), HANDLE, self.driver)
        self.cyl("shaft", (0, 0, -0.028), 0.0027, 0.036, METAL, self.driver)
        self.cyl("socket", (0, 0, -0.047), 0.0054, 0.012, DARK, self.driver)
        self.gauge = self.group("gauge_plunger")
        self.cyl("cap", (0, 0, -0.003), 0.010, 0.006, BLUE, self.gauge)
        self.cyl("stem", (0, 0, -0.012), 0.003, 0.015, METAL, self.gauge)
        self.probe = self.group("gauge_probe")
        self.cyl("foot", (0, 0, 0.001), 0.003, 0.005, BLUE, self.probe)
        self.pointer = self.group("gauge_pointer")
        self.box("needle", (0, 0, 0), (0.0018, 0.011, 0.002), RED, self.pointer)
        self.belt = UsdGeom.Mesh.Define(self.stage, f"{ROOT}/timing_belt")
        self.belt.CreateSubdivisionSchemeAttr("none")
        self.belt.CreateDoubleSidedAttr(True)
        self.belt.CreateDisplayColorAttr([Gf.Vec3f(*RUBBER)])

    def update(self, contacts):
        rig = contacts.rig
        motor_x = task.DRIVEN_X + rig.span
        y = rig.offset
        self.move(self.motor, (motor_x, y, task.TOP), rig.yaw)
        self.move(self.driver, contacts.driver_xyz, contacts.driver_yaw)
        self.move(self.tab, contacts.pull_xyz)
        for group, bx, angle in zip(self.bolts, task.BOLT_X, rig.bolts, strict=True):
            self.move(group, (bx, task.BOLT_Y, task.BOLT_Z - angle * BOLT_PITCH / (2 * pi)), -angle)
        self.move(self.gauge, (*task.GAUGE_XY, task.GAUGE_CAP_Z - contacts.gauge_travel))
        loaded = midspan_deflection(rig.span) if rig.gauge_loaded else 0.0
        sag = (max(0, (START_SPAN + 0.00075) - rig.span) * rig.span / 2) ** 0.5
        d = max(sag, loaded)
        self.move(self.probe, ((task.DRIVEN_X + motor_x) / 2, 0, task.AXIS_Z + PITCH_RADIUS - d + 0.004))
        reading = loaded if rig.gauge_loaded else rig.checked_mm / 1000 if rig.checked_mm is not None else 0
        self.move(self.pointer, (GAUGE_START + GAUGE_RATIO * min(0.007, reading), 0.051, task.TOP + 0.077))
        self.update_belt(motor_x, y, d, sag, rig.seated)

    def update_belt(self, motor_x, offset, sag_top, sag_bottom, seated):
        r = PITCH_RADIUS
        z0 = task.AXIS_Z
        x0 = task.DRIVEN_X
        axial = offset if seated else offset + 0.011
        points = []
        for u in np.linspace(0, 1, 13, endpoint=False):
            points.append((x0 + u * (motor_x - x0), axial * u, z0 + r - 4 * u * (1 - u) * sag_top))
        for a in np.linspace(pi / 2, -pi / 2, 16, endpoint=False):
            points.append((motor_x + r * cos(a), axial, z0 + r * sin(a)))
        for u in np.linspace(1, 0, 13, endpoint=False):
            points.append((x0 + u * (motor_x - x0), axial * u, z0 - r + 4 * u * (1 - u) * sag_bottom))
        for a in np.linspace(-pi / 2, -3 * pi / 2, 16, endpoint=False):
            points.append((x0 + r * cos(a), 0, z0 + r * sin(a)))
        center = np.asarray(points)
        tangent = np.roll(center, -1, axis=0) - np.roll(center, 1, axis=0)
        tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
        normal = np.column_stack([-tangent[:, 2], np.zeros(len(center)), tangent[:, 0]])
        vertices = np.stack([
            center + normal * 0.0009 + (0, -BELT_WIDTH / 2, 0),
            center + normal * 0.0009 + (0, BELT_WIDTH / 2, 0),
            center - normal * 0.0009 + (0, BELT_WIDTH / 2, 0),
            center - normal * 0.0009 + (0, -BELT_WIDTH / 2, 0),
        ], axis=1).reshape(-1, 3).astype(np.float32)
        if not self.belt.GetFaceVertexCountsAttr().HasAuthoredValue():
            faces = []
            for i in range(len(center)):
                j = (i + 1) % len(center)
                for k in range(4):
                    a, b = 4 * i + k, 4 * i + (k + 1) % 4
                    c, d = 4 * j + (k + 1) % 4, 4 * j + k
                    faces.extend([a, b, c, d])
            self.belt.CreateFaceVertexCountsAttr([4] * len(center) * 4)
            self.belt.CreateFaceVertexIndicesAttr(faces)
        self.belt.CreatePointsAttr().Set(Vt.Vec3fArray.FromNumpy(vertices))
