"""Contact-based CPU prototypes for two independent toilet maintenance tasks."""

import argparse
import json
import math
from pathlib import Path

import mujoco as mj
import numpy as np

DT = 0.002
WHITE = ".86 .90 .91 1"
TOOL = ".99 .55 .14 1"
STEEL = ".45 .59 .64 1"
WATER = ".14 .65 .81 .83"
PAPER = ".95 .90 .79 1"
WASTE = ".33 .18 .12 1"
JETS = [(-0.072, 0.054), (-0.072, -0.054), (0.072, 0.054), (0.072, -0.054)]
NAMES = ["Front left", "Front right", "Rear left", "Rear right"]
PROFILE = [
    (58, 49, -81, 0),
    (64, 55, -71, 0),
    (96, 74, -58, 0),
    (119, 94, -39, 0),
    (142, 108, -18, 0),
    (154, 118, -10, 0),
    (147, 111, 2, 0),
    (164, 128, 6, 0),
    (182, 144, 4, 0),
    (191, 151, -4, 0),
    (193, 152, -17, 0),
    (185, 144, -30, 4),
    (169, 127, -43, 10),
    (145, 106, -58, 21),
    (113, 83, -76, 45),
    (105, 78, -93, 75),
    (108, 78, -107, 80),
    (119, 81, -117, 82),
    (133, 89, -129, 80),
    (136, 90, -133, 79),
]
RIM = 0.218
DOCK = (-0.245, -0.144, 0.123)
LIP = 0.207
JET_STROKE = 0.023
JET_TILT = math.radians(20)
TRAP_STROKE = 0.105


def values(sequence):
    return " ".join(f"{number:.9g}" for number in sequence)


def geom(name, shape, position, size, color, *, collision=True, extra=""):
    mask = "" if collision else ' contype="0" conaffinity="0"'
    return (
        f'<geom name="{name}" type="{shape}" pos="{values(position)}" size="{values(size)}"'
        f' rgba="{color}"{mask} {extra}/>'
    )


def bowl():
    """Use the old rim profile as visuals and preserve the opening in the colliders."""
    segments = 48
    vertices = []
    for a, b, z, shift in PROFILE:
        vertices.extend(
            (
                (shift + a * math.cos(t)) * 0.001,
                b * math.sin(t) * 0.001,
                RIM + z * 0.001,
            )
            for t in np.linspace(0, 2 * math.pi, segments, endpoint=False)
        )
    faces = []
    for j in range(len(PROFILE)):
        k = (j + 1) % len(PROFILE)
        for i in range(segments):
            nxt = (i + 1) % segments
            a, b, c, d = (
                j * segments + i,
                j * segments + nxt,
                k * segments + nxt,
                k * segments + i,
            )
            faces.extend(((a, d, c), (a, c, b)))
    asset = f'<mesh name="ceramic" vertex="{values(np.ravel(vertices))}" face="{" ".join(map(str,np.ravel(faces)))}"/>'
    visual = (
        '<geom name="bowl" type="mesh" mesh="ceramic" rgba=".85 .90 .92 1"'
        ' contype="0" conaffinity="0"/>'
    )
    collisions = []
    for layer, (rx, ry, z, radius) in enumerate(
        (
            (0.161, 0.125, 0.223, 0.008),
            (0.157, 0.121, 0.208, 0.006),
            (0.120, 0.097, 0.180, 0.005),
            (0.096, 0.077, 0.161, 0.006),
        )
    ):
        angles = np.linspace(0, math.tau, 36, endpoint=False)
        for i, a in enumerate(angles):
            x, y = rx * math.cos(a), ry * math.sin(a)
            half = 0.5 * math.hypot(rx * math.sin(a), ry * math.cos(a)) * math.tau / 36
            yaw = math.atan2(ry * math.cos(a), -rx * math.sin(a))
            collisions.append(
                geom(
                    f"bowl_{layer}_{i}",
                    "box",
                    (x, y, z),
                    (half + 0.001, radius, 0.005),
                    "0 0 0 0",
                    extra=f'euler="0 0 {yaw}"',
                )
            )
    return asset, visual + "".join(collisions)


def loft_mesh(name, rings, hollow=False):
    """Join equal-length outlines into a closed mesh for the rear fittings."""
    count = len(rings[0])
    vertices = [point for ring in rings for point in ring]
    faces = []
    for j in range(len(rings) if hollow else len(rings) - 1):
        k = (j + 1) % len(rings)
        for i in range(count):
            nxt = (i + 1) % count
            a, b, c, d = j * count + i, j * count + nxt, k * count + nxt, k * count + i
            faces.extend(((a, b, c), (a, c, d)))
    if not hollow:
        for offset, ring, reverse in (
            (0, rings[0], True),
            ((len(rings) - 1) * count, rings[-1], False),
        ):
            centre = len(vertices)
            vertices.append(np.mean(ring, axis=0))
            for i in range(count):
                a, b = offset + i, offset + (i + 1) % count
                faces.append((centre, b, a) if reverse else (centre, a, b))
    triangles = " ".join(map(str, np.ravel(faces)))
    return f'<mesh name="{name}" vertex="{values(np.ravel(vertices))}" face="{triangles}"/>'


def rounded_outline(centre, half_x, half_y, radius, z):
    points = []
    for i, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        for a in np.linspace(i * math.pi / 2, (i + 1) * math.pi / 2, 8):
            points.append(
                (
                    centre + sx * (half_x - radius) + radius * math.cos(a),
                    sy * (half_y - radius) + radius * math.sin(a),
                    z,
                )
            )
    return points


def seat_and_cistern():
    """Model an open seat ring, raised lid and close-coupled ceramic tank."""
    angles = np.linspace(0, math.tau, 48, endpoint=False)
    seat_sections = (
        (0.170, 0.131, 0.225),
        (0.176, 0.137, 0.230),
        (0.174, 0.135, 0.239),
        (0.168, 0.129, 0.243),
        (0.143, 0.109, 0.243),
        (0.140, 0.106, 0.239),
        (0.140, 0.106, 0.231),
        (0.145, 0.111, 0.225),
    )
    seat = [
        [(rx * math.cos(a), ry * math.sin(a), z) for a in angles]
        for rx, ry, z in seat_sections
    ]
    pitch = math.radians(13)
    raised = []
    for inset, thickness in (
        (0.93, -0.008),
        (1.0, -0.004),
        (1.0, 0.004),
        (0.93, 0.008),
    ):
        section = []
        for a in angles:
            length = max(0.017, 0.150 + 0.150 * inset * math.cos(a))
            width = 0.118 * inset * math.sin(a)
            section.append(
                (
                    0.158 + length * math.sin(pitch) - thickness * math.cos(pitch),
                    width,
                    0.234 + length * math.cos(pitch) + thickness * math.sin(pitch),
                )
            )
        raised.append(section)
    tank = [
        rounded_outline(0.264, hx, hy, r, z)
        for hx, hy, r, z in (
            (0.044, 0.099, 0.017, 0.236),
            (0.052, 0.118, 0.022, 0.249),
            (0.064, 0.132, 0.025, 0.382),
            (0.062, 0.130, 0.024, 0.400),
        )
    ]
    tank_cover = [
        rounded_outline(0.264, hx, hy, r, z)
        for hx, hy, r, z in (
            (0.064, 0.133, 0.024, 0.401),
            (0.065, 0.134, 0.024, 0.405),
            (0.062, 0.131, 0.023, 0.412),
            (0.057, 0.126, 0.021, 0.416),
        )
    ]
    assets = "".join(
        (
            loft_mesh("seat_ring", seat, hollow=True),
            loft_mesh("raised_lid", raised),
            loft_mesh("cistern", tank),
            loft_mesh("cistern_cover", tank_cover),
        )
    )
    objects = [
        '<geom name="seat_ring" type="mesh" mesh="seat_ring" rgba=".95 .96 .95 1"'
        ' contype="0" conaffinity="0"/>',
        '<geom name="raised_lid" type="mesh" mesh="raised_lid" rgba=".95 .96 .95 1"/>',
        '<geom name="cistern" type="mesh" mesh="cistern" rgba=".86 .90 .91 1"/>',
        '<geom name="cistern_cover" type="mesh" mesh="cistern_cover" rgba=".94 .96 .95 1"/>',
        geom("flush_button", "cylinder", (0.267, 0, 0.417), (0.013, 0.002), STEEL),
        geom("rear_deck", "box", (0.222, 0, 0.225), (0.076, 0.100, 0.011), WHITE),
        geom("rear_pedestal", "box", (0.241, 0, 0.158), (0.045, 0.085, 0.063), WHITE),
    ]
    for i, a in enumerate(angles):
        rx, ry = 0.158, 0.122
        yaw = math.atan2(ry * math.cos(a), -rx * math.sin(a))
        half = math.pi / len(angles) * math.hypot(rx * math.sin(a), ry * math.cos(a))
        objects.append(
            geom(
                f"seat_contact_{i}",
                "box",
                (rx * math.cos(a), ry * math.sin(a), 0.234),
                (half + 0.001, 0.011, 0.008),
                "0 0 0 0",
                extra=f'euler="0 0 {yaw}"',
            )
        )
    for side in (-1, 1):
        objects.append(
            geom(
                f"seat_hinge_{side}",
                "cylinder",
                (0.160, side * 0.057, 0.244),
                (0.010, 0.016),
                STEEL,
                extra='zaxis="0 1 0"',
            )
        )
    return assets, "".join(objects)


def carriage():
    actor = ['<body name="carriage" pos="0 0 0" gravcomp="1">']
    for name, axis in zip("xyz", ("1 0 0", "0 1 0", "0 0 1")):
        actor.append(
            f'<joint name="{name}" type="slide" axis="{axis}" damping="8" armature=".04"/>'
        )
    actor.append(
        '<joint name="yaw" type="hinge" axis="0 0 1" damping=".06" armature=".002"/>'
    )
    actor.append(
        geom(
            "knuckle",
            "box",
            (0, 0, 0.036),
            (0.024, 0.040, 0.011),
            STEEL,
            collision=False,
            extra='mass=".09"',
        )
    )
    actor.append(
        geom(
            "column", "cylinder", (0, 0, 0.105), (0.009, 0.057), STEEL, collision=False
        )
    )
    controls = [
        f'<position name="{name}" joint="{name}" kp="1250" kv="36" forcerange="-25 25"/>'
        for name in "xyz"
    ]
    controls.append(
        '<position name="yaw" joint="yaw" kp="12" kv="1.5" forcerange="-3 3"/>'
    )
    for side in (-1, 1):
        tag = "left" if side == -1 else "right"
        actor.extend(
            [
                f'<body name="{tag}_finger" pos="0 {side*.028} .001" gravcomp="1">',
                f'<joint name="{tag}" type="slide" axis="0 {-side} 0" damping="2" range="0 .019"/>',
                geom(
                    f"{tag}_pad",
                    "box",
                    (0, 0, 0),
                    (0.010, 0.004, 0.014),
                    ".12 .20 .24 1",
                    extra='mass=".025" friction="2.3 .06 .008"',
                ),
                geom(
                    f"{tag}_support",
                    "box",
                    (0, 0, 0.021),
                    (0.011, 0.003, 0.012),
                    STEEL,
                    collision=False,
                ),
                "</body>",
            ]
        )
        controls.append(
            f'<position name="{tag}" joint="{tag}" kp="850" kv="8" forcerange="-8 8"/>'
        )
    return "".join(actor) + "</body>", "".join(controls)


def probe():
    parts = [
        geom(
            "probe_handle",
            "box",
            (0, 0, 0),
            (0.009, 0.011, 0.012),
            TOOL,
            extra='mass=".025" friction="2.3 .06 .008"',
        ),
        geom(
            "probe_arm",
            "box",
            (0.015, 0, -0.017),
            (0.018, 0.004, 0.003),
            TOOL,
            extra='mass=".004"',
        ),
        geom(
            "probe_drop",
            "box",
            (0, 0, -0.008),
            (0.004, 0.005, 0.011),
            TOOL,
            extra='mass=".002"',
        ),
    ]
    for side in (-1, 1):
        parts.append(
            geom(
                f"tine_{side}",
                "box",
                (0.041, side * 0.0052, -0.018),
                (0.014, 0.0016, 0.0018),
                TOOL,
                extra='mass=".002"',
            )
        )
    return (
        f'<body name="tool" pos="{values(DOCK)}"><freejoint/>'
        + "".join(parts)
        + "</body>"
    )


def jet_fixture(blocked):
    objects = []
    for i, (x, y) in enumerate(JETS):
        ux, uy = np.array((x, y)) / math.hypot(x, y)
        axis = values(
            (ux * math.sin(JET_TILT), uy * math.sin(JET_TILT), math.cos(JET_TILT))
        )
        objects.append(
            geom(
                f"outlet_{i}",
                "cylinder",
                (x, y, LIP + 0.010),
                (0.010, 0.010),
                STEEL,
                collision=False,
                extra=f'zaxis="{axis}"',
            )
        )
        # A four-sided nozzle leaves a native aperture and a tool-access corridor.
        for j in (-1, 1):
            objects.append(
                geom(
                    f"guide_{i}_{j}",
                    "box",
                    (x, y + j * 0.008, LIP + 0.011),
                    (0.007, 0.002, 0.008),
                    STEEL,
                )
            )
        if i == blocked:
            objects.append(
                f'<body name="plug" pos="{values((x,y,LIP))}" zaxis="{axis}" gravcomp="1">'
            )
            objects.append(
                f'<joint name="plug" type="slide" axis="0 0 -1" range="0 {JET_STROKE}" '
                'frictionloss=".32" damping=".45"/>'
            )
            for name, offset, radius, height in (
                ("plug_insert", 0.002, 0.0045, 0.007),
                ("plug_stem", -0.010, 0.0013, 0.010),
                ("plug_flange", -0.018, 0.0075, 0.002),
            ):
                objects.append(
                    geom(
                        name,
                        "cylinder",
                        (0, 0, offset),
                        (radius, height),
                        WASTE,
                        extra='mass=".004"',
                    )
                )
            objects.append("</body>")
    return "".join(objects)


def plunger():
    x, y, z = DOCK
    parts = [
        geom(
            "plunger_handle",
            "box",
            (0, 0, 0),
            (0.009, 0.011, 0.012),
            TOOL,
            extra='mass=".032" friction="2.3 .06 .008"',
        ),
        geom(
            "plunger_shaft",
            "cylinder",
            (0, 0, -0.019),
            (0.004, 0.019),
            STEEL,
            extra='mass=".013"',
        ),
        geom(
            "plunger_cup",
            "cylinder",
            (0, 0, -0.042),
            (0.019, 0.006),
            ".20 .26 .32 1",
            extra='mass=".025" friction="1.3 .03 .003"',
        ),
    ]
    return (
        f'<body name="tool" pos="{values((x,y,z))}"><freejoint/>'
        + "".join(parts)
        + "</body>"
    )


def trap_fixture():
    elements = [
        geom(
            "trap_base",
            "box",
            (0.06, 0, 0.078),
            (0.12, 0.035, 0.007),
            ".22 .34 .40 1",
            collision=False,
        ),
        geom(
            "sink_inlet",
            "cylinder",
            (0, 0, 0.147),
            (0.027, 0.012),
            STEEL,
            collision=False,
        ),
        '<body name="diaphragm" pos="0 0 .153">',
        '<joint name="diaphragm" type="slide" axis="0 0 -1" range="0 .019" '
        'stiffness="370" damping="4"/>',
        geom(
            "diaphragm",
            "cylinder",
            (0, 0, 0),
            (0.017, 0.004),
            ".14 .49 .58 1",
            extra='mass=".03"',
        ),
        "</body>",
        '<body name="clog" pos=".025 0 .105">',
        f'<joint name="clog" type="slide" axis="1 0 0" range="0 {TRAP_STROKE}" '
        'frictionloss=".68" damping="3.6"/>',
    ]
    for name, pos, size, color in (
        ("waste_a", (0.002, 0, 0), (0.016, 0.013, 0.014), WASTE),
        ("waste_b", (0.015, 0.003, -0.004), (0.013, 0.009, 0.009), WASTE),
        ("paper_a", (-0.005, 0, 0.010), (0.013, 0.014, 0.006), PAPER),
        ("paper_b", (0.006, -0.009, 0.002), (0.008, 0.009, 0.011), PAPER),
    ):
        elements.append(geom(name, "ellipsoid", pos, size, color, extra='mass=".009"'))
    elements.append("</body>")
    for side in (-1, 1):
        elements.append(
            geom(
                f"pipe_side_{side}",
                "box",
                (0.078, side * 0.020, 0.103),
                (0.085, 0.004, 0.021),
                ".18 .45 .54 .32",
                collision=False,
            )
        )
    elements.append(
        geom(
            "pipe_lid",
            "box",
            (0.078, 0, 0.126),
            (0.085, 0.020, 0.002),
            ".20 .48 .58 .24",
            collision=False,
        )
    )
    return "".join(elements)


def make_model(kind, blocked):
    asset, toilet = bowl()
    rear_assets, rear_fittings = seat_and_cistern()
    actor, motors = carriage()
    tool, fixture = (
        (probe(), jet_fixture(blocked))
        if kind == "rim"
        else (plunger(), trap_fixture())
    )
    stand_z = DOCK[2] - (0.026 if kind == "rim" else 0.048)
    stand = geom(
        "dock",
        "box",
        (*DOCK[:2], stand_z - 0.005),
        (0.046, 0.021, 0.005),
        ".20 .32 .38 1",
    )
    if kind == "rim":
        # The shaft is the lowest part of the parked probe.
        stand = geom(
            "dock",
            "box",
            (DOCK[0] + 0.024, DOCK[1], DOCK[2] - 0.027),
            (0.046, 0.020, 0.005),
            ".20 .32 .38 1",
        )
    return mj.MjModel.from_xml_string(f"""
<mujoco model="toilet-maintenance">
 <compiler angle="radian"/>
 <option timestep="{DT}" integrator="implicitfast" cone="elliptic" iterations="65"/>
 <visual><global offwidth="960" offheight="540"/><map znear=".005"/>
         <quality shadowsize="1024"/></visual>
 <default><geom friction=".85 .02 .003" condim="4" solref=".007 1" margin=".0002"/></default>
 <asset>{asset}{rear_assets}</asset>
 <worldbody>
  <light pos="-.45 -.35 .8" dir=".45 .3 -.8" diffuse=".82 .82 .82" castshadow="true"/>
  <light pos=".48 .22 .60" dir="-.45 -.3 -.65" diffuse=".42 .46 .51" castshadow="false"/>
  <geom name="desk" type="plane" size="1 1 .1" rgba=".10 .16 .20 1"/>
  {toilet}{rear_fittings}{stand}{actor}{tool}{fixture}
 </worldbody>
 <actuator>{motors}</actuator>
</mujoco>
""")


class Trial:
    def __init__(self, kind, blocked=1, recording=None):
        self.kind = kind
        self.model = make_model(kind, blocked)
        self.data = mj.MjData(self.model)
        self.blocked = blocked
        self.target = np.array([DOCK[0], DOCK[1], 0.265, 0.0, 0.0, 0.0])
        self.data.qpos[:3] = self.target[:3]
        self.data.ctrl[:] = self.target
        self.rinse = np.zeros(4)
        self.water_level = 1.0
        self.pressure = 0.0
        self.peak_pressure = 0.0
        self.finger_contacts = 0
        self.tool_contacts = 0
        self.phase = "Start"
        self.frames = recording
        self.trace = []
        if recording:
            recording.attach(self)
        mj.mj_forward(self.model, self.data)

    def contact(self, first, second):
        """Return native contact involving the two named sets of geometries."""
        for c in self.data.contact:
            a, b = self.model.geom(c.geom1).name, self.model.geom(c.geom2).name
            if (a in first and b in second) or (b in first and a in second):
                return True
        return False

    def joint(self, name):
        return float(self.data.joint(name).qpos[0])

    def update(self):
        if self.contact({"left_pad", "right_pad"}, {"probe_handle", "plunger_handle"}):
            self.finger_contacts += 1
        if self.kind == "rim":
            if self.contact({"tine_-1", "tine_1"}, {"plug_flange", "plug_stem"}):
                self.tool_contacts += 1
        else:
            piston = self.data.joint("diaphragm")
            self.pressure = 0.0
            if self.contact({"plunger_cup"}, {"diaphragm"}):
                self.tool_contacts += 1
                # Pulse through a filled trap during the actual compression stroke.
                self.pressure = min(8.0, max(0.0, float(piston.qvel[0])) * 71.0)
                self.peak_pressure = max(self.pressure, self.peak_pressure)
            self.data.qfrc_applied[self.model.joint("clog").dofadr[0]] = self.pressure

    def step(self, goal, seconds, phase, observe=None):
        self.phase = phase
        before = self.target.copy()
        steps = round(seconds / DT)
        for i in range(steps):
            s = (i + 1) / steps
            weight = s * s * (3 - 2 * s)
            self.data.ctrl[:] = before + weight * (goal - before)
            self.update()
            mj.mj_step(self.model, self.data)
            if observe:
                observe()
            if round(self.data.time / DT) % 20 == 0:
                self.trace.append(self.reading())
                if self.frames:
                    self.frames.draw()
        self.target = goal.copy()

    def move(self, xyz, seconds, phase, grip=None, yaw=None):
        goal = self.target.copy()
        goal[:3] = xyz
        if yaw is not None:
            goal[3] = yaw
        if grip is not None:
            goal[4:] = 0.0175 if grip else 0.0
        self.step(goal, seconds, phase)

    def hold(self, seconds, phase, observe=None):
        self.step(self.target.copy(), seconds, phase, observe)

    def pick(self, lift_height=0.291):
        self.move(DOCK, 0.65, "Approach tool")
        self.move(DOCK, 0.35, "Grip tool", True)
        self.move((*DOCK[:2], lift_height), 0.70, "Lift tool")

    def reading(self):
        reading = {
            "seconds": round(self.data.time, 3),
            "phase": self.phase,
            "tool": self.data.body("tool").xpos.tolist(),
            "tool_contacts": self.tool_contacts,
        }
        if self.kind == "rim":
            reading.update(
                plug_mm=round(1000 * self.joint("plug"), 2),
                collected_ml=np.round(self.rinse, 2).tolist(),
            )
        else:
            reading.update(
                clog_mm=round(1000 * self.joint("clog"), 2),
                water_percent=round(100 * self.water_level, 2),
            )
        return reading

    def rim_flow(self):
        flows = np.full(4, 100.0)
        flows[self.blocked] *= 0.2 + 0.8 * np.clip(self.joint("plug") / 0.014, 0.0, 1.0)
        return flows

    def collect(self, phase):
        self.rinse[:] = 0
        self.hold(
            1.0, phase, lambda: np.add(self.rinse, DT * self.rim_flow(), out=self.rinse)
        )
        return self.rinse.copy()

    def run_rim(self, control):
        before = self.collect("Measure each outlet")
        selected = int(np.argmin(before))
        outlet = np.array(JETS[selected])
        radial = outlet / np.linalg.norm(outlet)
        across = np.array([-radial[1], radial[0]])
        angle = math.atan2(radial[1], radial[0])
        inside = outlet - 0.070 * radial
        engaged = (
            outlet
            - (0.043 + 0.011 * math.sin(JET_TILT)) * radial
            + (0.023 * across if control else 0.0)
        )
        extracted = engaged - 0.023 * math.sin(JET_TILT) * radial
        self.pick()
        # Lift over the lip, descend inside the bowl and slide the fork around
        # the exposed stem above its flange.
        self.move((*inside, 0.281), 1.05, "Move above the weak outlet", yaw=angle)
        self.move((*inside, LIP + 0.008), 0.65, "Lower inside the bowl")
        self.move((*engaged, LIP + 0.008), 0.75, "Engage obstruction")
        self.move((*extracted, LIP - 0.019), 0.74, "Withdraw obstruction")
        self.move((*inside, LIP - 0.019), 0.65, "Release the cleared outlet")
        self.move((*inside, 0.280), 0.65, "Clear the bowl")
        after = self.collect("Retest under the same head")
        passed = (
            self.tool_contacts > 0
            and self.joint("plug") >= 0.016
            and np.all((after >= 80) & (after <= 120))
        )
        self.hold(0.65, "Flow restored" if passed else "Outlet remains restricted")
        return {
            "score": 1.0 if passed else 0.25,
            "before_ml": before.tolist(),
            "after_ml": after.tolist(),
            "diagnosed": NAMES[selected],
            "extraction_mm": self.joint("plug") * 1000,
        }

    def drain(self, phase):
        self.water_level = 1.0

        def update_level():
            clear = np.clip((self.joint("clog") - 0.025) / 0.052, 0.0, 1.0)
            self.water_level = max(0.0, self.water_level - DT * (0.09 + 0.70 * clear))

        self.hold(1.25, phase, update_level)
        return self.water_level

    def run_trap(self, control):
        before = self.drain("Test drainage before repair")
        # The cup hangs below the gripper and must pass above the seat ring.
        carry_height = 0.325
        self.pick(carry_height)
        x = 0.050 if control else 0.0
        self.move((x, 0, carry_height), 0.80, "Centre plunger over the drain")
        self.move((x, 0, 0.207), 0.60, "Seat cup")
        for _ in range(4):
            self.move((x, 0, 0.177), 0.24, "Compress the sealed cup")
            self.move((x, 0, 0.204), 0.33, "Release stroke")
        self.move((x, 0, carry_height), 0.70, "Lift cup clear")
        after = self.drain("Retest drainage")
        passed = (
            before >= 0.70
            and self.joint("clog") > 0.075
            and after < 0.15
            and self.tool_contacts > 0
        )
        self.hold(0.65, "Drainage restored" if passed else "Trapway remains restricted")
        return {
            "score": float(passed),
            "before_water_percent": 100 * before,
            "after_water_percent": 100 * after,
            "clog_travel_mm": self.joint("clog") * 1000,
            "peak_pressure_force_n": self.peak_pressure,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=("rim", "trap"))
    parser.add_argument("--blocked", type=int, choices=range(4), default=1)
    parser.add_argument("--control", action="store_true")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--video", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    recording = None
    if args.video:
        from recording import Recording

        recording = Recording(args.out / "demo.mp4")
    t = Trial(args.kind, args.blocked, recording)
    result = t.run_rim(args.control) if args.kind == "rim" else t.run_trap(args.control)
    result.update(
        finger_contact_steps=t.finger_contacts, tool_contact_steps=t.tool_contacts
    )
    (args.out / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    (args.out / "trace.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in t.trace)
    )
    if recording:
        recording.close()
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
