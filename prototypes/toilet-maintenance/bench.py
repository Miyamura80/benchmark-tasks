"""Contact-based CPU prototypes for two independent toilet maintenance tasks."""

import argparse
from dataclasses import dataclass
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
WASTE = ".33 .18 .12 1"
# Centres follow the inside edge of the porcelain rim in four quadrants.
JETS = [(-0.1056, 0.0792), (-0.1056, -0.0792), (0.1056, 0.0792), (0.1056, -0.0792)]
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
JET_HEIGHT = RIM - 0.007
JET_TILT = math.radians(58)
SCALE_OCCLUSION = (0.21, 0.16, 0.24, 0.19)
SCALE_BREAK_WORK = (0.000075, 0.000015, 0.00020, 0.00004)
RINSE_SECONDS = 2.0
JET_OPEN_FLOW = 100.0
TRAP_EXIT = 0.193
TRAP_STROKES = 8


@dataclass(frozen=True)
class TrapPiece:
    name: str
    material: str
    x: float
    y: float
    z: float
    size: tuple[float, float, float]
    friction: float
    damping: float


TRAP_PIECES = (
    TrapPiece(
        "paper_1", "paper", 0.052, -0.009, 0.114, (0.012, 0.009, 0.008), 0.31, 8.424
    ),
    TrapPiece(
        "feces_1", "feces", 0.060, 0.007, 0.100, (0.014, 0.010, 0.008), 0.23, 5.616
    ),
    TrapPiece(
        "paper_2", "paper", 0.072, 0.009, 0.115, (0.013, 0.009, 0.008), 0.46, 8.10
    ),
    TrapPiece(
        "feces_2", "feces", 0.081, -0.008, 0.100, (0.016, 0.009, 0.008), 0.27, 5.832
    ),
    TrapPiece(
        "paper_3", "paper", 0.087, -0.009, 0.116, (0.012, 0.009, 0.007), 0.38, 7.668
    ),
    TrapPiece(
        "feces_3", "feces", 0.101, 0.007, 0.102, (0.014, 0.008, 0.009), 0.26, 5.292
    ),
)


def values(sequence):
    return " ".join(f"{number:.9g}" for number in sequence)


def jet_axis(outlet):
    radial = np.asarray(outlet) / np.linalg.norm(outlet)
    return np.r_[radial * math.sin(JET_TILT), math.cos(JET_TILT)]


def scale_patch(i, outlet):
    """A rough chip bonded to the bore lining until the scraper rubs it loose."""
    axis = jet_axis(outlet)
    across = np.array((-outlet[1], outlet[0], 0.0)) / np.linalg.norm(outlet)
    across = np.cross(axis, np.cross(across, axis))
    radial = np.cross(across, axis)
    angle = math.radians((24, 111, 203, 298)[i])
    offset = (
        0.0028 * (math.cos(angle) * radial + math.sin(angle) * across) - 0.0008 * axis
    )
    position = np.array([*outlet, JET_HEIGHT]) + offset
    name = f"scale_{i}"
    color = (".83 .78 .66 1", ".89 .84 .74 1", ".78 .72 .59 1", ".86 .79 .66 1")[i]
    size = (
        (0.0024, 0.0017, 0.0019),
        (0.0018, 0.0020, 0.0014),
        (0.0026, 0.0017, 0.0018),
        (0.0021, 0.0019, 0.0015),
    )[i]
    pieces = [
        f'<body name="{name}" pos="{values(position)}" '
        f'xyaxes="{values((*radial, *across))}"><freejoint/>',
        geom(
            name,
            "ellipsoid",
            (0, 0, 0),
            size,
            color,
            extra='mass=".0012" material="scale_finish"',
        ),
    ]
    for j, offset in enumerate(
        ((-0.50, 0.18, -0.48), (0.31, 0.55, -0.53), (0.44, -0.35, -0.62))
    ):
        pieces.append(
            geom(
                f"{name}_grain_{j}",
                "ellipsoid",
                tuple(o * s for o, s in zip(offset, size)),
                tuple(0.42 * s for s in size),
                ".96 .92 .81 1",
                collision=False,
                extra='mass="0" material="scale_finish"',
            )
        )
    return "".join(pieces) + "</body>"


def geom(name, shape, position, size, color, *, collision=True, extra=""):
    mask = "" if collision else ' contype="0" conaffinity="0"'
    return (
        f'<geom name="{name}" type="{shape}" pos="{values(position)}" size="{values(size)}"'
        f' rgba="{color}"{mask} {extra}/>'
    )


def bowl(kind):
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
        # The thin bands of colliders need an actual passage into each rim bore.
        # Subdivide the two upper bands so the openings do not remove whole wall panels.
        has_bores = kind == "rim" and layer < 2
        count = 120 if has_bores else 36
        angles = np.linspace(0, math.tau, count, endpoint=False)
        bores = [math.atan2(y / ry, x / rx) for x, y in JETS] if has_bores else []
        for i, a in enumerate(angles):
            x, y = rx * math.cos(a), ry * math.sin(a)
            tangent = math.hypot(rx * math.sin(a), ry * math.cos(a))
            half = 0.5 * tangent * math.tau / count
            if any(
                abs(math.remainder(a - bore, math.tau)) * tangent < 0.007 + half
                for bore in bores
            ):
                continue
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
    parts.append(
        geom(
            "scraper_edge",
            "box",
            (0.037, 0, -0.018),
            (0.008, 0.0045, 0.0015),
            ".52 .62 .63 1",
            extra='mass=".004"',
        )
    )
    return (
        f'<body name="tool" pos="{values(DOCK)}"><freejoint/>'
        + "".join(parts)
        + "</body>"
    )


def jet_socket_mesh():
    angles = np.linspace(0, math.tau, 32, endpoint=False)
    rings = [
        [(radius * math.cos(a), radius * math.sin(a), z) for a in angles]
        for radius, z in (
            (0.0064, 0.000),
            (0.0066, 0.004),
            (0.0038, 0.004),
            (0.0038, 0.001),
            (0.0045, 0.000),
        )
    ]
    return loft_mesh("jet_socket", rings, hollow=True)


def jet_fixture(blocked):
    objects = []
    for i, outlet in enumerate(JETS):
        axis = values(jet_axis(outlet))
        mouth = values((*outlet, JET_HEIGHT))
        objects.extend(
            (
                f'<body name="socket_{i}" pos="{mouth}" zaxis="{axis}">',
                f'<geom name="outlet_{i}" type="mesh" mesh="jet_socket"'
                ' rgba=".85 .90 .92 1" contype="0" conaffinity="0"/>',
                geom(
                    f"bore_{i}",
                    "cylinder",
                    (0, 0, 0.0012),
                    (0.0038, 0.0003),
                    ".13 .18 .19 1",
                    collision=False,
                ),
            )
        )
        # The contact guide sits inside the ceramic and leaves the bore open.
        for j in range(4):
            a = j * math.pi / 2
            objects.append(
                geom(
                    f"guide_{i}_{j}",
                    "box",
                    (0.0057 * math.cos(a), 0.0057 * math.sin(a), 0.006),
                    (0.0012, 0.0042, 0.0045),
                    "0 0 0 0",
                    extra=f'euler="0 0 {a}"',
                )
            )
        objects.append("</body>")
        if i == blocked:
            objects.extend(scale_patch(j, outlet) for j in range(len(SCALE_OCCLUSION)))
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


def folded_sheet(name, size, layer, phase):
    """A thin, torn sheet with raised creases and an irregular edge."""
    a, b, c = size
    n = 17
    vertices = []
    for side in (-1, 1):
        for v in np.linspace(-1, 1, n):
            for u in np.linspace(-1, 1, n):
                edge_x = (
                    0.86 + 0.10 * math.sin(8 * v + phase) + 0.036 * math.cos(21 * v)
                )
                edge_y = (
                    0.79 + 0.13 * math.sin(7 * u + 2 * phase) + 0.039 * math.cos(19 * u)
                )
                x = a * (u * edge_x - 0.13 * v * math.sin(phase))
                y = b * (v * edge_y + 0.16 * u * math.sin(phase))
                crest = 0.32 * math.sin(3 * u + 1.8 * v + phase)
                crease = 0.16 * abs(math.sin(5 * u - 4 * v + phase))
                crinkle = 0.07 * math.sin(13 * u + 10 * v + phase)
                z = (
                    c * (layer + crest + crease + crinkle - 0.29 * abs(v))
                    + side * 0.00017
                )
                vertices.append((x, y, z))
    faces = []
    top = n * n
    for j in range(n - 1):
        for i in range(n - 1):
            a, b, c, d = j * n + i, j * n + i + 1, (j + 1) * n + i + 1, (j + 1) * n + i
            faces.extend(
                (
                    (a, c, b),
                    (a, d, c),
                    (top + a, top + b, top + c),
                    (top + a, top + c, top + d),
                )
            )
    border = list(range(n)) + [j * n + n - 1 for j in range(1, n)]
    border += [(n - 1) * n + i for i in range(n - 2, -1, -1)]
    border += [j * n for j in range(n - 2, 0, -1)]
    for a, b in zip(border, border[1:] + border[:1]):
        faces.extend(((a, b, top + b), (a, top + b, top + a)))
    triangles = " ".join(map(str, np.ravel(faces)))
    return f'<mesh name="{name}" vertex="{values(np.ravel(vertices))}" face="{triangles}"/>'


def trap_fixture():
    assets = ['<material name="wet_tissue" specular=".18" shininess=".12"/>']
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
    ]
    for index, p in enumerate(TRAP_PIECES):
        paper = p.material == "paper"
        elements.extend(
            [
                f'<body name="{p.name}" pos="{p.x} {p.y} {p.z}">',
                f'<joint name="{p.name}" type="slide" axis="1 0 0" range="0 {0.39-p.x}" '
                f'frictionloss="{p.friction}" damping="{p.damping}"/>',
                geom(
                    p.name,
                    "ellipsoid",
                    (0, 0, 0),
                    p.size,
                    "0 0 0 0" if paper else WASTE,
                    extra='mass=".018" contype="2" conaffinity="2"',
                ),
            ]
        )
        if paper:
            for layer, (offset, shade) in enumerate(
                (
                    (-0.52, ".72 .68 .57 1"),
                    (-0.03, ".84 .81 .71 1"),
                    (0.44, ".94 .91 .82 1"),
                )
            ):
                name = f"{p.name}_fold_{layer}"
                assets.append(
                    folded_sheet(name, p.size, offset, index * 0.72 + layer * 1.53)
                )
                elements.append(
                    f'<geom name="{name}" type="mesh" mesh="{name}" rgba="{shade}"'
                    ' material="wet_tissue" contype="0" conaffinity="0" mass="0"/>'
                )
            elements.append("</body>")
            continue
        # Uneven lobes show the fecal pieces in the cutaway.
        a, b, c = p.size
        lobes = ((-0.22 * a, 0.31 * b, 0.38 * c), (0.28 * a, -0.27 * b, 0.31 * c))
        shades = (".44 .23 .11 1", ".27 .13 .07 1")
        for i, (pos, shade) in enumerate(zip(lobes, shades)):
            elements.append(
                geom(
                    f"{p.name}_lobe_{i}",
                    "ellipsoid",
                    pos,
                    (0.54 * a, 0.63 * b, 0.61 * c),
                    shade,
                    collision=False,
                )
            )
        elements.append("</body>")
    for side in (-1, 1):
        elements.append(
            geom(
                f"pipe_side_{side}",
                "box",
                (0.098, side * 0.024, 0.103),
                (0.10, 0.004, 0.021),
                ".18 .45 .54 .32",
                extra='contype="2" conaffinity="2"',
            )
        )
    elements.append(
        geom(
            "pipe_lid",
            "box",
            (0.098, 0, 0.126),
            (0.10, 0.024, 0.002),
            ".20 .48 .58 .24",
            extra='contype="2" conaffinity="2"',
        )
    )
    elements.append(
        geom(
            "trap_exit",
            "box",
            (TRAP_EXIT, 0, 0.081),
            (0.0015, 0.023, 0.004),
            ".34 .78 .65 1",
            collision=False,
        )
    )
    elements.append(
        geom(
            "catch_tray",
            "box",
            (0.293, 0, 0.079),
            (0.105, 0.033, 0.005),
            ".17 .30 .34 1",
            collision=False,
        )
    )
    return "".join(assets), "".join(elements)


def make_model(kind, blocked):
    asset, toilet = bowl(kind)
    rear_assets, rear_fittings = seat_and_cistern()
    jet_asset = (
        jet_socket_mesh()
        + '<material name="scale_finish" specular=".06" shininess=".06"/>'
        if kind == "rim"
        else ""
    )
    bonds = (
        "".join(
            f'<weld name="bond_{i}" body1="scale_{i}" solref=".002 1"/>'
            for i in range(len(SCALE_OCCLUSION))
        )
        if kind == "rim"
        else ""
    )
    actor, motors = carriage()
    if kind == "rim":
        tool, fixture_assets, fixture = probe(), "", jet_fixture(blocked)
    else:
        tool = plunger()
        fixture_assets, fixture = trap_fixture()
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
 <asset>{asset}{rear_assets}{jet_asset}{fixture_assets}</asset>
 <worldbody>
  <light pos="-.45 -.35 .8" dir=".45 .3 -.8" diffuse=".82 .82 .82" castshadow="true"/>
  <light pos=".48 .22 .60" dir="-.45 -.3 -.65" diffuse=".42 .46 .51" castshadow="false"/>
  <geom name="desk" type="plane" size="1 1 .1" rgba=".10 .16 .20 1"/>
  {toilet}{rear_fittings}{stand}{actor}{tool}{fixture}
 </worldbody>
 <actuator>{motors}</actuator>
 <equality>{bonds}</equality>
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
        self.rinsing = False
        self.scale_work = np.zeros(len(SCALE_OCCLUSION))
        self.last_edge = None
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

    def trap_positions(self):
        return {p.name: self.data.body(p.name).xpos[0] for p in TRAP_PIECES}

    def trap_cleared(self):
        positions = self.trap_positions()
        return [
            p.name for p in TRAP_PIECES if positions[p.name] - p.size[0] >= TRAP_EXIT
        ]

    def update(self):
        if self.contact({"left_pad", "right_pad"}, {"probe_handle", "plunger_handle"}):
            self.finger_contacts += 1
        if self.kind == "rim":
            self.scrape()
        else:
            piston = self.data.joint("diaphragm")
            self.pressure = 0.0
            if self.contact({"plunger_cup"}, {"diaphragm"}):
                self.tool_contacts += 1
                # Pulse through a filled trap during the actual compression stroke.
                self.pressure = min(8.0, max(0.0, float(piston.qvel[0])) * 71.0)
                self.peak_pressure = max(self.pressure, self.peak_pressure)
            cleared = self.trap_cleared()
            # The pressure pulse acts on each remaining obstruction in proportion
            # to its cross-section. Adjacent pieces can also push each other.
            occupied = [p for p in TRAP_PIECES if p.name not in cleared]
            section = sum(p.size[1] * p.size[2] for p in occupied)
            for p in TRAP_PIECES:
                force = (
                    self.pressure * p.size[1] * p.size[2] / section
                    if p in occupied
                    else 0.0
                )
                self.data.qfrc_applied[self.model.joint(p.name).dofadr[0]] = force

    def scrape(self):
        edge = self.data.geom("scraper_edge").xpos.copy()
        slide = np.zeros(3) if self.last_edge is None else edge - self.last_edge
        self.last_edge = edge
        edge_id = self.model.geom("scraper_edge").id
        touched = False
        for i, contact in enumerate(self.data.contact):
            if edge_id not in (contact.geom1, contact.geom2):
                continue
            other = contact.geom2 if contact.geom1 == edge_id else contact.geom1
            name = self.model.geom(other).name
            if not name.startswith("scale_"):
                continue
            touched = True
            index = int(name.rsplit("_", 1)[1])
            bond = self.model.equality(f"bond_{index}").id
            if not self.data.eq_active[bond]:
                continue
            normal = contact.frame[:3]
            rubbing = np.linalg.norm(slide - np.dot(slide, normal) * normal)
            force = np.zeros(6)
            mj.mj_contactForce(self.model, self.data, i, force)
            self.scale_work[index] += max(0.0, force[0] - 0.08) * rubbing
            if self.scale_work[index] >= SCALE_BREAK_WORK[index]:
                self.data.eq_active[bond] = False
        self.tool_contacts += int(touched)

    def scale_remaining(self):
        mouth = np.array([*JETS[self.blocked], JET_HEIGHT])
        return [
            i
            for i in range(len(SCALE_OCCLUSION))
            if self.data.eq_active[self.model.equality(f"bond_{i}").id]
            or np.linalg.norm(self.data.body(f"scale_{i}").xpos - mouth) < 0.007
        ]

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
                scale_remaining=len(self.scale_remaining()),
                scrape_work_mj=np.round(1000 * self.scale_work, 4).tolist(),
                collected_ml=np.round(self.rinse, 2).tolist(),
                flow_ml_s=(
                    np.round(self.rim_flow(), 2).tolist() if self.rinsing else [0.0] * 4
                ),
            )
        else:
            reading.update(
                clog_x_mm={
                    name: round(1000 * x, 2)
                    for name, x in self.trap_positions().items()
                },
                cleared_pieces=self.trap_cleared(),
                water_percent=round(100 * self.water_level, 2),
            )
        return reading

    def rim_flow(self):
        flows = np.full(4, JET_OPEN_FLOW)
        flows[self.blocked] *= 1.0 - sum(
            SCALE_OCCLUSION[i] for i in self.scale_remaining()
        )
        return flows

    def collect(self, phase):
        self.rinse[:] = 0
        self.rinsing = True
        self.hold(
            RINSE_SECONDS,
            phase,
            lambda: np.add(self.rinse, DT * self.rim_flow(), out=self.rinse),
        )
        self.rinsing = False
        return self.rinse.copy()

    def run_rim(self, control):
        before = self.collect("First rinse · identify the weak jet")
        selected = int(np.argmin(before))
        outlet = np.array(JETS[selected])
        radial = outlet / np.linalg.norm(outlet)
        across = np.array([-radial[1], radial[0]])
        angle = math.atan2(radial[1], radial[0])
        inside = outlet - 0.070 * radial
        engaged = outlet - 0.038 * radial + (0.023 * across if control else 0.0)
        approach_z = JET_HEIGHT + 0.022
        self.pick()
        self.move((*inside, 0.281), 1.05, "Move above the weak outlet", yaw=angle)
        self.move((*inside, approach_z), 0.65, "Lower inside the bowl")
        self.move(
            (*(engaged - 0.008 * across), approach_z), 0.75, "Set blade under the crust"
        )
        for offset, height, depth in (
            (0.008, 0.0, 0.0),
            (-0.008, 0.006, 0.0),
            (0.008, 0.009, 0.0),
            (-0.008, 0.010, -0.002),
            (0.008, 0.008, -0.002),
            (-0.008, 0.005, 0.001),
        ):
            self.move(
                (*(engaged + offset * across + depth * radial), approach_z + height),
                0.68,
                "Scrape limescale",
            )
        self.move((*inside, approach_z), 0.60, "Withdraw the scraper")
        self.move((*inside, 0.280), 0.65, "Clear the bowl")
        self.move((*DOCK[:2], 0.285), 0.80, "Park the probe clear of the rinse")
        after = self.collect("Second rinse · verify each jet")
        reference = JET_OPEN_FLOW * RINSE_SECONDS
        passed = (
            self.tool_contacts > 0
            and not self.scale_remaining()
            and np.all((after >= 0.8 * reference) & (after <= 1.2 * reference))
        )
        self.hold(0.65, "Flow restored" if passed else "Outlet remains restricted")
        return {
            "score": 1.0 if passed else 0.25,
            "before_ml": before.tolist(),
            "after_ml": after.tolist(),
            "diagnosed": NAMES[selected],
            "scale_chips_removed": len(SCALE_OCCLUSION) - len(self.scale_remaining()),
            "scrape_work_mj": np.round(1000 * self.scale_work, 4).tolist(),
        }

    def drain(self, phase):
        self.water_level = 1.0

        def update_level():
            open_fraction = (len(self.trap_cleared()) / len(TRAP_PIECES)) ** 2
            self.water_level = max(
                0.0, self.water_level - DT * (0.04 + 0.75 * open_fraction)
            )

        self.hold(1.25, phase, update_level)
        return self.water_level

    def run_trap(self, control, strokes=TRAP_STROKES):
        before = self.drain("Test drainage before repair")
        # The cup hangs below the gripper and must pass above the seat ring.
        carry_height = 0.325
        self.pick(carry_height)
        x = 0.050 if control else 0.0
        self.move((x, 0, carry_height), 0.80, "Centre plunger over the drain")
        self.move((x, 0, 0.207), 0.60, "Seat cup")
        for i in range(strokes):
            self.move((x, 0, 0.177), 0.24, f"Compression stroke {i+1} of {strokes}")
            self.move((x, 0, 0.204), 0.33, "Release stroke")
        self.move((x, 0, carry_height), 0.70, "Lift cup clear")
        after = self.drain("Retest drainage")
        passed = (
            before >= 0.70
            and len(self.trap_cleared()) == len(TRAP_PIECES)
            and after < 0.15
            and self.tool_contacts > 0
        )
        self.hold(0.65, "Drainage restored" if passed else "Trapway remains restricted")
        return {
            "score": float(passed),
            "before_water_percent": 100 * before,
            "after_water_percent": 100 * after,
            "cleared_pieces": self.trap_cleared(),
            "final_x_mm": {
                name: round(1000 * x, 2) for name, x in self.trap_positions().items()
            },
            "strokes": strokes,
            "peak_pressure_force_n": self.peak_pressure,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=("rim", "trap"))
    parser.add_argument("--blocked", type=int, choices=range(4), default=1)
    parser.add_argument("--control", action="store_true")
    parser.add_argument(
        "--strokes", type=int, choices=range(1, 16), default=TRAP_STROKES
    )
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--video", action="store_true")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    recording = None
    if args.video:
        from recording import Recording

        recording = Recording(args.out / "demo.mp4")
    t = Trial(args.kind, args.blocked, recording)
    result = (
        t.run_rim(args.control)
        if args.kind == "rim"
        else t.run_trap(args.control, args.strokes)
    )
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
