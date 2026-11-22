"""Clear cowl leaves on a full-car scene with two articulated I2RT YAMs."""
from pathlib import Path
import argparse
import json
import os
import subprocess
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco as mj
import numpy as np
from PIL import Image

from car import CAR_LIFT, car_parts
from dual_yam import ARM_PREFIXES, BASE_X, BASE_Y, BASE_Z, Arm, append_yams

LEAVES = (
    (-0.34, 0.126, 0.92, "0.55 0.26 0.076 1"),
    (-0.135, 0.083, 1.02, "0.47 0.18 0.044 1"),
    (0.075, 0.104, 0.80, "0.61 0.40 0.14 1"),
    (0.302, 0.124, 1.12, "0.39 0.21 0.064 1"),
)
DRAIN_X = (-0.34, 0.302)
GRILLE_SLATS = (-.027, -.015, -.003, .009, .021)
SLAT_HALF_WIDTH = .0027
DRAIN_GAPS = tuple(zip(
    [-.035] + [x + SLAT_HALF_WIDTH for x in GRILLE_SLATS],
    [x - SLAT_HALF_WIDTH for x in GRILLE_SLATS] + [.035],
))
DRAIN_Y = (.124, .153)
DRAIN_TOP = .135
DRAIN_FLOOR = .094
BLOCKED_FRACTION = .05
DOWN = np.array([0., 0., -1.])
TRAY_X = 0
TRAY_Y = -.246
TRAY_Z = .155
HEIGHT = np.array([0, 0, CAR_LIFT])
DT = 0.002


def geom(kind, pos, size, rgba, attrs=""):
    return f'<geom type="{kind}" pos="{pos}" size="{size}" rgba="{rgba}" {attrs}/>'


def box(pos, size, rgba, attrs=""):
    return geom("box", pos, size, rgba, attrs)


def visual_box(pos, size, rgba, attrs=""):
    return box(pos, size, rgba, f'contype="0" conaffinity="0" {attrs}')


def surface_mesh(name, columns, rows, surface, thickness):
    """Create a closed thin shell by sampling a gently crowned surface."""
    grid = [surface(x, y) for y in np.linspace(0, 1, rows)
            for x in np.linspace(-1, 1, columns)]
    vertices = grid + [(x, y, z - thickness) for x, y, z in grid]
    count = len(grid)
    faces = []
    for r in range(rows - 1):
        for c in range(columns - 1):
            a = r * columns + c
            b, d = a + 1, a + columns
            e = d + 1
            faces.extend([(a, b, d), (b, e, d),
                          (a + count, d + count, b + count),
                          (b + count, d + count, e + count)])
    outline = (list(range(columns)) +
               [r * columns + columns - 1 for r in range(1, rows)] +
               list(range(count - 2, count - columns - 1, -1)) +
               [r * columns for r in range(rows - 2, 0, -1)])
    for a, b in zip(outline, outline[1:] + outline[:1]):
        faces.extend([(a, a + count, b), (b, a + count, b + count)])
    verts = " ".join(f"{v:.5f}" for vertex in vertices for v in vertex)
    triangles = " ".join(str(v) for face in faces for v in face)
    return f'<mesh name="{name}" vertex="{verts}" face="{triangles}"/>'


def stroke(a, b, width, color, *, flat=False):
    points = " ".join(f"{v:.5f}" for v in (*a, *b))
    shape = "cylinder" if flat else "capsule"
    return (f'<geom type="{shape}" fromto="{points}" size="{width}" rgba="{color}" '
            'contype="0" conaffinity="0"/>')


def bodywork():
    paint = "0.18 0.33 0.40 1"
    glass = "0.19 0.32 0.36 1"
    trim = "0.069 0.08 0.081 1"

    def hood(x, y):
        return (.44 * x, -.363 + .359 * y,
                .125 + .015 * (1 - x*x) - .018 * (1 - y)**2)

    def windscreen(x, y):
        return ((.60 - .086 * y) * x, .163 + .649 * y + .010*x*x,
                .132 + .453 * y - .017*x*x)

    assets = [surface_mesh("hood_skin", 33, 21, hood, .008),
              surface_mesh("screen_skin", 29, 21, windscreen, .005)]
    visuals = [
        f'<geom type="mesh" mesh="hood_skin" rgba="{paint}" material="paint" contype="0" conaffinity="0"/>',
        f'<geom type="mesh" mesh="screen_skin" rgba="{glass}" material="glass" contype="0" conaffinity="0"/>',
    ]
    for x in (-1, 1):
        for i in range(12):
            a, b = windscreen(x, i / 12), windscreen(x, (i + 1) / 12)
            visuals.append(stroke(a, b, .016, paint))
            a2 = (a[0] * .981, a[1] - .002, a[2] + .002)
            b2 = (b[0] * .981, b[1] - .002, b[2] + .002)
            visuals.append(stroke(a2, b2, .005, trim))
        for i in range(20):
            a = hood(.81 * x, i / 20)
            b = hood(.81 * x, (i + 1) / 20)
            a = (a[0], a[1], a[2] + .001)
            b = (b[0], b[1], b[2] + .001)
            visuals.append(stroke(a, b, .0025, ".26 .40 .46 1"))
    for i in range(28):
        for y, radius, color in ((1, .013, paint), (0, .006, trim)):
            visuals.append(stroke(windscreen(-1 + i / 14, y),
                                  windscreen(-1 + (i + 1) / 14, y), radius, color))

    # Wiper arms and low-profile rubber blades sit on the glass at their park position.
    for root, tip, blade in (
        ((-.333, .172, .133), (-.019, .207, .181), (-.031, .209, .177)),
        ((.111, .174, .139), (.351, .196, .164), (.356, .194, .160)),
    ):
        visuals.append(geom("cylinder", f"{root[0]} {root[1]} {root[2]}", ".011 .006", trim,
                            'contype="0" conaffinity="0"'))
        elbow = (root[0] + .095, root[1] + .015, root[2] + .026)
        visuals.append(stroke(root, elbow, .0045, trim, flat=True))
        visuals.append(stroke(elbow, tip, .003, trim, flat=True))
        visuals.append(stroke((blade[0] - .13, blade[1] + .008, blade[2] - .004),
                              (blade[0] + .10, blade[1] - .005, blade[2] - .005),
                              .0034, "0.038 0.042 0.043 1", flat=True))
    return assets, visuals


def leaf_mesh(index, scale):
    # Two surfaces share an irregular tapered outline and a raised midrib.
    outline = [
        (0, -.031), (.005, -.022), (.016, -.013), (.018, -.003),
        (.014, .004), (.018, .013), (.010, .017), (.007, .023),
        (0, .034), (-.006, .024), (-.013, .019), (-.012, .012),
        (-.018, .008), (-.016, -.004), (-.012, -.014), (-.003, -.021),
    ]
    vertices = []
    for i, (x, y) in enumerate(outline):
        ridge = -.001 + abs(y) * (.038 + .010 * index)
        curl = .0014 * np.sin(i * 1.32 + index * .8)
        vertices.append((x * scale, y * scale, ridge + curl))
    vertices.extend([(0, 0, .004), (0, 0, -.002)])
    n = len(outline)
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces.extend([(n, i, j), (n + 1, j, i)])
    verts = " ".join(f"{v:.5f}" for vertex in vertices for v in vertex)
    triangles = " ".join(str(v) for face in faces for v in face)
    return f'<mesh name="leaf{index}" vertex="{verts}" face="{triangles}"/>'


def vein(a, b, width, color):
    points = " ".join(f"{v:.5f}" for v in (*a, *b))
    return (f'<geom type="capsule" fromto="{points}" size="{width}" rgba="{color}" '
            'contype="0" conaffinity="0" mass="0"/>')


def bench_supports():
    """Direct supports at each arm base and an independent frame below the cowl."""
    steel = ".28 .33 .34 1"
    parts = [box("0 .035 .680", ".410 .370 .010", ".48 .37 .25 1", 'name="bench_deck"')]
    for x in (-.38, .38):
        for y in (-.30, .35):
            parts.append(box(f"{x} {y} .335", ".019 .019 .335", steel, 'name="bench_leg_%s_%s"' % (x,y)))
    for side in (-1, 1):
        x = side * BASE_X
        for name,z,half in (("foot",.012,".105 .105 .012"), ("column",.430,".045 .045 .406"),
                            ("plate",.854,".065 .065 .018")):
            parts.append(box(f"{x} {BASE_Y} {z}", half, steel, f'name="bench_{name}_{side}"'))
    # Spacers support the model cowl and tray without filling either drain.
    for x in (-.29,.27):
        parts.append(box(f"{x} .055 .719", ".025 .060 .029", steel, f'name="bench_riser_{x}"'))
    for x in (-.08,.08):
        parts.append(box(f"{x} -.246 .7575", ".018 .065 .0675", steel, f'name="bench_tray_{x}"'))
    return parts


def make_scene(bench=False):
    steel = "0.105 0.125 0.133 1"
    trim = "0.081 0.087 0.086 1"
    paint = "0.16 0.25 0.30 1"
    black = "0.075 0.084 0.088 1"
    car_assets, car_vis = ([], bench_supports()) if bench else car_parts()
    assets, panels = ([], []) if bench else bodywork()
    if bench:
        # Show the local hood edge and angled backing on the removable insert.
        panels.append(visual_box("0 .256 .194", ".443 .130 .011", ".18 .31 .36 1", 'euler="43 0 0"'))
        panels.append(visual_box("0 -.064 .113", ".380 .063 .008", paint))
    assets.extend(car_assets)
    assets.extend(leaf_mesh(i, leaf[2]) for i, leaf in enumerate(LEAVES))
    parts = [
        *panels,
        # The drain slots are open through the floor, with narrow ribs between them.
        *([box("0 .047 .080", ".45 .077 .02", steel, 'name="cowl_floor"'),
           box("0 -.0975 .080", ".390 .0675 .02", steel, 'name="bench_front_floor"')]
          if bench else [box("0 -.0205 .080", ".45 .1445 .02", steel, 'name="cowl_floor"')]),
        box("0 -.064 .113", ".380 .063 .008" if bench else ".445 .063 .008", paint, 'name="hood_edge" group="3"'),
        box("0 .162 .101", ".45 .011 .028", trim, 'name="glass_seal"'),
        box("0 .256 .194", ".443 .130 .011", trim,
            'euler="43 0 0" name="windscreen" group="3"'),
        box("-.445 .100 .111", ".009 .066 .023", trim),
        box(".445 .100 .111", ".009 .066 .023", trim),
        box("0 .153 .083", ".446 .007 .016", steel),
        *([] if bench else [box("-.436 -.21 .062", ".017 .145 .054", paint),
                            box(".436 -.21 .062", ".017 .145 .054", paint)]),
        box(f"{TRAY_X} {TRAY_Y} {TRAY_Z - .010}", ".125 .103 .010", "0.28 0.34 0.33 1", 'name="tray_floor"'),
    ]
    blind_sections = [(-.45, DRAIN_X[0] - .035),
                      (DRAIN_X[0] + .035, DRAIN_X[1] - .035),
                      (DRAIN_X[1] + .035, .45)]
    for left, right in blind_sections:
        center = (left + right) / 2
        width = (right - left) / 2
        parts.append(box(f"{center} .1395 .080", f"{width} .0155 .02", steel))
    for x in DRAIN_X:
        parts.append(visual_box(f"{x} .139 .064", ".036 .027 .003", ".025 .03 .03 1"))
        # The shallow lip above the grate clips a leaf tip, requiring a withdrawal
        # towards the hood before lifting. Grate slats give a visible clear state.
        for offset in GRILLE_SLATS:
            parts.append(box(f"{x + offset} .141 .103", f"{SLAT_HALF_WIDTH} .020 .003", "0.15 0.17 0.17 1"))
        parts.append(box(f"{x} .150 .120", ".033 .007 .004", black, 'name="drain_lip_' + ("left" if x < 0 else "right") + '"'))
    for x in (TRAY_X - .125, TRAY_X + .125):
        parts.append(box(f"{x} {TRAY_Y} {TRAY_Z + .011}", ".006 .106 .016", "0.23 0.30 0.29 1"))
    for y in (TRAY_Y-.103, TRAY_Y+.103):
        parts.append(box(f"{TRAY_X} {y} {TRAY_Z + .011}", ".127 .006 .016", "0.23 0.30 0.29 1"))
    for i, (x, y, scale, color) in enumerate(LEAVES):
        collision = geom("ellipsoid", "0 0 0", f"{.009*scale} {.027*scale} .0025", "0 0 0 0",
                         'friction="1.5 .01 .0002" mass=".0012"')
        contact_patch = box("0 0 0", f"{.014*scale} {.015*scale} .0033", "0 0 0 0",
                            'friction="1.5 .01 .0002" mass=".0003"')
        mesh = f'<geom name="leaf_surface{i}" type="mesh" mesh="leaf{i}" rgba="{color}" contype="0" conaffinity="0" mass="0"/>'
        midrib = vein((0, -.029 * scale, .0019), (0, .027 * scale, .0041), .00085, "0.69 0.48 0.24 1")
        stem = vein((0, -.038 * scale, .0028), (0, -.026 * scale, .0017),
                    .00085, "0.42 0.29 0.17 1")
        veins = [
            vein((0, yy * scale, .0033), (xx * scale, (yy + .007) * scale, .0021), .00041, "0.60 0.39 0.16 1")
            for yy in (-.014, -.001, .011) for xx in (-.011, .011)
        ]
        parts.append(f'<body name="leaf{i}" pos="{x} {y} .109"><freejoint/>{collision}{contact_patch}{mesh}{midrib}{stem}{"".join(veins)}</body>')
    for side in (() if bench else (-1,1)):
        post_x = side*.920
        car_vis.extend((
            visual_box(f"{post_x} {BASE_Y} .032", ".128 .122 .030", ".24 .26 .26 1"),
            visual_box(f"{post_x} {BASE_Y} .429", ".046 .048 .401", ".33 .36 .36 1"),
            visual_box(f"{side*.693} {BASE_Y} .827", ".257 .047 .028", ".22 .27 .27 1"),
            visual_box(f"{side*BASE_X} {BASE_Y} .854", ".073 .067 .016", ".29 .32 .33 1"),
        ))
    xml = f'''
<mujoco model="Windshield cowl">
  <compiler angle="degree"/>
  <option timestep="{DT}" integrator="implicitfast" iterations="100" cone="elliptic"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <headlight ambient=".27 .27 .27" diffuse=".38 .38 .38" specular=".18 .18 .18"/>
    <quality shadowsize="2048"/>
    <map shadowclip="2.4" znear=".011"/>
  </visual>
  <asset>
    <texture name="floor" type="2d" builtin="checker" rgb1=".36 .37 .36" rgb2=".33 .34 .33" width="64" height="64"/>
    <material name="glass" specular=".83" shininess=".78" rgba=".43 .54 .57 1"/>
    <material name="paint" specular=".47" shininess=".74" rgba=".43 .54 .57 1"/>
    <material name="workspace" texture="floor" texrepeat="6 6" shininess=".12"/>
    {"".join(assets)}
  </asset>
  <worldbody>
    <light pos="-2 -2 4" dir=".2 .3 -1" diffuse=".80 .77 .72" castshadow="true"/>
    <light pos="2 .6 3" dir="-1 -.6 -.7" diffuse=".55 .56 .62" castshadow="false"/>
    <light pos="0 2 4" dir="0 -.2 -1" diffuse=".28 .28 .26" castshadow="false"/>
    <geom type="plane" size="8 8 .1" material="workspace"/>
    {"".join(car_vis)}
    <frame pos="0 0 {CAR_LIFT}">
      {"".join(parts)}
    </frame>
  </worldbody>
</mujoco>
'''
    scene = ET.fromstring(xml)
    append_yams(scene)
    return ET.tostring(scene, encoding='unicode')


def grille_coverage(model, data, center_x, leaf_geoms):
    """Estimate leaf coverage of the open slots from weighted vertical rays."""
    covered_width = 0.0
    total_width = 0.0
    for left, right in DRAIN_GAPS:
        width = right - left
        hits = 0
        for x_fraction in (.125, .375, .625, .875):
            for y_fraction in (.1, .3, .5, .7, .9):
                point = np.array([
                    center_x + left + x_fraction * width,
                    DRAIN_Y[0] + y_fraction * (DRAIN_Y[1] - DRAIN_Y[0]),
                    CAR_LIFT + DRAIN_TOP,
                ])
                hits += any(
                    0 <= mj.mj_rayMesh(model, data, geom_id, point, DOWN)
                    <= DRAIN_TOP - DRAIN_FLOOR
                    for geom_id in leaf_geoms
                )
        covered_width += width * hits / 20
        total_width += width
    return covered_width / total_width


def score(model, data):
    positions = np.array([data.body(f"leaf{i}").xpos for i in range(len(LEAVES))]) - HEIGHT
    inside = ((abs(positions[:, 0] - TRAY_X) <= .115) &
              (abs(positions[:, 1] - TRAY_Y) <= .088) &
              (positions[:, 2] >= TRAY_Z - .002) & (positions[:, 2] <= TRAY_Z + .027))
    leaf_geoms = [model.geom(f"leaf_surface{i}").id for i in range(len(LEAVES))]
    coverage = [grille_coverage(model, data, x, leaf_geoms) for x in DRAIN_X]
    captured = int(np.count_nonzero(inside))
    drains_clear = sum(fraction < BLOCKED_FRACTION for fraction in coverage)
    return {
        "leaves_in_tray": captured,
        "drains_clear": drains_clear,
        "drain_leaf_coverage": [round(fraction, 4) for fraction in coverage],
        "score": (3 * captured + 2 * drains_clear) / 16,
    }


def run(output, *, show_trace=False, bench=False):
    model = mj.MjModel.from_xml_string(make_scene(bench=bench))
    data = mj.MjData(model)
    visibility = mj.MjvOption()
    for group in (3, 4, 5):
        visibility.geomgroup[group] = 0
    visibility.sitegroup[4] = 0
    arms = [Arm(model, prefix, side) for prefix, side in zip(ARM_PREFIXES, (-1, 1))]
    rest = [np.array([-.333, -.114, .268]), np.array([.333, -.114, .268])]
    xyz = [p.copy() for p in rest]
    openings = [.030, .030]
    for i, arm in enumerate(arms):
        error = arm.initialize(data, xyz[i] + HEIGHT, openings[i])
        if error > .003:
            raise RuntimeError(f"{arm.prefix} arm start pose error: {error:.4f} m")
    mj.mj_forward(model, data)

    camera = mj.MjvCamera()
    camera.type = mj.mjtCamera.mjCAMERA_FREE
    renderer = ffmpeg = None
    if output:
        output.mkdir(parents=True, exist_ok=True)
        renderer = mj.Renderer(model, height=540, width=960)
        ffmpeg = subprocess.Popen([
            "ffmpeg", "-y", "-v", "error", "-f", "rawvideo", "-pixel_format", "rgb24",
            "-video_size", "960x540", "-framerate", "25", "-i", "-", "-c:v", "libx264",
            "-preset", "fast", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            str(output / "windshield-cowl.mp4"),
        ], stdin=subprocess.PIPE)

    worst_ik = 0.
    support_ids = {i for i in range(model.ngeom) if (model.geom(i).name or "").startswith("bench_")}
    moving_arm = {i for i in range(model.ngeom)
                  if model.body_weldid[model.geom_bodyid[i]] != 0
                  and model.body(model.geom_bodyid[i]).name.startswith(ARM_PREFIXES)}
    support_contacts = 0

    def move(which, target, opening, duration, *, view="work", focus=0):
        nonlocal worst_ik, support_contacts
        arm = arms[which]
        start = xyz[which]
        target = np.array(target)
        steps = round(duration / DT)
        if view == "car":
            camera.lookat[:] = [0, .36, .72]
            camera.distance, camera.azimuth, camera.elevation = 4.62, 141, -29
            if bench:
                camera.lookat[:] = [0, 0, .65]
                camera.distance, camera.azimuth, camera.elevation = 2.45, 112, -23
        elif view == "drains":
            camera.lookat[:] = [-.017, .118, .800]
            camera.distance, camera.azimuth, camera.elevation = 1.09, 90, -29
        elif view == "pinch":
            camera.lookat[:] = [focus, .101, .84]
            camera.distance, camera.azimuth, camera.elevation = .71, 94, -22
        else:
            camera.lookat[:] = [0, -.049, .87]
            camera.distance, camera.azimuth, camera.elevation = 1.70, 109, -41
        for step in range(1, steps+1):
            if step % 5 == 0 or step == steps:
                t = step / steps
                smooth = t*t*(3-2*t)
                center = start + smooth*(target-start) + HEIGHT
                aperture = openings[which] + smooth*(opening-openings[which])
                error = arm.solve(center)
                worst_ik = max(worst_ik, error)
                arm.command(data, aperture)
            mj.mj_step(model, data)
            if bench:
                for contact in data.contact:
                    a, b = contact.geom
                    if contact.dist <= 0 and ((a in support_ids and b in moving_arm)
                                              or (b in support_ids and a in moving_arm)):
                        support_contacts += 1
            if renderer and round(data.time / DT) % 20 == 0:
                renderer.update_scene(data, camera=camera, scene_option=visibility)
                ffmpeg.stdin.write(renderer.render().tobytes())
        xyz[which], openings[which] = target, opening
        if show_trace:
            current = np.array(data.site_xpos[arm.site]) + data.site_xmat[arm.site].reshape(3,3) @ [.044,0,-.002]
            observed = [data.body(f"leaf{i}").xpos.round(4).tolist() for i in range(len(LEAVES))]
            print(round(data.time, 2), arm.prefix, target.tolist(), opening, float(np.linalg.norm(current-target-HEIGHT)), observed)

    move(0, rest[0], .030, 1.32, view="car")
    initial = score(model, data)
    if renderer:
        Image.fromarray(renderer.render()).save(output / "setup.png")
    move(0, rest[0], .030, 1.10, view="drains")
    if renderer:
        Image.fromarray(renderer.render()).save(output / "drains-before.png")
    for i, (_, _, scale, _) in enumerate(LEAVES):
        which = 0 if i < 2 else 1
        slide = .027 if i in (0,3) else 0.
        close = .014 * scale - .0015
        x, y, z = data.body(f"leaf{i}").xpos.copy() - HEIGHT
        pinch_z = max(.1045, z + .0005)
        if slide:
            y = min(y - .011, .112)
        drop_x = [-.075,-.031,.032,.073][i]
        drop_y = TRAY_Y + (.031 if i%2 else -.027)
        move(which,[x, y, .229], .034, .64)
        move(which,[x, y, pinch_z], .034, .76, view="pinch", focus=x)
        move(which,[x, y, pinch_z], close, .45, view="pinch", focus=x)
        if slide:
            move(which,[x, y-slide, pinch_z], close, .50, view="pinch", focus=x)
        move(which,[x, y-slide, .236], close, .52, view="pinch", focus=x)
        move(which,[drop_x, drop_y, .243], close, 1.10)
        move(which,[drop_x, drop_y, .217], close, .40)
        move(which,[drop_x, drop_y, .217], .035, .46)
        move(which,[drop_x, drop_y, .270], .035, .33)
        if i in (1,3):
            move(which,rest[which], .030, .83)
    move(0, rest[0], .030, 1.0)
    if renderer:
        Image.fromarray(renderer.render()).save(output / "result.png")
    move(0, rest[0], .030, 1.60, view="drains")
    if renderer:
        Image.fromarray(renderer.render()).save(output / "drains-after.png")
    move(0, rest[0], .030, 1.00, view="car")
    final = score(model, data)
    result = {
        "initial": initial,
        "final": final,
        "seconds": round(data.time, 2),
        "max_weighted_ik_residual": round(float(worst_ik), 6),
        "embodiment": "Two 6-axis I2RT YAM models with linear grippers",
    }
    if bench:
        result["layout"] = "bench"
        result["moving_arm_support_contacts"] = support_contacts
    if renderer:
        renderer.close()
        ffmpeg.stdin.close()
        if ffmpeg.wait() != 0:
            raise RuntimeError("ffmpeg failed to encode the recording")
        (output / "score.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, help="Export a demo MP4 and setup/result screenshots")
    parser.add_argument("--trace", action="store_true", help="Print end-effector and leaf positions at each waypoint")
    parser.add_argument("--bench", action="store_true", help="Use the freestanding cowl insert and direct supports")
    args = parser.parse_args()
    result = run(args.output, show_trace=args.trace, bench=args.bench)
    print(json.dumps(result))
    if result["initial"]["score"] != 0 or result["final"]["score"] != 1 or result.get("moving_arm_support_contacts", 0):
        raise SystemExit(1)
