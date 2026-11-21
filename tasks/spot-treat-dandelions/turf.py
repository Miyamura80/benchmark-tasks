"""Build the turf, dandelion geometry and assessor's overhead canopy map."""

import math
import numpy as np
from PIL import Image, ImageDraw

W, H = 300, 200
CX, CY, Z = 0.060, -0.085, 0.190
CENTERS = np.array([(61, 70), (150, 139), (232, 66)])
SCALE = 3


def world(u, v, height=0.0):
    return np.array([CX + (u - W / 2) / 1000, CY + (H / 2 - v) / 1000, Z + height])


def _surface_map(meshes):
    """Rasterize the highest triangle at the centre of each 1 mm assessor pixel."""
    height = np.zeros((H, W), np.float32)
    labels = np.zeros((H, W), np.uint8)
    for plant, vertices, triangles in meshes:
        vs = np.asarray(vertices)
        for ids in triangles:
            tri = vs[list(ids)]
            xmin, ymin = np.floor(tri[:, :2].min(axis=0)).astype(int)
            xmax, ymax = np.ceil(tri[:, :2].max(axis=0)).astype(int)
            if xmin >= xmax or ymin >= ymax:
                continue
            a, b, c = tri
            e1, e2 = b - a, c - a
            det = e1[0] * e2[1] - e2[0] * e1[1]
            if abs(det) < 1e-8:
                continue
            yy, xx = np.mgrid[ymin:ymax, xmin:xmax] + 0.5
            x, y = xx - a[0], yy - a[1]
            l = (x * e2[1] - y * e2[0]) / det
            m = (y * e1[0] - x * e1[1]) / det
            z = a[2] + l * e1[2] + m * e2[2]
            region = height[ymin:ymax, xmin:xmax]
            visible = (l >= 0) & (m >= 0) & (l + m <= 1) & (z > region)
            region[visible] = z[visible]
            labels[ymin:ymax, xmin:xmax][visible] = plant
    return np.ceil(height).astype(np.uint8), labels


def layout(seed=0):
    rng = np.random.default_rng(seed)
    centers = CENTERS + (rng.integers(-6, 7, (3, 2)) if seed else 0)
    size = (W * SCALE, H * SCALE)
    grass = Image.new("RGB", size, (43, 69, 33))
    draw = ImageDraw.Draw(grass)
    for _ in range(36000):
        x, y = rng.uniform((0, 0), size)
        hue = rng.uniform(0.62, 1.45)
        dx, dy = rng.uniform((-6, -17), (8, -3))
        draw.line(
            (x, y, x + dx, y + dy),
            fill=tuple(int(v * hue) for v in (57, 91, 39)),
            width=int(rng.integers(1, 4)),
        )
    overhead = grass.copy()
    draw = ImageDraw.Draw(overhead)
    meshes = []

    def pixels(points):
        return [(round(p[0] * SCALE), round(p[1] * SCALE)) for p in points]

    def paint(plant, vertices, faces, edge, color):
        meshes.append((plant, vertices, faces))
        draw.polygon(pixels(edge), fill=color)

    for plant, (cx, cy) in enumerate(centers, 1):
        c = np.array((cx, cy))
        phase = rng.uniform(0, 2 * math.pi)
        for k in range(10 + plant):
            angle = phase + k * 2.39996 + rng.uniform(-0.23, 0.23)
            axis = np.array([math.cos(angle), math.sin(angle)])
            normal = np.array([-axis[1], axis[0]])
            length = rng.uniform(23, 33) * (1.0 if k < 8 else 0.76)
            wid = rng.uniform(4.1, 5.7)
            bend = rng.uniform(-0.17, 0.17) * length
            arch = rng.uniform(3.0, 6.0) if k < 8 else rng.uniform(6.0, 11.0)
            lobes = int(rng.integers(3, 5))
            # Tips point back toward the crown. Paired lobes and one broad terminal lobe.
            rows = [(0.0, 0.05, 0.0), (0.17, 0.09, 0.0)]
            for t in np.linspace(0.23, 0.68, lobes):
                rows.extend(((t, 0.29, 0.0), (t + 0.048, 0.70 + 0.23 * t, 0.105)))
            rows.extend(
                (
                    (0.78, 0.30, 0.0),
                    (0.84, 0.77, 0.065),
                    (0.93, 0.42, 0.0),
                    (1.0, 0.022, 0.0),
                )
            )
            asym = rng.uniform(0.77, 1.22, 2)
            verts = []

            def rib(t):
                return c + axis * (length * t) + normal * bend * t**1.7

            for t, f, back in rows:
                pt = rib(t)
                z = 1.25 + arch * math.sin(t * math.pi * 0.91) + 1.3 * t
                for j in (-1, 0, 1):
                    p = (
                        pt
                        + normal * j * wid * f * asym[int(j > 0)]
                        - axis * (abs(j) * back * length)
                    )
                    verts.append((*p, z + (1 - abs(j)) * (0.4 + f * 0.75)))
            fs = []
            for r in range(len(rows) - 1):
                for side in range(2):
                    i = 3 * r + side
                    fs.extend(((i, i + 1, i + 3), (i + 1, i + 4, i + 3)))
            contour = verts[0::3] + list(reversed(verts[2::3]))
            shade = rng.uniform(0.79, 1.14) + (k > 7) * 0.09
            color = tuple(min(255, round(v * shade)) for v in (72, 125, 45))
            paint(plant, verts, fs, contour, color)
            draw.line(
                pixels([rib(t) for t in np.linspace(0.02, 0.92, 12)]),
                fill=tuple(round(v * shade) for v in (131, 155, 73)),
                width=2,
            )
            for ix, (t, w, back) in enumerate(rows):
                if back:
                    for side in (0, 2):
                        draw.line(
                            pixels([rib(max(0.05, t - 0.09)), verts[3 * ix + side]]),
                            fill=tuple(round(v * shade) for v in (94, 139, 56)),
                            width=1,
                        )
        # Short flower stalks represent recently mown lawn plants. The green bud shares
        # the same rosette outline and coverage gate as the two open heads.
        angle = phase + 0.4
        stem_axis = np.array((math.cos(angle), math.sin(angle)))
        stem_perp = np.array((-stem_axis[1], stem_axis[0]))
        reach, altitude = (
            (11.0, 19.0) if plant == 1 else ((8.0, 14.0) if plant == 2 else (6.5, 12.0))
        )
        tip = c + stem_axis * reach
        verts = [
            (*(c - stem_perp * 0.85), 1.6),
            (*(c + stem_perp * 0.85), 1.6),
            (*(tip + stem_perp * 0.72), altitude),
            (*(tip - stem_perp * 0.72), altitude),
        ]
        paint(plant, verts, [(0, 1, 2), (0, 2, 3)], verts, (134, 151, 59))
        # Bracts cup the base of the open heads and enclose the tapered green bud.
        n = 18
        rad = (8.8 if plant == 1 else 7.1) if plant < 3 else 3.2
        verts = [(*tip, altitude - 2.2)]
        for j in range(n):
            a = 2 * math.pi * j / n
            q = tip + rad * (0.53 if plant < 3 else 1) * np.array(
                [math.cos(a), math.sin(a)]
            )
            verts.append((*q, altitude + 1.4 * (j % 2)))
        apex = tip + (0.4 if plant < 3 else 2.2) * stem_axis
        verts.append((*apex, altitude + (1.8 if plant < 3 else 5.7)))
        fs = []
        for j in range(n):
            i = 1 + j
            nidx = 1 + (j + 1) % n
            fs.extend(((0, i, nidx), (i, nidx, n + 1)))
        paint(plant, verts, fs, verts[1:-1], (101, 132, 43))
        if plant == 3:
            for v in verts[1:-1:2]:
                draw.line(pixels([v, verts[-1]]), fill=(145, 157, 55), width=1)
            continue
        # Several concentric whorls of individually tapered ray florets.
        for layer, (length, number, rise) in enumerate(
            (
                (1.0, 62, 1.0),
                (0.83, 52, 1.8),
                (0.63, 44, 2.4),
                (0.44, 32, 2.8),
                (0.26, 22, 3.0),
            )
        ):
            for j in range(number):
                a = (
                    phase
                    + 2
                    * math.pi
                    * (j + 0.4 * layer + rng.uniform(-0.24, 0.24))
                    / number
                )
                d = np.array([math.cos(a), math.sin(a)])
                nrm = np.array([-d[1], d[0]])
                r = rad * length * rng.uniform(0.90, 1.07)
                base = tip + d * (r * 0.23)
                end = tip + d * r
                w = rng.uniform(0.30, 0.62) * (1 - 0.09 * layer)
                h = altitude + rise
                shape = [
                    (*(base - nrm * 0.16), h),
                    (*(end - d * 0.5 - nrm * w), h + 1.2),
                    (*end, h + 1.35),
                    (*(end - d * 0.45 + nrm * w), h + 1.4),
                    (*(base + nrm * 0.16), h),
                ]
                col = (
                    int(rng.integers(233, 255)),
                    int(rng.integers(178, 221) - layer * 4),
                    int(rng.integers(13, 44)),
                )
                paint(plant, shape, [(0, 1, 2), (0, 2, 3), (0, 3, 4)], shape, col)
    heights, labels = _surface_map(meshes)
    return overhead, labels, centers, [(v, f) for _, v, f in meshes], heights, grass


def uv_pair(wet):
    before = np.full((*wet.shape, 3), (21, 12, 50), np.uint8)
    after = before.copy()
    after[wet] = (45, 190, 60)
    return before, after
