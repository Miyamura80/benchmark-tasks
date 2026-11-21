"""YAM workbench with a bracket-mounted downward spot sprayer and replaceable turf."""

import io
import xml.etree.ElementTree as ET
import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw
from turf import W, H, CX, CY, Z, layout, uv_pair
from verify import score
from yam_station import station_xml, geom, element


class Lawn:
    def __init__(self, seed=0):
        (
            self.texture,
            self.labels,
            self.centers,
            self.leaves,
            self.heights,
            self.grass,
        ) = layout(seed)
        self.seed = seed

    def compile(self):
        tree, scene = station_xml()
        # Map the exact ground coordinates to explicit UVs, avoiding cube texture wrapping.
        asset = tree.find("asset")
        element(asset, "texture", name="grass_photo", type="2d", file="grass.png")
        element(
            asset,
            "material",
            name="grass_material",
            texture="grass_photo",
            reflectance="0",
            specular=".05",
        )
        element(
            asset,
            "mesh",
            name="grass_mesh",
            vertex="-.15 -.10 0 .15 -.10 0 .15 .10 0 -.15 .10 0 0 0 -.002",
            face="0 1 2 0 2 3 0 4 1 1 4 2 2 4 3 3 4 0",
            texcoord="0 1 1 1 1 0 0 0 .5 .5",
        )
        element(asset, "texture", name="rosette_photo", type="2d", file="plants.png")
        element(
            asset,
            "material",
            name="rosette_material",
            texture="rosette_photo",
            specular=".08",
        )
        geom(
            scene,
            "table",
            "box",
            (0, 0, 0.06),
            (0.41, 0.36, 0.022),
            (0.32, 0.30, 0.25, 1),
        )
        geom(
            scene,
            "tray",
            "box",
            (CX, CY, 0.132),
            (0.153, 0.103, 0.055),
            (0.16, 0.18, 0.15, 1),
        )
        geom(
            scene,
            "sod",
            "box",
            (CX, CY, Z - 0.002),
            (0.15, 0.10, 0.002),
            (0.22, 0.16, 0.09, 1),
        )
        element(
            scene,
            "geom",
            name="canopy",
            type="mesh",
            mesh="grass_mesh",
            material="grass_material",
            pos=(CX, CY, Z + 0.0004),
            contype="0",
            conaffinity="0",
            group="1",
        )

        def text(values):
            return " ".join(str(v) for row in values for v in row)

        vertices, faces, uv = [], [], []
        for leaf, triangles in self.leaves:
            index = len(vertices)
            for u, v, h in leaf:
                vertices.append(((u - W / 2) / 1000, (H / 2 - v) / 1000, h / 1000))
                uv.append((u / W, v / H))
            for a, b, c in triangles:
                # World Y is reversed relative to the overhead photograph. Wind the
                # visible side of each thin leaf and ray floret towards the camera.
                p, q, r = (leaf[i] for i in (a, b, c))
                direction = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (
                    r[0] - p[0]
                )
                if direction > 0:
                    b, c = c, b
                faces.append((index + a, index + b, index + c))
        element(
            asset,
            "mesh",
            name="leaves",
            vertex=text(vertices),
            face=text(faces),
            texcoord=text(uv),
        )
        element(
            scene,
            "geom",
            name="foliage",
            type="mesh",
            mesh="leaves",
            material="rosette_material",
            pos=(CX, CY, Z),
            contype=0,
            conaffinity=0,
        )
        rng = np.random.default_rng(127)
        verts, tris = [], []
        for x, y in rng.uniform([1, 1], [W - 1, H - 1], (2600, 2)):
            if self.labels[int(y), int(x)]:
                continue
            a = len(verts)
            u, v = (x - W / 2) / 1000, (H / 2 - y) / 1000
            verts.extend(
                (
                    (u - 0.00045, v, 0),
                    (u + 0.00045, v, 0),
                    (
                        u + rng.uniform(-0.002, 0.002),
                        v + 0.001,
                        rng.uniform(0.002, 0.005),
                    ),
                )
            )
            tris.append((a, a + 1, a + 2))
        element(asset, "mesh", name="blades", vertex=text(verts), face=text(tris))
        element(
            scene,
            "geom",
            name="grass_blades",
            type="mesh",
            mesh="blades",
            rgba=".25 .37 .13 1",
            pos=(CX, CY, Z + 0.0005),
            contype=0,
            conaffinity=0,
        )
        element(
            tree.find("worldbody"),
            "camera",
            name="survey",
            pos=(CX, CY, 0.64),
            xyaxes="1 0 0 0 1 0",
            fovy="43",
        )
        site = tree.find(".//site[@name='right_tcp']")
        wrist = next(body for body in tree.iter("body") if site in list(body))
        tool = element(wrist, "body", name="sprayer")
        # tcp frame's z axis points toward the lawn at the nominal working orientation.
        geom(
            tool,
            "tool_bracket",
            "box",
            (0, 0, -0.016),
            (0.027, 0.012, 0.007),
            (0.22, 0.23, 0.21, 1),
            contype=0,
            conaffinity=0,
        )
        geom(
            tool,
            "reservoir",
            "ellipsoid",
            (0.039, 0, -0.027),
            (0.022, 0.024, 0.043),
            (0.48, 0.69, 0.67, 1),
            contype=0,
            conaffinity=0,
        )
        geom(
            tool,
            "pump",
            "box",
            (0.026, 0, 0.012),
            (0.027, 0.012, 0.009),
            (0.91, 0.88, 0.77, 1),
            contype=0,
            conaffinity=0,
        )
        geom(
            tool,
            "tip",
            "cylinder",
            (0, 0, 0.026),
            (0.006, 0.009),
            (0.27, 0.30, 0.29, 1),
            contype=0,
            conaffinity=0,
        )
        element(
            tool, "site", name="outlet", pos=(0, 0, 0.036), size=".001", rgba="0 0 0 0"
        )
        images = {}
        for name, im in (("grass.png", self.grass), ("plants.png", self.texture)):
            buf = io.BytesIO()
            im.save(buf, format="PNG")
            images[name] = buf.getvalue()
        return mj.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"), images)

    def reset(self, s):
        self.rng = np.random.default_rng(901 + self.seed)
        self.wet = Image.new("1", (W, H))
        self.pulses = []
        self.armed = True
        self.result = score(*uv_pair(np.asarray(self.wet)), self.labels)

    def before_step(self, s):
        pass

    def after_step(self, s):
        opening = np.mean([s.joint(f"right_finger_{i}") for i in (-1, 1)])
        if opening > 0.032:
            self.armed = True
        if not self.armed or opening > 0.016:
            return
        self.armed = False
        nozzle = s.data.site("outlet")
        origin = nozzle.xpos.copy()
        basis = nozzle.xmat.reshape(3, 3)
        rays = self.rng.normal(0, 0.17, (1800, 2)) @ basis[:, :2].T + basis[:, 2]
        down = rays[:, 2] < -1e-6
        t = (Z - origin[2]) / np.where(down, rays[:, 2], -1)
        valid = down & (t > 0) & (t < 0.45) & (origin[2] > Z + 0.002)
        # Trace front to back through the visible canopy in 1 mm depth slices.
        rays = rays[valid]
        live = np.ones(len(rays), bool)
        pen = ImageDraw.Draw(self.wet)
        landed = 0
        for height_mm in range(int(self.heights.max()), -1, -1):
            ids = np.flatnonzero(live)
            ts = (Z + height_mm / 1000 - origin[2]) / rays[ids, 2]
            pos = origin + ts[:, None] * rays[ids]
            x = np.floor((pos[:, 0] - CX) * 1000 + W / 2).astype(int)
            y = np.floor((CY - pos[:, 1]) * 1000 + H / 2).astype(int)
            inside = (ts >= 0) & (x >= 0) & (x < W) & (y >= 0) & (y < H)
            slots = np.flatnonzero(inside)
            struck = slots[self.heights[y[slots], x[slots]] >= height_mm]
            for j in struck:
                pen.point((int(x[j]), int(y[j])), fill=1)
            live[ids[struck]] = False
            landed += len(struck)
        self.pulses.append(
            dict(
                time_s=float(s.data.time),
                origin_m=origin.tolist(),
                axis=basis[:, 2].tolist(),
                hits_in_tray=int(landed),
            )
        )
        self.result = score(*uv_pair(np.asarray(self.wet)), self.labels)

    def measure(self, s):
        return dict(**self.result, pulses=len(self.pulses))
