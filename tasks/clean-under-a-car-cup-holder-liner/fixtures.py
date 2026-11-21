"""Dual-YAM detailing station with a lift-out liner and contact-driven residue."""

import math

import mujoco as mj
import numpy as np

from fixture_base import BLACK, DARK, RUBBER, STEEL, VIS, Fixture, body, box, cyl, element
from station import DT
import detail

TRIM = (0.205, 0.216, 0.234, 1)
EDGE = (0.36, 0.375, 0.39, 1)
LINER = (0.095, 0.102, 0.117, 1)
SPONGE = (0.88, 0.67, 0.14, 1)
BROWN = (0.52, 0.249, 0.085, 0.85)
MIN_CONTACT_N = 0.10
FULL_CONTACT_N = 0.60
WIPE_DISTANCE = 0.003


class FreeGripFixture(Fixture):
    """Bounded compliant support of an actual two-finger grasp."""

    loose_parts = ()

    def reset(self, s):
        super().reset(s)
        self.carries = {}
        self.was_grasped = set()

    def before_step(self, s):
        super().before_step(s)
        for side in ("left", "right"):
            command = s.data.actuator(f"{side}_finger_-1").ctrl[0]
            if command > 0.020:
                self.carries.pop(side, None)
                continue
            tcp = s.data.site(f"{side}_tcp")
            hand_rotation = tcp.xmat.reshape(3, 3)
            if side not in self.carries and command < 0.008:
                held = {entry[0] for entry in self.carries.values()}
                for name in self.loose_parts:
                    if name in held or len(self.fingertips(s, name, side)) != 2:
                        continue
                    p = s.data.body(name)
                    offset = hand_rotation.T @ (p.xpos - tcp.xpos)
                    orient = hand_rotation.T @ p.xmat.reshape(3, 3)
                    self.carries[side] = (name, offset, orient)
                    self.was_grasped.add(name)
                    break
            if side not in self.carries:
                continue
            name, offset, orient = self.carries[side]
            part = s.data.body(name)
            error = tcp.xpos + hand_rotation @ offset - part.xpos
            if np.linalg.norm(error) > 0.030:
                self.carries.pop(side)
                continue
            velocity = s.data.joint(name + "_free").qvel
            s.data.xfrc_applied[part.id, :3] = np.clip(
                1050 * error - 8 * velocity[:3] + (0, 0, s.model.body_mass[part.id] * 9.81),
                -13, 13,
            )
            quat, rotation_error = np.empty(4), np.empty(3)
            mj.mju_mat2Quat(quat, (hand_rotation @ orient @ part.xmat.reshape(3, 3).T).ravel())
            if quat[0] < 0:
                quat *= -1
            mj.mju_quat2Vel(rotation_error, quat, 1)
            angular = part.xmat.reshape(3, 3) @ velocity[3:]
            s.data.xfrc_applied[part.id, 3:] = np.clip(
                0.27 * rotation_error - 0.003 * angular, -0.072, 0.072,
            )



def ring(parent, name, center, radius, width, depth, color, segments=24, **kwargs):
    """Make an open ring with tangent faces and bevel-like edges."""
    half_tangent = (radius + width) * math.tan(math.pi / segments) + 0.00025
    for index in range(segments):
        angle = math.tau * index / segments
        c, s = math.cos(angle), math.sin(angle)
        box(
            parent,
            f"{name}_{index}",
            (center[0] + radius * c, center[1] + radius * s, center[2]),
            (width, half_tangent, depth),
            color,
            quat=(math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)),
            **kwargs,
        )


class CupHolder(FreeGripFixture):
    title = "Clean under a car cup-holder liner"
    instruction = (
        "Lift the rubber liner out by its tab, scrub the sticky residue with the "
        "sponge, then put the liner back flat in the cup holder."
    )
    focus = (0.090, 0.023, 0.246)
    x_offset = -0.067
    cx, cy = (0.110, 0.033)
    floor_z = 0.222
    seat_z = floor_z + 0.003
    parked = (0.024, 0.140, 0.211)
    sponge_stock = (0.045, -0.120, 0.215)
    loose_parts = ("liner", "sponge")
    dots = (
        (-0.017, -0.019), (0.002, -0.018), (0.018, -0.013),
        (-0.019, 0.001), (0.001, 0.002), (0.019, 0.005),
        (-0.017, 0.019), (0.004, 0.021), (0.018, 0.016),
    )

    def build(self, world):
        box(world, "console", (0.116, 0.004, 0.200),
            (0.137, 0.199, 0.010), TRIM)
        box(world, "console_side", (0.249, 0.004, 0.224),
            (0.008, 0.197, 0.022), DARK)
        box(world, "console_edge_left", (-0.020, 0.003, 0.220),
            (0.004, 0.190, 0.008), EDGE)
        box(world, "console_edge_right", (0.241, 0.003, 0.248),
            (0.003, 0.190, 0.002), EDGE)
        for index in range(15):
            box(world, f"console_stitch_{index}", (0.230, -0.180 + 0.023 * index, 0.244),
                (0.001, 0.006, 0.0008), (0.54, 0.49, 0.43, 1), **VIS)
        for name, x, y in (("target", self.cx, self.cy), ("adjacent", .149, -.073)):
            extra = .005 if name == "target" else 0.0
            cyl(world, f"{name}_well_bottom", (x, y, self.floor_z - .005),
                .044 + extra, .005, BLACK, **VIS)
            box(world, f"{name}_well_floor", (x, y, self.floor_z - .005),
                (.041 + extra, .041 + extra, .005), BLACK, group=5)
            ring(world, f"{name}_well_wall", (x, y, .231), .047 + extra, .005, .010, TRIM)
            ring(world, f"{name}_rim", (x, y, .243), .049 + extra, .006, .0023, EDGE)
            ring(world, f"{name}_rim_shadow", (x, y, .241), .054 + extra, .003, .0015, DARK, **VIS)
        for index, (dx, dy) in enumerate(self.dots):
            cyl(world, f"residue_{index}",
                (self.cx + dx, self.cy + dy, self.floor_z + .00015),
                .0064 if index % 2 == 1 else .0051, .00019, BROWN, **VIS)

        px, py, pz = self.parked
        box(world, "parking_pad", (px, py, pz - .005),
            (.046, .047, .002), (0.31, 0.33, 0.33, 1))
        liner = body(world, "liner", (self.cx, self.cy, self.seat_z))
        element(liner, "freejoint", name="liner_free")
        cyl(liner, "rubber_disk", (0, 0, 0), .037, .003, LINER, **VIS)
        for i, (x, y) in enumerate(((0, 0), (-.023, 0), (.023, 0), (0, -.023), (0, .023))):
            box(liner, f"disk_contact_{i}", (x, y, 0),
                (.0135, .0135, .0028), LINER, mass=.0046, group=5)
        ring(liner, "liner_bead", (0, 0, .002), .034,
             .00115, .0016, RUBBER, **VIS)
        for row in range(-5, 6):
            y = row * .0058
            half = math.sqrt(.030**2 - y**2)
            box(liner, f"liner_rib_{row}", (0, y, .0034),
                (half, .00062, .00046), RUBBER, **VIS)
        box(liner, "pull_tab", (0, .006, .011),
            (.010, .006, .011), LINER, mass=.002, friction="1.8 .01 .0007")
        box(liner, "tab_tip", (0, .006, .020),
            (.010, .0064, .0020), RUBBER, mass=.001)
        for row in range(-2, 3):
            box(liner, f"tab_rib_{row}", (row * .003, .006, .022),
                (.00035, .0047, .00057), EDGE, **VIS)

        sx, sy, sz = self.sponge_stock
        box(world, "sponge_tray", (sx, sy, sz - .008), (.036, .034, .003), DARK)
        sponge = body(world, "sponge", self.sponge_stock)
        element(sponge, "freejoint", name="sponge_free")
        box(sponge, "sponge_pad", (0, 0, 0), (.0135, .0115, .005), SPONGE, mass=.007)
        box(sponge, "sponge_handle", (0, 0, .026), (.010, .0067, .021), STEEL, mass=.013)
        for j in range(9):
            y = (j - 4) * .00225
            box(sponge, f"sponge_edge_texture_{j}", (.01351, y, 0),
                (.00015, .00046, .0042), (0.65, 0.41, 0.071, 1), **VIS)

    def decorate(self, tree, world, assets):
        material_assets = tree.find("asset")
        if material_assets is None:
            material_assets = element(tree, "asset")
        hidden_prefixes = ("console", "target_", "adjacent_", "residue_")
        for geom in world.findall("geom"):
            if geom.attrib["name"].startswith(hidden_prefixes):
                geom.set("group", "5")
        liner = world.find("body[@name='liner']")
        sponge = world.find("body[@name='sponge']")
        for part in (liner, sponge):
            for geom in part.findall("geom"):
                geom.set("group", "5")
        detail.console(material_assets, world)
        detail.insert(material_assets, liner)
        detail.scrubber(sponge)
        detail.smears(material_assets, world, self.dots, self.floor_z, self.cx, self.cy)

    def reset(self, s):
        super().reset(s)
        self.load = np.zeros(len(self.dots))
        self.exposed = False
        self.reseat_hold = 0.0
        self.last_sponge = None
        self.pressure = 0.0

    def before_step(self, s):
        s.data.xfrc_applied[:] = 0
        super().before_step(s)

    def after_step(self, s):
        liner = s.data.body("liner")
        sponge = s.data.body("sponge")
        self.exposed |= bool(
            (abs(liner.xpos[0] - self.cx - self.x_offset) > .060
             or abs(liner.xpos[1] - self.cy) > .065)
            and "liner" in self.was_grasped
        )
        previous = sponge.xpos[:2].copy() if self.last_sponge is None else self.last_sponge
        self.last_sponge = sponge.xpos[:2].copy()
        moved = min(float(np.linalg.norm(self.last_sponge - previous)), .002)
        force = np.empty(6)
        self.pressure = 0.0
        for index, contact in enumerate(s.data.contact):
            names = {s.model.geom(contact.geom1).name, s.model.geom(contact.geom2).name}
            if names == {"sponge_pad", "target_well_floor"}:
                mj.mj_contactForce(s.model, s.data, index, force)
                self.pressure += max(0.0, force[0])
        holding_sponge = any(v[0] == "sponge" for v in self.carries.values())
        if self.exposed and holding_sponge and self.pressure >= MIN_CONTACT_N:
            rotation = sponge.xmat.reshape(3, 3)
            load_factor = min(self.pressure / FULL_CONTACT_N, 1.0)
            for index, (dx, dy) in enumerate(self.dots):
                position = np.array([self.cx + self.x_offset + dx,
                                     self.cy + dy, self.floor_z])
                along, across, normal = rotation.T @ (position - sponge.xpos)
                if abs(along) < .016 and abs(across) < .013 and abs(normal + .005) < .008:
                    self.load[index] += moved * load_factor
                    remaining = max(0.0, 1 - self.load[index] / WIPE_DISTANCE)
                    for layer, opacity in enumerate((.77, .71, .56)):
                        s.model.geom(f"spill_{index}_{layer}").rgba[3] = opacity * remaining

        offset = liner.xpos - [self.cx + self.x_offset, self.cy, self.seat_z]
        rotation = liner.xmat.reshape(3, 3)
        seated = (
            np.linalg.norm(offset[:2]) < .009 and abs(offset[2]) < .003
            and rotation[2, 2] > .976 and rotation[1, 1] > .91
            and not any(v[0] == "liner" for v in self.carries.values())
        )
        self.reseat_hold = self.reseat_hold + DT if seated else 0.0

    def measure(self, s):
        clean = int(np.count_nonzero(self.load >= WIPE_DISTANCE))
        seated = self.reseat_hold >= .35
        score = self.score((self.exposed, clean >= 4, clean == len(self.dots), seated))
        return dict(score=score, liner_removed=self.exposed,
                    spots_cleaned=clean, liner_seated=seated,
                    sponge_contact_n=round(self.pressure, 2))

    def transfer_liner(self, s, destination, phrase):
        x, y, z = s.position("liner")
        origin = (x - self.x_offset, y + .006, z + .014)
        target = (destination[0], destination[1] + .006, destination[2] + .014)
        self.take(s, "left", origin, "Pinch the rubber pull tab", lift=.042)
        self.go(s, "left", (origin[0], origin[1], origin[2]+.054),
                "Lift the liner clear of the rim", time=.92)
        self.go(s, "left", (target[0], target[1], target[2]+.048), phrase, time=1.15)
        self.go(s, "left", target, "Lower the liner", time=.7)
        self.leave(s, "left", target, "Release the liner", lift=.045)

    def demo(self, s, incomplete=False, air_wipe=False):
        self.transfer_liner(s, self.parked, "Place the liner on the console pad")
        self.go(s, "left", (.06, .173, .307), "Clear the cup holder", time=.8)
        sx, sy, sz = self.sponge_stock
        self.take(s, "right", (sx, sy, sz + .031), "Grip the narrow sponge", lift=.043)
        self.go(s, "right", (sx, sy, sz+.093), "Lift the sponge", time=.85)
        self.go(s, "right", (self.cx, self.cy, .315),
                "Bring the sponge above the cup holder", time=1.05)
        wipe_z = .256 + (.035 if air_wipe else 0.0)
        self.go(s, "right", (self.cx - .020, self.cy - .020, wipe_z),
                "Press the sponge onto the well floor", time=.85)
        for dx, dy in ((.020, -.020), (.020, 0), (-.020, 0),
                       (-.020, .020), (.020, .020)):
            self.go(s, "right", (self.cx + dx, self.cy + dy, wipe_z),
                    "Scrub the spills with overlapping passes", time=.65)
        self.go(s, "right", (self.cx + .020, self.cy + .020, .310),
                "Lift the sponge clear of the rim")
        self.go(s, "right", (sx, sy, .298), "Return the sponge to its tray")
        self.go(s, "right", (sx, sy, sz+.031), "Lower onto the tray")
        self.leave(s, "right", (sx, sy, sz+.031), "Release the sponge")
        self.go(s, "right", (sx, -.156, .328), "Clear the opening")
        if not incomplete:
            self.transfer_liner(s, (self.cx, self.cy, self.seat_z),
                                "Align the liner above the cleaned recess")
            self.go(s, "left", (.06, .153, .325), "Leave the liner flush")
        s.wait(.55)
