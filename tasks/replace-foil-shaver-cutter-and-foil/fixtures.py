"""Three contact-driven repair stations for dual-YAM tabletop trials."""

import math

import mujoco as mj
import numpy as np

from station import DT
import realism
from fixture_base import (
    BLACK, BLUE, DARK, ORANGE, RUBBER, STEEL, TEAL, VIS, WHITE,
    Fixture, Handle, body, box, capsule, cyl, element, geom, joint, plate,
)

BRUSH = (0.88, 0.72, 0.52, 1)
FIBER = (0.66, 0.61, 0.50, 1)


def slide(parent, name, axis, limit, **kwargs):
    return joint(parent, name, "slide", axis, limit, **kwargs)


def slide_force(s, name, target, gain, damping, cap):
    return np.clip(gain * (target - s.joint(name)) - damping * s.speed(name), -cap, cap)


def touching(s, first, second):
    a, b = s.model.body(first).id, s.model.body(second).id
    body_ids = s.model.geom_bodyid
    return any(
        c.dist < 0.001 and {int(body_ids[c.geom1]), int(body_ids[c.geom2])} == {a, b}
        for c in s.data.contact
    )


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


class Shaver(FreeGripFixture):
    title = "Replace a foil shaver’s cutter and foil"
    instruction = "Remove the worn foil and cutter. Fit the new cutter and foil, briefly run the shaver to check the assembly, then switch it off."
    focus = (0.057, 0.008, 0.270)
    head = (0.084, 0.018)
    cutter_z, foil_z = 0.253, 0.250
    cutter_stock = (0.158, -0.084, 0.242)
    foil_stock = (0.166, 0.126, 0.240)
    old_foil_tray = (0.010, 0.140, 0.224)
    old_cutter_tray = (0.003, -0.135, 0.224)
    button = (0.080, -0.046, 0.256)
    loose_parts = ("worn_foil", "worn_cutter", "fresh_foil", "fresh_cutter")

    def build(self, w):
        plate(w, sx=0.20)
        hx, hy = self.head
        box(w, "rubber_cradle", (0.067, -0.017, 0.218), (0.055, 0.085, 0.018), RUBBER)
        box(w, "shaver_case", (0.080, -0.016, 0.237), (0.044, 0.071, 0.011), BLACK)
        for i in range(5):
            box(w, f"body_fluting_{i}", (0.075, -0.072 + .007 * i, .249), (.020, .0014, .0007), RUBBER, **VIS)
        for x, y, z in [self.cutter_stock, self.foil_stock, self.old_foil_tray, self.old_cutter_tray]:
            box(w, f"tray_{x}_{y}", (x, y, z - .007), (.041, .026, .006), BLUE)
        box(w, "head_seat", (hx, hy, .249), (.036, .0177, .002), DARK)
        stem = body(w, "drive", (hx, hy, .255))
        slide(stem, "motor_drive", (1, 0, 0), (-.004, .004), damping=.12)
        box(stem, "keyed_spindle", (0, 0, 0), (.005, .004, .005), STEEL, mass=.014)
        for sign in (-1, 1):
            # Stops register the skirt and keep both sides of the foil level.
            box(w, f"foil_guide_{sign}", (hx + sign*.046, hy, .252), (.002, .016, .005), BLACK)
        for name, pos, worn in (
            ("worn_cutter", (*self.head, self.cutter_z), True),
            ("fresh_cutter", self.cutter_stock, False),
        ):
            part = body(w, name, pos)
            element(part, "freejoint", name=name+"_free")
            color = (0.49, 0.43, 0.36, 1) if worn else STEEL
            for sign in (-1, 1):
                box(part, f"{name}_foot{sign}", (sign*.018, 0, .003), (.011, .009, .004), color, mass=.008)
            box(part, f"{name}_bridge", (0, 0, .0105), (.029, .011, .0035), color, mass=.009)
            for i in range(19):
                x = (i - 9)*.0029
                capsule(part, f"{name}_tooth{i}", (x,-.009,.014), (x,.009,.014), .0012, color, **VIS)
        for name, pos, worn in (
            ("worn_foil", (*self.head, self.foil_z), True),
            ("fresh_foil", self.foil_stock, False),
        ):
            part = body(w, name, pos)
            element(part, "freejoint", name=name+"_free")
            color = (0.59, 0.52, 0.43, 1) if worn else (.71,.75,.77,1)
            for sign in (-1, 1):
                box(part, f"{name}_skirt{sign}", (0,sign*.020,.007), (.041,.0026,.008), color, mass=.007)
                box(part, f"{name}_latch{sign}", (sign*.041,0,.009), (.0028,.018,.009), DARK, mass=.006)
            for i in range(19):
                x = (i - 9)*.0034
                for j in range(8):
                    u, v = j*math.pi/8, (j+1)*math.pi/8
                    capsule(part, f"{name}_foil{i}_{j}", (x, .020*math.cos(u),.014+.022*math.sin(u)), (x, .020*math.cos(v),.014+.022*math.sin(v)), .00065, color, **VIS)
            for j in range(9):
                angle = j*math.pi/8
                y, z = .020*math.cos(angle), .014+.022*math.sin(angle)
                capsule(part, f"{name}_mesh{j}", (-.033,y,z), (.033,y,z), .0005, color, **VIS)
            # Mechanical upper bridge carries grasp loads. The thin perforated arch is visual.
            box(part, f"{name}_arch_backing", (0,0,.033), (.031,.010,.0024), color, mass=.006)
        switch = body(w, "switch", self.button)
        slide(switch, "button_travel", (0,0,-1), (0,.010), stiffness=245, damping=1.1)
        box(switch, "switch_top", (0,0,0), (.012,.009,.004), TEAL, mass=.020)

    def decorate(self, tree, world, assets):
        realism.shaver(tree, world, assets)

    def reset(self, s):
        super().reset(s)
        self.powered = False
        self.button_down = False
        self.powered_exposed = False
        self.drive_min = self.drive_max = self.stroke_time = 0.0

    def aligned(self, s, name, z, eps):
        p = s.position(name)
        r = s.data.body(name).xmat.reshape(3,3)
        return bool(
            abs(p[0] - (self.head[0] + self.x_offset)) < eps
            and abs(p[1] - self.head[1]) < eps
            and abs(p[2] - z) < .006
            and r[2,2] > .97 and abs(r[0,0]) > .97
        )

    def foil_registered(self, s, name):
        if not self.aligned(s, name, self.foil_z, .0055):
            return False
        part = s.data.body(name)
        rotation = part.xmat.reshape(3, 3)
        for sign in (-1, 1):
            local = np.array((sign*.041, 0, 0))
            seat = np.array((self.head[0]+self.x_offset, self.head[1], self.foil_z)) + local
            tab = part.xpos + rotation @ local
            if np.max(np.abs(tab[1:] - seat[1:])) > .0065:
                return False
        return True

    def fitted(self, s):
        return (
            self.aligned(s, "fresh_cutter", self.cutter_z, .006),
            self.foil_registered(s, "fresh_foil"),
        )

    def before_step(self, s):
        s.data.xfrc_applied[:] = 0
        super().before_step(s)
        covered = any(self.foil_registered(s, name) for name in ("worn_foil", "fresh_foil"))
        carried = {entry[0] for entry in self.carries.values()}
        for name in ("worn_foil", "fresh_foil"):
            if name in carried:
                continue
            part = s.data.body(name)
            seat = np.array((self.head[0]+self.x_offset, self.head[1], self.foil_z))
            offset = part.xpos-seat
            rotation = part.xmat.reshape(3,3)
            if (np.max(np.abs(offset[:2])) < .009 and -.003 < offset[2] < .007
                    and rotation[2,2] > .962 and abs(rotation[0,0]) > .97):
                # Compliant lead-in faces centralize the skirt while the two
                # spring clips pull it down. A cap outside the mouth gets no force.
                velocity = s.data.joint(name+"_free").qvel[:3]
                force = np.clip(-340*offset[:2]-2.7*velocity[:2], -2.6, 2.6)
                s.data.xfrc_applied[part.id,:3] += (*force,-.58)
        for name in ("worn_cutter", "fresh_cutter"):
            if name not in carried and self.aligned(s, name, self.cutter_z, .009):
                part = s.data.body(name)
                s.data.xfrc_applied[part.id, 2] -= 2.1
        if self.powered and covered:
            target = .0037 * math.sin(math.tau * 15 * s.data.time)
            self.force(s, "motor_drive", slide_force(s, "motor_drive", target, 3400, 1.9, 15))
            # The cutter's slotted foot follows its drive under the spring collar.
            for name in ("worn_cutter", "fresh_cutter"):
                if self.aligned(s, name, self.cutter_z, .007):
                    part = s.data.body(name)
                    error = self.head[0] + self.x_offset + s.joint("motor_drive") - part.xpos[0]
                    s.data.xfrc_applied[part.id, 0] += np.clip(3200*error-2.4*s.data.joint(name+"_free").qvel[0], -8, 8)
        else:
            self.force(s, "motor_drive", slide_force(s, "motor_drive", 0.0, 720, 2.2, 6))

    def after_step(self, s):
        travel = s.joint("button_travel")
        if travel < .0015:
            self.button_down = False
        if travel > .004 and not self.button_down:
            self.button_down = True
            self.powered = not self.powered
        covered = any(self.foil_registered(s, name) for name in ("worn_foil", "fresh_foil"))
        if self.powered and not covered:
            self.powered_exposed = True
        if self.powered and all(self.fitted(s)):
            self.stroke_time += DT
            # Measure the cutter itself, with the real sleeve in place.
            x = s.position("fresh_cutter")[0]
            if self.stroke_time <= DT:
                self.drive_min = self.drive_max = x
            self.drive_min, self.drive_max = min(x,self.drive_min), max(x,self.drive_max)

    def visual(self, s, scene):
        p = (self.button[0] + self.x_offset, self.button[1] - .017, .255)
        simple_geom(scene, mj.mjtGeom.mjGEOM_ELLIPSOID, (.004,.0017,.0012), p,
                    (.32,.91,.50,1) if self.powered else (.26,.38,.36,1))

    def measure(self, s):
        new_cutter, new_foil = self.fitted(s)
        free = set(x[0] for x in self.carries.values())
        old_off = all(
            np.linalg.norm(s.position(name)[:2] - (tray[0]+self.x_offset,tray[1])) < .026
            and name not in free and abs(s.position(name)[2] - tray[2]) < .009
            for name, tray in (("worn_cutter", self.old_cutter_tray), ("worn_foil", self.old_foil_tray))
        )
        span = self.drive_max - self.drive_min
        complete = self.stroke_time >= .35 and span >= .001 and not self.powered
        score = self.score((old_off, new_cutter, new_foil and not free, complete))
        return dict(score=0.0 if self.powered_exposed else score,
                    old_parts_removed=old_off, cutter_seated=new_cutter,
                    foil_seated=new_foil, motor_on=self.powered,
                    cutter_stroke_mm=round(span*1000,2), test_seconds=round(self.stroke_time,3),
                    powered_exposed=self.powered_exposed,
                    grasped_parts=sorted(self.was_grasped))

    def transfer(self, s, side, name, target, phrase, height):
        x,y,z = s.position(name)
        origin = (x-self.x_offset,y,z+height)
        self.take(s, side, origin, "Grip "+name.replace("_"," "), lift=.038)
        finish = (target[0], target[1], target[2] + height)
        self.go(s, side, (origin[0],origin[1],origin[2]+.030), "Lift "+name.replace("_"," "), time=.70)
        self.go(s, side, (finish[0],finish[1],finish[2]+.031), phrase, time=1.1)
        self.go(s, side, finish, "Lower into the seat", time=.85)
        self.leave(s,side,finish,"Release the part",lift=.034)
        s.wait(.18)

    def demo(self, s, incomplete=False):
        self.transfer(s,"left","worn_foil",self.old_foil_tray,"Carry old foil to the empty tray",.029)
        self.transfer(s,"right","worn_cutter",self.old_cutter_tray,"Carry old cutter to the empty tray",.009)
        self.transfer(s,"right","fresh_cutter",(*self.head,self.cutter_z),"Align the new cutter over the drive",.009)
        # Park the right arm to clear the foil approach from the opposite side.
        self.go(s,"right",(.11,-.12,.321),"Clear the shaver head")
        self.transfer(s,"left","fresh_foil",(*self.head,self.foil_z),"Align the foil skirt with both retainers",.023)
        self.go(s,"left",(.07,.115,.326),"Clear the assembled shaver")
        if not incomplete:
            x,y,z=self.button
            self.go(s,"right",(x,y,z+.033),"Approach the power switch",opening=.0)
            for phase in ("Start the covered cutter", "Switch the shaver off"):
                self.go(s,"right",(x,y,z-.014),phase,time=.55)
                self.go(s,"right",(x,y,z+.024),"Release the button",time=.4)
                s.wait(.55)
        s.wait(.55)


def simple_geom(scene, kind, size, position, color, matrix=None, material=None):
    if scene.ngeom >= scene.maxgeom:
        raise RuntimeError("Fixture render geometry limit reached")
    mj.mjv_initGeom(scene.geoms[scene.ngeom], kind, np.asarray(size), np.asarray(position),
                    np.eye(3).ravel() if matrix is None else np.asarray(matrix).ravel(),
                    np.asarray(color, dtype=np.float32))
    if material is not None:
        scene.geoms[scene.ngeom].matid = material
        scene.geoms[scene.ngeom].texcoord = 1
    scene.ngeom += 1




def line(scene,start,end,radius,color):
    if scene.ngeom >= scene.maxgeom:
        raise RuntimeError("Fixture render geometry limit reached")
    connector=scene.geoms[scene.ngeom]
    mj.mjv_initGeom(connector,mj.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.asarray(color,dtype=np.float32))
    mj.mjv_connector(connector,mj.mjtGeom.mjGEOM_CAPSULE,radius,np.asarray(start,dtype=float),np.asarray(end,dtype=float))
    scene.ngeom+=1


