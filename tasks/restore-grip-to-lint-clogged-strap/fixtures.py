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


class Fastener(Fixture):
    title = "Restore grip to a lint-clogged hook-and-loop strap"
    instruction = (
        "Brush the lint off the hook side, close the strap over the loop side "
        "using the clamp, then release the test weight. Leave the weight suspended."
    )
    focus = (.085, -.035, .245)
    brush = (.006, -.074, .273)
    clamp = (.087, .118, .258)
    weight_latch = (.173, -.132, .251)
    lint_centers = (.034, .054, .074, .094, .114, .134)
    handles = (
        Handle("brush", "brush_sweep", "right", (1, 0, 0), max_force=20),
        Handle("flap", "closure_align", "left", (0, -1, 0), max_force=18),
        Handle("weight_latch", "latch_travel", "right", (0, -1, 0), max_force=17),
    )

    def build(self, world):
        plate(world, sx=.22)
        box(world, "base", (.077, -.023, .205), (.117, .161, .005), DARK)
        box(world, "hook_webbing", (.085, -.074, .214), (.064, .024, .004), BLACK)
        for i in range(25):
            for j in range(8):
                x, y = .025 + i*.005, -.094 + j*.0055
                capsule(world, f"hook_{i}_{j}", (x,y,.218), (x,y+.0011,.2201),
                        .00055, (.17,.18,.19,1), **VIS)
        box(world, "lint_collection_trough", (.084,-.127,.213), (.064,.018,.002), BLUE)
        for i, x in enumerate(self.lint_centers):
            tuft = body(world, f"tuft_{i}", (x, -.074, .221))
            slide(tuft, f"lint_{i}", (0,-1,0), (0,.056), damping=.12, frictionloss=.004)
            box(tuft, f"lint_contact_{i}", (0,0,.001), (.0028,.009,.004), FIBER,
                mass=.0028, solref=".012 1")
            for j in range(18):
                theta = j*2.399 + i*.26
                x1, y1 = .003*math.sin(theta), (j%9-4)*.0037
                capsule(tuft, f"fiber_{i}_{j}", (x1,y1,.003),
                        (x1+.006*math.cos(theta), y1+.005*math.sin(theta),
                         .002+abs(math.sin(theta))*.004), .00055, FIBER, **VIS)
        brush = body(world, "brush", self.brush)
        slide(brush, "brush_sweep", (1,0,0), (0,.159), damping=2, frictionloss=.8)
        slide(brush, "brush_press", (0,0,-1), (0,.026), damping=.65, stiffness=88)
        box(brush, "brush_handle", (0,0,0), (.013,.011,.008), BRUSH, mass=.023)
        box(brush, "brush_stock", (0,0,-.020), (.010,.026,.005), BRUSH, mass=.017)
        for i in range(11):
            for j in range(3):
                x, y = (j-1)*.0046, (i-5)*.0043
                capsule(brush, f"bristle_{i}_{j}", (x,y,-.023), (x+.0024,y,-.039),
                        .00125, WHITE, mass=.00018, friction=".20 .004 .001", solref=".018 1")
        flap = body(world, "flap", self.clamp)
        slide(flap, "closure_align", (0,-1,0), (0,.083), damping=2.0, frictionloss=1.0)
        slide(flap, "closure_press", (0,0,-1), (0,.026), stiffness=130,
              damping=.7, frictionloss=.16)
        box(flap, "strap_tab", (0,0,0), (.014,.011,.007), TEAL, mass=.019)
        capsule(flap, "clamp_arm", (0,-.003,0), (0,-.112,-.005), .0038, STEEL, mass=.010)
        box(flap, "loop_webbing", (-.002,-.112,-.013), (.063,.020,.0035), (.32,.35,.39,1),
            mass=.035)
        for i in range(23):
            box(flap, f"weave_{i}", (-.059+i*.0052, -.112, -.0092), (.0005,.018,.0003),
                (.39,.42,.46,1), **VIS)

        carrier = body(world, "load_end", (.155,-.074,.223))
        slide(carrier, "strap_slip", (1,0,0), (0,.036), damping=.12)
        box(carrier, "loaded_tab", (.009,0,.007), (.010,.008,.003), RUBBER, mass=.010)
        bearing = body(world, "test_weight", (.234,-.074,.254))
        slide(bearing, "weight_drop", (0,0,-1), (0,.038), damping=.15)
        cyl(bearing,"proof_mass",(0,0,0),.017,.016,STEEL,mass=.215)
        gate = body(world, "weight_latch", self.weight_latch)
        slide(gate, "latch_travel", (0,-1,0), (0,.031), damping=2.2, frictionloss=1.5)
        box(gate,"test_release",(0,0,0),(.012,.010,.008),ORANGE,mass=.029)

    def decorate(self, tree, world, assets):
        realism.fastener(tree, world, assets)

    def constraints(self, tree):
        eq = element(tree, "equality")
        element(eq, "joint", joint1="weight_drop", joint2="strap_slip",
                polycoef="0 1 0 0 0", solref=".005 1")

    def reset(self, s):
        super().reset(s)
        self.cleaned = set()
        self.brushed = {}
        self.closed = False
        self.was_closed = False
        self.proof = 0.0
        self.weight_released = False

    def press_tool(self, s, handle, axis, side, preload):
        tcp, _, _ = self.grips[handle]
        target = np.clip(tcp[2]-s.data.site(f"{side}_tcp").xpos[2]+preload, 0, .025)
        self.force(s, axis, slide_force(s, axis, target, 1050, 6, 18))

    def before_step(self, s):
        super().before_step(s)
        if "brush" in self.grips:
            self.press_tool(s, "brush", "brush_press", "right", .003)
            if s.speed("brush_sweep") > .003 and s.joint("brush_press") > .006:
                for i in range(len(self.lint_centers)):
                    if i not in self.cleaned and touching(s, "brush", f"tuft_{i}"):
                        self.brushed.setdefault(i, s.joint("brush_sweep"))
        for i, start in self.brushed.items():
            name = f"lint_{i}"
            # The angled bristles move lint sideways as the brush passes. Once
            # it clears the hooks, the sloped chute carries it into the trough.
            advance = max(0, s.joint("brush_sweep") - start)
            if i in self.cleaned or s.joint(name) > .025:
                target = .045 + .0013*(i%5)
            else:
                target = min(.041, advance*2.6)
            self.force(s, name, slide_force(s, name, target, 12, .38, .54))
        if "flap" in self.grips:
            self.press_tool(s, "flap", "closure_press", "left", .006)
        if self.closed:
            self.force(s, "closure_press",
                       slide_force(s, "closure_press", .024, 300, 3, 4.2))
            clear_area = sum(np.clip((s.joint(f"lint_{i}")-.004)/.035, 0, 1)
                             for i in range(len(self.lint_centers)))
            overlap = np.clip(1-s.joint("strap_slip")/.126, 0, 1)
            alignment = np.clip((s.joint("closure_align")-.065)/.016, 0, 1)
            shear_limit = (.33+.72*clear_area)*overlap*alignment
            self.force(s, "strap_slip",
                       slide_force(s, "strap_slip", 0, 570, .95, shear_limit))
        if not self.weight_released:
            self.force(s, "weight_drop",
                       slide_force(s, "weight_drop", 0, 3700, 16, 14))

    def after_step(self, s):
        for i in self.brushed:
            if s.joint(f"lint_{i}") > .035:
                self.cleaned.add(i)
        overlap = s.joint("closure_align") > .074 and s.joint("strap_slip") < .017
        if s.joint("closure_press") > .017 and overlap:
            self.closed = self.was_closed = True
        if s.joint("closure_press") < .010 or not overlap:
            self.closed = False
        if s.joint("latch_travel") > .018:
            self.weight_released = True
        if self.weight_released and self.closed and s.joint("weight_drop") < .010:
            self.proof += DT
        else:
            self.proof = 0.0

    def measure(self, s):
        clean = len(self.cleaned)
        held = self.proof >= .65
        score = .5*clean/len(self.lint_centers)
        if clean == len(self.lint_centers):
            score += .25*int(self.closed) + .25*int(held and not self.grips)
        return dict(score=round(score,4), lint_tufts_removed=clean,
                    strap_closed=self.closed, load_released=self.weight_released,
                    weight_drop_mm=round(s.joint("weight_drop")*1000,1),
                    load_held_s=round(self.proof,2), pressed_before_load=self.was_closed,
                    lint_travel_mm=[round(s.joint(f"lint_{i}")*1000,1)
                                    for i in range(len(self.lint_centers))])

    def visual(self, s, scene):
        slip = s.joint("strap_slip")
        panel = s.position("flap") + (-.002 + slip, -.112, -.013)
        simple_geom(scene, mj.mjtGeom.mjGEOM_BOX, (.063,.020,.0035), panel,
                    (1,1,1,1), material=s.model.mat("strap_loops").id)
        for side in (-1, 1):
            for k in range(29):
                p = panel + (-.058+.0041*k, side*.0177, .0037)
                line(scene, p, p+(.0025,0,0), .00025, (.59,.61,.57,1))
        tail = s.position("load_end") + (.004,0,.007)
        lip = panel + (.061,0,.0004)
        if s.joint("closure_align") > .064:
            # Webbing bends over the jaw and ends at the clevis on the pull cord.
            middle = (lip+tail)/2 + (0,0,.003)
            for first, last in ((lip,middle),(middle,tail)):
                vector = last-first
                basis = np.array((vector, (0,1,0), np.cross(vector,(0,1,0))))
                basis /= np.linalg.norm(basis,axis=1)[:,None]
                simple_geom(scene, mj.mjtGeom.mjGEOM_BOX,
                            (np.linalg.norm(vector)/2+.0004,.009,.0009),
                            (first+last)/2, (.24,.27,.28,1), basis.T)
        weight = s.position("test_weight") + (0,0,.021)
        centre = weight + (-.010,0,.296-weight[2])
        rope = [.010*np.array((math.cos(a),0,math.sin(a)))+centre
                for a in np.linspace(2.316,0,10)]
        points = [tail,*rope,weight]
        for first,last in zip(points,points[1:]):
            line(scene,first,last,.00058,(.19,.20,.18,1))
        line(scene, tail-.003*np.array((1,0,0)), tail+.004*np.array((1,0,0)),
             .0020, (.68,.71,.72,1))

    def demo(self, s, incomplete=False):
        if not incomplete:
            x,y,z = self.brush
            self.take(s, "right", (x,y,z-.006), "Grip the stiff brush")
            self.go(s, "right", (x,y,z-.010), "Set the bristles against the hook strip")
            self.go(s, "right", (.159,y,z-.010), "Sweep the lint into the waste trough", time=3.8)
            self.go(s, "right", (.159,y,z+.009), "Lift the brush clear")
            self.go(s, "right", (x,y,z+.009), "Return the brush to its parking position", time=2)
            self.leave(s, "right", (x,y,z+.007), "Let go of the brush")
            self.go(s, "right", (.11,-.16,.315), "Clear the test fixture")
        x,y,z = self.clamp
        self.take(s, "left", (x,y,z-.006), "Grip the clamp carrying the loop side")
        self.go(s, "left", (x,y-.085,z+.007), "Align the loop side over the hooks", time=1.8)
        self.go(s, "left", (x,y-.085,z-.031), "Press the two fastening faces together", time=.85)
        s.wait(.25)
        self.leave(s, "left", (x,y-.085,z-.031), "Release the closed strap")
        self.go(s, "left", (.06,.119,.328), "Clear the load test")
        x,y,z = self.weight_latch
        self.take(s, "right", (x,y,z), "Grip the test weight latch")
        finish = (x,y-.028,z)
        self.go(s, "right", finish, "Release the 215 gram weight")
        self.leave(s, "right", finish, "Withdraw from the test weight")
        s.wait(1.1, "Check whether the unsupported weight stays suspended")


def line(scene,start,end,radius,color):
    if scene.ngeom >= scene.maxgeom:
        raise RuntimeError("Fixture render geometry limit reached")
    connector=scene.geoms[scene.ngeom]
    mj.mjv_initGeom(connector,mj.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.asarray(color,dtype=np.float32))
    mj.mjv_connector(connector,mj.mjtGeom.mjGEOM_CAPSULE,radius,np.asarray(start,dtype=float),np.asarray(end,dtype=float))
    scene.ngeom+=1


