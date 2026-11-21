"""Boot-liner hair removal fixture using the dual-YAM station."""
import numpy as np

from fixture_base import Fixture, Handle, VIS, body, box, capsule, joint, plate
from station import element
from vacuum import BIN_CENTER, INTAKE_DIRECTION, INTAKE_OFFSET, add_vacuum_geometry, suction_force

TOOL_ROTATION = np.array(((0, 1, 0), (-1, 0, 0), (0, 0, 1)))
INTAKE = TOOL_ROTATION @ INTAKE_OFFSET
BIN_CENTER = TOOL_ROTATION @ BIN_CENTER
INTAKE_DIRECTION = TOOL_ROTATION @ INTAKE_DIRECTION

CARPET = (.13, .145, .15, 1)
TRIM = (.075, .085, .09, 1)
BRUSH = (.28, .52, .48, 1)
RUBBER = (.055, .10, .095, 1)
COLORS = ((.80, .66, .51, 1), (.57, .44, .35, 1), (.93, .82, .69, 1))
AXES = ((1, 0, 0), (0, 1, 0), (0, 0, 1))


class BootLiner(Fixture):
    title = "Remove pet hair from a cloth boot liner"
    instruction = (
        "Brush the pet hair loose from both strips of carpet, including the strip beside "
        "the raised trim. Vacuum the hair, then return the vacuum to its dock and switch it off."
    )
    focus = (.065, -.015, .240)
    brush = (-.034, -.066, .268)
    dock = (.104, .136, .277)
    switch = (.030, .022, .025)
    pile_x = .146
    rows = (-.074, -.029)
    seeds = tuple((x, y) for y in (-.074, -.029) for x in (.004, .030, .056, .082, .108))
    handles = (
        Handle("brush", "brush_x", "right", AXES[0], max_force=24),
        Handle("vacuum", "vacuum_x", "left", AXES[0], max_force=28),
    )

    def build(self, world):
        rng = np.random.default_rng(581)
        plate(world, x=.07, sx=.19)
        box(world, "molded_boot_floor", (.052, -.047, .207), (.128, .069, .008), TRIM)
        box(world, "carpeted_liner", (.051, -.047, .216), (.116, .061, .004), CARPET)
        box(world, "raised_trim", (.051, -.112, .235), (.126, .009, .018), TRIM)
        box(world, "boot_sill", (-.076, -.045, .220), (.011, .069, .023), TRIM)
        for i in range(22):
            box(world, f"sill_tread_{i}", (-.077, -.100 + i*.0052, .244),
                (.0075, .001, .00055), (.20, .22, .23, 1), **VIS)
        for i in range(480):
            x, y = rng.uniform([-.062, -.106], [.164, .010])
            dx, dy = rng.normal(0, .0008, size=2)
            gray = rng.uniform(.12, .25)
            capsule(world, f"pile_thread_{i}", (x, y, .2201),
                    (x+dx, y+dy, .2212), .00042, (gray*.88, gray, gray, 1), **VIS)
        vacuum = body(world, "vacuum", self.dock)
        joint(vacuum, "vacuum_x", "slide", AXES[0], (-.055, .150), damping=2)
        joint(vacuum, "vacuum_y", "slide", AXES[1], (-.225, .060), damping=2)
        joint(vacuum, "vacuum_z", "slide", AXES[2], (-.018, .145), damping=2)
        housing = element(vacuum, "body", name="vacuum_housing", quat=".70710678 0 0 -.70710678")
        add_vacuum_geometry(housing)
        housing.remove(housing.find("geom[@name='vacuum_switch']"))
        box(vacuum, "vacuum_switch_bezel", (.024, .020, .012), (.016, .012, .010),
            (.10, .11, .12, 1), **VIS)
        box(vacuum, "vacuum_handle_collision", (0, 0, 0), (.013, .011, .009),
            (0, 0, 0, 0), mass=.26)
        switch = body(vacuum, "vacuum_switch", self.switch)
        joint(switch, "vacuum_switch_travel", "slide", (0, 0, -1), (0, .006),
              damping=.26, stiffness=325)
        box(switch, "vacuum_power_button", (0, 0, 0), (.009, .008, .003),
            (.38, .53, .57, 1), mass=.007, solref=".004 1")
        box(world, "vacuum_dock", (self.dock[0], self.dock[1]+.03, .213),
            (.032, .065, .012), (.24, .27, .30, 1))

        brush = body(world, "brush", self.brush)
        joint(brush, "brush_x", "slide", AXES[0], (-.004, .192), damping=2)
        joint(brush, "brush_y", "slide", AXES[1], (-.010, .060), damping=2)
        joint(brush, "brush_z", "slide", AXES[2], (-.028, .055), damping=2, stiffness=40)
        box(brush, "rubber_brush_grip", (0, 0, 0), (.015, .011, .009), BRUSH, mass=.048)
        box(brush, "brush_stock", (0, 0, -.022), (.016, .018, .005), BRUSH, mass=.036)
        for column in range(3):
            for tooth in range(11):
                x, y = -.010 + column*.010, -.015 + tooth*.003
                capsule(brush, f"rubber_tooth_{column}_{tooth}", (x, y, -.0255),
                        (x+.0025, y, -.037), .00175, RUBBER, mass=.0002)
        for i, (x, y) in enumerate(self.seeds):
            clump = body(world, f"hair_{i}", (x, y, .225))
            for axis, axis_vector in zip("xyz", AXES):
                joint(clump, f"hair_{i}_{axis}", "slide", axis_vector, (-.32, .32), damping=.065)
            box(clump, f"fiber_proxy_{i}", (0, 0, 0), (.012, .010, .005),
                (0, 0, 0, 0), mass=.0038, contype=0, conaffinity=0)
            for j in range(28):
                p = rng.uniform([-.011, -.009, -.002], [.011, .009, .003])
                q = p + rng.uniform([-.006, -.003, -.002], [.006, .003, .004])
                mid = (p + q)/2 + (0, .0015, .0014)
                for k, (a, b) in enumerate(((p, mid), (mid, q))):
                    capsule(clump, f"fur_{i}_{j}_{k}", a, b, .00023, COLORS[j%3], **VIS)

    def reset(self, s):
        super().reset(s)
        self.gathered = set()
        self.collected = set()
        self.entrained = set()
        self.powered = False
        self.switch_pressed = False
        self.had_vacuum = False
        self.dock_pose = np.zeros(3)
        self.vacuum_offset = np.zeros(3)
        self.peak_suction = 0.
        self.nearest_uncollected_m = float("inf")
        self.had_brush = False
        self.last_brush = s.model.body("brush").pos.copy()

    def drive(self, s, name, target, gain, damping, limit):
        magnitude = gain*(target-s.joint(name)) - damping*s.speed(name)
        self.force(s, name, np.clip(magnitude, -limit, limit))

    def before_step(self, s):
        super().before_step(s)
        brush = s.position("brush") - (self.x_offset, 0, 0)
        if "brush" in self.grips:
            initial_tcp, _, _ = self.grips["brush"]
            tcp = s.data.site("right_tcp").xpos
            for axis, idx in (("y", 1), ("z", 2)):
                self.drive(s, f"brush_{axis}", self.brush_grip_position[idx] +
                           tcp[idx] - initial_tcp[idx], 1350, 18, 24)
        else:
            self.brush_grip_position = np.array([s.joint(f"brush_{axis}") for axis in "xyz"])
            for axis, target in zip("xyz", self.brush_grip_position):
                self.drive(s, f"brush_{axis}", target, 700, 10, 20)
        if self.had_brush is False and "brush" in self.grips:
            self.brush_grip_position = np.array([s.joint(f"brush_{axis}") for axis in "xyz"])
        self.had_brush = "brush" in self.grips
        adv = brush[0]-self.last_brush[0]
        self.last_brush = brush
        vacuum = s.position("vacuum") - (self.x_offset, 0, 0)
        previous = self.had_vacuum
        self.had_vacuum = "vacuum" in self.grips
        vacuum_state = np.array([s.joint(f"vacuum_{axis}") for axis in "xyz"])
        if self.had_vacuum:
            initial_tcp, _, _ = self.grips["vacuum"]
            tcp = s.data.site("left_tcp").xpos
            if not previous:
                self.vacuum_offset = vacuum_state
            for axis, index in (("y", 1), ("z", 2)):
                self.drive(s, f"vacuum_{axis}", self.vacuum_offset[index] +
                           tcp[index] - initial_tcp[index], 1700, 23, 34)
        else:
            if previous:
                self.dock_pose = np.zeros(3) if np.linalg.norm(vacuum_state) < .045 else vacuum_state
            for axis, target in zip("xyz", self.dock_pose):
                self.drive(s, f"vacuum_{axis}", target, 2000, 28, 38)
        switch_travel = s.joint("vacuum_switch_travel")
        if switch_travel > .0036 and not self.switch_pressed:
            self.powered = not self.powered
            self.switch_pressed = True
        elif switch_travel < .0010:
            self.switch_pressed = False
        intake = vacuum + np.asarray(INTAKE)
        bin_position = vacuum + np.asarray(BIN_CENTER)
        for i, (x, y) in enumerate(self.seeds):
            home = np.array((x, y, .225))
            position = s.position(f"hair_{i}") - (self.x_offset, 0, 0)
            if (self.had_brush and adv > 0.000004 and brush[2] < .259
                    and abs(brush[1]-y) < .021 and abs(brush[0]-position[0]) < .025
                    and i not in self.collected):
                self.gathered.add(i)
            if i in self.collected:
                # The enclosed bin carries captured debris with the hand vacuum.
                offset = ((i % 5 - 2)*.002, (i // 5 - .5)*.009, 0.)
                for axis, delta in zip("xyz", bin_position + offset - home):
                    self.drive(s, f"hair_{i}_{axis}", delta, 76, .66, 1.25)
                continue
            distance = np.linalg.norm(position - intake)
            if self.had_vacuum:
                self.nearest_uncollected_m = min(self.nearest_uncollected_m, distance)
            velocity = np.array([s.speed(f"hair_{i}_{axis}") for axis in "xyz"])
            suction = suction_force(intake, position, velocity,
                                    powered=self.powered and i in self.gathered,
                                    intake_direction=INTAKE_DIRECTION)
            suction_magnitude = np.linalg.norm(suction)
            self.peak_suction = max(self.peak_suction, suction_magnitude)
            if suction_magnitude > .045:
                self.entrained.add(i)
            if self.powered and i in self.entrained and distance < .007:
                self.collected.add(i)
                continue
            target = home.copy()
            if i in self.gathered:
                target[0] = min(self.pile_x + (i % 5 - 2)*.002,
                                max(position[0], brush[0]+.013) if
                                abs(brush[1]-y)<.021 and brush[2]<.259 and adv>0
                                else position[0])
            # Loosened hair lifts when local suction exceeds its light floor
            # support. There is no attraction before brush engagement.
            for axis, displacement, air in zip("xyz", target-home, suction):
                if suction_magnitude > .001:
                    self.force(s, f"hair_{i}_{axis}", air)
                else:
                    self.drive(s, f"hair_{i}_{axis}", displacement, 56, .60, 1.1)

    def visual(self, s, scene):
        indicator = s.model.geom("vacuum_indicator").id
        color = (.34, .96, .68, 1) if self.powered else (.21, .29, .28, 1)
        for geometry in scene.geoms[:scene.ngeom]:
            if geometry.objid == indicator:
                geometry.rgba[:] = color

    def measure(self, s):
        vacuum_position = s.position("vacuum")
        bin_position = vacuum_position + BIN_CENTER
        settled = sum(
            np.linalg.norm(s.position(f"hair_{index}") - bin_position) < .016
            for index in self.collected
        )
        docked = bool(
            not self.had_vacuum and
            np.linalg.norm([s.joint(f"vacuum_{axis}") for axis in "xyz"]) < .012
        )
        all_secured = settled == len(self.seeds) and docked and not self.powered
        return dict(score=round(.9 * settled / len(self.seeds) + .1 * all_secured, 3),
                    bundles_settled_in_bin=int(settled),
                    vacuum_docked=docked,
                    sources_displaced=len(self.gathered),
                    bundles_in_bin=len(self.collected),
                    vacuum_power=self.powered,
                    switch_mm=round(s.joint("vacuum_switch_travel")*1e3, 1),
                    peak_suction_mN=round(float(self.peak_suction)*1e3, 1),
                    nearest_uncollected_mm=(
                        round(float(self.nearest_uncollected_m)*1e3, 1)
                        if np.isfinite(self.nearest_uncollected_m) else None
                    ),
                    intake_mm=np.round((s.position("vacuum")-(self.x_offset, 0, 0)
                                       +np.asarray(INTAKE))*1e3).astype(int).tolist(),
                    left_mm=np.round((s.data.site("left_tcp").xpos-(self.x_offset, 0, 0))*1e3)
                                  .astype(int).tolist(),
                    vacuum_gripped=self.had_vacuum,
                    hair=np.round([s.position(f"hair_{i}")-(self.x_offset, 0, 0)
                                   for i in range(len(self.seeds))], 3).tolist())

    def sweep_carpet(self, s):
        brush_x, _, brush_z = self.brush
        self.take(s, "right", self.brush, "Grip the rubber detailing brush")
        for row, y in enumerate(self.rows):
            label = "the raised trim" if row == 0 else "the open carpet"
            self.go(s, "right", (brush_x, y, brush_z+.015), f"Position over {label}")
            self.go(s, "right", (brush_x, y, .251), "Lower rubber teeth into the pile")
            self.go(s, "right", (.145, y, .251), f"Rake the embedded hair along {label}", time=2.7)
            self.go(s, "right", (.145, y, .310), "Lift the brush off the gathered clump")
        self.go(s, "right", (-.036, -.079, .300), "Move the brush out of the working area")
        self.leave(s, "right", (-.036, -.079, .300), "Release the brush outside the liner")
    def demo(self, s, incomplete=False, skip_brush=False):
        if not skip_brush:
            self.sweep_carpet(s)
        power_point = np.array(self.dock) + self.switch
        if not incomplete:
            self.go(s, "left", power_point+(0, 0, .040), "Approach the power switch", opening=.02)
            self.go(s, "left", power_point-(0, 0, .011), "Press the vacuum power switch", time=.65)
            self.go(s, "left", power_point+(0, 0, .042), "Lift off the switch")
        self.take(s, "left", self.dock, "Grip the cordless car vacuum", lift=.055)
        for y in self.rows:
            sweep_start = .057 if skip_brush else .136
            sweep_end = .127 if skip_brush else .130
            contact = (sweep_start-INTAKE[0], y-INTAKE[1]+.005, .237-INTAKE[2])
            near_end = (sweep_end-INTAKE[0], contact[1], contact[2])
            self.go(s, "left", (contact[0], contact[1], .325),
                    "Carry the vacuum over the gathered hair", time=1.25)
            self.go(s, "left", contact, "Lower the nozzle to the carpet", time=1.05)
            sweep_label = "Pass over the embedded hair" if skip_brush else "Draw the loose hair through the intake"
            self.go(s, "left", near_end, sweep_label, time=1.15)
            s.wait(.75)
            self.go(s, "left", (near_end[0], near_end[1], .325), "Raise the nozzle clear")
        self.go(s, "left", (self.dock[0], self.dock[1], .333), "Return to the vacuum dock")
        self.go(s, "left", self.dock, "Seat the vacuum in the dock")
        self.leave(s, "left", self.dock, "Release the vacuum")
        if not incomplete:
            self.go(s, "left", power_point+(0, 0, .035), "Return to the power switch", opening=.02)
            self.go(s, "left", power_point-(0, 0, .011), "Switch off the vacuum", time=.65)
            self.go(s, "left", power_point+(0, 0, .045), "Clear the dock")
