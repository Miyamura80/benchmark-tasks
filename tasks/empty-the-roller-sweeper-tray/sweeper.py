"""Lift out and empty the removable debris tray of a floor sweeper."""

import math
import numpy as np
import mujoco as mj
from fixtures import (
    Fixture,
    box,
    cylinder,
    body,
    capsule,
    element,
    ball,
    tray,
    DARK,
    BLACK,
    TEAL,
    ORANGE,
    STEEL,
    NO_CONTACT,
    DT,
    HZ,
)


class Sweeper(Fixture):
    title = "Empty the roller sweeper's debris tray"
    instruction = "Pull the loaded tray out along its guides, tip the dry spill into the bin, and slide the empty tray fully home."
    skill = "Extract and tip a loaded tray."
    focus = (0.016, -0.12, 0.22)
    origin = np.array([0.025, -0.12, 0.184])
    bin_xy = (0.019, -0.246)
    size = (0.125, 0.112)

    def build(self, world):
        x, y, z = self.origin
        box(world,"worktop",(.04,-.14,.122),(.31,.34,.008),(.4,.36,.30,1))
        tray(
            world, *self.bin_xy, prefix="bin", z=0.148, sx=self.size[0], sy=self.size[1]
        )
        floor=world.find("geom[@name='bin']")
        floor.set("pos",f"{self.bin_xy[0]} {self.bin_xy[1]} .1405")
        floor.set("size",f"{self.size[0]} {self.size[1]} .010")
        for sign in (-1, 1):
            box(
                world,
                f"bin_end{sign}",
                (self.bin_xy[0] + sign * self.size[0], self.bin_xy[1], 0.177),
                (0.003, self.size[1], 0.031),
                TEAL,
            )
            box(
                world,
                f"bin_rim{sign}",
                (self.bin_xy[0], self.bin_xy[1] + sign * self.size[1], .185 if sign<0 else .161),
                (self.size[0], .003, .039 if sign<0 else .015),
                TEAL,
            )
        box(
            world,
            "sweeper chassis",
            (x, y + 0.034, z - 0.006),
            (0.077, 0.075, 0.005),
            DARK,
        )
        box(
            world,
            "brush cover",
            (x, y + 0.076, z + 0.012),
            (0.082, 0.024, 0.015),
            (0.15, 0.37, 0.43, 1),
        )
        # Open upper service hatch exposes the lift-out tray and brush.
        capsule(
            world,
            "brush core",
            (x - 0.071, y + 0.057, z + 0.018),
            (x + 0.071, y + 0.057, z + 0.018),
            0.011,
            BLACK,
        )
        for i in range(23):
            a = i * 0.8
            for sign in (-1, 1):
                off = np.array([0, math.sin(a), math.cos(a)]) * sign * 0.017
                p = np.array([x - 0.062 + i * 0.0057, y + 0.057, z + 0.018])
                capsule(
                    world,
                    f"bristle{sign}_{i}",
                    p,
                    p + off,
                    0.0026,
                    ORANGE,
                    **NO_CONTACT,
                )
        for sign in (-1, 1):
            cylinder(
                world,
                f"wheel{sign}",
                (x + sign * 0.084, y + 0.082, z),
                (0.027, 0.007),
                BLACK,
                euler=(0, math.pi / 2, 0),
            )
        capsule(
            world,
            "lower handle",
            (x, y + 0.081, z + 0.01),
            (x + 0.049, y + 0.198, z + 0.132),
            0.006,
            STEEL,
        )
        # Two return lips retain the back corners until the tray clears the runners.
        for side in (-1, 1):
            box(world, f"guide{side}", (x+side*.073,y+.034,z+.021),
                (.002,.006,.024), DARK)
            box(world, f"guide_lip{side}", (x+side*.066,y+.033,z+.043),
                (.008,.004,.0025), DARK)
        box(world,"back_stop",(x,y+.044,z+.016),(.069,.002,.018),DARK)
        box(world,"extension_shelf",(x,y-.048,z-.006),(.076,.031,.005),DARK)
        # Enclose the seated tray beneath the opaque motor cover. The exposed
        # pull end remains accessible and the open front accepts the drawer.
        box(world,"motor_cover",(x+.031,y+.024,z+.050),(.039,.035,.004),(.12,.30,.36,1))
        for i in range(5):
            box(world,f"vent{i}",(x+.018+i*.007,y+.031,z+.0542),(.0015,.014,.00045),DARK,**NO_CONTACT)
        part = body(world, "drawer", self.origin)
        element(part, "freejoint", name="drawer_pose")
        box(
            part,
            "drawer_floor",
            (0, 0, 0.003),
            (0.067, 0.031, 0.003),
            (0.67, 0.70, 0.68, 1),
            mass=0.043,
        )
        for sign in (-1, 1):
            box(
                part,
                f"side{sign}",
                (0, sign * 0.035, 0.014),
                (0.067, 0.002, 0.011),
                (0.75, 0.77, 0.73, 1),
                mass=0.009,
                euler=(-sign * 0.59, 0, 0),
            )
            if sign < 0:
                lip = part.find("geom[@name='side-1']")
                lip.set("pos", "0 -.039 .008")
                lip.set("euler", "1.25 0 0")
            box(
                part,
                f"end{sign}",
                (sign * 0.066, 0, 0.018),
                (0.002, 0.031, 0.015),
                (0.68, 0.72, 0.68, 1),
                mass=0.007,
            )
        box(
            part,
            "handle",
            (-0.067, 0, 0.037),
            (0.009, 0.023, 0.018),
            ORANGE,
            mass=0.016,
            friction="2 .03 .004",
        )
        # Individually moving rice grains, split lentils and oat flakes.
        # Each group can be weighed separately in the bench trial.
        for i in range(18):
            row, col = divmod(i, 6)
            dx, dy = -.047 + col*.018, -.014 + row*.014
            if self.variant:
                dx += .002*math.sin(i)
                dy += .002*math.cos(i)
            dim, weight, color = (
                ((.0034,.0013,.0012), .000025, (.88,.82,.66,1)),
                ((.0027,.0025,.0015), .00004, (.77,.37,.13,1)),
                ((.0047,.0026,.0010), .000035, (.78,.64,.38,1)),
            )[(row+col)%3]
            crumb=body(world,f"debris{i}",(x+dx,y+dy,z+.011))
            element(crumb,"freejoint",name=f"piece{i}")
            ball(crumb,f"crumb{i}",(0,0,0),dim,color,mass=weight,
                 euler=(.05,.04,.49*i),priority=1,condim=6,solref=".012 1",friction=".45 .0006 .00045")

    def reset(self, s):
        for i in range(18):
            dof=s.model.joint(f"piece{i}").dofadr[0]
            # Stabilize submillimetre contact impulses in platelet rotation.
            s.model.dof_armature[dof+3:dof+6]=1e-8
        s.model.opt.timestep=.0005
        self.left_guides=False
        self.clearance = 0.0
        self.max_tilt = 0.0
        self.settled = 0.0
        self.docked = 0.0
        self.last_support=np.full(18,-1.0)
        self.bin_id = s.model.geom("bin").id
        self.piece_ids = np.array([s.model.geom(f"crumb{i}").id for i in range(18)])
        self.piece_bodies = np.array([s.model.body(f"debris{i}").id for i in range(18)])
        self.linear_dofs = np.array([s.model.joint(f"piece{i}").dofadr[0] for i in range(18)])[:,None] + np.arange(3)
        self.contact_times = np.empty(s.model.ngeom)
        self.back_wall = s.data.geom("side1")
        self.wall_half = s.model.geom("side1").size.copy()
        self.stop_face = s.model.geom("back_stop").pos[1] - s.model.geom("back_stop").size[1]
        self.drawer_floor = s.model.geom("drawer_floor").id
        self.chassis = s.model.geom("sweeper chassis").id

    def count(self, s):
        a, b = s.data.contact.geom1, s.data.contact.geom2
        self.contact_times.fill(-1)
        self.contact_times[b[a == self.bin_id]] = s.data.time
        self.contact_times[a[b == self.bin_id]] = s.data.time
        np.maximum(self.last_support, self.contact_times[self.piece_ids], out=self.last_support)
        # Submillimetre flakes can lose contact for a few substeps while resting.
        # A 20 ms grace period still requires low speed and floor-level height.
        p = s.data.xpos[self.piece_bodies]
        speed = np.linalg.norm(s.data.qvel[self.linear_dofs], axis=1)
        inside = np.all(np.abs(p[:,:2]-self.bin_xy) < np.asarray(self.size)-.004, axis=1)
        return int(np.count_nonzero((s.data.time-self.last_support<=.020) & inside &
                                    (p[:,2]>.150) & (p[:,2]<.157) & (speed<.04)))

    def rear_corner_gaps(self):
        # Project each end of the drawer's inclined rear wall onto the stop normal.
        axis = self.back_wall.xmat.reshape(3, 3)[1]
        reach = abs(axis[1]) * self.wall_half[1] + abs(axis[2]) * self.wall_half[2]
        centre_gap = self.stop_face - self.back_wall.xpos[1] - reach
        skew = axis[0] * self.wall_half[0]
        return centre_gap - skew, centre_gap + skew

    def resting_on_chassis(self, s):
        a, b = s.data.contact.geom1, s.data.contact.geom2
        return bool(np.any(((a == self.drawer_floor) & (b == self.chassis)) |
                           ((b == self.drawer_floor) & (a == self.chassis))))

    def after_step(self, s):
        p=s.position("drawer")
        if p[1]<self.origin[1]-.048 and abs(p[2]-self.origin[2])<.014:
            self.left_guides=True
        self.clearance = max(self.clearance,p[2]-self.origin[2])
        self.max_tilt = max(
            self.max_tilt, math.acos(np.clip(s.data.body("drawer").xmat[8], -1, 1))
        )
        self.settled = self.settled + s.model.opt.timestep if self.count(s) == 18 else 0.0
        part = s.data.body("drawer")
        in_seat = np.linalg.norm(part.xpos - self.origin) < 0.010 and np.trace(
            part.xmat.reshape(3, 3)
        ) > 1 + 2 * math.cos(math.radians(10))
        clear = all(
            np.linalg.norm(s.data.site(f"{side}_tcp").xpos - s.data.geom("handle").xpos)
            > 0.065
            for side in ("right", "left")
        )
        docked = False
        if self.settled and in_seat and clear and self.resting_on_chassis(s):
            gaps = self.rear_corner_gaps()
            docked = min(gaps) >= -0.001 and max(gaps) <= 0.002
        self.docked = self.docked + s.model.opt.timestep if docked else 0.0

    def measure(self, s):
        n = int(self.count(s))
        return dict(
            score=(
                (
                    1.0
                    if self.settled >= 0.5 and self.docked >= 0.4
                    else min(n / 18, 0.75)
                )
                if self.left_guides and self.clearance > 0.035 and self.max_tilt > math.radians(45)
                else 0.0
            ),
            debris_in_bin=n,
            cleared_guides=self.left_guides,
            tray_reseated=self.docked >= 0.4,
            rear_gaps_mm=[round(1000 * gap, 2) for gap in self.rear_corner_gaps()],
            chassis_support=self.resting_on_chassis(s),
            tray_lift_mm=round(1000 * self.clearance, 1),
            tray_tilt_deg=round(math.degrees(self.max_tilt), 1),
            settled_s=round(self.settled, 2),
        )

    def demo(self, s, incomplete=False, *, short_return=False):
        arm = "right"
        x, y, z = s.data.geom("handle").xpos.copy()
        s.move_arm(
            arm,
            (x, y, z + 0.084),
            1.2,
            "Approach the lift-out tray",
            opening=0.90,
            yaw=1.5708,
        )
        s.move_arm(
            arm,
            (x, y, z + 0.002),
            0.9,
            "Lower the fingers around the orange grip",
            yaw=1.5708,
        )
        s.move_arm(
            arm,
            (x, y, z + 0.002),
            0.65,
            "Pinch the tray handle",
            opening=0.02,
            yaw=1.5708,
        )
        s.move_arm(arm,(x,y-.064,z+.002),1.40,
                   "Slide the loaded tray clear of both guide lips",yaw=1.5708)
        s.move_arm(
            arm,
            (x, y-.064, z + 0.094),
            0.88,
            "Lift the tray clear of the roller head",
            yaw=1.5708,
        )
        s.move_arm(
            arm,
            (x, -0.198, z + 0.094),
            0.85,
            "Carry the tray over the waste bin",
            yaw=1.5708,
        )
        if incomplete:
            s.wait(1.4, "Hold the loaded tray level")
            return
        start = s.data.site("right_tcp").xmat.reshape(3, 3).copy()
        goal = s.data.site("right_tcp").xpos.copy()
        targets = s.targets.copy()
        wrist_path = []
        for i in range(1, 43):
            angle = 1.08 * i / 42
            c, si = math.cos(angle), math.sin(angle)
            R = np.array([[1, 0, 0], [0, c, -si], [0, si, c]]) @ start
            angles, _, _ = s.solve(arm, goal, initial=targets[1, :6], rotation=R)
            targets[1, :6] = angles
            s.phase = "Tip the debris into the waste bin"
            s.action(targets)
            wrist_path.append(targets.copy())
        s.wait(1.6, "Let rice, lentils and flakes settle")
        s.phase = "Turn the empty tray upright"
        for pose in reversed(wrist_path):
            s.action(pose)
        s.move_arm(
            arm,
            (x, y-.064, z + 0.094),
            1.1,
            "Return the empty tray above the sweeper",
            yaw=1.5708,
        )
        s.move_arm(
            arm, (x, y-.064, z + 0.002), 1.1, "Lower the tray onto the runners", yaw=1.5708
        )
        return_y = y - 0.001 if short_return else y + 0.009
        s.move_arm(arm, (x, return_y, z + 0.002), 1.55,
                   "Stop the drawer short" if short_return else "Slide the tray home beneath both guide lips",
                   yaw=1.5708)
        s.move_arm(
            arm, (x, return_y, z + 0.002), 0.45, "Release the tray", opening=0.95, yaw=1.5708
        )
        s.move_arm(arm, (x, return_y, z + 0.085), 0.85, "Withdraw the gripper", yaw=1.5708)
        s.wait(0.5, "Verify the empty tray and waste bin")
