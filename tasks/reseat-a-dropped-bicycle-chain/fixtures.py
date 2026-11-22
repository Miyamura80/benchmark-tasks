"""A closed roller chain with constrained pitch and independent toothed sprockets."""

import math
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from station import element, geom, station_xml

PITCH = 0.01270
ROLLER = 0.00385
TEETH = (28, 16)
RADII = PITCH / (2 * np.sin(np.pi / np.array(TEETH)))
SPAN = 32
AXLES = np.array(
    [
        [-0.008, -0.048, 0.271],
        [
            -0.008,
            -0.048 + math.sqrt((SPAN * PITCH) ** 2 - np.diff(RADII)[0] ** 2),
            0.271,
        ],
    ]
)
DROP = 0.013
STEEL = (0.58, 0.64, 0.68, 1)
DARK = (0.17, 0.20, 0.22, 1)
BRASS = (0.79, 0.52, 0.16, 1)
XAXIS = (0, math.pi / 2, 0)
VISUAL = dict(contype=0, conaffinity=0, mass=0)
WHEEL = dict(
    contype=4,
    conaffinity=11,
    condim=3,
    solref=".003 1",
    solimp=".98 .998 .0003",
    friction=".27 .001 .0001",
)
LINK = dict(
    contype=2,
    conaffinity=13,
    condim=3,
    solref=".003 1",
    solimp=".98 .998 .0003",
    friction=".35 .001 .0001",
)
PLATE = dict(
    contype=8,
    conaffinity=7,
    condim=3,
    solref=".003 1",
    solimp=".98 .998 .0003",
    friction=".35 .001 .0001",
)


def make_loop(drop=DROP):
    arcs = []
    for j, (center, radius, teeth) in enumerate(zip(AXLES, RADII, TEETH)):
        start = math.pi / 2 if j == 0 else -math.pi / 2
        angles = start + np.arange(teeth // 2 + 1) * math.tau / teeth
        points = np.tile(center, (len(angles), 1))
        points[:, 1] += radius * np.cos(angles)
        points[:, 2] += radius * np.sin(angles)
        if j == 0:
            points[:, 0] += drop
        arcs.append(points)
    front, rear = arcs
    return np.vstack(
        (
            front[:-1],
            np.linspace(front[-1], rear[0], SPAN, endpoint=False),
            rear[:-1],
            np.linspace(rear[-1], front[0], SPAN, endpoint=False),
        )
    )


def wheel(world, name, center, radius, teeth, index, rear_drag):
    part = element(world, "body", name=name, pos=center)
    element(
        part,
        "joint",
        name=name + "_angle",
        type="hinge",
        axis=(1, 0, 0),
        limited="false",
        frictionloss=rear_drag if index else 0.003,
        damping=0.0006,
        armature=0.00014,
    )
    geom(
        part,
        name + "_root",
        "cylinder",
        (0, 0, 0),
        (radius - 0.0041, 0.0012),
        DARK,
        euler=XAXIS,
        mass=0.10 if not index else 0.055,
        group=5,
        **WHEEL,
    )
    # The contact root is a solid disk. Render the stamped rim and spider.
    for k in range(teeth):
        a = math.pi / 2 + k * math.tau / teeth
        ring_r = radius - 0.007
        geom(
            part,
            f"{name}_rim_{k}",
            "box",
            (0, ring_r * math.cos(a), ring_r * math.sin(a)),
            (0.0012, 0.0045, ring_r * math.tan(math.pi / teeth) + 0.0003),
            STEEL,
            euler=(a, 0, 0),
            **VISUAL,
        )
    for k in range(5):
        a = k * math.tau / 5
        radial = (radius - 0.006) / 2
        geom(
            part,
            f"{name}_spoke_{k}",
            "box",
            (0, radial * math.cos(a), radial * math.sin(a)),
            (0.0015, radial, 0.0032),
            DARK,
            euler=(a, 0, 0),
            **VISUAL,
        )
        geom(
            part,
            f"{name}_bolt_{k}",
            "cylinder",
            (-0.0022, 0.68 * radius * math.cos(a), 0.68 * radius * math.sin(a)),
            (0.0026, 0.0013),
            STEEL,
            euler=XAXIS,
            **VISUAL,
        )
    for k in range(teeth):
        angle = math.pi / 2 + (k + 0.5) * math.tau / teeth
        pos = (
            0,
            (radius - 0.0005) * math.cos(angle),
            (radius - 0.0005) * math.sin(angle),
        )
        geom(
            part,
            f"{name}_tooth_{k}",
            "box",
            pos,
            (0.00130, 0.0042, 0.0018),
            STEEL,
            euler=(angle, 0, 0),
            mass=0.0004,
            **WHEEL,
        )
    # A contrasting physical index on each hub makes manual rotation counts visible.
    geom(
        part,
        name + "_index",
        "box",
        (-0.002, 0, 0.50 * radius),
        (0.0005, 0.0016, 0.37 * radius),
        BRASS,
        **VISUAL,
    )
    geom(
        part,
        name + "_hub",
        "cylinder",
        (0, 0, 0),
        (0.009, 0.009),
        STEEL,
        euler=XAXIS,
        **WHEEL,
    )
    if index == 1:
        for side in (-1, 1):
            geom(
                part,
                f"keeper_{side}",
                "cylinder",
                (side * 0.0060, 0, 0),
                (radius + 0.0045, 0.0008),
                (0.45, 0.50, 0.51, 0.12),
                euler=XAXIS,
                mass=0.01,
                **WHEEL,
            )
    if index == 0:
        # The bench crank uses a compact counterweight inside the tooth circle.
        geom(
            part,
            "crank_counterweight",
            "cylinder",
            (-0.015, 0.026, 0),
            (0.012, 0.004),
            DARK,
            euler=XAXIS,
            mass=0.13385,
            **WHEEL,
        )
        geom(
            part,
            "crank_axle",
            "cylinder",
            (-0.013, 0, 0),
            (0.007, 0.016),
            STEEL,
            euler=XAXIS,
            **VISUAL,
        )
        geom(
            part,
            "crank_arm",
            "capsule",
            (-0.023, -0.029, 0),
            (0.005, 0.029),
            STEEL,
            euler=(math.pi / 2, 0, 0),
            mass=0.07,
            **WHEEL,
        )
        geom(
            part,
            "pedal",
            "cylinder",
            (-0.084, -0.058, 0),
            (0.008, 0.056),
            BRASS,
            euler=XAXIS,
            mass=0.025,
            **WHEEL,
        )


class Chain:
    title = "Reseat a dropped bicycle chain"
    instruction = "Guide the dropped chain onto the front teeth and turn the crank two full turns with the rear sprocket driven throughout."
    focus = (-0.008, 0.035, 0.273)

    def __init__(
        self,
        variant=0,
        *,
        drop=DROP,
        rear_drag=0.002,
        lift_pick=False,
    ):
        self.lift_pick = lift_pick
        self.points = make_loop(drop)
        self.rear_drag = rear_drag
        self.variant = variant

    def compile(self):
        tree, world = station_xml()
        base = tree.find(".//body[@name='left_base']")
        base.set("pos", ".320 .115 .175")
        base.set("quat", "0 0 0 1")
        if self.lift_pick:
            base.set("pos", "-0.29 0.13 0.29")
            base.set("quat", "1 0 0 0")
            geom(
                world,
                "pick_arm_riser",
                "box",
                (-0.29, 0.13, .2125),
                (0.075, 0.065, 0.0575),
                DARK,
            )
        tree.find(".//geom[@name='pedestal_1']").set("pos", "-0.29 0.13 .077")
        tree.find(".//body[@name='right_base']").set("pos", "-.286 -.180 .175")
        tree.find(".//geom[@name='pedestal_-1']").set("pos", "-.286 -.180 .077")
        opt = tree.find("option")
        opt.set("jacobian", "sparse")
        opt.set("iterations", "45")
        opt.set("tolerance", "1e-8")
        # Keep inertias of small links explicit; each pin has a roller and two plates.
        w = tree.find("worldbody")
        contact = tree.find("contact")
        # Slip-on pedal socket. Four rigid walls capture the pedal spindle while
        # leaving its axis open, so insertion and withdrawal require real motion.
        carrier = tree.find(".//body[@name='right_grasp_site']")
        for axis in (1, 2):
            for sign in (-1, 1):
                c = np.array([0.0, 0.0, 0.040])
                c[axis] += sign * 0.0115
                half = np.array([0.007, 0.0135, 0.0135])
                half[axis] = 0.0020
                geom(
                    carrier,
                    f"socket_{axis}_{sign}",
                    "box",
                    c,
                    half,
                    STEEL,
                    mass=0.003,
                    solref=".003 1",
                )
        geom(
            carrier,
            "socket_stem",
            "box",
            (0, 0, 0.017),
            (0.004, 0.006, 0.011),
            DARK,
            mass=0.012,
        )
        # Two removable parallel jaws. Offsets below are in the TCP frame
        # at zero opening. Each jaw follows its own stock finger slide.
        for sign, name, finger_origin, axes in [
            (1, "left_tip_left", [-.0238981, -.0450619, -.0801401],
             [[0,1,0],[0,0,-1],[-1,0,0]]),
            (-1, "left_tip_right", [.0238981, .0450619, -.0801401],
             [[1,0,0],[0,0,1],[0,-1,0]]),
        ]:
            finger = tree.find(f".//body[@name='{name}']")
            rotation = np.asarray(axes)
            def local(p):
                return rotation.T @ (np.asarray(p) - finger_origin)
            pad = np.array([-.085, .108 + sign * .0023, .075])
            shoulder = pad + [0, sign*.004, -.018]
            root = np.array([0, sign*.009, -.006])
            middle = np.array([-.048, .094 + sign*.009, .012])
            for k, (a, b) in enumerate(((root,middle),(middle,shoulder),(shoulder,pad))):
                element(finger, "geom", name=f"tong_{sign}_{k}", type="capsule",
                        fromto=(*local(a),*local(b)), size=(.002,), rgba=STEEL, mass=.0015)
            rotation_q = np.zeros(4)
            mj.mju_mat2Quat(rotation_q, rotation.T.ravel().astype(float))
            geom(finger, f"tong_pad_{sign}", "box", local(pad),
                 (.0075,.0018,.003), DARK, quat=rotation_q, mass=.0006,
                 friction="1.0 .005 .0001", solref=".003 1")
        for name in ("front", "rear"):
            element(contact, "exclude", body1="workbench_fixture", body2=name)
        for k, (name, pos, r, n) in enumerate(
            zip(("front", "rear"), AXLES, RADII, TEETH)
        ):
            wheel(w, name, pos, r, n, k, self.rear_drag)
        # Fixed rear triangle, with the drive plane at x=-8 mm.
        # Tubes, stays and bottom bracket are ordinary collision geometry.
        front, rear = AXLES
        bottom = front + [0.045, 0, 0]
        seat = bottom + [0, 0.080, 0.320]
        head = bottom + [0, -0.300, 0.295]
        paint = (0.14, 0.38, 0.44, 1)
        def tube(name, start, end, radius):
            element(world, "geom", name=name, type="capsule",
                    fromto=(*start, *end), size=(radius,), rgba=paint)
        geom(world, "bottom_bracket_shell", "cylinder", bottom, (0.0205, 0.034),
             STEEL, euler=XAXIS)
        tube("seat_tube", bottom, seat, .014)
        tube("down_tube", bottom, head, .018)
        tube("top_tube", seat, head, .013)
        for side in (-1, 1):
            dropout = rear + [0.045 + side * 0.062, 0, 0]
            tube(f"chainstay_{side}", bottom + [side*.026,.010,-.004],
                 dropout, .008)
            tube(f"seatstay_{side}", seat, dropout, .006)
        geom(world, "rear_axle", "cylinder", rear+[.045,0,0],
             (.005,.067), STEEL, euler=XAXIS)
        geom(world, "stand", "box", rear+[.130,0,-.140],
             (.013,.025,.140), DARK)
        tendon = tree.find("tendon")
        if tendon is None:
            tendon = element(tree, "tendon")
        for i, p in enumerate(self.points):
            b = element(w, "body", name=f"link{i}", pos=p)
            for axis, vec in zip("xyz", np.eye(3)):
                element(
                    b,
                    "joint",
                    name=f"pin{i}_{axis}",
                    type="slide",
                    axis=vec,
                    limited="false",
                    damping=0.013,
                    armature="0",
                )
            element(
                b,
                "inertial",
                pos=(0, 0, 0),
                mass=0.0033,
                diaginertia=(7e-8, 3e-8, 7e-8),
            )
            geom(
                b,
                f"roller{i}",
                "cylinder",
                (0, 0, 0),
                (ROLLER, 0.00160),
                BRASS,
                euler=XAXIS,
                **LINK,
            )
            element(b, "site", name=f"pin{i}", size=".0001", rgba="0 0 0 0")
            for sign in [-1, 1]:
                geom(
                    b,
                    f"cheek{i}_{sign}",
                    "cylinder",
                    (sign * 0.00255, 0, 0),
                    (0.0041, 0.00065),
                    STEEL if i % 2 else DARK,
                    euler=XAXIS,
                    **PLATE,
                )
                element(
                    b,
                    "site",
                    name=f"plate{i}_{sign}",
                    pos=(sign * 0.00255, 0, 0),
                    size=".0001",
                    rgba="0 0 0 0",
                )
            j = (i + 1) % len(self.points)
            element(contact, "exclude", body1=f"link{i}", body2=f"link{j}")
            pitch = element(
                tendon,
                "spatial",
                name=f"pitch{i}",
                limited="true",
                range=(PITCH - 0.000055, PITCH + 0.000055),
                springlength=PITCH,
                stiffness=30,
                damping=0.009,
                solreflimit=".003 1",
                solimplimit=".985 .998 .00005",
                width=".00015",
                rgba="0 0 0 0",
            )
            for k in (i, j):
                element(pitch, "site", site=f"pin{k}")
            for sign in [-1, 1]:
                plate = element(
                    tendon, "spatial", width=".0017", rgba=STEEL if i % 2 else DARK
                )
                for k in (i, j):
                    element(plate, "site", site=f"plate{k}_{sign}")
            # Limit jackknifing at a pin; a bicycle chain can articulate through the
            # 22.5-degree rear tooth pitch but cannot fold back through its neighbour.
            elbow = element(
                tendon,
                "spatial",
                limited="true",
                range=(0.0230, 0.0262),
                solreflimit=".006 1",
                width=".0001",
                rgba="0 0 0 0",
            )
            for k in (i, (i + 2) % len(self.points)):
                element(elbow, "site", site=f"pin{k}")
        return mj.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"))

    def reset(self, s):
        self.ids = np.array(
            [s.model.body(f"link{i}").id for i in range(len(self.points))]
        )
        self.xdofs = s.model.body_dofadr[self.ids].copy()
        # Park both arms on the drive side, with their tools separated.
        for side, pos in [
            ("right", AXLES[0] + [-0.165, -0.058, 0.040]),
            ("left", AXLES[0] + [-0.125, 0.190, 0.120]),
        ]:
            idx = ("left", "right").index(side)
            options = [(yaw, s.solve(side, pos, yaw)) for yaw in ((math.pi/2,) if side=="left" else (0, math.pi))]
            if side == "left":
                rng = np.random.default_rng(32)
                limits = np.array([j.range for j in s.joints[side]])
                for _ in range(60):
                    yaw = math.pi / 2
                    start = rng.uniform(limits[:,0]*.85, limits[:,1]*.85)
                    options.append((yaw,s.solve(side,pos,yaw,start)))
                    if options[-1][1][1] < .001 and options[-1][1][2] < .02:
                        break
            yaw, (angles, error, _) = min(
                options, key=lambda pair: pair[1][1] + 0.03 * pair[1][2]
            )
            if error > 0.004:
                raise RuntimeError(f"{side} parking pose is unreachable")
            s.yaw_choices[side] = (0, yaw)
            s.yaw_targets[side] = yaw
            s.targets[idx, :6] = angles
            s.targets[idx, 6] = 0.12
            for j, a in zip(s.joints[side], angles):
                s.data.qpos[j.qposadr[0]] = a
            for finger in s.fingers[side]:
                s.data.joint(finger).qpos[0] = 0.0475 * 0.12
        s.set_targets(s.targets)
        self.visited = set()
        self.started_dropped = False
        self.pedal_id = s.model.geom("pedal").id
        self.socket_sides = [s.model.geom(f"socket_1_{sign}").id for sign in (-1, 1)]
        guide_parts = [f"tong_{sign}_{k}" for sign in (1,-1) for k in range(3)]
        guide_parts += ["tong_pad_1","tong_pad_-1"]
        self.guide_ids = [s.model.geom(name).id for name in guide_parts]
        self.origin = None
        self.turns = 0.0
        self.rear_turns = 0.0
        self.counts = (0, 0)

    def before_step(self, s):
        # Passive lateral plate bending. This internal force sums to zero and
        # depends only on neighboring links, independent of both sprockets.
        x = (
            s.data.qpos[s.model.jnt_qposadr[s.model.body_jntadr[self.ids]]]
            + self.points[:, 0]
        )
        v = s.data.qvel[self.xdofs]
        edge = 9.0 * x + 0.008 * v
        curvature = np.roll(edge, 1) + np.roll(edge, -1) - 2 * edge
        s.data.qfrc_applied[self.xdofs] = (
            2 * curvature - np.roll(curvature, 1) - np.roll(curvature, -1)
        )

    def observe_mesh(self, s):
        rollers = s.data.xpos[self.ids]
        counts = []
        for index, (p, r, n, name) in enumerate(
            zip(AXLES, RADII, TEETH, ("front", "rear"))
        ):
            d = rollers - p
            phase = (
                (np.arctan2(d[:, 2], d[:, 1]) - s.joint(name + "_angle") - math.pi / 2)
                * n
                / math.tau
            )
            pockets = np.rint(phase).astype(int)
            arc_error = np.abs(phase - pockets) * math.tau * r / n
            seated = (
                (abs(d[:, 0]) < 0.0028)
                & (abs(np.hypot(d[:, 1], d[:, 2]) - r) < 0.0025)
                & (arc_error < 0.0029)
            )
            counts.append(len(np.unique(pockets[seated] % n)))
            if index == 0:
                self.front_links = np.flatnonzero(seated)
        self.counts = tuple(counts)

    def sample(self, s):
        self.observe_mesh(s)
        if s.steps == 0 and self.counts[0] == 0 and self.counts[1] >= 3:
            self.started_dropped = True
        # The tool and chain must separate before the proof starts. Use the
        # distance from each jaw segment to the roller and cheek envelope.
        clearance = float("inf")
        for geom_id in self.guide_ids:
            local = (
                s.data.xpos[self.ids] - s.data.geom_xpos[geom_id]
            ) @ s.data.geom_xmat[geom_id].reshape(3, 3)
            size = s.model.geom_size[geom_id]
            if s.model.geom_type[geom_id] == mj.mjtGeom.mjGEOM_CAPSULE:
                local[:, 2] -= np.clip(local[:, 2], -size[1], size[1])
                distances = np.linalg.norm(local, axis=1) - size[0]
            else:
                distances = np.linalg.norm(np.maximum(abs(local) - size, 0), axis=1)
            # Bound the roller and both cheeks with an enclosing sphere.
            gap = distances.min() - .0053
            clearance = min(clearance, gap)
        angles = np.array([s.joint("front_angle"), s.joint("rear_angle")])
        ratio = TEETH[0] / TEETH[1]
        if self.origin is not None:
            self.turns, self.rear_turns = (angles - self.origin) / math.tau
        if (
            not self.started_dropped
            or self.counts[0] < 4
            or self.counts[1] < 3
            or clearance < 0.010
            or abs(self.rear_turns - ratio * self.turns) > 0.040
        ):
            self.origin = None
            self.visited.clear()
            self.turns = self.rear_turns = 0.0
        else:
            if self.origin is None:
                self.origin = angles.copy()
            self.visited.update(self.front_links)
        socket_x = s.data.geom_xpos[self.socket_sides, 0].mean()
        self.pedal_free = (
            abs(socket_x - s.data.geom_xpos[self.pedal_id, 0]) > 0.056 + 0.007 + 0.003
        )

    def proven(self):
        return (
            self.turns >= 2
            and self.rear_turns >= 3.5
            and len(self.visited) == len(self.ids)
            and self.counts[0] >= 8
            and self.counts[1] >= 5
        )

    def measure(self, s):
        f, r = self.counts
        score = 0.0
        if self.started_dropped and f >= 4 and r >= 3:
            score = 0.25
        if self.turns >= 0.25:
            score = 0.5
        if self.proven() and self.pedal_free:
            score = 1.0
        return dict(
            score=score,
            front_pockets=f,
            rear_pockets=r,
            crank_turns=round(self.turns, 4),
            driven_turns=round(self.rear_turns, 4),
            distinct_front_links=len(self.visited),
        )

    def control(self, s):
        s.wait(3, "Idle control")

    def detail_control(self, s):
        self.demo(s, partial=True)

    def demo(self, s, partial=False):
        from policy import reseat

        reseat(s, partial)
