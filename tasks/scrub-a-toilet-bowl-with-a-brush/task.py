"""Scrub four marked regions inside a toilet bowl and return the brush."""
from math import cos, pi, sin, sqrt

import os

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import Box, TABLE_TOP

from geom import Geometry

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MAX_STEPS = 1400
INSTRUCTION = (
    "Use the toilet brush to scrub away all four brown marks from inside the bowl, "
    "including the marks under the rim. Return the brush to its holder and let go."
)
CASES = ("nominal", "air-pass", "one-side")
CASE = os.environ.get("ROBOCURVE_FIXTURE_CASE", "nominal")

CENTER = (.263, -.124)
RIM = TOP + .101
BRUSH_LENGTH = .064                 # Grip centre to bristle centre
BRISTLE_RADIUS = .023
HOLDER = (.155, -.306, TOP + .112)
LIFT = RIM + BRUSH_LENGTH + .026
# Geometry from waterline to inner lip, followed by the outer ceramic.
INNER_PROFILE = (
    (.058, .049, RIM - .081), (.064, .055, RIM - .071), (.096, .074, RIM - .058),
    (.119, .094, RIM - .039), (.142, .108, RIM - .018),
    (.154, .118, RIM - .010), (.147, .111, RIM + .002),
)
OUTER_PROFILE = (
    (.162, .127, RIM + .006), (.179, .140, RIM + .005), (.190, .149, RIM - .003),
    (.193, .152, RIM - .013), (.177, .140, RIM - .047),
    (.141, .111, RIM - .078), (.112, .090, RIM - .099),
)
# Two stains per side, one under the lip and one on the sloping wall.
STAINS = (
    (-2.48, 5), (-1.02, 4), (1.02, 4), (2.48, 5),
)
ANGULAR_OFFSETS = (-.15, -.075, 0, .075, .15)
TARGET = .92
PORCELAIN = (.91, .90, .85)
SOIL = (.24, .095, .031)
SEAT_COLOR = (.97, .97, .95)
TANK_CENTER = (.487, CENTER[1])

OBJECTS = [
    Box("bowl_support", (*CENTER, TOP + .011), size=(.340, .222, .022),
        color=(.45, .49, .51)),
    Box("holder_pad", (*HOLDER[:2], TOP + .006), size=(.074, .074, .012),
        color=(.15, .27, .32)),
]


def configure_demo(case):
    global CASE
    CASE = case


def surface(row, theta):
    a, b, z = INNER_PROFILE[row]
    return np.array([CENTER[0] + a * cos(theta), CENTER[1] + b * sin(theta), z])


def brush_path(row, theta, air=False):
    patch = surface(row, theta)
    inset = .013 if not air else .063
    return tuple(patch + (-inset * cos(theta), -inset * sin(theta), BRUSH_LENGTH + .004))


def demo():
    c, o = FINGER_CLOSED, FINGER_OPEN

    def w(duration, pos, grip, name):
        return Waypoint(duration, pos, grip, name)

    def clean(indices, park):
        path = [
            w(1.9, (*HOLDER[:2], LIFT), o, "approach brush holder"),
            w(1.35, HOLDER, o, "reach handle"),
            w(.8, HOLDER, c, "grip handle"),
            w(1.2, (*HOLDER[:2], LIFT), c, "lift brush"),
        ]
        for index in indices:
            centre, row = STAINS[index]
            sweep = [.24, .12, 0, -.12, -.24, -.12, 0, .12, .24]
            start = brush_path(row, centre + sweep[0], CASE == "air-pass")
            path += [
                w(1.15, (*start[:2], LIFT), c, "move above stain"),
                w(.90, start, c, "lower brush to bowl wall"),
            ]
            path += [w(.34, brush_path(row, centre + offset, CASE == "air-pass"), c, "scrub along wall")
                     for offset in sweep[1:]]
            path.append(w(.85, (*start[:2], LIFT), c, "lift clear of rim"))
        path += [
            w(1.25, (*HOLDER[:2], LIFT), c, "return above holder"),
            w(1.25, HOLDER, c, "insert brush in holder"),
            w(.85, HOLDER, o, "release handle"),
            w(.9, (*HOLDER[:2], LIFT), o, "clear brush"),
            w(1.3, park, o, "retract"),
        ]
        return path

    right_park, left_park = (.25, -.235, TOP + .178), (.25, .235, TOP + .178)
    right = clean((0, 1) if CASE == "one-side" else (0, 1, 2, 3), right_park)
    left = [w(2, left_park, o, "park left arm"), w(sum(p.duration for p in right), left_park, o, "hold")]
    return {"right": right, "left": left}


class State:
    def __init__(self, case="nominal"):
        self.brush_grip = np.array(HOLDER)
        self.head = self.brush_grip - (0, 0, BRUSH_LENGTH)
        self.owner = None
        self.parked = True
        self.dirt = np.ones((len(STAINS), len(ANGULAR_OFFSETS)))
        self.patches = np.array([[surface(row, centre + offset) for offset in ANGULAR_OFFSETS]
                                 for centre, row in STAINS])

    def advance(self, hands, dt):
        if self.owner is None:
            for name, hand in hands.items():
                if hand.open_fraction < .30 and hand.upright and np.linalg.norm(hand.position - self.brush_grip) < .014:
                    self.owner = name
                    self.parked = False
                    break
        if self.owner is not None:
            h = hands[self.owner]
            previous = self.head
            self.brush_grip = h.position.copy()
            self.head = self.brush_grip + h.rotation @ np.array([0., 0., -BRUSH_LENGTH])
            self.scrub(previous, self.head, dt)
            if h.open_fraction >= .58:
                self.owner = None
                self.parked = bool(np.linalg.norm(self.brush_grip - HOLDER) < .016 and h.upright)
                if self.parked:
                    self.brush_grip = np.array(HOLDER)
                    self.head = self.brush_grip - (0, 0, BRUSH_LENGTH)
                else:
                    self.brush_grip[2] = TOP + .025
                    self.head = self.brush_grip + (-BRUSH_LENGTH, 0, 0)
        cleaned = 1 - self.dirt.mean(axis=1)
        complete = bool(np.all(cleaned >= TARGET))
        if complete and self.parked:
            score = 1.0
        else:
            score = round(float(np.sum(cleaned)) * .20, 4)
        return {
            "score": score, "patches_percent": [round(float(v) * 100, 1) for v in cleaned],
            "brush_holder": self.parked, "brush_arm": self.owner,
            "brush_xyz": [round(float(v), 5) for v in self.head],
            "caption": (
                "Brush returned and all four marks clean" if complete and self.parked else
                "Return the brush to its holder" if complete else
                f"{int(np.sum(cleaned >= TARGET))} of 4 marks scrubbed"
            ),
        }

    def scrub(self, before, after, dt):
        movement = after - before
        xy_length = float(np.linalg.norm(movement[:2]))
        if not .0013 <= xy_length <= .035:
            return
        # A brush must remain inside the bowl. A handle swung through the
        # exterior ceramic cannot remove a patch from its reverse side.
        delta = after[:2] - CENTER
        horizontal_radius = sqrt((delta[0] / .154) ** 2 + (delta[1] / .118) ** 2)
        if horizontal_radius > .975 or after[2] > RIM + .004 or after[2] < RIM - .063:
            return
        toward_patch = self.patches - before
        phase = np.clip(np.sum(toward_patch * movement, axis=2) / np.dot(movement, movement), 0., 1.)
        nearest = before + phase[:, :, None] * movement
        separation = np.linalg.norm(self.patches - nearest, axis=2)
        self.dirt = np.maximum(0., self.dirt - xy_length / .042 * (separation <= BRISTLE_RADIUS + .004))


def rings(profiles, count=80):
    points = [(CENTER[0] + a * cos(i * 2 * pi / count),
               CENTER[1] + b * sin(i * 2 * pi / count), z)
              for a, b, z in profiles for i in range(count)]
    faces = [(row * count + i, row * count + (i + 1) % count,
              (row + 1) * count + (i + 1) % count, (row + 1) * count + i)
             for row in range(len(profiles) - 1) for i in range(count)]
    return points, faces


def rounded_housing(center, levels):
    """Loft rounded rectangles with independent depth, width, and corner radii."""
    points = []
    for z, depth, width, radius in levels:
        for corner in range(4):
            angle = corner * pi / 2
            cx = center[0] + (depth / 2 - radius) * (1 if corner in (0, 3) else -1)
            cy = center[1] + (width / 2 - radius) * (1 if corner < 2 else -1)
            for index in range(9):
                theta = angle + pi * index / 16
                points.append((cx + radius * cos(theta), cy + radius * sin(theta), z))
    n = 36
    faces = [(row * n + i, row * n + (i + 1) % n,
              (row + 1) * n + (i + 1) % n, (row + 1) * n + i)
             for row in range(len(levels) - 1) for i in range(n)]
    faces.extend([tuple(reversed(range(n))), tuple(range(len(points) - n, len(points)))])
    return points, faces


class View:
    def __init__(self, state):
        g = self.g = Geometry()
        g.mesh("VitreousChinaBowl", *rings(INNER_PROFILE + OUTER_PROFILE), PORCELAIN)
        seat = ((.151, .115, RIM + .007), (.153, .117, RIM + .014),
                (.160, .124, RIM + .019), (.175, .137, RIM + .019),
                (.182, .144, RIM + .014), (.184, .146, RIM + .006))
        g.mesh("OpenSeatRing", *rings(seat), SEAT_COLOR)
        for side in (-1, 1):
            g.cylinder(f"SeatHinge{side}", (.426, CENTER[1] + side * .069, RIM + .012),
                       .014, .032, SEAT_COLOR, axis="Y")
        g.mesh("PorcelainPlinth", *rounded_housing(CENTER, (
            (TOP, .344, .226, .040), (TOP + .005, .352, .234, .038),
            (TOP + .023, .352, .234, .038), (TOP + .040, .294, .214, .037),
        )), PORCELAIN)
        g.box("CisternFoot", (*TANK_CENTER, TOP + .053), (.078, .165, .106), PORCELAIN)
        g.mesh("RearCistern", *rounded_housing(TANK_CENTER, (
            (TOP + .088, .073, .198, .019), (TOP + .100, .091, .230, .022),
            (TOP + .196, .097, .245, .023), (TOP + .207, .094, .244, .022),
        )), PORCELAIN)
        g.mesh("CisternLidSeam", *rounded_housing(TANK_CENTER, (
            (TOP + .207, .094, .244, .022), (TOP + .211, .095, .245, .022),
        )), (.53, .56, .55))
        g.mesh("OverhangingCisternLid", *rounded_housing(TANK_CENTER, (
            (TOP + .211, .102, .252, .024), (TOP + .216, .108, .257, .025),
            (TOP + .225, .106, .255, .024), (TOP + .232, .095, .243, .021),
        )), SEAT_COLOR)
        g.cylinder("FlushButtonCollar", (*TANK_CENTER, TOP + .233), .013, .003, (.44, .48, .50))
        g.cylinder("FlushButton", (*TANK_CENTER, TOP + .235), .009, .003, (.74, .77, .78))
        g.sphere("Water", (*CENTER, RIM - .075), (.059, .050, .002), (.13, .37, .43))
        g.cylinder("BrushStand", (*HOLDER[:2], TOP + .032), .030, .050, (.10, .23, .27))
        g.cylinder("HolderLiner", (*HOLDER[:2], TOP + .058), .026, .006, (.10, .13, .15))
        self.stains = []
        for i, (centre, row) in enumerate(STAINS):
            cluster = []
            a, b, z = INNER_PROFILE[row]
            for j, offset in enumerate(ANGULAR_OFFSETS):
                theta = centre + offset
                points = [(CENTER[0] + (a - .0026 - band * .005) * cos(theta + angular),
                           CENTER[1] + (b - .0026 - band * .005) * sin(theta + angular),
                           z - .004 - band * .009)
                          for band, angular in ((-.65, -.043), (-.73, .040), (.74, .042), (.85, -.039))]
                cluster.append(g.mesh(f"Deposit{i}_{j}", points, [(0, 1, 2, 3)], SOIL))
            self.stains.append(cluster)
        self.head = g.group("BrushHead")
        g.sphere("Bristles", (0, 0, 0), (.023, .023, .026), (.16, .32, .32), self.head)
        for i in range(12):
            theta = i * 2 * pi / 12
            g.sphere(f"BristleTuft{i}", (.019 * cos(theta), .019 * sin(theta), 0),
                     (.006, .006, .019), (.25, .41, .40), self.head)
        self.grip = g.group("BrushGrip")
        g.cylinder("RubberGrip", (0, 0, .067), .011, .169, (.12, .53, .54), self.grip)
        self.shaft = g.mesh("BrushShaft", [(0, 0, 0)] * 24,
                            [(i, (i + 1) % 12, (i + 1) % 12 + 12, i + 12) for i in range(12)],
                            (.57, .63, .62))
        self.update(state)

    def update(self, state):
        g = self.g
        g.move(self.head, state.head)
        g.move(self.grip, state.brush_grip)
        direction = state.brush_grip - state.head
        direction /= max(.0001, np.linalg.norm(direction))
        axis = np.cross(direction, (0, 1, 0))
        axis /= max(.0001, np.linalg.norm(axis))
        across = np.cross(direction, axis)
        points = [position + .0045 * (axis * cos(i * 2 * pi / 12) + across * sin(i * 2 * pi / 12))
                  for position in (state.head, state.brush_grip) for i in range(12)]
        g.points(self.shaft, points)
        for i, cluster in enumerate(self.stains):
            for j, patch in enumerate(cluster):
                remaining = float(state.dirt[i, j])
                g.show(patch, remaining > .06)
                color = tuple(base * (1 - remaining) + dirty * remaining for base, dirty in zip(PORCELAIN, SOIL))
                g.color(patch, color)
