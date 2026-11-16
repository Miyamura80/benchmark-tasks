"""Use a bent brush to reach and clean two shielded patches under a toilet lip."""
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
    "Use the angled brush to remove both brown marks from the underside of the rim. "
    "Lower the head through the opening and reach behind the ceramic lip. Return the brush upright."
)
CASES = ("nominal", "air-pass", "exposed-only")
CASE = os.environ.get("ROBOCURVE_FIXTURE_CASE", "nominal")

CENTER = (.263, -.124)
RIM = TOP + .134
BRUSH_LENGTH = .057                 # Grip centre to bristle centre
BRISTLE_RADIUS = .015
BRUSH_OFFSET = .031
MIN_IMPULSE_NS = .00005
SCRUB_TRAVEL_M = .050
HOLDER = (.155, -.306, TOP + .094)
LIFT = RIM + BRUSH_LENGTH + .024
# Geometry from waterline to inner lip, followed by the outer ceramic.
INNER_PROFILE = (
    (.058, .049, RIM - .081), (.064, .055, RIM - .071), (.096, .074, RIM - .058),
    (.119, .094, RIM - .039), (.142, .108, RIM - .018),
    (.160, .129, RIM - .020), (.135, .095, RIM - .014), (.135, .095, RIM + .004),
)
# The vitreous-china shell descends from the rounded rim into a taller,
# narrow trapway and widens again at its floor mount. Each row includes its
# centre shift toward the cistern so the front has a clear undercut.
OUTER_PROFILE = (
    (.164, .128, RIM + .006, 0), (.182, .144, RIM + .004, 0),
    (.191, .151, RIM - .004, 0), (.193, .152, RIM - .017, 0),
    (.185, .144, RIM - .030, .004), (.169, .127, RIM - .043, .010),
    (.145, .106, RIM - .058, .021), (.113, .083, RIM - .076, .045),
    (.105, .078, RIM - .093, .075), (.108, .078, TOP + .027, .080),
    (.119, .081, TOP + .017, .082), (.133, .089, TOP + .005, .080),
    (.136, .090, TOP + .001, .079),
)
# Exposed control marks and sole scored targets, respectively.
EXPOSED = ((-2.52, 3.), (-1.45, 3.))
HIDDEN = ((-2.52, 5.55), (-1.45, 5.55))
STAINS = EXPOSED + HIDDEN
ANGULAR_OFFSETS = (-.15, -.075, 0, .075, .15)
TARGET = .92
PORCELAIN = (.91, .93, .92)
SOIL = (.24, .095, .031)
SEAT_COLOR = (.97, .97, .96)
TANK_CENTER = (.502, CENTER[1])

OBJECTS = [
    Box("bowl_fixture", (0., 0., 0.), size=(.01, .01, .01)),
    Box("angled_brush", HOLDER, size=(.018, .022, .034), mass=.042),
]


def configure_demo(case):
    global CASE
    CASE = case


def surface(row, theta):
    i = int(row)
    f = row - i
    profile = np.asarray(INNER_PROFILE)
    a, b, z = (1-f)*profile[i] + f*profile[min(i+1, len(profile)-1)]
    return np.array([CENTER[0] + a * cos(theta), CENTER[1] + b * sin(theta), z])


def brush_pose(row, theta, air=False):
    patch = surface(row, theta)
    radial = patch[:2] - CENTER
    normal = radial / np.linalg.norm(radial)
    yaw = float(np.arctan2(*normal[::-1]))
    # Get behind the lip from below, with the stem inside the opening.
    depth = BRISTLE_RADIUS - .002 if row > 5 else .004
    inset = BRUSH_OFFSET + (.003 if row > 5 else BRISTLE_RADIUS - .002)
    if air:
        depth += .025
        inset += .020
    return (*tuple(patch[:2] - inset*normal), patch[2] + BRUSH_LENGTH - depth), yaw


def demo():
    closed, opened = FINGER_CLOSED, FINGER_OPEN
    park = (.25, -.245, TOP + .190)
    right = [
        Waypoint(2., (*HOLDER[:2], LIFT), opened, "approach"),
        Waypoint(1.4, HOLDER, opened, "lower to collar"),
        Waypoint(.9, HOLDER, closed, "pinch collar"),
        Waypoint(1.3, (*HOLDER[:2], LIFT), closed, "lift"),
    ]
    targets = EXPOSED if CASE == "exposed-only" else HIDDEN
    for centre, row in targets:
        start, yaw = brush_pose(row, centre-.18, CASE == "air-pass")
        inward = np.array([cos(yaw), sin(yaw)]) * .030
        entry = (*tuple(np.array(start[:2])-inward), start[2]-.012)
        right.extend([
            Waypoint(1.5, (*entry[:2], LIFT), closed, "centre over opening", yaw),
            Waypoint(1.3, entry, closed, "lower inside opening", yaw),
            Waypoint(1.0, (*start[:2], start[2]-.010), closed, "reach behind lip", yaw),
        ])
        for offset in (-.18, -.09, 0., .09, .18, .09, 0., -.09, -.18, -.09, 0., .09, .18):
            point, angle = brush_pose(row, centre + offset, CASE == "air-pass")
            right.append(Waypoint(.48, point, closed, "scrub with loaded head", angle))
        end, angle = brush_pose(row, centre+.18, CASE == "air-pass")
        inward = np.array([cos(angle), sin(angle)]) * .032
        exit_xy = tuple(np.array(end[:2])-inward)
        right.extend([
            Waypoint(.75, (*end[:2], end[2]-.015), closed, "unload head", angle),
            Waypoint(.8, (*exit_xy, end[2]-.015), closed, "withdraw behind lip", angle),
            Waypoint(.9, (*exit_xy, LIFT), closed, "lift out", angle),
        ])
    right.extend([
        Waypoint(1.7, (*HOLDER[:2], LIFT), closed, "over holder"),
        Waypoint(1.25, HOLDER, closed, "lower into holder"),
        Waypoint(.85, HOLDER, opened, "release"),
        Waypoint(1.3, park, opened, "clear holder"),
    ])
    left_park = (.25, .235, TOP + .178)
    return {"right": right, "left": [Waypoint(2., left_park, opened, "clear work area"),
                                     Waypoint(sum(p.duration for p in right), left_park, opened, "hold")]}


class State:
    def __init__(self, case="nominal"):
        self.previous = None
        self.dirt = np.ones((len(STAINS), len(ANGULAR_OFFSETS)))
        self.patches = np.array([[surface(row, centre + offset) for offset in ANGULAR_OFFSETS]
                                 for centre, row in STAINS])

    def advance(self, brush, rotation, contacts):
        head = brush.pos + rotation @ np.array([BRUSH_OFFSET, 0., -BRUSH_LENGTH])
        tips = set()
        loaded = []
        for pair, point, impulse in contacts:
            if "BrushHandle" in pair:
                tips.update(pair & {"tip_left", "tip_right"})
            if pair == {"BrushBristles", "CeramicSurface"} and impulse >= MIN_IMPULSE_NS:
                loaded.append(point)
        held = len(tips) == 2
        self.scrub(head, loaded if held else ())
        park_error = np.linalg.norm(brush.pos - HOLDER)
        parked = (not tips and park_error < .014 and rotation[2, 2] >= .96
                  and brush.resting(max_speed=.025, max_spin=.25))
        cleaned = 1 - self.dirt.mean(axis=1)
        hidden = cleaned[len(EXPOSED):]
        complete = bool(np.all(hidden >= TARGET))
        return dict(score=1. if complete and parked else round(float(.4*np.sum(hidden)), 4),
                    exposed_percent=np.round(100*cleaned[:len(EXPOSED)], 1).tolist(),
                    hidden_percent=np.round(100*hidden, 1).tolist(),
                    brush_holder=bool(parked), held=held, loaded_contacts=len(loaded),
                    brush_xyz=np.round(head, 5).tolist(),
                    caption="Under-rim marks clean" if complete else "Scrub behind the rim")


    def scrub(self, head, contacts):
        before, self.previous = self.previous, head
        if len(contacts) == 0 or before is None:
            return
        distance = float(np.linalg.norm((head - before)[:2]))
        if not .0003 <= distance <= .025:
            return
        gaps = np.linalg.norm(self.patches[:, :, None, :] - np.asarray(contacts), axis=3)
        loaded = np.any(gaps <= .014, axis=2)
        self.dirt = np.maximum(0., self.dirt - distance/SCRUB_TRAVEL_M*loaded)


def rings(profiles, count=96):
    points = []
    for a, b, z, *shift in profiles:
        offset = shift[0] if shift else 0.
        for i in range(count):
            theta = i * 2 * pi / count
            points.append((CENTER[0] + offset + a*cos(theta),
                           CENTER[1] + b*sin(theta), z))
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
        for index, side in enumerate((-1, 1)):
            g.cylinder(f"SeatHinge{index}", (.432, CENTER[1] + side*.065, RIM + .009),
                       .009, .026, SEAT_COLOR, axis="Y")
            g.sphere(f"BoltCover{index}", (.348, CENTER[1] + side*.073, TOP + .011),
                     (.010,.007,.004), PORCELAIN)
        g.mesh("RearDeck", *rounded_housing((.442, CENTER[1]), (
            (RIM - .096, .067, .114, .025), (RIM - .072, .096, .132, .027),
            (RIM - .039, .125, .173, .029), (RIM - .012, .144, .209, .027),
            (RIM + .006, .139, .211, .027),
        )), PORCELAIN)
        g.mesh("RearCistern", *rounded_housing(TANK_CENTER, (
            (RIM + .002, .087, .215, .022), (RIM + .015, .105, .253, .026),
            (RIM + .042, .121, .276, .031), (RIM + .119, .128, .289, .032),
            (RIM + .174, .128, .289, .030), (RIM + .181, .125, .287, .029),
        )), PORCELAIN)
        g.mesh("CisternLidSeam", *rounded_housing(TANK_CENTER, (
            (RIM + .181, .124, .286, .029), (RIM + .183, .125, .287, .029),
        )), (.61, .64, .63))
        g.mesh("OverhangingCisternLid", *rounded_housing(TANK_CENTER, (
            (RIM + .183, .131, .293, .032), (RIM + .187, .138, .300, .033),
            (RIM + .193, .136, .298, .033), (RIM + .201, .125, .286, .030),
        )), SEAT_COLOR)
        g.cylinder("FlushButtonCollar", (*TANK_CENTER, RIM + .202), .015, .003, (.47, .49, .50))
        g.cylinder("FlushButton", (*TANK_CENTER, RIM + .204), .011, .003, (.75, .78, .79))
        g.sphere("Water", (*CENTER, RIM - .075), (.059, .050, .002), (.39, .65, .70))
        self.stains = []
        for i, (centre, row) in enumerate(STAINS):
            cluster = []
            for j, offset in enumerate(ANGULAR_OFFSETS):
                theta = centre + offset
                points = [surface(row+dr, theta+dt) + (0, 0, -.0006 if row > 5 else .0006)
                          for dr, dt in ((-.24, -.039), (-.24, .039), (.24, .039), (.24, -.039))]
                cluster.append(g.mesh(f"Deposit{i}_{j}", points, [(0, 1, 2, 3)], SOIL))
            self.stains.append(cluster)
        self.update(state)

    def update(self, state):
        g = self.g
        for i, cluster in enumerate(self.stains):
            for j, patch in enumerate(cluster):
                remaining = float(state.dirt[i, j])
                g.show(patch, remaining > .06)
                color = tuple(base * (1 - remaining) + dirty * remaining for base, dirty in zip(PORCELAIN, SOIL))
                g.color(patch, color)
