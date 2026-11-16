"""Strip a used fitted sheet into a laundry zone and fit a clean one."""
from itertools import product
from math import pi

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
    "Change the bassinet sheet. Take the used coral sheet off the mattress and put it in the "
    "laundry zone. Fit the clean blue sheet over all four corners, then release it."
)
CASES = ("nominal", "miss-one-pocket", "leave-used-sheet")
CASE = os.environ.get("ROBOCURVE_FIXTURE_CASE", "nominal")

X0, X1, Y = .206, .481, .330
ZTOP, ZBOTTOM = TOP + .059, TOP + .015
# The four pocket hems run around the underside of the mattress.
SIGNS = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])
CORNERS = np.array([[X0, -Y], [X1, -Y], [X1, Y], [X0, Y]])
SEAT = np.column_stack((CORNERS + SIGNS * .008, np.full(4, ZBOTTOM + .004)))
CLEAN_FOLD = np.array([[.134, -.186, TOP + .022], [.149, -.235, TOP + .036],
                       [.149, .235, TOP + .036], [.134, .186, TOP + .022]])
HOVER = TOP + .109
PAD = .013
TAB_OFFSETS = np.column_stack((np.zeros(4), SIGNS[:, 1] * .037, np.zeros(4)))
GRID_X, GRID_Y = 7, 13
CORNER_I = [0, (GRID_X - 1) * GRID_Y, GRID_X * GRID_Y - 1, GRID_Y - 1]
LAUNDRY_END = .196
TOP_CORNER_I = [GRID_Y+1, (GRID_X-2)*GRID_Y+1, (GRID_X-2)*GRID_Y+GRID_Y-2, 2*GRID_Y-2]
TOP_CORNERS = np.column_stack((CORNERS - SIGNS * (.040, .048), np.full(4, ZTOP + .003)))

OBJECTS = [
    Box("bassinet_foam_mattress", ((X0 + X1) / 2, 0, (ZTOP + ZBOTTOM) / 2),
        size=(X1-X0, 2*Y, ZTOP-ZBOTTOM), color=(.81, .82, .82)),
    Box("clean_sheet_shelf", (.141, 0, TOP + .010), size=(.045, .526, .020),
        color=(.17, .31, .32)),
    Box("laundry_zone_pad", (.090, 0, TOP + .003), size=(.207, .600, .006),
        color=(.31, .34, .27)),
]


def configure_demo(case):
    global CASE
    CASE = case


def demo():
    c, o = FINGER_CLOSED, FINGER_OPEN

    def w(duration, point, grip, name, yaw=0.):
        return Waypoint(duration, tuple(point), grip, name, yaw)

    def approach(path, point, title, yaw=0.):
        opening = .55*o if yaw else o
        path.extend([
            w(1.35, (*point[:2], HOVER), opening, "approach " + title, yaw),
            w(1.20, point + (0, 0, PAD), opening, "lower to " + title, yaw),
            w(.72, point + (0, 0, PAD), c, "pinch " + title, yaw),
        ])

    def unhook(path, index):
        pocket = SEAT[index] + TAB_OFFSETS[index]
        dx, dy = SIGNS[index] * .030
        outside = pocket + (dx, dy, PAD)
        yaw = float(SIGNS[index,1] * pi / 2) if SIGNS[index,0] > 0 else 0.
        approach(path, pocket, "used pull tab", yaw)
        path.extend([
            w(.88, outside, c, "stretch elastic hem out", yaw),
            w(.87, (*outside[:2], HOVER), c, "lift used hem above mattress", yaw),
        ])
        return outside

    def install(path, index, skip=False):
        pocket = SEAT[index] + TAB_OFFSETS[index]
        dx, dy = SIGNS[index] * .025
        pickup = CLEAN_FOLD[index] + TAB_OFFSETS[index]
        yaw = float(SIGNS[index,1] * pi / 2) if SIGNS[index,0] > 0 else 0.
        approach(path, pickup, "clean pull tab", yaw)
        path.extend([
            w(.92, (*pickup[:2], HOVER), c, "lift clean pocket", yaw),
            w(1.55, (pocket[0]+dx, pocket[1]+dy, HOVER), c, "stretch clean pocket over corner", yaw),
            w(.94, (pocket[0]+dx, pocket[1]+dy, pocket[2]+PAD), c, "pull hem below mattress top", yaw),
            w(.77, pocket+(0, 0, PAD), c, "draw hem inward under corner", yaw) if not skip else
            w(.77, (pocket[0]+dx, pocket[1]+dy, HOVER), c, "leave pocket loose", yaw),
            w(.69, pocket+(0, 0, PAD), o, "let elastic pocket hold", yaw) if not skip else
            w(.69, (pocket[0]+dx, pocket[1]+dy, HOVER), o, "release loose pocket", yaw),
            w(.87, (pocket[0]+dx, pocket[1]+dy, HOVER), o, "clear seated pocket", yaw),
        ])

    result = {}
    for side, near, far, sign in (("right", 0, 1, -1.), ("left", 3, 2, 1.)):
        path = []
        park = (.18, sign * .274, HOVER)
        if CASE != "leave-used-sheet":
            p = unhook(path, near)
            path += [
                w(.90, (.295, p[1] - sign * .055, HOVER), c, "fold released near pocket onto sheet"),
                w(.72, (.295, p[1] - sign * .055, HOVER), o, "release used near pocket"),
            ]
            unhook(path, far)
            path += [
                w(1.80, (.060, sign*.289, HOVER), c, "pull soiled sheet entirely off mattress"),
                w(1.02, (.059, sign*.272, TOP + .046), c, "lower used sheet onto laundry pad"),
                w(.90, (.059, sign*.272, TOP + .046), o, "release used sheet into laundry zone"),
                w(.90, (.059, sign*.272, HOVER), o, "clear laundry zone"),
            ]
            trailing = np.array([.202, sign*.258, TOP+.021])
            approach(path, trailing, "trailing used corners")
            path += [
                w(.72, (*trailing[:2], HOVER), c, "lift trailing fabric"),
                w(1.45, (.060, sign*.245, HOVER), c, "clear all used fabric from mattress"),
                w(.92, (.059, sign*.227, TOP+.030), c, "lay the trailing fabric on the laundry pad"),
                w(.75, (.059, sign*.227, TOP+.030), o, "release used sheet completely"),
                w(.92, (.059, sign*.227, HOVER), o, "clear used sheet"),
            ]
            drape = np.array([.216, sign*.114, ZTOP+.003])
            approach(path, drape, "remaining used fabric at near edge")
            path += [
                w(.70, (*drape[:2], HOVER), c, "lift remaining used fabric"),
                w(1.35, (.073, sign*.130, HOVER), c, "clear mattress edge"),
                w(.92, (.073, sign*.130, TOP+.031), c, "lay remaining fabric on pad"),
                w(.70, (.073, sign*.130, TOP+.031), o, "release remaining fabric"),
                w(.80, (.073, sign*.130, HOVER), o, "clear laundry pad"),
            ]
        install(path, far)
        install(path, near, skip=CASE == "miss-one-pocket" and side == "right")
        path += [w(1.3, park, o, "release and clear mattress"),
                 w(1.1, park, o, "settle sheet")]
        result[side] = path
    return result


def grid_faces(nx, ny):
    return [(x*ny+y, (x+1)*ny+y, (x+1)*ny+y+1, x*ny+y+1)
            for x in range(nx-1) for y in range(ny-1)]


def make_positions():
    return np.array([(x, y, ZTOP+.003) for x in np.linspace(X0-.012, X1+.012, GRID_X)
                     for y in np.linspace(-Y-.012, Y+.012, GRID_Y)])


def make_links():
    links = []
    for x, y in product(range(GRID_X), range(GRID_Y)):
        i = x*GRID_Y+y
        for dx, dy, stiff in ((1, 0, .90), (0, 1, .90), (1, 1, .38), (1, -1, .38),
                              (2, 0, .14), (0, 2, .14)):
            xx, yy = x+dx, y+dy
            if 0 <= xx < GRID_X and 0 <= yy < GRID_Y:
                links.append((i, xx*GRID_Y+yy, stiff))
    return zip(*links)


A, B, STIFFNESS = (np.array(values, dtype=dtype)
                   for values, dtype in zip(make_links(), (int, int, float)))
REST = np.linalg.norm(make_positions()[B] - make_positions()[A], axis=1)
DEGREE = np.bincount(np.concatenate((A, B)), minlength=GRID_X*GRID_Y)[:, None]


class Sheet:
    def __init__(self, clean):
        self.clean = clean
        self.xyz = make_positions()
        if clean:
            for i in range(GRID_X):
                s = i / (GRID_X-1)
                width = .186 + .049*s
                for j in range(GRID_Y):
                    y = (j/(GRID_Y-1)*2 - 1)*width
                    self.xyz[i*GRID_Y+j] = (.134+.015*s+(.004 if i%2 else 0), y,
                                            TOP+.022+.014*s+.008*(i%2))
            self.xyz[CORNER_I] = CLEAN_FOLD
        else:
            self.xyz[CORNER_I] = SEAT
        self.velocity = np.zeros_like(self.xyz)
        self.latched = np.full(4, not clean, dtype=bool)
        self.folded = np.full(4, clean, dtype=bool)
        self.grasps = [None] * 4
        self.threaded = [False] * 4
        self.started = not clean
        self.body_grasps = {}

    def grip(self, hands, occupied, dt):
        anchors = {}
        for i, vertex in enumerate(CORNER_I):
            owner = self.grasps[i]
            if owner is None:
                for name, hand in hands.items():
                    tip = hand.position - (0, 0, PAD) - TAB_OFFSETS[i]
                    if (name not in occupied and hand.upright and hand.open_fraction < .31
                            and np.linalg.norm(tip-self.xyz[vertex]) < .015):
                        owner = self.grasps[i] = name
                        occupied.add(name)
                        self.folded[i] = False
                        self.started = True
                        break
            if owner is None:
                continue
            hand = hands[owner]
            tip = hand.position - (0, 0, PAD) - TAB_OFFSETS[i]
            clearance = (tip[:2] - CORNERS[i])*SIGNS[i]
            outside = bool(np.all(clearance > .017))
            if self.latched[i] and outside and tip[2] < ZTOP + .012:
                self.latched[i] = False
                self.threaded[i] = False
            if not self.latched[i]:
                if outside and ZBOTTOM -.008 < tip[2] < ZTOP -.011:
                    self.threaded[i] = True
                if (self.threaded[i] and np.all(clearance > -.008) and np.all(clearance < .020)
                        and ZBOTTOM -.008 < tip[2] < ZBOTTOM + .021):
                    self.latched[i] = True
                    self.threaded[i] = False
            if hand.open_fraction >= .58:
                self.grasps[i] = None
                self.threaded[i] = False
            elif not self.latched[i]:
                anchors[vertex] = tip
        for i, vertex in enumerate(CORNER_I):
            if self.latched[i]:
                anchors[vertex] = SEAT[i]
                if self.clean:
                    anchors[TOP_CORNER_I[i]] = TOP_CORNERS[i]
            elif self.folded[i]:
                anchors[vertex] = CLEAN_FOLD[i]
        if not self.clean and self.started and not bool(np.any(self.latched)):
            candidates = np.array([i for i in range(len(self.xyz)) if i not in CORNER_I], dtype=int)
            for name, hand in hands.items():
                tip = hand.position - (0, 0, PAD)
                if name not in occupied and name not in self.body_grasps and hand.open_fraction < .31 and hand.upright:
                    gaps = np.linalg.norm(self.xyz[candidates] - tip, axis=1)
                    best = int(np.argmin(gaps))
                    if gaps[best] < .020 and int(candidates[best]) not in self.body_grasps.values():
                        self.body_grasps[name] = int(candidates[best])
                if name not in self.body_grasps:
                    continue
                vertex = self.body_grasps[name]
                occupied.add(name)
                if hand.open_fraction >= .58:
                    del self.body_grasps[name]
                else:
                    anchors[vertex] = tip
        if self.started:
            self.solve(anchors, dt)

    def solve(self, anchors, dt):
        indices = np.array(list(anchors), dtype=int)
        values = np.array(list(anchors.values())).reshape(-1, 3)
        dt = dt/2
        vertices = self.xyz
        for _ in range(2):
            before = vertices.copy()
            vertices += self.velocity * dt
            vertices[:, 2] -= 7.0*dt*dt
            for _ in range(10):
                diffs = vertices[B] - vertices[A]
                length = np.linalg.norm(diffs, axis=1)
                strain = length-REST
                strain = np.maximum(0., strain) + .28*np.minimum(0., strain)
                correction = (STIFFNESS * strain/np.maximum(length, 1e-8))[:, None]*diffs
                fixes = np.zeros_like(vertices)
                np.add.at(fixes, A, correction)
                np.add.at(fixes, B, -correction)
                vertices += 1.5*fixes/DEGREE
                vertices[indices] = values
                self.collide(vertices, before)
            self.velocity = (vertices-before) / dt * .79
            self.velocity[indices] = 0.

    @staticmethod
    def collide(v, before):
        # The mattress and the two table pads are unilateral collision surfaces.
        inside = (v[:, 0] >= X0+.001) & (v[:, 0] <= X1-.001) & (np.abs(v[:, 1]) < Y-.001)
        reached = inside & (v[:, 2] < ZTOP+.002)
        from_above = reached & (before[:, 2] >= ZTOP-.011)
        v[from_above, 2] = ZTOP+.003
        side = reached & ~from_above
        for index in np.flatnonzero(side):
            x, y, z = v[index]
            distances = [x-X0, X1-x, y+Y, Y-y, ZTOP-z]
            closest = int(np.argmin(distances))
            if closest == 0:
                v[index, 0] = X0-.003
            elif closest == 1:
                v[index, 0] = X1+.003
            elif closest == 2:
                v[index, 1] = -Y-.003
            elif closest == 3:
                v[index, 1] = Y+.003
            else:
                v[index, 2] = ZTOP+.003
        on_shelf = (np.abs(v[:, 0]-.141)<.023) & (np.abs(v[:, 1])<.265)
        floor = np.where(on_shelf, TOP+.021, np.where(v[:, 0]<LAUNDRY_END, TOP+.007, TOP+.003))
        v[:, 2] = np.maximum(v[:, 2], floor)


class State:
    def __init__(self, case="nominal"):
        self.old = Sheet(clean=False)
        self.clean = Sheet(clean=True)

    def advance(self, hands, dt):
        occupied = set(self.old.body_grasps) | {owner for owner in self.old.grasps + self.clean.grasps if owner is not None}
        self.clean.grip(hands, occupied, dt)
        self.old.grip(hands, occupied, dt)
        # The used fabric must have cleared the sleeping surface. Bulk of the
        # fabric must rest in the laundry area, with both arms released.
        old = self.old.xyz
        old_mattress = ((old[:, 0] >= X0) & (old[:, 0] <= X1)
                        & (np.abs(old[:, 1]) <= Y) & (old[:, 2] >= ZBOTTOM-.003))
        laundry = (old[:, 0] < LAUNDRY_END) & (np.abs(old[:, 1]) < .303) & (old[:, 2] < TOP+.035)
        off = not bool(np.any(old_mattress) or np.any(self.old.latched) or any(self.old.grasps) or self.old.body_grasps)
        sorted_old = off and bool(np.mean(laundry) >= .65)
        fit = int(np.sum(self.clean.latched))
        released = not any(self.clean.grasps)
        cloth = self.clean.xyz
        interior = [i*GRID_Y+j for i in range(1, GRID_X-1) for j in range(1, GRID_Y-1)]
        panel = cloth[interior]
        coverage = float(np.mean((panel[:, 2] >= ZTOP-.002) & (panel[:, 2] <= ZTOP+.030)
                                 & (panel[:, 0] >= X0) & (panel[:, 0] <= X1) & (np.abs(panel[:, 1]) <= Y)))
        score = 0.
        if off:
            score += .17
        if sorted_old:
            score += .08
        if off and released:
            score += .155 * fit
        if sorted_old and fit == 4 and coverage >= .88 and released:
            score = 1.
        return {
            "score": round(score, 4), "used_pockets": int(np.sum(self.old.latched)),
            "clean_pockets": fit, "used_removed": off, "laundry": sorted_old,
            "laundry_percent": round(float(np.mean(laundry)) * 100, 1),
            "old_centroid": [round(float(v),4) for v in old.mean(axis=0)],
            "old_x_max": round(float(old[:,0].max()),4),
            "old_outliers": [[round(float(v),4) for v in pt] for pt in old[np.argsort(-old[:,0])[:4]]],
            "old_body_grasps": self.old.body_grasps.copy(),
            "surface_percent": round(coverage*100, 1),
            "clean_corner_xyz": [[round(float(v),4) for v in pt] for pt in cloth[CORNER_I]],
            "old_corner_xyz": [[round(float(v),4) for v in pt] for pt in old[CORNER_I]],
            "clean_grasps": self.clean.grasps.copy(), "used_grasps": self.old.grasps.copy(),
            "caption": f"{fit} of 4 clean pockets seated" if off else
                       f"{int(np.sum(self.old.latched))} used pockets still on mattress",
        }


# Render a denser fabric surface from the same simulated vertices. The sewn
# skirt wraps the mattress wherever the adjacent elastic pockets are seated.
RENDER_X, RENDER_Y = 29, 49
U, V = np.meshgrid(np.linspace(0, 1, RENDER_X), np.linspace(0, 1, RENDER_Y), indexing="ij")
HEM = np.r_[np.arange(RENDER_Y), np.arange(1, RENDER_X) * RENDER_Y + RENDER_Y - 1,
            np.arange(RENDER_Y - 2, -1, -1) + (RENDER_X - 1) * RENDER_Y,
            np.arange(RENDER_X - 2, 0, -1) * RENDER_Y]
# Move each perimeter sample one grid point inward, including the four corners.
HEM_INNER = (np.clip(HEM // RENDER_Y, 1, RENDER_X - 2) * RENDER_Y
             + np.clip(HEM % RENDER_Y, 1, RENDER_Y - 2))
HEM_FACES = [(i, (i + 1) % len(HEM), (i + 1) % len(HEM) + len(HEM), i + len(HEM))
             for i in range(len(HEM))]


def interpolate_axis(vertices, count):
    """Interpolate with bounded cubic tangents so the folds have soft creases."""
    t = np.linspace(0, len(vertices) - 1, count)
    i = np.minimum(t.astype(int), len(vertices) - 2)
    t = (t - i)[:, None, None]
    a, b = vertices[i], vertices[i + 1]
    before = vertices[np.maximum(i - 1, 0)]
    after = vertices[np.minimum(i + 2, len(vertices) - 1)]
    bend = .5 * t * (1 - t) * ((1 - t) * (b - before) - t * (after - a))
    return np.clip(a + t*t*(3-2*t)*(b-a) + bend, np.minimum(a,b), np.maximum(a,b))


def fabric_surface(sheet, fitted_corners):
    xyz = sheet.xyz.reshape(GRID_X, GRID_Y, 3)
    dense = interpolate_axis(interpolate_axis(xyz, RENDER_X).swapaxes(0, 1), RENDER_Y).swapaxes(0, 1)
    if not np.any(fitted_corners > .28):
        return dense.reshape(-1, 3)

    # An elastic mattress skirt is short, while the upper panel stays wide.
    # Each seated corner only draws in its connected part of the visual mesh.
    fit = np.stack(((1-U)*(1-V), U*(1-V), U*V, (1-U)*V), axis=-1) @ fitted_corners
    fit = np.clip((fit-.28)/.65, 0, 1)
    fit *= fit * (3-2*fit)
    skirt_x = np.clip(1 - np.minimum(U, 1-U)/.070, 0, 1)
    skirt_y = np.clip(1 - np.minimum(V, 1-V)/.060, 0, 1)
    skirt = np.maximum(skirt_x, skirt_y)**1.5
    # Project the skirt over the top edge and down the side in separate
    # bands, leaving a rounded shoulder instead of a diagonal plane.
    x = np.interp(U, [0, .035, .072, .928, .965, 1],
                  [X0-.008, X0-.006, X0-.002, X1+.002, X1+.006, X1+.008])
    y = np.interp(V, [0, .03, .061, .939, .97, 1],
                  [-Y-.008, -Y-.006, -Y-.002, Y+.002, Y+.006, Y+.008])
    wrinkle = .0012 * np.sin(9*pi*U + 3*pi*V) * np.sin(12*pi*V)
    z = ZTOP+.003 - (ZTOP+.003-SEAT[0,2])*skirt + wrinkle*(1-skirt)
    fitted = np.stack((x, y, z), axis=-1)
    return (dense + fit[..., None] * (fitted-dense)).reshape(-1, 3)


class View:
    def __init__(self, state):
        g = self.g = Geometry()
        xmid = (X0 + X1)/2
        g.box("WoodSupport", (xmid, 0, TOP+.007), (X1-X0+.024, 2*Y+.020, .014), (.54,.37,.24))
        g.box("LaundryBackStripe", (.015, 0, TOP+.007), (.007,.596,.003), (.69,.65,.23))
        for j, y in enumerate((-.292, .292)):
            g.box(f"LaundryEdge{j}", (.093,y,TOP+.007), (.187,.007,.003), (.69,.65,.23))
        self.sheets = []
        self.hems = []
        self.tags = []
        self.fitted_corners = [sheet.latched.astype(float) for sheet in (state.old, state.clean)]
        for i, (sheet, color, seam_color) in enumerate((
            (state.old,(.63,.25,.20),(.52,.18,.15)),
            (state.clean,(.18,.46,.63),(.13,.35,.52)),
        )):
            points = fabric_surface(sheet, self.fitted_corners[i])
            mesh = g.mesh(f"Fabric{i}", points, grid_faces(RENDER_X, RENDER_Y), color)
            self.sheets.append(mesh)
            hem = g.mesh(f"ElasticHem{i}", self.hem_points(points), HEM_FACES, seam_color)
            self.hems.append(hem)
            tabs = [g.mesh(f"PullTab{i}_{j}", self.tab_points(sheet, j),
                           [(k,k+1,k+6,k+5) for k in range(4)], color)
                    for j in range(4)]
            self.tags.append(tabs)
        self.update(state)

    @staticmethod
    def hem_points(surface):
        exterior = surface[HEM]
        interior = surface[HEM_INNER]
        offset = np.array([0, 0, .001])
        return np.concatenate((exterior + offset, exterior + .47*(interior-exterior) + offset))

    @staticmethod
    def tab_points(sheet, i):
        base = sheet.xyz[CORNER_I[i]]
        lengths = np.linspace(0, 1, 5)
        center = base + lengths[:, None] * TAB_OFFSETS[i]
        # The free fabric tab droops under its own weight. A pinch straightens it.
        drop = .009 if sheet.grasps[i] is None else .001
        center[:, 2] += .002 - drop * lengths**2
        width = .008 - .002 * lengths**2
        cross = width[:, None] * (1., 0., .12)
        return np.concatenate((center-cross, center+cross))

    def update(self,state):
        for i,sheet in enumerate((state.old,state.clean)):
            corners = self.fitted_corners[i]
            # Let the fabric settle over several physics frames when a corner
            # catches or releases, instead of snapping the visible panel.
            corners += .38 * (sheet.latched - corners)
            points = fabric_surface(sheet, corners)
            self.g.points(self.sheets[i], points)
            self.g.points(self.hems[i], self.hem_points(points))
            for j, tab in enumerate(self.tags[i]):
                self.g.points(tab, self.tab_points(sheet, j))
