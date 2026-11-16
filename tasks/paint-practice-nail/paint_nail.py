"""Use the upright, preloaded red applicator to paint the pale nail on the centre finger of the silicone practice hand."""
import numpy as np

from yam_arena.station import BASES
from yam_arena.task_file import Box, TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MIDLINE_Y = (BASES["left"][1] + BASES["right"][1]) / 2
XY = np.array([0.235, MIDLINE_Y])
SURFACE = TOP + 0.024
TOOL = Box(
    "polish_applicator", (0.163, 0.175, TOP + 0.019),
    color=(0.78, 0.04, 0.13), mass=0.023, size=(0.027, 0.024, 0.038),
)
def hand_position(dx, dy, height):
    """Place static geometry relative to the middle nail's plan-view centre."""
    return (float(XY[0] + dx), float(XY[1] + dy), TOP + height)


# Simplified colliders under the sculpted silicone shell.
HAND_COLLIDERS = [
    Box("middle_finger_proxy", hand_position(0.014, 0, 0.016), size=(0.052, 0.019, 0.016)),
    Box("index_finger_proxy", hand_position(0.025, 0.021, 0.015), size=(0.049, 0.017, 0.014)),
    Box("ring_finger_proxy", hand_position(0.025, -0.019, 0.015), size=(0.052, 0.017, 0.014)),
    Box("little_finger_proxy", hand_position(0.034, -0.036, 0.014), size=(0.044, 0.014, 0.012)),
    Box("thumb_proxy", hand_position(0.045, 0.046, 0.015), size=(0.031, 0.018, 0.014), yaw=-0.34),
    Box("palm_proxy", hand_position(0.077, -0.003, 0.017), size=(0.053, 0.065, 0.018)),
    Box("wrist_proxy", hand_position(0.113, 0.005, 0.0145), size=(0.033, 0.042, 0.013)),
]
REST = Box(
    "manicure_rest", hand_position(0.054, 0.009, 0.003),
    color=(0.32, 0.35, 0.39), size=(0.163, 0.113, 0.006),
)
OBJECTS = [REST, *HAND_COLLIDERS, TOOL]

CELL = 0.00075
RADIUS = 0.0028
TIP_EXTENSION = 0.004
TOOL_REACH = TOOL.size[2] / 2 + TIP_EXTENSION
CONTACT_TOLERANCE = 0.0015
COVERAGE = 0.85
SPILL = 0.02
SUCCESS_HOLD_S = 0.3
OFFSETS = np.stack(np.meshgrid(np.arange(-40, 41), np.arange(-15, 16), indexing="ij"), -1).reshape(-1, 2) * CELL
# Rounded rectangle 17 x 12 mm with 3 mm corner radii.
NAIL = np.linalg.norm(np.maximum(np.abs(OFFSETS) - [0.0055, 0.003], 0), axis=1) <= 0.003
SKIN = ~NAIL


class Paint:
    """Accumulate a swept applicator footprint in the fixture's coordinate frame."""

    def __init__(self):
        self.painted = np.zeros(len(NAIL), dtype=bool)
        self.previous = None

    def step(self, xy, contact):
        if not contact:
            self.previous = None
            return False
        xy = np.asarray(xy) - XY
        before, self.previous = self.previous, xy
        if before is None:
            return False
        delta = xy - before
        length = np.linalg.norm(delta)
        if not 0.00008 <= length <= 0.004:
            return False
        projection = np.clip((OFFSETS - before) @ delta / length**2, 0, 1)
        swept = np.linalg.norm(OFFSETS - before - projection[:, None] * delta, axis=1) <= RADIUS
        change = bool(np.any(swept & ~self.painted))
        self.painted |= swept
        return change

    def metrics(self):
        area = np.count_nonzero(NAIL)
        return np.count_nonzero(self.painted & NAIL) / area, np.count_nonzero(self.painted & SKIN) / area


paint = Paint()


def success(objects):
    covered, spilled = paint.metrics()
    return bool(covered >= COVERAGE and spilled <= SPILL)


def score(objects):
    covered, spilled = paint.metrics()
    return min(0.99, covered * max(0.0, 1.0 - 4.0 * spilled))
