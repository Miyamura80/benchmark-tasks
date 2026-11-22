"""SI dimensions of the prepared pressure-sensitive mailer."""

import math
import numpy as np

ORIGIN = np.array([0.32, 0.0, 0.748])
HEIGHT = 0.085
LENGTH = 0.220
WIDTH = 0.145
ZONES = np.linspace(-0.088, 0.088, 5)
RIM_Y = -0.061
LID_Z = 0.0863
HINGE_Y = WIDTH / 2
REST = 33.0
BOARD = 0.0022


def rx(deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def parts():
    hinge = ORIGIN + [0, HINGE_Y, LID_Z]
    R = rx(-REST)
    rows = {
        "carton": (ORIGIN, np.eye(3), 0.16),
        "lid": (hinge + R @ [0, -0.061, 0], R, 0.035),
    }
    for i, x in enumerate(ZONES):
        rows[f"lip_{i}"] = (hinge + R @ [x, -0.132, 0.0], R, 0.0032)
    return rows
