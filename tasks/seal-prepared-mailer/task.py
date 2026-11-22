"""Press the exposed factory-fitted tape across a preloaded mailer's closing flap."""

from yam_arena.task_file import Box
from pathlib import Path
import sys

# yam_arena loads a file by path without adding its directory to sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parent))
try:
    from carton_seal.layout import *
    from carton_seal.adapter import install, score, success
finally:
    sys.path.pop(0)
install()


def vec(p):
    return tuple(map(float, p))


SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0
OBJECTS = [
    Box(
        n,
        pos=tuple(map(float, p)),
        size=(0.04, 0.02, 0.002),
        mass=m,
        color=(0.69, 0.45, 0.25),
    )
    for n, (p, R, m) in parts().items()
]
OBJECTS.extend(
    [
        Box(
            "blue_product",
            pos=vec(ORIGIN + [-0.045, 0.01, 0.019]),
            size=(0.064, 0.044, 0.025),
            mass=0.09,
            color=(0.26, 0.44, 0.56),
        ),
        Box(
            "rust_product",
            pos=vec(ORIGIN + [0.045, 0, 0.025]),
            size=(0.055, 0.044, 0.038),
            mass=0.09,
            color=(0.69, 0.34, 0.20),
        ),
        Box(
            "west_guide",
            pos=vec(ORIGIN + [-0.114, 0, 0.009]),
            size=(0.004, 0.043, 0.018),
            color=(0.35, 0.35, 0.37),
        ),
        Box(
            "east_guide",
            pos=vec(ORIGIN + [0.114, 0, 0.009]),
            size=(0.004, 0.043, 0.018),
            color=(0.35, 0.35, 0.37),
        ),
    ]
)
