"""Time belongs to physics samples, independent of how often hooks are read."""

import math
import numpy as np


class ReleaseHold:
    def __init__(self, control_period=0.05):
        self.max_interval = control_period * 1.02
        self.time = None
        self.position = None
        self.ready_since = None
        self.duration = 0.0
        self.overloaded = False

    def update(self, time, position, angle, bonds, inside, clear, pads):
        position = np.asarray(position, dtype=float)
        pads = np.asarray(pads, dtype=float)
        if (
            position.shape != (3,)
            or pads.shape != (5,)
            or not math.isfinite(time)
            or not math.isfinite(angle)
            or not np.all(np.isfinite(position))
            or not np.all(np.isfinite(pads))
            or np.any(pads < 0)
        ):
            raise ValueError("Invalid native mailer measurement")
        dt = None if self.time is None else time - self.time
        if dt is not None and dt <= 0:
            raise ValueError("Expected consecutive post-step measurements")
        self.overloaded |= float(pads.sum()) > 30.0
        consecutive = dt is not None and dt <= self.max_interval
        still = consecutive and np.linalg.norm(position - self.position) / dt < 0.010
        ready = (
            set(bonds) == set(range(5))
            and abs(angle) <= 3
            and inside
            and clear
            and still
            and not self.overloaded
        )
        if ready:
            if self.ready_since is None:
                self.ready_since = time
        else:
            self.ready_since = None
        self.duration = time - self.ready_since if self.ready_since is not None else 0.0
        self.time, self.position = time, position.copy()

    @property
    def score(self):
        return float(self.duration >= 2.0 - 1e-8)
