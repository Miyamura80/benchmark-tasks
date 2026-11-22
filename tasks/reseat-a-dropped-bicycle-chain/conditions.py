"""Submitted frame configuration, with SI units."""
from fixtures import Chain

CASES = ("frame",)

def build(case="frame", tool="jaws", variant=0):
    if (case, tool, variant) != ("frame", "jaws", 0):
        raise ValueError("Expected the frame, offset jaws and nominal reset")
    return Chain(variant, drop=.011, rear_drag=.001, lift_pick=True)
