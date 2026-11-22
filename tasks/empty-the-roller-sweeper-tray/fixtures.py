"""Shared primitives for the roller-sweeper fixture."""

import math
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from station import DT, HZ, element, geom, station_xml

STEEL = (0.59, 0.64, 0.69, 1)
DARK = (0.13, 0.16, 0.19, 1)
BLACK = (0.08, 0.09, 0.11, 1)
BLUE = (0.14, 0.33, 0.52, 1)
TEAL = (0.04, 0.59, 0.57, 1)
ORANGE = (0.98, 0.42, 0.10, 1)
YELLOW = (0.98, 0.74, 0.18, 1)
RUBBER = (0.24, 0.25, 0.27, 1)
WHITE = (0.81, 0.85, 0.86, 1)
BOARD = (0.35, 0.43, 0.45, 1)
WATER = (0.12, 0.70, 0.94, 0.85)
NO_CONTACT = {"contype": 0, "conaffinity": 0, "mass": 0}


def box(parent, name, pos, half, color, **kwargs):
    return geom(parent, name, "box", pos, half, color, **kwargs)


def cylinder(parent, name, pos, size, color, **kwargs):
    return geom(parent, name, "cylinder", pos, size, color, **kwargs)


def body(parent, name, pos):
    return element(parent, "body", name=name, pos=pos)




def hinge(parent, name, axis, limits, **kwargs):
    return element(
        parent, "joint", name=name, type="hinge", axis=axis, range=limits, **kwargs
    )


def ball(parent, name, pos, size, color, **kwargs):
    return geom(parent, name, "ellipsoid", pos, size, color, **kwargs)


def capsule(parent, name, start, end, radius, color, **kwargs):
    return element(
        parent,
        "geom",
        name=name,
        type="capsule",
        fromto=(*start, *end),
        size=radius,
        rgba=color,
        **kwargs,
    )


def stand(parent, xy=(0.015, 0), half=(0.16, 0.19), top=0.200):
    for i, x in enumerate((xy[0] - half[0] + 0.023, xy[0] + half[0] - 0.023)):
        box(
            parent, f"foot{i}", (x, xy[1], 0.182), (0.017, half[1] - 0.015, 0.017), DARK
        )
    return box(parent, "bench_pad", (*xy, top - 0.004), (*half, 0.004), BOARD)


def tray(parent, x, y, prefix="tray", z=0.201, sx=0.046, sy=0.054):
    box(parent, prefix, (x, y, z), (sx, sy, 0.0025), TEAL)
    for i, sign in enumerate((-1, 1)):
        box(
            parent,
            f"{prefix}_x{i}",
            (x + sign * sx, y, z + 0.006),
            (0.002, sy, 0.008),
            TEAL,
        )
        box(
            parent,
            f"{prefix}_y{i}",
            (x, y + sign * sy, z + 0.006),
            (sx, 0.002, 0.008),
            TEAL,
        )


class Fixture:
    def __init__(self, variant=0):
        self.variant = variant

    def compile(self):
        tree, world = station_xml()
        self.asset_meshes(tree.find("asset"))
        self.build(world)
        self.constraints(tree)
        for item in list(world):
            if item.find("freejoint") is not None:
                world.remove(item)
                item.set("childclass", "fixture")
                tree.find("worldbody").append(item)
        return mj.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"))

    def asset_meshes(self, assets):
        pass

    def constraints(self, tree):
        pass

    def reset(self, sim):
        pass

    def before_step(self, sim):
        pass

    def after_step(self, sim):
        pass

    def control(self, sim):
        sim.wait(20, "Unrepaired starting condition")

    def detail_control(self, sim):
        self.demo(sim, incomplete=True)

    def visual(self, sim, scene):
        pass














