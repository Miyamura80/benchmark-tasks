"""Handheld automotive vacuum geometry and a bounded fine-debris suction proxy."""

import xml.etree.ElementTree as ET

import numpy as np

INTAKE_OFFSET = np.array([.075, 0, -.035])
BIN_CENTER = np.array([.017, 0, -.021])
INTAKE_DIRECTION = np.array([.45, 0, -.893])
INTAKE_DIRECTION /= np.linalg.norm(INTAKE_DIRECTION)


def add_vacuum_geometry(node):
    """Build noncolliding housing details around a grip-centred tool body."""
    charcoal = (.09, .10, .105, 1)
    rubber = (.04, .047, .050, 1)
    amber = (.92, .40, .10, 1)
    clear = (.26, .38, .40, .33)

    def geom(name, kind, color, **attributes):
        fields = {
            key: " ".join(map(str, value)) if isinstance(value, (tuple, list)) else str(value)
            for key, value in attributes.items()
        }
        ET.SubElement(node, "geom", name="vacuum_" + name, type=kind,
                      rgba=" ".join(map(str, color)), contype="0",
                      conaffinity="0", group="1", mass="0", **fields)

    geom("grip", "capsule", rubber, fromto=(0, -.013, 0, 0, .013, 0), size=.012)
    geom("handle_left", "capsule", charcoal,
         fromto=(-.020, 0, -.014, -.012, 0, .002), size=.008)
    geom("handle_right", "capsule", charcoal,
         fromto=(.022, 0, -.013, .012, 0, .002), size=.008)
    geom("motor_housing", "ellipsoid", charcoal, pos=(-.018, 0, -.021), size=(.032, .022, .019))
    geom("rear_rubber", "ellipsoid", rubber, pos=(-.038, 0, -.022), size=(.014, .020, .017))
    geom("battery_seam", "box", amber, pos=(-.034, 0, -.034), size=(.012, .018, .002))
    geom("filter_core", "capsule", (.50, .47, .38, 1),
         fromto=(-.001, 0, -.020, .023, 0, -.020), size=.012)
    geom("dust_bin", "capsule", clear, fromto=(.002, 0, -.021, .033, 0, -.021), size=.017)
    geom("bin_release", "box", amber, pos=(.016, 0, -.005), size=(.005, .008, .002))
    geom("bin_collar", "capsule", charcoal, fromto=(.030, 0, -.021, .039, 0, -.023), size=.014)
    geom("nozzle_taper", "capsule", charcoal,
         fromto=(.038, 0, -.022, .062, 0, -.028), size=.009)
    geom("crevice_nozzle", "box", charcoal, pos=(.061, 0, -.029),
         size=(.013, .010, .0048), euler=(0, .26, 0))
    geom("intake_throat", "box", (.013, .015, .017, 1), pos=(.074, 0, -.034),
         size=(.0022, .0077, .0028), euler=(0, .46, 0))
    for side in (-1, 1):
        geom(f"lip_side_{side}", "box", rubber, pos=(.075, side*.0085, -.035),
             size=(.0025, .0012, .004), euler=(0, .46, 0))
        geom(f"lip_edge_{side}", "box", rubber,
             pos=(.075+side*.0016, 0, -.035+side*.0033),
             size=(.0025, .0085, .001), euler=(0, .46, 0))
    for side in (-1, 1):
        for index in range(5):
            geom(f"exhaust_{side}_{index}", "capsule", rubber,
                 fromto=(-.022-index*.004, side*.020, -.016,
                         -.024-index*.004, side*.020, -.027), size=.001)
    geom("switch", "ellipsoid", amber, pos=(.012, 0, .008), size=(.005, .006, .002))
    geom("indicator", "sphere", (.24, .31, .31, 1), pos=(.019, .004, -.005), size=.0024)


def suction_force(intake, debris_position, debris_velocity, *, powered, in_bin=False,
                  intake_direction=INTAKE_DIRECTION):
    """Pull loose debris only in front of a powered intake.

    Distance and alignment reduce force. This proxy omits bulk airflow and
    entrainment of particles still bound to carpet fibers.
    """
    offset = np.asarray(debris_position) - np.asarray(intake)
    distance = np.linalg.norm(offset)
    if not powered or in_bin or distance <= 1e-6 or distance >= .040:
        return np.zeros(3)
    direction = offset / distance
    forward = np.dot(direction, intake_direction)
    if forward < .12:
        return np.zeros(3)
    strength = .42 * (1 - distance / .040) ** 2 * forward
    return -strength * direction - .035 * strength * np.asarray(debris_velocity)
