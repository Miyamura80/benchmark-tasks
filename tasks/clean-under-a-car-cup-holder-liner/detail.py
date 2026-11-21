"""Smooth visible surfaces around the console's contact mesh."""

import math

import numpy as np

from fixture_base import VIS, body, box, capsule
from fixture_base import cyl, element

PLASTIC = (.18, .171, .16, 1)
WELL = (.057, .064, .071, 1)
LINED = (.091, .100, .108, 1)
LEATHER = (.19, .136, .103, 1)
SEAM = (.39, .296, .227, 1)
METAL = (.48, .52, .55, 1)
BLACK = (.039, .045, .048, 1)
FOAM = (.737, .577, .156, 1)
CLOTH = (.63, .475, .113, 1)
TOOL = (.215, .266, .269, 1)
TICKS = (.69, .704, .68, 1)


def shell(assets, parent, name, sections, color, reverse=False, **attrs):
    """Skin circular cross-sections including height and radius changes."""
    points = np.array(sections)
    count = points.shape[1]
    faces = []
    for j in range(len(points) - 1):
        for i in range(count):
            a, b = j * count + i, j * count + (i + 1) % count
            faces.extend(((a, b, a + count), (b, b + count, a + count)))
    if reverse:
        faces = [triangle[::-1] for triangle in faces]
    element(assets, "mesh", name=name, vertex=points.flatten(),
            face=" ".join(map(str, np.asarray(faces).flatten())))
    return element(parent, "geom", name=name, type="mesh", mesh=name, rgba=color,
                   **attrs, **VIS)


def loops(x, y, profile, sections=96):
    a = np.arange(sections) * math.tau / sections
    return [np.column_stack((x + radius * np.cos(a), y + radius * np.sin(a),
                             np.full(sections, z))) for radius, z in profile]


def well(assets, parent, name, x, y, extra, rect):
    inner = .041 + extra
    opening = inner + .003
    outer = opening + .004
    shell(assets, parent, f"{name}_wall_skin",
          loops(x, y, [(inner - .001, .2219), (inner, .224), (inner + .001, .229),
                       (opening, .2405), (opening + .0007, .2416)]),
          WELL, reverse=True)
    shell(assets, parent, f"{name}_bezel",
          loops(x, y, [(opening + .0006, .2414), (opening + .0013, .2434),
                       (opening + .0022, .2444), (outer - .0007, .2440),
                       (outer, .2427), (outer + .0004, .2420)]),
          METAL, reverse=True)
    inner_loop = loops(x, y, [(outer, .2421)])[0]
    xy = (inner_loop[:, :2] - (x, y)) / outer
    xmin, xmax, ymin, ymax = rect
    with np.errstate(divide="ignore"):
        stretch = np.minimum(
            np.where(xy[:, 0] >= 0, (xmax-x) / xy[:, 0], (xmin-x) / xy[:, 0]),
            np.where(xy[:, 1] >= 0, (ymax-y) / xy[:, 1], (ymin-y) / xy[:, 1]),
        )
    perimeter = np.column_stack((xy * stretch[:, None] + (x, y), np.full(len(xy), .242)))
    shell(assets, parent, f"{name}_surround", [inner_loop, perimeter], PLASTIC, reverse=True)
    cyl(parent, f"{name}_grained_floor", (x, y, .2213), inner + .0006, .00064,
        (.081, .085, .088, 1), **VIS)
    for i in range(3):
        cyl(parent, f"{name}_floor_inset_{i}", (x, y, .2219), .032 - .004 * i,
            .00014, (.084 + .004*i, .088 + .004*i, .093 + .004*i, 1), **VIS)


def edge_roll(parent, name, x, y1, y2, z, radius, color):
    capsule(parent, name, (x, y1, z), (x, y2, z), radius, color, **VIS)


def console(assets, parent):
    # Keep collision layout intact. The skins supply the flared bores and molded PC-ABS surfaces.
    well(assets, parent, "primary", .110, .033, .005, (.017, .212, -.022, .094))
    well(assets, parent, "secondary", .149, -.073, 0, (.093, .212, -.131, -.022))
    box(parent, "armrest_cut", (.145, .151, .226),
        (.067, .057, .016), PLASTIC, **VIS)
    box(parent, "gear_selector_panel", (.116, -.166, .226),
        (.096, .035, .016), PLASTIC, **VIS)
    box(parent, "shelf_panel", (.039, .004, .2097),
        (.061, .196, .00032), PLASTIC, **VIS)
    box(parent, "utility_pocket_end", (.049, -.131, .221),
        (.044, .001, .011), PLASTIC, **VIS)
    box(parent, "utility_pocket_side", (.092, -.076, .225),
        (.001, .055, .017), PLASTIC, **VIS)
    box(parent, "armrest_wall", (.077, .150, .225),
        (.0015, .056, .017), PLASTIC, **VIS)
    edge_roll(parent, "padded_knee_rail", .227, -.191, .189, .238, .018, LEATHER)
    edge_roll(parent, "padded_near_bead", -.022, -.192, .189, .222, .006, WELL)
    edge_roll(parent, "armrest_leading_edge", .117, .191, .198, .226, .001, SEAM)
    for i in range(29):
        y = -.184 + .013 * i
        edge_roll(parent, f"leather_stitch_{i}", .219, y, y + .005, .254, .00038, SEAM)
    box(parent, "gear_selector_gaiter", (.098, -.171, .245),
        (.040, .023, .004), BLACK, **VIS)
    box(parent, "gear_selector_channel", (.093, -.171, .249),
        (.029, .004, .0018), (.107, .108, .112, 1), **VIS)
    capsule(parent, "gear_selector_stem", (.105, -.171, .250), (.107, -.171, .273),
            .006, METAL, **VIS)
    box(parent, "gear_selector_knob", (.108, -.171, .274),
        (.016, .015, .007), LEATHER, **VIS)
    for i in range(4):
        x = .071 + .016 * i
        box(parent, f"selector_index_{i}", (x, -.154, .2495), (.001, .003, .0003),
            (.73, .32, .094, 1) if i == 1 else TICKS, **VIS)

    # Open hand-rest pocket above the wells, with molded slots and a rubber bottom.
    box(parent, "pocket_outline", (.143, .154, .242),
        (.048, .029, .0028), WELL, **VIS)
    box(parent, "pocket_inset", (.143, .154, .244),
        (.045, .025, .0012), BLACK, **VIS)
    for i in range(9):
        x = .103 + .010 * i
        box(parent, f"pocket_rib_{i}", (x, .154, .2455),
            (.0006, .021, .00045), LINED, **VIS)


def insert(assets, parent):
    shell(assets, parent, "molded_tpe_insert",
          loops(0, 0, [(.0367, -.0028), (.037, -.0017), (.037, 0),
                       (.0367, .003), (.0358, .0045), (.0335, .0046),
                       (.0331, .0031), (.0314, .0027)]), LINED)
    cyl(parent, "liner_floor_skin", (0, 0, .0014), .0325, .0013, BLACK, **VIS)
    for i in range(-5, 6):
        y = i * .0054
        half = math.sqrt(.0314**2 - y**2)
        capsule(parent, f"molded_rib_{i}", (-half, y, .003), (half, y, .003),
                .00065, LINED, **VIS)
    # Rounded overmold on the same two-finger pinch volume.
    capsule(parent, "pull_web_base", (-.009, .006, .004), (.009, .006, .004),
            .0031, LINED, **VIS)
    for sign in (-1, 1):
        capsule(parent, f"rubber_web_{sign}", (sign*.0075, .0085, .006),
                (sign*.0075, .006, .018), .0029, LINED, **VIS)
    capsule(parent, "soft_tab_bar", (-.007, .006, .019), (.007, .006, .019),
            .0043, LINED, **VIS)
    for i in range(5):
        x = (i - 2) * .003
        capsule(parent, f"soft_tab_grain_{i}", (x, .0017, .018), (x, .0019, .0215),
                .00026, TICKS, **VIS)


def scrubber(parent):
    cyl(parent, "tool_neck_fillet", (0, 0, .007), .0110, .0055, TOOL, **VIS)
    box(parent, "foam_block", (0, 0, -.0004), (.0127, .0107, .00445), FOAM, **VIS)
    for sign in (-1, 1):
        capsule(parent, f"foam_hem_x_{sign}", (sign*.0125, -.0095, -.0008),
                (sign*.0125, .0095, -.0008), .00125, CLOTH, **VIS)
        capsule(parent, f"foam_hem_y_{sign}", (-.011, sign*.0102, -.0005),
                (.011, sign*.0102, -.0005), .00125, CLOTH, **VIS)
    for j in range(12):
        x = (j - 5.5) * .0021
        for sign in (-1, 1):
            capsule(parent, f"pile_{sign}_{j}", (x, sign * .0109, -.0032),
                    (x + .0006, sign * .0112, .0032), .00022, FOAM, **VIS)
    box(parent, "molded_handle_core", (0, 0, .024), (.008, .0051, .0178), TOOL, **VIS)
    for sign in (-1, 1):
        capsule(parent, f"molded_handle_edge_{sign}", (sign*.007, 0, .013),
                (sign*.007, 0, .043), .006, TOOL, **VIS)
        for i in range(5):
            z = .024 + .0035*i
            capsule(parent, f"handle_texture_{i}_{sign}", (-.007, sign*.0057, z),
                    (.007, sign*.0057, z), .00055, BLACK, **VIS)


def smears(assets, parent, points, floor, x, y):
    """Dark, irregular spill stain with a thicker outer meniscus and satellite spots."""
    for i, (dx, dy) in enumerate(points):
        location = np.array((x+dx, y+dy))
        for layer, (radius, color) in enumerate(((.0067, (.23,.122,.051,.77)),
                                               (.0052, (.31,.171,.084,.71)),
                                               (.0031, (.38,.213,.109,.56)))):
            phase = np.arange(36) * math.tau / 36
            spread = radius * (1 + .18*np.sin(3*phase + i*.9) +
                               .10*np.sin(5*phase + i*.8))
            boundary = location + np.column_stack((np.cos(phase)*spread,
                                                    np.sin(phase)*spread*.77))
            z = floor + .00016 + layer * .00010 + i * .000012
            # A micron-scale dome keeps the visual mesh volumetric.
            lower = np.column_stack((boundary, np.full(len(phase), z - .00005)))
            top = np.column_stack((location + (boundary - location)*.20,
                                   np.full(len(phase), z + .00004)))
            crown = np.column_stack((location + (boundary - location)*.02,
                                     np.full(len(phase), z + .00007)))
            shell(assets, parent, f"spill_{i}_{layer}", [lower, top, crown], color)
