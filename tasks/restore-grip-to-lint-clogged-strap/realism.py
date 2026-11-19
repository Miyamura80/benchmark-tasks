"""Procedural render geometry and surface materials for the repair fixtures."""

from io import BytesIO
import math

import numpy as np
from PIL import Image

from fixture_base import VIS, box, capsule, cyl, element, geom

VISUAL = {**VIS, "group": 2}
RUBBER = (.09, .105, .12, 1)
ALUMINIUM = (.68, .72, .75, 1)
ABS = (.86, .845, .79, 1)


def hide(parent, *names):
    for name in names:
        item = parent.find(f"geom[@name='{name}']")
        if item is None:
            raise ValueError(f"Missing collision proxy {name}")
        item.set("group", "5")


def mesh(asset, parent, name, vertices, faces, color, uv=None, material=None):
    attrs = {
        "vertex": " ".join(f"{v:.6f}" for vertex in vertices for v in vertex),
        "face": " ".join(str(v) for face in faces for v in face),
    }
    if uv is not None:
        attrs["texcoord"] = " ".join(f"{v:.6f}" for point in uv for v in point)
    element(asset, "mesh", name=name, **attrs)
    style = {"material": material} if material else {}
    element(parent, "geom", name=name, type="mesh", mesh=name, rgba=color,
            **VISUAL, **style)


def png_texture(asset, assets, name, source, *, repeat=(1, 1), specular=.15, shiny=.15):
    filename = name + ".png"
    data = BytesIO()
    source.save(data, format="PNG")
    assets[filename] = data.getvalue()
    element(asset, "texture", name=name, type="2d", file=filename)
    element(asset, "material", name=name, texture=name, texrepeat=repeat,
            specular=specular, shininess=shiny)
    return name


def woven_texture(base, step=5, seed=61):
    """Produce over-under highlights without sampling a reference image."""
    rng = np.random.default_rng(seed)
    y, x = np.indices((192, 192))
    vertical = .66 + .34*np.sin((x % step)*math.pi/step)
    horizontal = .66 + .34*np.sin((y % step)*math.pi/step)
    pattern = np.where(((x//step+y//step) % 2) == 0, vertical, horizontal)
    rgb = np.array(base)[None, None, :] * pattern[:, :, None]
    rgb += rng.normal(0, 3.4, size=pattern.shape)[:, :, None]
    return Image.fromarray(np.uint8(np.clip(rgb, 0, 255)))








def screw(parent, name, x, y, z, radius=.003):
    cyl(parent, name, (x,y,z), radius, .00065, ALUMINIUM, **VISUAL)
    box(parent, name+"_slot", (x,y,z+.00072), (radius*.64,.00036,.00012), RUBBER,
        euler=(0,0,.35), **VISUAL)






def tube_bundle(asset, parent, name, paths, radius, color):
    """Join small profiles into a mesh to keep the render geometry count low."""
    vertices, faces = [], []
    sides = 5
    for path in paths:
        start = len(vertices)
        for i, point in enumerate(path):
            tangent = np.array(path[min(i+1,len(path)-1)]) - path[max(i-1,0)]
            tangent /= np.linalg.norm(tangent)
            u = np.cross(tangent, (1,0,0))
            u *= radius/np.linalg.norm(u)
            v = np.cross(tangent, u)
            for j in range(sides):
                angle = j*math.tau/sides
                vertices.append(point + u*math.cos(angle) + v*math.sin(angle))
        for i in range(len(path)-1):
            for j in range(sides):
                a = start+i*sides+j
                b = start+i*sides+(j+1) % sides
                faces.extend([(a,b,a+sides), (b,b+sides,a+sides)])
    mesh(asset, parent, name, vertices, faces, color)


def fastener(tree, world, assets):
    asset = tree.find("asset")
    strap = png_texture(asset, assets, "strap_weave", woven_texture((48,49,43),6),
                        repeat=(7,3), specular=.04)
    png_texture(asset, assets, "strap_loops", woven_texture((90,93,81),5,42),
                repeat=(8,3), specular=.04)
    world.find("geom[@name='base']").set("rgba", ".26 .28 .26 1")
    pad = world.find("geom[@name='hook_webbing']")
    pad.set("material", strap)
    pad.set("rgba", "1 1 1 1")
    hide(world, *(f"hook_{i}_{j}" for i in range(25) for j in range(8)))
    hook_paths = []
    for i in range(66):
        for j in range(24):
            x = .024+i*.00184
            y = -.094+j*.00173+(i%2)*.00038
            hook_paths.append([
                (x,y,.218), (x,y,.21855), (x,y+.00012,.21915),
                (x,y+.00038,.21943), (x,y+.00071,.21929), (x,y+.00081,.2190),
            ])
    tube_bundle(asset, world, "moulded_hooks", hook_paths, .00014, (.13,.14,.12,1))
    for sign in (-1,1):
        y = -.074 + sign*.0221
        capsule(world, f"woven_selvedge_{sign}", (.023,y,.2178), (.147,y,.2178),
                .00085, (.207,.210,.17,1), **VISUAL)
        for i in range(30):
            x = .026 + i*.0041
            capsule(world, f"base_stitch_{sign}_{i}", (x,y,.2182), (x+.0025,y,.2182),
                    .00017, (.46,.44,.37,1), **VISUAL)
    rng = np.random.default_rng(138)
    palette = [(.48,.44,.38,1),(.53,.54,.49,1),(.62,.59,.53,1),
               (.39,.37,.34,1),(.42,.45,.46,1),(.67,.63,.56,1)]
    # These sit below the hook tips and remain after removing the loose lint.
    for k in range(62):
        x,y = rng.uniform(.026,.144),rng.uniform(-.091,-.056)
        dx,dy = rng.uniform(-.002,.002),rng.uniform(-.0014,.0014)
        capsule(world, f"embedded_short_fiber_{k}", (x,y,.2184),
                (x+dx,y+dy,.2189), .00009, palette[k%len(palette)], **VISUAL)
    for i in range(6):
        tuft = world.find(f"body[@name='tuft_{i}']")
        for g in tuft.findall("geom"):
            g.set("group", "5")
        spread, offset = ((.62,-.003),(1.01,.005),(.51,-.006),
                          (1.25,-.001),(.85,.006),(.51,-.004))[i]
        for tint, color in enumerate(palette):
            fibers = []
            for _ in range(8+int(16*spread)):
                angle = rng.uniform(0, math.tau)
                length = rng.uniform(.0021,.0088)*spread
                root = np.array((rng.normal(0,.0022), offset+rng.normal(0,.0041),
                                 rng.uniform(-.0021,-.0003)))
                fiber = [root]
                for k in range(1,7):
                    heading = angle + k*rng.uniform(.27,.55)
                    displacement = (math.cos(heading), math.sin(heading),
                                    .51*math.sin(k*math.pi/7))
                    fiber.append(fiber[-1]+length/6*np.array(displacement))
                fibers.append(fiber)
            tube_bundle(asset, tuft, f"lint_fibres_{i}_{tint}", fibers,
                        .000092 if tint % 2 else .000133, color)
        for k in range(8):
            geom(tuft, f"lint_felt_{i}_{k}", "ellipsoid",
                 (rng.normal(0,.0027)*spread,offset+rng.normal(0,.0034),
                  rng.uniform(-.0018,-.0005)),
                 (.0012*spread,.0014*spread,.00041), palette[(i+k)%len(palette)],
                 **VISUAL)
    brush = world.find("body[@name='brush']")
    hide(brush, "brush_handle", "brush_stock")
    geom(brush, "beech_shaped_grip", "ellipsoid", (0,0,.002),
         (.013,.011,.009), (.43,.27,.16,1), **VISUAL)
    box(brush, "brush_back", (0,0,-.020), (.0105,.026,.0040),
        (.40,.26,.16,1), **VISUAL)
    for sign in (-1,1):
        capsule(brush, f"brush_beech_edge_{sign}", (sign*.009,-.026,-.020),
                (sign*.009,.026,-.020), .002, (.51,.34,.21,1), **VISUAL)
    capsule(brush, "turned_brush_neck", (0,0,-.020), (0,0,.001),
            .006, (.45,.29,.16,1), **VISUAL)
    for i in range(11):
        for j in range(3):
            item = brush.find(f"geom[@name='bristle_{i}_{j}']")
            item.set("group", "5")
            x, y = (j-1)*.0046, (i-5)*.0043
            for k, (dx,dy) in enumerate(((-.00075,.0004),(.0003,.0004),(0,-.0007),
                                       (.00072,-.0002),(-.0002,.0008))):
                capsule(brush, f"nylon_fiber_{i}_{j}_{k}", (x+dx,y+dy,-.023),
                        (x+dx+.0024,y+dy-.0018,-.0385), .00029,
                        (.38,.37,.30,1) if k%2 else (.59,.54,.43,1), **VISUAL)
    flap = world.find("body[@name='flap']")
    hide(flap, "strap_tab", "clamp_arm", "loop_webbing",
         *(f"weave_{i}" for i in range(23)))
    cyl(flap, "knurled_clamp_handle", (0,0,0), .012,.008,
        (.26,.30,.32,1), **VISUAL)
    for k in range(16):
        t = k*math.tau/16
        geom(flap, f"handle_knurl_{k}", "sphere", (.011*math.cos(t),.011*math.sin(t),.006),
             (.0011,), ALUMINIUM, **VISUAL)
    capsule(flap, "clamp_link", (0,-.009,-.002), (0,-.111,-.006), .0035,
            (.47,.52,.54,1), **VISUAL)
    box(flap, "clamp_saddle", (-.002,-.112,-.009), (.009,.015,.002),
        (.36,.41,.44,1), **VISUAL)
    world.find("geom[@name='lint_collection_trough']").set("rgba", ".24 .27 .25 1")
    for sign in (-1,1):
        screw(world, f"webbing_mount_{sign}", .022 if sign < 0 else .146, -.101, .218)
        capsule(world, f"tray_flange_{sign}", (.021,-.127+sign*.017,.217),
                (.148,-.127+sign*.017,.2155), .0015, (.35,.39,.36,1), **VISUAL)
    box(world, "pulley_mount", (.215,-.064,.253), (.012,.006,.044),
        (.31,.36,.38,1), **VISUAL)
    cyl(world, "proof_pulley", (.224,-.074,.296), .010,.004, (.27,.31,.33,1),
        euler=(math.pi/2,0,0), **VISUAL)
    cyl(world, "proof_pulley_hub", (.224,-.079,.296), .003,.001, ALUMINIUM,
        euler=(math.pi/2,0,0), **VISUAL)
    bearing = world.find("body[@name='test_weight']")
    hide(bearing, "proof_mass")
    cyl(bearing, "steel_proof_mass", (0,0,0), .0163,.014, (.63,.68,.69,1), **VISUAL)
    for sign in (-1,1):
        cyl(bearing, f"weight_ring_{sign}", (0,0,sign*.0135),
            .0167,.001, (.42,.49,.52,1), **VISUAL)
    cyl(bearing, "weight_eye", (0,0,.018), .005,.003, ALUMINIUM, **VISUAL)
    box(world, "proof_drop_stop", (.234,-.074,.196), (.021,.022,.003), RUBBER, **VISUAL)


