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




def foil_texture(worn):
    rng = np.random.default_rng(32 if worn else 45)
    y, x = np.indices((384, 768))
    # Brushed stainless catches a broad reflection at the crown and narrow
    # reflections along the two shoulders. Keep the pores identical in size.
    shade = (142 + 64*np.exp(-((y-105)/76)**2)
             + 30*np.exp(-((y-286)/26)**2) + 5*np.sin(x/21))
    if worn:
        shade -= (36*np.exp(-((x-207)**2/27500+(y-129)**2/7300))
                  + 28*np.exp(-((x-541)**2/24500+(y-245)**2/9000)))
        shade -= 8*(np.sin(x/3.2)*np.cos(y/13) > .85)
    shade += rng.normal(0, 2.0, size=shade.shape)
    shade += rng.normal(0, 2.2, size=(384,1))
    channels = np.stack([shade + (8 if worn else -4), shade, shade + (-6 if worn else 6)], axis=-1)
    return Image.fromarray(np.uint8(np.clip(channels, 0, 255)))


def shell(asset, parent, name, center_x, stations, color):
    count = 36
    vertices = []
    for y, width, mid_z, thickness in stations:
        for angle in np.arange(count)*math.tau/count:
            vertices.append((center_x + width*math.cos(angle), y,
                             mid_z + thickness*math.sin(angle)))
    faces = []
    for row in range(len(stations)-1):
        start = row*count
        for i in range(count):
            a, b = start+i, start+(i+1) % count
            faces.extend([(a, a+count, b), (b, a+count, b+count)])
    for n in range(1, count-1):
        faces.extend([(0, n, n+1),
                      ((len(stations)-1)*count, (len(stations)-1)*count+n+1,
                       (len(stations)-1)*count+n)])
    mesh(asset, parent, name, vertices, faces, color)


def arch(asset, parent, name, length, radius, height, bottom, material):
    """Make a continuous domed foil with staggered through-holes."""
    vertices, uv, faces = [], [], []

    def point(u, v):
        x, angle = (2*u-1)*length, v*math.pi
        curve = 1.0 - .035*(abs(x)/length)**7
        vertices.append((x,radius*curve*math.cos(angle),
                         bottom+height*curve*math.sin(angle)))
        uv.append((u,v))

    perimeter = [(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)]
    cols, rows = 42, 27
    for row in range(rows):
        for col in range(cols):
            u,v = (col+.5)/cols,(row+.5)/rows
            idx = len(vertices)
            for a,b in perimeter:
                point(u+a*.5/cols,v+b*.5/rows)
            if row in (0,rows-1) or col in (0,cols-1):
                point(u,v)
                faces.extend((idx+k,idx+8,idx+(k+1)%8) for k in range(8))
                continue
            skew = (.13 if row%2 else -.13)/cols
            for k in range(8):
                a = k*math.tau/8
                point(u+skew+.29*math.cos(a)/cols,v+.30*math.sin(a)/rows)
            for k in range(8):
                a,b = idx+k,idx+(k+1)%8
                faces.extend([(a,a+8,b),(b,a+8,b+8)])
    mesh(asset, parent, name, vertices, faces, (1, 1, 1, 1), uv, material)


def screw(parent, name, x, y, z, radius=.003):
    cyl(parent, name, (x,y,z), radius, .00065, ALUMINIUM, **VISUAL)
    box(parent, name+"_slot", (x,y,z+.00072), (radius*.64,.00036,.00012), RUBBER,
        euler=(0,0,.35), **VISUAL)


def rounded_tray(asset, parent, name, x, y, z):
    rings = []
    for half_x, half_y, corner, height in (
        (.039, .024, .006, 0), (.041, .026, .008, .0024),
        (.041, .026, .008, .007), (.038, .023, .0068, .007),
        (.036, .021, .005, .0018),
    ):
        ring = []
        for sx, sy, a in ((1,1,0),(-1,1,90),(-1,-1,180),(1,-1,270)):
            for angle in np.radians(np.linspace(a, a+90, 7)):
                ring.append((x+sx*(half_x-corner)+corner*math.cos(angle),
                             y+sy*(half_y-corner)+corner*math.sin(angle), z+height))
        rings.append(ring)
    size = len(rings[0])
    faces = []
    for row in range(len(rings)-1):
        for i in range(size):
            a, b = row*size+i, row*size+(i+1)%size
            faces.extend([(a,b,a+size),(b,b+size,a+size)])
    last = (len(rings)-1)*size
    faces.extend((last,last+i,last+i+1) for i in range(1,size-1))
    mesh(asset, parent, name, sum(rings, []), faces, (.209,.223,.221,1))
    for k in range(9):
        xx = x+(k-4)*.0073
        capsule(parent, f"{name}_liner_{k}", (xx,y-.018,z+.002), (xx,y+.018,z+.002),
                .00035, (.286,.294,.275,1), **VISUAL)


def shaver(tree, world, assets):
    asset = tree.find("asset")
    hide(world, "rubber_cradle", "shaver_case", "head_seat", "foil_guide_-1", "foil_guide_1",
         *(f"body_fluting_{i}" for i in range(5)))
    element(asset, "material", name="shaver_satin", specular=".42",
            shininess=".3", rgba=(1,1,1,1))
    shell(asset, world, "shaver_ergonomic_housing", .080, [
        (-.106,.006,.234,.005), (-.101,.017,.234,.009),
        (-.091,.024,.234,.012), (-.073,.026,.235,.013),
        (-.054,.025,.235,.014), (-.039,.024,.235,.014),
        (-.022,.025,.235,.015), (-.009,.030,.236,.015),
        (.004,.039,.235,.014), (.020,.043,.235,.013),
        (.036,.041,.235,.011), (.041,.037,.235,.008),
    ], (.058,.087,.093,1))
    world.find("geom[@name='shaver_ergonomic_housing']").set("material", "shaver_satin")
    shell(asset, world, "shaver_soft_grip", .080, [
        (-.099,.004,.244,.002), (-.094,.012,.244,.0036),
        (-.082,.017,.246,.0051), (-.064,.018,.246,.0058),
        (-.044,.017,.246,.0064), (-.026,.016,.246,.0061),
        (-.013,.010,.247,.004), (-.010,.002,.247,.002),
    ], (.065,.066,.064,1))
    for k in range(14):
        y = -.091 + k*.0041
        for sign in (-1,1):
            capsule(world, f"grip_moulding_{k}_{sign}", (.080+sign*.009,y,.250),
                    (.080+sign*.016,y+.0015,.2484), .00052, (.108,.113,.110,1),
                    **VISUAL)
    for sign in (-1,1):
        x = .080+sign*.025
        for i, ((y0,dx0), (y1,dx1)) in enumerate(zip(
            ((-.091,-.002),(-.067,0),(-.040,-.001),(-.015,.003),(.010,.014)),
            ((-.067,0),(-.040,-.001),(-.015,.003),(.010,.014),(.035,.014)),
        )):
            capsule(world, f"shell_seam_{sign}_{i}",
                    (x+sign*dx0,y0,.237),(x+sign*dx1,y1,.237),
                    .00074, (.27,.32,.32,1), **VISUAL)
        capsule(world, f"cradle_rib_{sign}", (.080+sign*.019,-.078,.218),
                (.080+sign*.019,-.022,.218), .0045, (.108,.117,.115,1), **VISUAL)
        screw(world, f"head_screw_{sign}", .084+sign*.036, .019, .251)
        box(world, f"head_latch_return_{sign}", (.084+sign*.043,.019,.249),
            (.003,.007,.0028), (.38,.43,.44,1), **VISUAL)
    box(world, "cradle_steel_base", (.080,-.025,.2045), (.041,.073,.0045),
        (.115,.131,.129,1), **VISUAL)
    for end in (-.072, .022):
        capsule(world, f"rubber_cross_support_{end}", (.049,end,.213), (.111,end,.213),
                .008, (.12,.14,.13,1), **VISUAL)
    capsule(world, "shaver_collar_trim", (.046,.032,.246), (.122,.032,.246),
            .0050, (.40,.46,.46,1), **VISUAL)
    box(world, "shaver_cutout", (.084,.018,.249), (.035,.018,.0017), RUBBER, **VISUAL)
    # A paired socket and pins show where the reciprocating cutter docks.
    for sign in (-1,1):
        cyl(world, f"seat_pin_{sign}", (.084+sign*.025,.018,.252),
            .0025,.0028, (.48,.50,.49,1), **VISUAL)
    cyl(world, "shaver_charging_port", (.080,-.103,.233), .0045,.0009,
        (.26,.30,.28,1), euler=(math.pi/2,0,0), **VISUAL)
    for name, worn in (("worn_foil", True), ("fresh_foil", False)):
        part = world.find(f"body[@name='{name}']")
        for item in list(part.findall("geom")):
            item.set("group", "5")
        material = png_texture(asset, assets, name+"_perforated_metal",
                               foil_texture(worn), specular=.48, shiny=.61)
        arch(asset, part, name+"_foil_skin", .039, .019, .022, .014, material)
        frame_color = (.28,.26,.23,1) if worn else (.14,.18,.18,1)
        for sign in (-1,1):
            capsule(part, f"{name}_lip_{sign}", (-.041,sign*.020,.011),
                    (.041,sign*.020,.011), .0026, frame_color, **VISUAL)
            capsule(part, f"{name}_folded_seam_{sign}", (-.039,sign*.0194,.015),
                    (.039,sign*.0194,.015), .00065, (.61,.64,.64,1), **VISUAL)
            end = sign*.041
            points = [np.array((end,.020*math.cos(t),.014+.021*math.sin(t)))
                      for t in np.linspace(0, math.pi, 17)]
            for i, (a, b) in enumerate(zip(points, points[1:])):
                capsule(part, f"{name}_rim_{sign}_{i}", a, b, .0016, frame_color, **VISUAL)
            end_plate = [(end,0,.0125), *(tuple(point) for point in points)]
            end_faces = [(0,i,i+1) for i in range(1,len(end_plate)-1)]
            # Give the end plate finite thickness for MuJoCo's mesh bounds.
            back = [(x-sign*.002,y,z) for x,y,z in end_plate]
            count = len(end_plate)
            panel_faces = end_faces + [(c+count,b+count,a+count) for a,b,c in end_faces]
            for i in range(count):
                j = (i+1)%count
                panel_faces.extend([(i,i+count,j),(j,i+count,j+count)])
            if sign > 0:
                panel_faces = [(a,c,b) for a,b,c in panel_faces]
            mesh(asset, part, f"{name}_end_plate_{sign}", end_plate+back, panel_faces,
                 frame_color)
            box(part, f"{name}_real_latch_{sign}", (end,0,.004),
                (.0028,.006,.0035), ALUMINIUM, **VISUAL)
            box(part, f"{name}_thumb_release_{sign}", (end,0,.019),
                (.0025,.008,.0037), frame_color, **VISUAL)
            for i in (-1,0,1):
                capsule(part, f"{name}_thumb_rib_{sign}_{i}",
                        (end+sign*.0026,i*.0027,.017),
                        (end+sign*.0026,i*.0027,.021), .00035,
                        (.38,.41,.41,1), **VISUAL)
    for name in ("worn_cutter", "fresh_cutter"):
        part = world.find(f"body[@name='{name}']")
        for item in part.findall("geom"):
            if "_tooth" in item.get("name") or "_bridge" in item.get("name"):
                item.set("group", "5")
        dark = (.30,.29,.27,1)
        metal = (.64,.60,.53,1) if name == "worn_cutter" else ALUMINIUM
        capsule(part, name+"_central_carriage", (-.027,0,.008), (.027,0,.008),
                .0045, dark, **VISUAL)
        for i in range(28):
            x = -.027+i*.002
            for j, t in enumerate(np.linspace(.04, math.pi-.04, 10)[:-1]):
                u = t + (math.pi-.08)/9
                capsule(part, f"{name}_blade_{i}_{j}",
                        (x,.010*math.cos(t),.008+.0076*math.sin(t)),
                        (x,.010*math.cos(u),.008+.0076*math.sin(u)),
                        .00052, metal, **VISUAL)
    switch = world.find("body[@name='switch']")
    hide(switch, "switch_top")
    geom(switch, "shaver_switch_cap", "ellipsoid", (0,0,.001), (.012,.008,.004),
         RUBBER, **VISUAL)
    for j in range(16):
        a = math.pi*.67 + j*math.pi*1.66/16
        b = a + math.pi*1.66/16
        capsule(switch, f"power_mark_{j}", (.003*math.sin(a),.003*math.cos(a),.005),
                (.003*math.sin(b),.003*math.cos(b),.005), .00027, ALUMINIUM, **VISUAL)
    capsule(switch, "power_mark_stem", (0,.0012,.005), (0,.004,.005), .00030,
            ALUMINIUM, **VISUAL)
    for index, tray in enumerate([g for g in world.findall("geom")
                                  if g.get("name", "").startswith("tray_")]):
        x, y, z = map(float, tray.get("pos").split())
        tray.set("group", "5")
        rounded_tray(asset, world, f"shaver_parts_tray_{index}", x, y, z+.004)






