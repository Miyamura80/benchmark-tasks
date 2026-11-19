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










def blind(tree, world, assets, roller_z):
    asset = tree.find("asset")
    material = png_texture(asset, assets, "shade_linen",
                           woven_texture((220,207,184),4),
                           repeat=(8,6), specular=.025)
    roller = world.find("body[@name='shade_roller']")
    shade_roll = roller.find("geom[@name='cloth_roll']")
    shade_roll.set("material", material)
    shade_roll.set("rgba", "1 1 1 1")
    side = .195
    post = (.32,.34,.32,1)
    for x in (-.095,.089):
        hide(world, f"stand_{x}", f"foot_{x}")
        box(world, f"foot_plate_{x}", (x,.055,.205), (.021,.039,.0045),
            post, **VISUAL)
        box(world, f"square_upright_{x}", (x,.071,.321), (.009,.010,.115),
            post, **VISUAL)
        box(world, f"upright_inset_{x}", (x,.0607,.325), (.0035,.0005,.104),
            (.23,.24,.22,1), **VISUAL)
        for offset in (-.023,.022):
            screw(world,f"stand_mount_{x}_{offset}",x,.055+offset,.210)
        capsule(world, f"brace_{x}", (x,.038,.215),(x,.065,.272),.004,
                post, **VISUAL)
    box(world, "blind_headrail", (.067,.049,roller_z+.019), (.144,.016,.012),
        (.65,.67,.65,1), **VISUAL)
    capsule(world, "headrail_lip", (-.079,.032,roller_z+.028),
            (.210,.032,roller_z+.028), .002, (.54,.56,.55,1), **VISUAL)
    for x in (-.061,.208):
        box(world, f"awning_bracket_{x}", (x,.020,roller_z+.009),
            (.0044,.045,.022), (.70,.71,.67,1), **VISUAL)
        screw(world, f"awning_mount_{x}", x,.048,roller_z+.024, radius=.0024)
    cyl(roller,"spindle_extension",(.121,0,0),.010,.030,ALUMINIUM,
        euler=(0,math.pi/2,0),**VISUAL)
    cyl(roller,"left_tube_end",(-.100,0,0),.027,.003,ABS,
        euler=(0,math.pi/2,0),**VISUAL)
    for side_x in (-.097,.094):
        cyl(roller, f"fabric_roll_edge_{side_x}", (side_x,0,0),
            .0324,.0018, (.72,.67,.57,1),euler=(0,math.pi/2,0),**VISUAL)
    for g in world.findall("geom"):
        if g.get("name","").startswith("clutch_cheek_"):
            g.set("group","5")
    # Molded clutch walls leave an opening where each run of the chain exits.
    for sign in (-1,1):
        x = side+sign*.008
        cyl(world, f"clutch_cheek_disk_{sign}", (x,-.014,roller_z), .032,.0031,
            ABS, euler=(0,math.pi/2,0), **VISUAL)
        cyl(world, f"clutch_axle_cap_{sign}", (x+sign*.0036,-.014,roller_z), .013,.0024,
            (.68,.69,.64,1), euler=(0,math.pi/2,0), **VISUAL)
    for i in range(20):
        a = .04+i*math.pi/20
        b = a+math.pi/20
        capsule(world, f"clutch_guard_arc_{i}",
                (side,-.014+.035*math.cos(a),roller_z+.035*math.sin(a)),
                (side,-.014+.035*math.cos(b),roller_z+.035*math.sin(b)),
                .0042, ABS, **VISUAL)
    hem = world.find("body[@name='hem']")
    hide(hem,"bottom_rail")
    box(hem,"bottom_extrusion", (0,0,0), (.097,.0030,.0047),
        (.53,.56,.54,1),**VISUAL)
    for z in (-.0037,.0037):
        capsule(hem,f"hem_lip_{z}",(-.095,-.0031,z),(.095,-.0031,z),
                .00082,ALUMINIUM,**VISUAL)
    for sign in (-1,1):
        geom(hem,f"hem_endcap_{sign}","ellipsoid",(sign*.099,0,0),
             (.0025,.0051,.0051), ABS, **VISUAL)
    for name in ("down_chain","up_chain","terminal","coupler"):
        obj = world.find(f"body[@name='{name}']")
        for g in obj.findall("geom"):
            g.set("group","5")
    world.find("geom[@name='connector_cradle']").set("rgba", ".34 .42 .45 1")
    for sign in (-1,1):
        box(world,f"service_jig_side_{sign}", (.124+sign*.016,-.099,.227),
            (.0024,.016,.005), (.48,.53,.56,1), **VISUAL)
        screw(world,f"jig_mount_{sign}",.124+sign*.014,-.081,.227)
