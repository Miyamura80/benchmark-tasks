"""Hard-surface compact hatchback surrounding the cowl test surface."""

import math

import numpy as np

CAR_LIFT = .69
PAINT = ".18 .33 .40 1"
PAINT_LIGHT = ".23 .39 .45 1"
RUBBER = ".034 .037 .038 1"
GLASS = ".105 .18 .204 1"
CHROME = ".57 .60 .62 1"
LAMP = ".77 .85 .87 1"
TAIL = ".53 .044 .029 1"


def decorate_box(name, position, halfsize, rgba, material="", add=""):
    return (f'<geom name="{name}" type="box" pos="{fmt(position)}" size="{fmt(halfsize)}" '
            f'rgba="{rgba}" contype="0" conaffinity="0" {material} {add}/>')


def rod(a, b, radius, rgba, shape="capsule"):
    return (f'<geom type="{shape}" fromto="{fmt((*a, *b))}" size="{radius}" '
            f'rgba="{rgba}" contype="0" conaffinity="0"/>')


def fmt(values):
    return " ".join(f"{v:.5f}" for v in values)


def create_mesh(name, sections):
    vertices = [p for section in sections for p in section]
    n = len(sections[0])
    triangles = []
    for j in range(len(sections) - 1):
        for i in range(n):
            a, b = j*n+i, j*n+(i+1)%n
            c, d = a+n, b+n
            triangles.extend([(a, b, c), (b, d, c)])
    for i in range(1, n - 1):
        end = (len(sections)-1)*n
        triangles.extend([(0, i+1, i), (end, end+i, end+i+1)])
    points = np.asarray(vertices)
    points -= np.mean(points, axis=0)
    a, b, c = points[np.asarray(triangles)].transpose(1, 0, 2)
    signed_volume = np.sum(a * np.cross(b, c))
    if signed_volume < 0:
        triangles = [(a, c, b) for a, b, c in triangles]
    verts = " ".join(fmt(v) for v in vertices)
    faces = " ".join(" ".join(map(str, t)) for t in triangles)
    return f'<mesh name="{name}" vertex="{verts}" face="{faces}"/>'


def use_mesh(name, rgba, material="paint"):
    return (f'<geom type="mesh" mesh="{name}" material="{material}" rgba="{rgba}" '
            'contype="0" conaffinity="0"/>')


def car_parts():
    anchors = np.array([
        (-1.515, .61, .42, .52),
        (-1.46, .725, .505, .63),
        (-1.33, .77, .58, .68),
        (-1.02, .775, .617, .724),
        (-.58, .765, .671, .782),
        (-.36, .754, .719, .813),
        (-.03, .746, .755, .829),
        (.19, .746, .777, .825),
        (.53, .750, .792, .829),
        (1.49, .748, .795, .831),
        (1.96, .732, .775, .808),
        (2.16, .67, .697, .769),
        (2.24, .572, .562, .754),
    ])
    axles = (-.91, 1.58)
    body_sections = []
    for y in np.linspace(anchors[0, 0], anchors[-1, 0], 137):
        halfwidth, shoulder, top = [np.interp(y, anchors[:,0], anchors[:,i]) for i in range(1, 4)]
        if -.36 < y < .21:
            # Body shoulders rise outside the open cowl, while the center stays
            # below the gutter floor. The task owns the hood and glass center.
            top = min(top, CAR_LIFT + .091)
        bottom = .227 + .037 * (abs(y + .28) / 2.52)
        for axle in axles:
            distance = (y - axle) / .295
            if abs(distance) < 1:
                bottom = max(bottom, .28 + .326 * math.sqrt(1 - distance*distance))
        body_sections.append([
            (-halfwidth*.85, y, bottom-.010),
            (halfwidth*.85, y, bottom-.010),
            (halfwidth*.98, y, bottom+.015),
            (halfwidth, y, shoulder-.026),
            (halfwidth*.97, y, shoulder+.025),
            (halfwidth*.85, y, top-.022),
            (halfwidth*.48, y, top-.010),
            (-halfwidth*.48, y, top-.010),
            (-halfwidth*.85, y, top-.022),
            (-halfwidth*.97, y, shoulder+.025),
            (-halfwidth, y, shoulder-.026),
            (-halfwidth*.98, y, bottom+.015),
        ])

    roof_anchors = np.array([
        (.79, .513, .897, 1.271),
        (.91, .521, .897, 1.294),
        (1.29, .527, .902, 1.327),
        (1.67, .51, .908, 1.294),
        (1.91, .492, .94, 1.198),
        (2.11, .468, .941, 1.039),
        (2.16, .441, .913, .944),
    ])
    roof_sections = []
    for y in np.linspace(.79, 2.16, 57):
        width, base, z = [np.interp(y, roof_anchors[:,0], roof_anchors[:,i]) for i in range(1,4)]
        roof_sections.append([
            (-width-.119, y, base-.08),
            (width+.119, y, base-.08),
            (width+.119, y, base-.04),
            (width+.102, y, base+.007),
            (width, y, z-.035),
            (width*.89, y, z-.007),
            (width*.49, y, z+.003),
            (-width*.49, y, z+.003),
            (-width*.89, y, z-.007),
            (-width, y, z-.035),
            (-width-.102, y, base+.007),
            (-width-.119, y, base-.04),
        ])
    assets = [create_mesh("hatch_body", body_sections), create_mesh("hatch_cabin", roof_sections)]
    parts = [use_mesh("hatch_body", PAINT), use_mesh("hatch_cabin", PAINT)]
    windows = [
        (.41, .80, .557, .512, .974, 1.242),
        (.82, 1.31, .513, .527, 1.247, 1.287),
        (1.34, 1.84, .525, .493, 1.283, 1.196),
        (1.84, 2.04, .493, .474, 1.195, 1.055),
    ]
    for side in (-1,1):
        # Fill the front door and lower A-pillar area below the raked screen.
        pillar = [[(side*.606,.39,.86), (side*.666,.39,.827),
                   (side*.666,.39,.891), (side*.591,.39,1.020)],
                  [(side*.523,.823,.884), (side*.656,.823,.826),
                   (side*.656,.823,.937), (side*.511,.823,1.273)]]
        name = f"front_cabin_side_{side+1}"
        assets.append(create_mesh(name, pillar))
        parts.append(use_mesh(name, PAINT))
    for side in (-1, 1):
        for i, (y0, y1, w0, w1, z0, z1) in enumerate(windows):
            xc0, xc1 = side * w0, side * w1
            lower0 = .897 if i == 0 else .947
            name = f"cab_glass_{side+1}_{i}"
            section0 = [(xc0+side*.003,y0,z0), (xc0+side*.106,y0,lower0),
                        (xc0+side*.082,y0,lower0), (xc0-side*.016,y0,z0)]
            section1 = [(xc1+side*.009,y1,z1), (xc1+side*.108,y1,.948),
                        (xc1+side*.084,y1,.948), (xc1-side*.014,y1,z1)]
            assets.append(create_mesh(name, [section0, section1]))
            parts.append(use_mesh(name, GLASS, "glass"))
            parts.append(rod(section0[1],section1[1],.0038,RUBBER))
        for y in (.76,1.57):
            x = side*.744
            parts.append(decorate_box(f"door_handle_{side+1}_{y}",(x,y,.764),(.011,.045,.008),PAINT_LIGHT))
            parts.append(rod((x,y+.10,.748),(x+side*.004,y+.082,.388),.002,".063 .105 .12 1"))
        mirror_mount = (side*.62, .367, .915)
        mirror_center = (side*.742, .376, .946)
        parts.append(rod(mirror_mount, mirror_center, .023, RUBBER))
        parts.append(decorate_box(f"mirror_{side+1}",mirror_center,(.051,.034,.024),PAINT_LIGHT,
                                  'material="paint"', f'euler="0 0 {side*10}"'))
        for axle in axles:
            x = side * .705
            face = side * .797
            parts.append(rod((x-side*.077,axle,.259), (face,axle,.259), .263, RUBBER, "cylinder"))
            parts.append(rod((face,axle,.259), (face+side*.006,axle,.259), .171, CHROME, "cylinder"))
            parts.append(rod((face+side*.008,axle,.259), (face+side*.011,axle,.259), .147,
                             ".12 .135 .145 1", "cylinder"))
            for k in range(6):
                a = k*math.tau/6
                parts.append(rod((face+side*.019,axle+.038*math.cos(a),.259+.038*math.sin(a)),
                                 (face+side*.019,axle+.153*math.cos(a),.259+.153*math.sin(a)),
                                 .013, CHROME))
            parts.append(rod((face+side*.017,axle,.259),(face+side*.023,axle,.259),
                             .051,CHROME,"cylinder"))
            for j in range(25):
                t0, t1 = j*math.pi/25,(j+1)*math.pi/25
                parts.append(rod((face+side*.002,axle+.283*math.cos(t0),.264+.286*math.sin(t0)),
                                 (face+side*.002,axle+.283*math.cos(t1),.264+.286*math.sin(t1)),
                                 .007,RUBBER))
        parts.append(rod((side*.32,-1.508,.546),(side*.669,-1.392,.603),.021,LAMP))
        parts.append(rod((side*.41,-1.50,.536),(side*.66,-1.392,.586),.016,".26 .31 .34 1"))
        parts.append(rod((side*.39,2.20,.694),(side*.648,2.144,.741),.026,TAIL))
        parts.append(rod((side*.43,2.205,.679),(side*.638,2.150,.728),.009,".90 .14 .081 1"))
    # Lower bumper and air intake are inset under the leading body shoulder.
    parts.append(decorate_box("front_grille",(0,-1.493,.407),(.37,.039,.055),RUBBER,
                              add='euler="-5 0 0"'))
    for z in (.373,.394,.415,.436):
        parts.append(rod((-.343,-1.536,z),(.343,-1.536,z),.004,".14 .15 .155 1","cylinder"))
    parts.append(decorate_box("front_plate",(0,-1.548,.493),(.112,.006,.029),".76 .79 .75 1"))
    parts.append(decorate_box("rear_plate",(0,2.241,.557),(.112,.008,.031),".76 .79 .75 1"))
    parts.append(rod((-.50,2.212,.401),(.50,2.212,.401),.030,RUBBER))
    parts.append(rod((-.34,1.86,1.209),(.34,1.86,1.209),.008,TAIL))
    return assets, parts
