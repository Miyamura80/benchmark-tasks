"""Native mating parts and press controls for the encoder fixture."""
from math import cos, degrees, pi, sin

from native import asset, collider
import task


PARTS = {"encoder_frame", "magnet_carrier", "sensor_bracket", "encoder_catch", "field_button"}
GUIDE_OFFSETS = (-.038, .038)
CATCH_MASS, BUTTON_MASS = .019, .008


def author(directory):
    from pxr import Gf, UsdGeom

    paths = {name: directory / f"{name}.usda" for name in PARTS}
    for path in paths.values():
        path.unlink(missing_ok=True)

    alloy, bracket, grip = (.61,.65,.70), (.16,.39,.27), (.85,.57,.09)

    def box(stage, name, center, size, color, angle=0., friction=None):
        shape = UsdGeom.Cube.Define(stage, f"/Object/{name}")
        shape.CreateSizeAttr(1.)
        shape.AddTranslateOp().Set(Gf.Vec3d(*center))
        shape.AddRotateZOp().Set(angle)
        shape.AddScaleOp().Set(Gf.Vec3f(*size))
        shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        collider(shape, friction)

    def cylinder(stage, name, center, radius, height, color):
        shape = UsdGeom.Cylinder.Define(stage, f"/Object/{name}")
        shape.CreateRadiusAttr(radius)
        shape.CreateHeightAttr(height)
        shape.AddTranslateOp().Set(Gf.Vec3d(*center))
        shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        collider(shape)

    def sleeve(stage, name, center, bore, length, wall, color, keyed=False):
        # Twelve convex contact blocks surround an open bore. A single flat
        # mates with the +X shaft flat when the carrier has the correct yaw.
        for i in range(12):
            angle = i * pi / 6
            internal = bore - .001 if keyed and i == 0 else bore
            radial = internal + wall / 2
            p = (center[0] + radial*cos(angle), center[1] + radial*sin(angle), center[2])
            box(stage, f"{name}{i}", p, (wall, 2*radial*.268+.00025, length), color, degrees(angle))

    stage = asset(paths["encoder_frame"])
    x, y = task.CENTER
    cylinder(stage, "MotorHousing", (x,y,(task.DECK+task.MOTOR_TOP)/2), .040, task.MOTOR_TOP-task.DECK, (.10,.29,.42))
    # Polygonal D shaft with a true flattened face and thickness contacts.
    shaft = UsdGeom.Mesh.Define(stage, "/Object/KeyedShaft")
    radius, flat = .004, .003
    rim = [(x+min(flat,radius*cos(i*pi/12)), y+radius*sin(i*pi/12)) for i in range(24)]
    points = [(a,b,z) for z in (task.MOTOR_TOP,task.SHAFT_TOP) for a,b in rim]
    faces = [tuple(reversed(range(24))), tuple(range(24,48))]
    faces.extend((i, (i+1)%24, (i+1)%24+24, i+24) for i in range(24))
    shaft.CreatePointsAttr([Gf.Vec3f(*v) for v in points])
    shaft.CreateFaceVertexCountsAttr([len(f) for f in faces])
    shaft.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    shaft.CreateDisplayColorAttr([Gf.Vec3f(*alloy)])
    shaft.CreateSubdivisionSchemeAttr("none")
    collider(shaft)
    from pxr import UsdPhysics
    UsdPhysics.MeshCollisionAPI.Apply(shaft.GetPrim()).CreateApproximationAttr("convexHull")
    for i, offset in enumerate(GUIDE_OFFSETS):
        cylinder(stage, f"Guide_{i}", (x,y+offset,task.TOP+.087), .004, .053, alloy)
        cylinder(stage, f"Shoulder_{i}", (x,y+offset,task.HEAD_SEAT[2]-.0525), .009, .037, (.19,.23,.25))
    box(stage, "CatchHousing", (*task.LATCH_XY,task.LATCH_OPEN-.037), (.028,.029,.040), alloy)
    box(stage, "ButtonHousing", (*task.TEST_XY,task.TEST_CAP-.029), (.062,.051,.040), (.20,.25,.28))
    stage.GetRootLayer().Save()

    stage = asset(paths["magnet_carrier"], .019)
    # Bore ceiling lands on the shaft tip when the magnet is at the right height.
    # Keep the grasp flange below the sensor plane. Its underside is the
    # stop face at the shaft tip, and its upper face is the magnet datum.
    box(stage, "CarrierGrip", (0,0,task.CAP_GRIP_Z_OFFSET), (.020,.020,.004),
        (.79,.53,.13), friction=1.1)
    sleeve(stage, "CarrierBore", (0,0,-.0144), .0045, .0148, .004,
           (.79,.53,.13), keyed=True)
    stage.GetRootLayer().Save()

    stage = asset(paths["sensor_bracket"], .056)
    box(stage, "PCB", (0,0,-.0236), (.047,.043,.0028), bracket)
    box(stage, "Crossbar", (0,0,-.018), (.067,.053,.008), alloy)
    for i, offset in enumerate(GUIDE_OFFSETS):
        sleeve(stage, f"GuideBore{i}_", (0.,offset,-.025), .0047, .018, .003, alloy)
        for j, dx in enumerate((-.013,.013)):
            box(stage, f"BridgeLink{i}_{j}", (dx, offset*.83, -.018), (.015,.039,.008), alloy)
    # Tab enters the gap beneath the lowered retaining tongue.
    box(stage, "LatchTab", (.049,-.050,-.019), (.022,.017,.006), alloy)
    box(stage, "BracketGrip", (0,0,0), (.026,.025,.017), grip, friction=1.1)
    stage.GetRootLayer().Save()

    stage = asset(paths["encoder_catch"], CATCH_MASS)
    box(stage, "RedCatch", (0,0,-.003), (.025,.021,.008), (.63,.07,.08))
    box(stage, "LatchTongue", (-.010,.012,-.005), (.028,.014,.005), (.27,.29,.30))
    stage.GetRootLayer().Save()

    stage = asset(paths["field_button"], BUTTON_MASS)
    cylinder(stage, "BlueTestKey", (0,0,-.003), .009, .008, (.13,.38,.72))
    stage.GetRootLayer().Save()
    return paths


def connect_slide(stage, fixture, body, origin, stroke, stiffness):
    """Anchor a native vertical slider to the static world before detection."""
    from pxr import Gf, UsdPhysics

    joint = UsdPhysics.PrismaticJoint.Define(stage, f"{fixture}/{body.name}Slide")
    joint.CreateBody1Rel().SetTargets([body])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*origin))
    joint.CreateAxisAttr("Z")
    joint.CreateLowerLimitAttr(-stroke)
    joint.CreateUpperLimitAttr(0.)
    api = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
    api.CreateStiffnessAttr(stiffness)
    api.CreateDampingAttr(.9)
    api.CreateTargetPositionAttr(0.)
    return api
