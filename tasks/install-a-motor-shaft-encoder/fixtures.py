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

    def sleeve(stage, name, center, bore, length, wall, color, keyed=False, lead=.002):
        # Twelve convex contact blocks surround an open bore. A single flat
        # mates with the +X shaft flat when the carrier has the correct yaw.
        # Stack three concentric rings to form a stepped lead-in below the
        # locating section without changing the seated axial stop.
        rows = ((.52, .0), (.30, lead / 2), (.18, lead))
        top = center[2] + length / 2
        for row, (fraction, extra) in enumerate(rows):
            depth = length * fraction
            for i in range(12):
                angle = i * pi / 6
                opening = bore + extra
                internal = opening - .001 if keyed and i == 0 else opening
                radial = internal + wall / 2
                p = (center[0] + radial*cos(angle), center[1] + radial*sin(angle), top - depth/2)
                box(stage, f"{name}{row}_{i}", p, (wall, 2*radial*.268+.00025, depth), color, degrees(angle))
            top -= depth


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
        # Machined lead-in on the exposed end of each steel guide pin.
        guide_z_top = task.TOP+.1135
        guide_z_shoulder = guide_z_top-.014
        cylinder(stage, f"Guide_{i}", (x,y+offset,(task.TOP+.0605+guide_z_shoulder)/2),
                 .004, guide_z_shoulder-(task.TOP+.0605), alloy)
        taper = UsdGeom.Mesh.Define(stage, f"/Object/GuideTaper_{i}")
        rings = [[(x+radius*cos(j*pi/8), y+offset+radius*sin(j*pi/8), z)
                  for j in range(16)] for radius,z in ((.004,guide_z_shoulder),(.001,guide_z_top))]
        faces = [tuple(reversed(range(16))), tuple(range(16,32))]
        faces += [(j,(j+1)%16,(j+1)%16+16,j+16) for j in range(16)]
        taper.CreatePointsAttr([Gf.Vec3f(*point) for ring in rings for point in ring])
        taper.CreateFaceVertexCountsAttr([len(face) for face in faces])
        taper.CreateFaceVertexIndicesAttr([j for face in faces for j in face])
        taper.CreateDisplayColorAttr([Gf.Vec3f(*alloy)])
        taper.CreateSubdivisionSchemeAttr("none")
        collider(taper)
        UsdPhysics.MeshCollisionAPI.Apply(taper.GetPrim()).CreateApproximationAttr("convexHull")
        cylinder(stage, f"Shoulder_{i}", (x,y+offset,task.HEAD_SEAT[2]-.0525), .009, .037, (.19,.23,.25))
    # Support the crossbar outboard of the PCB so the PCB can pass between the rails.
    rail_top = task.HEAD_SEAT[2] - .022
    for i, shift in enumerate((-.031, .031)):
        box(stage, f"CrossbarStop_{i}", (x + shift, y, rail_top - .011),
            (.004, .036, .022), alloy)
    catch_x, catch_y, catch_z = task.CATCH_OPEN
    box(stage, "CatchHousing", (catch_x-task.CATCH_SHIFT/2, catch_y, task.CATCH_CLOSED[2]-.031),
        (.046,.029,.040), alloy)
    box(stage, "ButtonHousing", (*task.TEST_XY,task.TEST_CAP-.036), (.062,.051,.040), (.20,.25,.28))
    stage.GetRootLayer().Save()

    stage = asset(paths["magnet_carrier"], .019)
    # Bore ceiling lands on the shaft tip when the magnet is at the right height.
    # Keep the grasp flange below the sensor plane. Its underside is the
    # stop face at the shaft tip, and its upper face is the magnet datum.
    box(stage, "CarrierGrip", (0,0,task.CAP_GRIP_Z_OFFSET), (.020,.020,.004),
        (.79,.53,.13), friction=1.1)
    sleeve(stage, "CarrierBore", (0,0,-.0144), .0055, .0148, .004,
           (.79,.53,.13), keyed=True)
    stage.GetRootLayer().Save()

    stage = asset(paths["sensor_bracket"], .056)
    box(stage, "PCB", (0,0,-.0236), (.047,.043,.0028), bracket)
    box(stage, "Crossbar", (0,0,-.018), (.067,.053,.008), alloy)
    for i, offset in enumerate(GUIDE_OFFSETS):
        sleeve(stage, f"GuideBore{i}_", (0.,offset,-.025), .0058, .018, .003, alloy)
        for j, dx in enumerate((-.016,.016)):
            box(stage, f"BridgeLink{i}_{j}", (dx, offset*.83, -.018), (.015,.039,.008), alloy)
    # The open catch sits outboard while this tab descends onto the fixture.
    box(stage, "TabArm", (.0345, -.049, -.019), (.026, .012, .006), alloy)
    box(stage, "LatchTab", task.TAB_POS, task.TAB_SIZE, alloy)
    box(stage, "GripPedestal", (0, 0, -.011), (.020, .020, .008), grip)
    box(stage, "BracketGrip", (0,0,0), (.026,.025,.017), grip, friction=1.1)
    stage.GetRootLayer().Save()

    stage = asset(paths["encoder_catch"], CATCH_MASS)
    box(stage, "RedCatch", (0,0,-.003), (.025,.021,.008), (.63,.07,.08))
    box(stage, "LatchTongue", task.TONGUE_POS, task.TONGUE_SIZE, (.27,.29,.30))
    stage.GetRootLayer().Save()

    stage = asset(paths["field_button"], BUTTON_MASS)
    cylinder(stage, "BlueTestKey", (0,0,-.003), .009, .008, (.13,.38,.72))
    stage.GetRootLayer().Save()
    return paths


def connect_slide(stage, fixture, body, origin, stroke, stiffness, tilt=0.):
    """Anchor a native slider to the static world before detection."""
    from pxr import Gf, UsdPhysics

    joint = UsdPhysics.PrismaticJoint.Define(stage, f"{fixture}/{body.name}Slide")
    joint.CreateBody1Rel().SetTargets([body])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*origin))
    joint.CreateAxisAttr("Z")
    frame = Gf.Quatf(cos(tilt / 2), Gf.Vec3f(0., sin(tilt / 2), 0.))
    joint.CreateLocalRot0Attr(frame)
    joint.CreateLocalRot1Attr(frame)
    joint.CreateLowerLimitAttr(-stroke)
    joint.CreateUpperLimitAttr(0.)
    api = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
    api.CreateStiffnessAttr(stiffness)
    api.CreateDampingAttr(.9)
    api.CreateTargetPositionAttr(0.)
    return api
