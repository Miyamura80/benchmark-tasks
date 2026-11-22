"""Author rigid board, flexible edge cells, ordinary creases and passive adhesive."""

from pxr import Usd, UsdGeom, UsdPhysics, Gf, PhysxSchema
from .layout import *
from pathlib import Path


def box(stage, path, xyz, size, color):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*xyz))
    c.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    phys = PhysxSchema.PhysxCollisionAPI.Apply(c.GetPrim())
    phys.CreateContactOffsetAttr(0.0002)
    phys.CreateRestOffsetAttr(0.0)


def export(root, name):
    path = root / (name + ".usda")
    s = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(s, 1.0)
    UsdGeom.SetStageUpAxis(s, "Z")
    obj = UsdGeom.Xform.Define(s, "/Object").GetPrim()
    s.SetDefaultPrim(obj)
    UsdPhysics.RigidBodyAPI.Apply(obj)
    UsdPhysics.MassAPI.Apply(obj).CreateMassAttr(parts()[name][2])
    brown = (0.66, 0.43, 0.24)
    dark = (0.55, 0.34, 0.18)
    if name == "carton":
        box(s, "/Object/floor", (0, 0, 0.002), (LENGTH, WIDTH, 0.004), dark)
        for z, sign in [("front", -1), ("back", 1)]:
            box(
                s,
                "/Object/" + z,
                (0, sign * (WIDTH / 2 - 0.0012), HEIGHT / 2),
                (LENGTH, 0.0024, HEIGHT),
                brown,
            )
        for z, sign in [("west", -1), ("east", 1)]:
            box(
                s,
                "/Object/" + z,
                (sign * (LENGTH / 2 - 0.0012), 0, HEIGHT / 2),
                (0.0024, WIDTH - 0.005, HEIGHT),
                brown,
            )
        for i, x in enumerate(ZONES):
            box(
                s,
                "/Object/rim_" + str(i),
                (x, RIM_Y, HEIGHT - 0.0011),
                (0.0437, 0.019, 0.0022),
                dark,
            )
    elif name == "lid":
        box(s, "/Object/panel", (0, 0, 0), (LENGTH, 0.122, BOARD), brown)
    else:
        box(s, "/Object/paper", (0, 0, 0), (0.0437, 0.0197, BOARD), brown)
        box(
            s,
            "/Object/adhesive",
            (0, RIM_Y - HINGE_Y + 0.132, -0.00115),
            (0.0433, 0.016, 0.00025),
            (0.80, 0.61, 0.30),
        )
    s.GetRootLayer().Save()
    return str(path)


def hinge(s, parent, name, a, b, a0, a1, limits, rest, k, c):
    j = UsdPhysics.RevoluteJoint.Define(s, parent + "/" + b + "/" + name)
    j.CreateBody0Rel().SetTargets([parent + "/" + a])
    j.CreateBody1Rel().SetTargets([parent + "/" + b])
    j.CreateLocalPos0Attr(Gf.Vec3f(*a0))
    j.CreateLocalPos1Attr(Gf.Vec3f(*a1))
    j.CreateAxisAttr("X")
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    j.CreateExcludeFromArticulationAttr(True)
    j.CreateCollisionEnabledAttr(False)
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(k * math.pi / 180)
    drive.CreateDampingAttr(c * math.pi / 180)
    drive.CreateTargetPositionAttr(rest)


def install(s, parent):
    hinge(
        s,
        parent,
        "crease",
        "carton",
        "lid",
        (0, HINGE_Y, LID_Z),
        (0, 0.061, 0),
        (-77.0, 2.0),
        -REST,
        0.53,
        0.031,
    )
    for i, x in enumerate(ZONES):
        hinge(
            s,
            parent,
            "flex",
            "lid",
            f"lip_{i}",
            (x, -0.061, 0.0),
            (0, 0.010, 0.0),
            (-8.0, 8.0),
            0.0,
            0.060,
            0.0025,
        )


def bond(s, parent, i, base_anchor, tab_anchor):
    j = UsdPhysics.SphericalJoint.Define(s, parent + f"/lip_{i}/activated_adhesive")
    j.CreateBody0Rel().SetTargets([parent + "/carton"])
    j.CreateBody1Rel().SetTargets([parent + f"/lip_{i}"])
    j.CreateLocalPos0Attr(Gf.Vec3f(*base_anchor))
    j.CreateLocalPos1Attr(Gf.Vec3f(*tab_anchor))
    j.CreateExcludeFromArticulationAttr(True)
    j.CreateBreakForceAttr(12.0)
    j.CreateCollisionEnabledAttr(True)
    return j
