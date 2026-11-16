"""Author bowl, bent brush and dock contacts before YAM initializes physics."""

from native import asset, collider
from task import BRISTLE_RADIUS, BRUSH_LENGTH, BRUSH_OFFSET, HOLDER, INNER_PROFILE, OUTER_PROFILE, PORCELAIN, TOP, rings


def author(directory):
    from pxr import Gf, UsdGeom, UsdPhysics

    ceramic, brush = (directory / f"{name}.usda" for name in ("bowl_fixture", "angled_brush"))
    ceramic.unlink(missing_ok=True)
    brush.unlink(missing_ok=True)

    model = asset(ceramic)
    bowl = UsdGeom.Mesh.Define(model, "/Object/CeramicSurface")
    points, faces = rings(INNER_PROFILE + OUTER_PROFILE)
    # The traversal climbs the inner wall. Reverse winding so the
    # contact-side normal of the inner wall points into the empty bowl.
    faces = [tuple(reversed(face)) for face in faces]
    bowl.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    bowl.CreateFaceVertexCountsAttr([len(face) for face in faces])
    bowl.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
    bowl.CreateSubdivisionSchemeAttr("none")
    bowl.CreateDoubleSidedAttr(True)
    bowl.CreateDisplayColorAttr([Gf.Vec3f(*PORCELAIN)])
    collider(bowl)
    UsdPhysics.MeshCollisionAPI.Apply(bowl.GetPrim()).CreateApproximationAttr("none")

    def box(stage, name, center, size, color, angle=0., friction=None):
        shape = UsdGeom.Cube.Define(stage, f"/Object/{name}")
        shape.CreateSizeAttr(1.)
        shape.AddTranslateOp().Set(Gf.Vec3d(*center))
        shape.AddRotateZOp().Set(angle)
        shape.AddScaleOp().Set(Gf.Vec3f(*size))
        shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        collider(shape, friction)

    # The holder has an open side at the offset head, and a collar saddle.
    box(model, "HolderChassis", (HOLDER[0]-.034, HOLDER[1], TOP+.020), (.047, .066, .016), (.13,.28,.31))
    for index, side in enumerate((-1,1)):
        box(model, f"HolderUpright{index}", (HOLDER[0]-.019, HOLDER[1]+side*.020, HOLDER[2]-.061),
            (.030,.010,.082), (.13,.28,.31))
        box(model, f"HolderLip{index}", (HOLDER[0]-.004, HOLDER[1]+side*.012, HOLDER[2]-.024),
            (.025,.010,.014), (.15,.30,.31))
    head_x = HOLDER[0] + BRUSH_OFFSET
    head_z = HOLDER[2] - BRUSH_LENGTH
    foot = head_z - BRISTLE_RADIUS
    box(model, "BristleCupBase", (head_x, HOLDER[1], (TOP + foot)/2),
        (.043,.044,foot-TOP), (.15,.30,.31))
    for i, (dx, dy) in enumerate(((.019,0.),(-.019,0.),(0.,.019),(0.,-.019))):
        size = (.005,.025,.011) if dx else (.025,.005,.011)
        box(model, f"BristleCupRim{i}", (head_x+dx,HOLDER[1]+dy,foot+.005), size, (.15,.30,.31))
    model.GetRootLayer().Save()

    model = asset(brush, .042)
    box(model, "BrushHandle", (0.,0.,0.), (.018,.022,.034), (.12,.53,.54), friction=1.2)
    # Brushes stay grippable from above; side collar rests on the forked stand.
    box(model, "BrushCollar", (0.,0.,-.014), (.023,.026,.006), (.12,.53,.54))
    box(model, "BrushStem", (0.,0.,-.035), (.008,.008,.041), (.57,.63,.62))
    box(model, "BentNeck", (.015, 0., -BRUSH_LENGTH+.005), (.036,.009,.009), (.57,.63,.62))
    head = UsdGeom.Sphere.Define(model, "/Object/BrushBristles")
    head.CreateRadiusAttr(BRISTLE_RADIUS)
    head.AddTranslateOp().Set(Gf.Vec3d(BRUSH_OFFSET,0.,-BRUSH_LENGTH))
    head.CreateDisplayColorAttr([Gf.Vec3f(.18,.41,.38)])
    collider(head)
    model.GetRootLayer().Save()
    return {"bowl_fixture": ceramic, "angled_brush": brush}
