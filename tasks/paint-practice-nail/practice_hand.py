"""USD render geometry for the silicone training hand and its polish."""
from pathlib import Path

import numpy as np

ROOT = "/World/PracticeHand"
MESH_PATH = Path(__file__).with_name("silicone_hand_mesh.npz")
NAIL_HALF_LENGTH = 0.0085
NAIL_HALF_WIDTH = 0.006
SKIN = (0.49, 0.258, 0.182)


def nail_relief(x, y):
    """Shallow transverse arch, in metres, over the calibrated contact plane."""
    return 0.0008 - 0.00165 * (y / NAIL_HALF_WIDTH) ** 2 - 0.00035 * (x / NAIL_HALF_LENGTH) ** 2


def create_mesh(stage, path, vertices, faces, smooth=True):
    from pxr import UsdGeom, Vt

    mesh = UsdGeom.Mesh.Define(stage, path)
    vertices = np.asarray(vertices, dtype=np.float32)
    faces = np.asarray(faces, dtype=np.int32)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(vertices))
    mesh.CreateFaceVertexCountsAttr([faces.shape[1]] * len(faces))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.reshape(-1)))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    if smooth:
        normals = np.zeros_like(vertices)
        edges = vertices[faces[:, 1:3]] - vertices[faces[:, :1]]
        face_normals = np.cross(edges[:, 0], edges[:, 1])
        for i in range(faces.shape[1]):
            np.add.at(normals, faces[:, i], face_normals)
        length = np.linalg.norm(normals, axis=1)
        normals /= np.maximum(length[:, None], 1e-14)
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(normals))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    return mesh


def material(stage, name, color, roughness, clearcoat=0.0, vertex_color=False):
    from pxr import Sdf, UsdShade

    path = f"{ROOT}/Looks/{name}"
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    diffuse = shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
    diffuse.Set(color)
    if vertex_color:
        reader = UsdShade.Shader.Define(stage, f"{path}/Color")
        reader.CreateIdAttr("UsdPrimvarReader_float3")
        reader.CreateInput("varname", Sdf.ValueTypeNames.String).Set("displayColor")
        reader.CreateOutput("result", Sdf.ValueTypeNames.Float3)
        diffuse.ConnectToSource(reader.ConnectableAPI(), "result")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.46)
    shader.CreateInput("clearcoat", Sdf.ValueTypeNames.Float).Set(clearcoat)
    shader.CreateInput("clearcoatRoughness", Sdf.ValueTypeNames.Float).Set(0.24)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def bind(mesh, finish):
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(finish)


def build_nail(stage, name, xy, z, scale, yaw, finish):
    from pxr import UsdGeom, UsdPhysics, Vt

    # Radius at the cuticle and at the free edge is sampled independently.
    angles = np.radians(np.arange(0, 360, 5))
    direction = np.column_stack([np.cos(angles), np.sin(angles)])
    corner = np.sign(direction) * np.array([0.0055, 0.003])
    perimeter = corner + 0.003 * direction
    # At the cardinal axes include both ends of the straight edge through the nearby
    # quadrant corners. All intermediate rings share the same winding.
    local = np.vstack([np.zeros((1, 2)), *(perimeter * radius for radius in (0.25, 0.50, 0.75, 0.92, 1.0))])
    sides = len(perimeter)
    faces = [(0, 1 + i, 1 + (i + 1) % sides) for i in range(sides)]
    for ring in range(4):
        first = 1 + sides * ring
        for i in range(sides):
            a, b = first + i, first + (i + 1) % sides
            faces += [(a, a + sides, b + sides), (a, b + sides, b)]
    rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    vertices = np.column_stack([local * scale @ rot.T + xy, z + scale * nail_relief(*local.T)])
    mesh = create_mesh(stage, f"{ROOT}/{name}", vertices, faces)
    # Cream free edge at the fingertip and a subtle pale crescent at the cuticle.
    x, y = local.T
    free_edge = np.clip((-x - 0.0061) / 0.002, 0, 1) * 0.82
    crescent = np.exp(-((x - 0.0058) / 0.0015) ** 2 - (y / 0.0038) ** 2) * 0.42
    fade = np.maximum(free_edge, crescent)[:, None]
    base_color = np.array([0.76, 0.47, 0.40])
    light_color = np.array([0.92, 0.77, 0.66])
    colors = np.asarray(base_color * (1 - fade) + light_color * fade, dtype=np.float32)
    mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex).Set(Vt.Vec3fArray.FromNumpy(colors))
    bind(mesh, finish)
    if name == "MiddleNail":
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")


def install(stage, task):
    from pxr import UsdGeom

    stage.RemovePrim(ROOT)
    proxy_names = {obj.name for obj in task.HAND_COLLIDERS}
    for prim in stage.Traverse():
        if prim.GetName() in proxy_names and prim.IsA(UsdGeom.Imageable):
            UsdGeom.Imageable(prim).MakeInvisible()
    with np.load(MESH_PATH, allow_pickle=False) as asset:
        vertices = asset["vertices"].astype(float) / 100_000
        vertices += np.array([*task.XY, task.SURFACE])
        hand = create_mesh(stage, f"{ROOT}/Silicone", vertices, asset["triangles"])
    bind(hand, material(stage, "Silicone", SKIN, 0.54))
    plain = material(stage, "NaturalNail", (0.76, 0.47, 0.40), 0.28, clearcoat=0.22, vertex_color=True)
    material(stage, "RedPolish", (0.57, 0.010, 0.031), 0.21, clearcoat=0.5)
    for name, x, y, z, scale, yaw in [
        ("MiddleNail", 0, 0, 0, 1.0, 0),
        ("IndexNail", 5.8, 22.3, -0.5, 0.88, -0.07),
        ("RingNail", 2.6, -19.6, -1.1, 0.90, 0.035),
        ("LittleNail", 17, -37.3, -2.5, 0.73, 0.12),
        ("ThumbNail", 36.2, 46.6, -0.9, 1.02, -0.34),
    ]:
        build_nail(stage, name, task.XY + np.array([x, y]) / 1000, task.SURFACE + z / 1000, scale, yaw, plain)


def update_polish(stage, task):
    from pxr import UsdShade

    path = f"{ROOT}/Coverage"
    cells = task.OFFSETS[task.paint.painted]
    if not len(cells):
        return
    corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * task.CELL / 2
    xy = cells[:, None, :] + corners
    x, y = xy[..., 0], xy[..., 1]
    # The edge band uses the finger's approximate elliptical surface.
    curved_skin = -0.0006 - 0.0072 * (1 - np.sqrt(np.clip(1 - (y / 0.0107) ** 2, 0, 1)))
    z = np.where(task.NAIL[task.paint.painted, None], nail_relief(x, y), curved_skin)
    vertices = np.dstack([xy + task.XY, z + task.SURFACE + 0.00028]).reshape(-1, 3)
    faces = np.arange(len(vertices), dtype=np.int32).reshape(-1, 4)
    mesh = create_mesh(stage, path, vertices, faces, smooth=False)
    bind(mesh, UsdShade.Material.Get(stage, f"{ROOT}/Looks/RedPolish"))
