"""Build the procedural silicone hand asset with Blender 4.3 in background mode."""
from pathlib import Path

import bpy
import numpy as np

# Cross-sections in millimetres: x, y, z, horizontal radius, vertical radius.
# The middle nail's centre is the origin, with the tips pointing towards -x.
PROFILES = [
    [(-13, 0, -8.5, .7, .7), (-12, 0, -8.1, 4.8, 3.9), (-9, 0, -7.8, 8.7, 6.6),
     (-4, 0, -7.8, 10.3, 7.2), (9, 0, -7.8, 10.6, 7.2), (20, .5, -7.6, 9.7, 7.4),
     (33, .7, -7.2, 9.4, 8.2), (48, 0, -7, 10.8, 9), (59, -1, -7, 9, 8)],
    [(-4, 22.5, -8, .7, .7), (-2, 22.5, -8, 6.5, 4.8), (4, 22.3, -7.8, 9, 6.8),
     (16, 21.6, -7.7, 9.1, 7.0), (29, 20.7, -7.35, 8.8, 7.7), (44, 18.5, -7, 10.8, 9),
     (56, 17, -7.2, 10, 8)],
    [(-7, -19, -8.6, .7, .7), (-5, -19.4, -8.5, 6.3, 4.8), (1, -19.7, -8.2, 8.7, 6.6),
     (13, -19.3, -8.1, 9, 6.8), (28, -18.3, -7.5, 8.7, 7.7), (44, -17.2, -7.4, 9.7, 8.5),
     (58, -16.8, -7.5, 10, 8)],
    [(8, -38, -9.6, .7, .7), (10, -38, -9.5, 4.7, 3.7), (16, -37.5, -9, 7.0, 5.9),
     (25, -36.2, -8.9, 7.2, 6.2), (40, -34.4, -8.8, 8.6, 6.5), (55, -29.8, -8.2, 9.4, 7)],
    [(25, 48.5, -10, .7, .7), (28, 48, -9.9, 7.1, 5.4), (34, 47.3, -9.3, 9.7, 7.1),
     (43, 45, -8.9, 11, 8.0), (56, 39, -8.2, 11, 8.4), (68, 31, -8.1, 13, 8.7),
     (83, 24, -8.2, 13, 8.1)],
    [(34, -4, -8, 7, 4), (42, -5.2, -7.7, 23, 7.5), (50, -6.4, -7.0, 36.5, 9),
     (62, -6, -6.7, 39.5, 9.4), (76, -2.9, -7.1, 38, 9.6), (88, .8, -7.9, 32.5, 9),
     (99, 3.2, -8.7, 27.0, 8.2), (110, 4.5, -9.4, 24.5, 7.6),
     (124, 5.2, -9.5, 23, 7.1), (129, 5.3, -9.6, 19, 5.6), (131, 5.4, -9.6, 12, 3)],
]


def loft(name, profile):
    stations = np.linspace(0, len(profile) - 1, (len(profile) - 1) * 4 + 1)
    sections = np.column_stack([
        np.interp(stations, np.arange(len(profile)), values)
        for values in np.asarray(profile).T
    ])
    sides = 40
    angle = np.arange(sides) * 2 * np.pi / sides
    vertices = [
        (x, y + dy * a, z + dz * b)
        for x, y, z, dy, dz in sections for a, b in zip(np.cos(angle), np.sin(angle))
    ]
    faces = [tuple(reversed(range(sides)))]
    for start in range(0, len(vertices) - sides, sides):
        for i in range(sides):
            a, b = start + i, start + (i + 1) % sides
            faces.append((a, b, b + sides, a + sides))
    faces.append(tuple(range(len(vertices) - sides, len(vertices))))
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    obj.select_set(True)
    return obj


def main():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    parts = [loft(f"hand_{i}", profile) for i, profile in enumerate(PROFILES)]
    bpy.context.view_layer.objects.active = parts[-1]
    bpy.ops.object.join()
    hand = bpy.context.object
    hand.data.remesh_voxel_size = 1.15
    bpy.ops.object.voxel_remesh()
    smooth = hand.modifiers.new("blend finger roots", "SMOOTH")
    smooth.factor = 0.75
    smooth.iterations = 5
    bpy.ops.object.modifier_apply(modifier=smooth.name)
    reduce = hand.modifiers.new("reduce planar triangles", "DECIMATE")
    reduce.ratio = 0.38
    bpy.ops.object.modifier_apply(modifier=reduce.name)
    hand.data.calc_loop_triangles()
    vertices = np.asarray([vertex.co[:] for vertex in hand.data.vertices])
    triangles = np.asarray([tri.vertices[:] for tri in hand.data.loop_triangles])
    assert len(vertices) < 65536
    path = Path(__file__).with_name("silicone_hand_mesh.npz")
    np.savez_compressed(path, vertices=np.round(vertices * 100).astype(np.int16),
                        triangles=triangles.astype(np.uint16))
    print(f"Wrote {len(vertices)} vertices and {len(triangles)} triangles to {path}")


if __name__ == "__main__":
    main()
