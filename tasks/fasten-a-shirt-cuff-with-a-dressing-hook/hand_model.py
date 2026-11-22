"""Continuous synthetic forearm and hand for the cuff fixture."""

import numpy as np
from skimage.measure import marching_cubes

from station import element


SKIN = (0.72, 0.50, 0.40, 1)
NAIL = (0.78, 0.60, 0.53, 1)


def add_hand(tree, parent):
    spacing = 0.0018
    lower = np.array((-0.161, -0.073, 0.192))
    upper = np.array((0.248, 0.065, 0.290))
    x, y, z = np.ogrid[tuple(slice(lo, hi, spacing) for lo, hi in zip(lower, upper))]
    field = np.full(np.broadcast_shapes(x.shape, y.shape, z.shape), 0.12)

    def blend(shape, smooth):
        nonlocal field
        mix = np.maximum(smooth-np.abs(field-shape), 0.0)/smooth
        field = np.minimum(field, shape)-0.25*smooth*mix**2

    def ellipsoid(center, axes, smooth=0.004):
        cx, cy, cz = center
        rx, ry, rz = axes
        metric = np.sqrt(((x-cx)/rx)**2 + ((y-cy)/ry)**2 + ((z-cz)/rz)**2)
        blend((metric-1)*min(axes), smooth)

    def segment(start, end, first, last, smooth=0.0029):
        ax, ay, az = start
        dx, dy, dz = np.subtract(end, start)
        t = np.clip(((x-ax)*dx+(y-ay)*dy+(z-az)*dz)/(dx*dx+dy*dy+dz*dz), 0, 1)
        radial = np.sqrt((x-ax-t*dx)**2 + (y-ay-t*dy)**2 + (z-az-t*dz)**2)
        blend(radial-(first+(last-first)*t), smooth)

    # The two tapered volumes blend at the wrist, then each segmented digit
    # joins the palm. Every finger, including the opposable thumb, is part
    # of the same closed surface.
    ellipsoid((-0.064, -0.005, 0.243), (0.092, 0.029, 0.023))
    ellipsoid((0.038, -0.005, 0.244), (0.094, 0.023, 0.018))
    ellipsoid((0.090, -0.008, 0.244), (0.040, 0.020, 0.013))
    ellipsoid((0.127, -0.009, 0.243), (0.044, 0.033, 0.015))
    ellipsoid((0.149, -0.007, 0.244), (0.020, 0.030, 0.012))
    fingers = (
        ((0.152, -0.032, 0.245), (0.181, -0.033, 0.244), (0.209, -0.034, 0.237), 0.0073, 0.0058),
        ((0.158, -0.012, 0.246), (0.192, -0.013, 0.246), (0.223, -0.014, 0.238), 0.0085, 0.0061),
        ((0.156, 0.008, 0.246), (0.187, 0.010, 0.244), (0.215, 0.012, 0.237), 0.0079, 0.0058),
        ((0.148, 0.026, 0.245), (0.173, 0.031, 0.241), (0.198, 0.033, 0.234), 0.0066, 0.0047),
        ((0.111, -0.033, 0.239), (0.124, -0.047, 0.235), (0.144, -0.056, 0.230), 0.0099, 0.0066),
    )
    for base, middle, tip, radius, finish in fingers:
        segment(base, middle, radius, radius*0.88)
        segment(middle, tip, radius*0.88, finish)
    vertices, faces, _, _ = marching_cubes(field, 0.0, spacing=(spacing,)*3,
                                            gradient_direction="ascent", allow_degenerate=False)
    vertices += lower
    asset = tree.find("asset")
    if asset is None:
        asset = element(tree, "asset")
    element(asset, "mesh", name="silicone_hand_mesh", vertex=vertices.ravel(),
            face=" ".join(str(i) for i in faces.ravel()), smoothnormal="true")
    element(parent, "geom", name="synthetic_hand_forearm", type="mesh", mesh="silicone_hand_mesh",
            rgba=SKIN, contype=0, conaffinity=0, mass=0)
    for i, (_, middle, tip, _, finish) in enumerate(fingers):
        cx, cy, cz = 0.14*np.asarray(middle)+0.86*np.asarray(tip)
        angle = float(np.arctan2(tip[1]-middle[1], tip[0]-middle[0]))
        element(parent, "geom", name=f"finger_nail_{i}", type="ellipsoid",
                pos=(cx, cy, cz+0.0048 if i == 0 else cz+finish*0.78),
                euler=(0, 0.12, angle), size=(0.0068, finish*0.60, 0.0013),
                rgba=NAIL, contype=0, conaffinity=0, mass=0)


def add_sleeve(tree, parent):
    """Cover the full forearm with a tapered shirt sleeve under the cuff."""
    angles = np.linspace(-np.pi+0.015, np.pi-0.015, 45)
    positions = np.linspace(-0.162, 0.094, 22)

    def profile(x, theta):
        y_radius = float(np.interp(x, [-0.162, -0.086, -0.022, 0.042, 0.094],
                                   [0.030, 0.0345, 0.032, 0.0285, 0.024]))
        z_radius = float(np.interp(x, [-0.162, -0.086, -0.022, 0.042, 0.094],
                                   [0.024, 0.0285, 0.026, 0.023, 0.018]))
        wrinkle = 0.0008*np.sin(10*theta+53*x)+0.0005*np.cos(167*x+2*theta)
        return y_radius+wrinkle, z_radius+wrinkle

    add_fabric_shell(tree, parent, "shirt_sleeve", positions, angles, profile,
                     (0.36, 0.49, 0.67, 1))
    # Supported end of the dressing form. The sleeve continues to the end of
    # the visible arm, with only the hand projecting beyond the wrist cuff.
    element(parent, "geom", name="sleeve_supported_end", type="ellipsoid",
            pos=(-0.161, -0.005, 0.244), size=(0.0047, 0.0295, 0.0237),
            rgba=(0.34, 0.46, 0.63, 1), contype=0, conaffinity=0, mass=0)


def add_cuff(tree, parent):
    """A sewn wrist band with a continuous outer and inner surface."""
    angles = np.linspace(-2.47, 2.52, 32)
    positions = np.linspace(0.029, 0.098, 11)

    def profile(x, theta):
        radius = 0.0342+0.00065*np.cos(5*theta+2*np.pi*(x-positions[0])/(positions[-1]-positions[0]))
        return radius, radius

    add_fabric_shell(tree, parent, "woven_cuff", positions, angles, profile,
                     (0.38, 0.51, 0.69, 1))


def add_fabric_shell(tree, parent, name, positions, angles, profile, color):
    vertex = []
    for layer in (0, 1):
        thickness = 0.0015*layer
        for x in positions:
            for theta in angles:
                ry, rz = profile(x, theta)
                vertex.append((x, -0.005+(ry-thickness)*np.sin(theta),
                               0.244+(rz-thickness)*np.cos(theta)))
    width = len(angles)
    length = len(positions)
    layer_size = width*length
    faces = []

    def quad(a, b, c, d):
        faces.extend(((a, c, b), (a, d, c)))

    for i in range(length-1):
        for j in range(width-1):
            a = i*width+j
            b, c, d = a+1, a+width+1, a+width
            quad(a, b, c, d)
            quad(d+layer_size, c+layer_size, b+layer_size, a+layer_size)
    for j in range(width-1):
        quad(j, j+layer_size, j+1+layer_size, j+1)
        k = (length-1)*width+j
        quad(k, k+1, k+1+layer_size, k+layer_size)
    for i in range(length-1):
        a = i*width
        b = a+width-1
        quad(a, a+width, a+width+layer_size, a+layer_size)
        quad(b, b+layer_size, b+width+layer_size, b+width)

    asset = tree.find("asset")
    mesh_name = f"{name}_mesh"
    element(asset, "mesh", name=mesh_name, vertex=np.asarray(vertex).ravel(),
            face=" ".join(str(i) for row in faces for i in row), smoothnormal="true")
    element(parent, "geom", name=name, type="mesh", mesh=mesh_name,
            rgba=color, contype=0, conaffinity=0, mass=0)
