"""Render-only surfaces and textures for the practice mannequins."""

from math import pi

import numpy as np

from station import element


def gaussian(value, center, width):
    return np.exp(-((value - center) / width) ** 2)


def mesh(assets, name, vertices, faces, uv):
    element(assets, "mesh", name=name, vertex=np.asarray(vertices).ravel(),
            texcoord=np.asarray(uv).ravel(),
            face=" ".join(str(i) for face in faces for i in face))


def texture(assets, name, size, color, specular, shininess):
    element(assets, "texture", name=name, type="2d", builtin="flat",
            width=size, height=size, rgb1=color, rgb2=color)
    element(assets, "material", name=name, texture=name,
            specular=specular, shininess=shininess)


def set_texture(model, texture_id, rgb):
    start = model.tex_adr[texture_id]
    pixels = np.rint(np.clip(rgb, 0., 1.) * 255).astype(np.uint8).ravel()
    model.tex_data[start:start + len(pixels)] = pixels


def complexion(x, y):
    """Paint lips, eyelids and fine brows in the model's face coordinates."""
    side = np.abs(y)
    rgb = np.empty((*np.broadcast_shapes(x.shape, y.shape), 3))
    rgb[:] = [0.81, 0.62, 0.52]

    def tint(color, opacity):
        alpha = np.clip(opacity, 0., 1.)[..., None]
        rgb[:] += alpha * (np.asarray(color) - rgb)

    tint([.83, .52, .46], .33 * gaussian(x, .004, .021) * gaussian(side, .034, .016))
    tint([.90, .68, .58], .25 * gaussian(x, -.055, .022) * gaussian(y, 0., .047))
    tint([.58, .38, .35], .25 * gaussian(x, -.031, .011) * gaussian(side, .030, .014))
    tint([.57, .37, .32], .23 * gaussian(x, .016, .005) * gaussian(side, .008, .004))
    tint([.34, .21, .19], .62 * gaussian(x, .019, .0018) * gaussian(side, .0068, .0026))

    eye_t = (side - .014) / .028
    eyelid = -.032 + .0032 * np.sin(pi * eye_t) + .0010 * eye_t
    eye_window = np.clip(5 * np.minimum(eye_t, 1 - eye_t), 0., 1.)
    tint([.49, .29, .26], .33 * eye_window * gaussian(x, eyelid - .003, .0015))
    tint([.22, .12, .10], .84 * eye_window * gaussian(x, eyelid, .00075))
    for lash in np.linspace(.11, .92, 13):
        lash_x = -.032 + .0032 * np.sin(pi * lash) + .001 * lash
        reach = .0012 + .0011 * lash
        tail = np.clip((x - lash_x) / reach, 0., 1.)
        lash_y = .014 + .028 * lash + .0014 * tail
        tint([.27, .14, .12], .62 * gaussian(x, lash_x + .0006, reach / 1.6)
             * gaussian(side, lash_y, .00028))

    brow_t = np.clip((side - .013) / .035, 0., 1.)
    brow_x = -.044 - .005 * np.sin(pi * brow_t) + .003 * brow_t**3
    brow_width = .0022 - .0015 * brow_t
    brow_window = np.clip(7 * np.minimum(brow_t, 1 - brow_t), 0., 1.)
    grain = .62 + .21 * np.sin(side * 4200 + x * 1500) + .09 * np.sin(side * 7100)
    tint([.22, .115, .087], .76 * brow_window * grain * gaussian(x, brow_x, brow_width))

    lip_t = np.clip(y / .022, -1., 1.)
    lip_round = np.maximum(0., 1. - lip_t**2)
    seam = .0515 - .0028 * lip_t**2 + .0007 * gaussian(y, 0., .003)
    upper = seam - lip_round * (.0026 + .0015 * gaussian(np.abs(y), .005, .003))
    lower = seam + .0056 * lip_round
    lip_mask = np.clip(np.minimum(x - upper, lower - x) / .0011, 0., 1.)
    tint([.57, .30, .29], .72 * lip_mask)
    tint([.43, .23, .23], .77 * lip_round * gaussian(x, seam, .00055))
    tint([.79, .52, .48], .40 * lip_mask * gaussian(x, seam + .0022, .0014))

    rng = np.random.default_rng(14)
    pores = rng.normal(0., .004, rgb.shape[:2])
    return rgb + pores[..., None]
