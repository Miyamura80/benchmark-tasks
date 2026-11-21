"""Prepare an evaluator-only leaf mask from the registered and cropped daylight photo."""

import argparse
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw


def main():
    import matplotlib.pyplot as plt

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("daylight", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    im = Image.open(args.daylight).convert("RGB")
    m = Image.new("L", im.size)
    fig, ax = plt.subplots()
    for label in (1, 2, 3):
        while True:
            ax.clear()
            ax.imshow(im)
            ax.imshow(
                np.ma.masked_equal(np.asarray(m), 0),
                vmin=0,
                vmax=3,
                cmap="autumn",
                alpha=0.55,
            )
            ax.set_title(
                f"Weed {label}: outline one leaf or leaf group. Enter to close; empty Enter finishes weed."
            )
            fig.canvas.draw()
            pts = plt.ginput(n=-1, timeout=0)
            if not pts:
                if label not in np.asarray(m):
                    raise ValueError("Every weed needs an outline")
                break
            if len(pts) < 3:
                raise ValueError("A polygon needs at least three points.")
            layer = Image.new("1", im.size)
            ImageDraw.Draw(layer).polygon(pts, fill=1)
            overlaps = (
                (np.asarray(m) > 0) & (np.asarray(m) != label) & np.asarray(layer)
            )
            if overlaps.any():
                raise ValueError("Two different weeds overlap in the annotation")
            m.paste(label, mask=layer)
    plt.close(fig)
    m.save(args.output)


if __name__ == "__main__":
    main()
