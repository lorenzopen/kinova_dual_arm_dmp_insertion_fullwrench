"""The Tetris chain in pictures, for a reader with no simulation in front of them.

    python fig_snapshots.py [--on DIR] [--move K]
        -> ch10_tetris/pipeline_snapshots.pdf   one move, start to end
        -> ch10_tetris/nest_poses.pdf           every piece standing in the nest

Pictures taken by trace_game.py (TinyRenderer, fixed cameras) during the game.
"""
import argparse
from pathlib import Path

import numpy as np
from PIL import Image

from style import FIG, plt, save

ap = argparse.ArgumentParser()
ap.add_argument("--on", default=str(FIG / "data" / "s7025_on"))
ap.add_argument("--move", type=int, default=3)
a = ap.parse_args()
snaps = Path(a.on) / "snaps"

STEPS = (("feeder", "1  arrives flat"), ("in_nest", "2  rolled up, put in the nest"),
         ("nest", "3  grasp on the thickness"), ("hang", "4  still hang: wrench"),
         ("mouth", "5  at the mouth of the slot"), ("released", "6  seated and released"))


def crop(img, keep=0.8):
    """The central part of a picture: the cameras are set wide on purpose."""
    h, w = img.shape[:2]
    dh, dw = int(h * (1 - keep) / 2), int(w * (1 - keep) / 2)
    return img[dh:h - dh, dw:w - dw]


def show(ax, path, title, keep=0.8):
    ax.imshow(crop(np.asarray(Image.open(path).convert("RGB")), keep))
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_title(title, fontsize=8, loc="left")


have = [(tag, t) for tag, t in STEPS if (snaps / f"m{a.move}_{tag}.png").exists()]
fig, axes = plt.subplots(2, 3, figsize=(7.2, 4.4), layout="constrained")
for ax, (tag, title) in zip(axes.flat, have):
    show(ax, snaps / f"m{a.move}_{tag}.png", title, 0.95 if tag in ("mouth", "released") else 0.8)
for ax in list(axes.flat)[len(have):]:
    ax.axis("off")
save(fig, "ch10_tetris/pipeline_snapshots")

nest = sorted((q for q in snaps.glob("m*_nest.png") if not q.name.endswith("in_nest.png")),
              key=lambda q: int(q.name[1:].split("_")[0]))
fig, axes = plt.subplots(1, len(nest), figsize=(7.2, 1.6), layout="constrained")
for ax, path in zip(np.atleast_1d(axes), nest):
    show(ax, path, f"move {path.name[1:].split('_')[0]}", 0.7)
save(fig, "ch10_tetris/nest_poses")
