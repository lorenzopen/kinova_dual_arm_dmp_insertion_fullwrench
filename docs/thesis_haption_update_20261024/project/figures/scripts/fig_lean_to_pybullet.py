"""From Yang Lin's lean model to PyBullet: the same route, frame by frame.

    python fig_lean_to_pybullet.py   -> ch10_tetris/lean_to_pybullet.pdf

World J s7161 move 5 (a 75 % narrowed slot, nominal grasp). Top row: the route
the BBO found in the lean quasi-static model (seated 0.5 mm). Bottom row: the
SAME route driven in PyBullet with the two tri-pads, the SOCP squeeze, the jam
sentry and the slow seat (seated 4.5 mm, 2 mm of slip in the pads). It is the
first rollout of the PyBullet refinement, which therefore stopped there
(dmp_realistic_6x6/tight/NARROW75_TRAINING.md, section 13).

Frames from the GIFs copied outside the repo (C:/Users/Lorenzo/Desktop/TESI_GIF).
"""
from pathlib import Path

import numpy as np
from PIL import Image

from style import plt, save

GIF = Path(r"C:\Users\Lorenzo\Desktop\TESI_GIF")
# (gif, row label, where along the gif each frame is taken, crop box): the PyBullet
# gif also films the carry to the mouth, so its frames are matched to the model's by eye
ROWS = (("J_cmpiidoff_s7161m5_LEAN_route_LEANMODEL.gif", "lean model\n(training)",
         (0.0, 0.35, 0.6, 0.8, 1.0), (20, 80, 556, 594)),
        ("J_cmpiidoff_s7161m5_LEAN_route_PB_mu1_js3_pybullet.gif", "PyBullet\n(tri-pads + SOCP)",
         (0.0, 0.6, 0.8, 0.9, 1.0), (150, 30, 570, 440)))
NAMES = ("start", "", "", "", "seated")


def frames(path, at, box):
    im = Image.open(path)
    out = []
    for u in at:
        im.seek(min(im.n_frames - 1, int(round(u * (im.n_frames - 1)))))
        out.append(np.asarray(im.convert("RGB").crop(box)))
    return out


fig, axes = plt.subplots(2, 5, figsize=(7.2, 3.2), layout="constrained")
for r, (name, label, at, box) in enumerate(ROWS):
    for c, img in enumerate(frames(GIF / name, at, box)):
        ax = axes[r, c]
        ax.imshow(img)
        ax.set_xticks([]); ax.set_yticks([])
        for s in ax.spines.values():
            s.set_visible(False)
        if c == 0:
            ax.set_ylabel(label)
        if r == 0:
            ax.set_title(NAMES[c], fontsize=8)
save(fig, "ch10_tetris/lean_to_pybullet")
