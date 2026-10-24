"""Forces and moment during one insertion: the straight line against the learnt DMP.

    python fig_insertion_forces.py [--move K] [--on DIR] [--off DIR]
        -> ch10_tetris/insertion_forces.pdf

Same game, same seed, same move (seed 7025 move 1 by default: the board is
still identical in the two arms, so the comparison is clean). From the trace of
trace_game.py, from 1.5 s before the route until the release:

  1. squeeze: the normal force each hand measures, and the target the SOCP asks
  2. the push the slot puts on the piece (horizontal), measured from the pads in
     quasi-statics, beside what Yang Lin's model predicted along the route
  3. the moment the pads carry about the pad axis - the weak channel (CLAUDE.md §8)
"""
import argparse

import numpy as np
from matplotlib.patches import Patch

from style import FIG, LEFT, RIGHT, GREY, plt, save

ap = argparse.ArgumentParser()
ap.add_argument("--move", type=int, default=1)
ap.add_argument("--on", default=str(FIG / "data" / "s7025_on"))
ap.add_argument("--off", default=str(FIG / "data" / "s7025_off"))
ap.add_argument("--out", default="ch10_tetris/insertion_forces")
ap.add_argument("--labels", nargs=2, default=("straight line", "learnt DMP"),
                help="column titles for --off and --on")
a = ap.parse_args()

STEPS = ["start", "transit", "approach", "squeeze", "move", "roll", "hang", "route", "seat",
         "release", "descent"]
SHADE = {"descent": "0.94", "route": "#e3eef8", "seat": "#f3e9dc"}


def load(folder):
    z = np.load(f"{folder}/trace.npz")
    D, cols = z["data"], list(z["columns"])
    return {k: D[:, i] for i, k in enumerate(cols)}


def window(T, move):
    """Grasp #2 of that move, from the first descent sample to the release."""
    m = (T["move"] == move) & (T["stage"] == 1)
    names = np.array([STEPS[int(s)] for s in T["step"]])
    keep = m & np.isin(names, ["descent", "route", "seat", "release"])
    i = np.nonzero(keep)[0]
    route = i[names[i] == "route"]
    if len(route):                       # 1.5 s of the free-air descent is enough context
        i = i[i >= route[0] - 150]
    return i, names


arms = [(a.labels[0], a.off), (a.labels[1], a.on)]
fig, axes = plt.subplots(3, 2, figsize=(7.2, 5.2), sharex="col", sharey="row", layout="constrained")
for c, (label, folder) in enumerate(arms):
    T = load(folder)
    i, names = window(T, a.move)
    t = T["t"][i] - T["t"][i[0]]
    # step bands
    for ax in axes[:, c]:
        start = 0
        for k in range(1, len(i) + 1):
            if k == len(i) or names[i[k]] != names[i[start]]:
                col = SHADE.get(names[i[start]])
                if col:
                    ax.axvspan(t[start], t[k - 1], color=col, lw=0)
                start = k
    ax = axes[0, c]
    ax.plot(t, T["fL"][i], color=LEFT, lw=1.0, label="left hand")
    ax.plot(t, T["fR"][i], color=RIGHT, lw=1.0, label="right hand")
    ax.plot(t, T["target"][i], color="k", lw=0.9, ls="--", label="SOCP target")
    ax.set_title(label)
    axes[0, 0].set_ylabel("squeeze [N]")

    ax = axes[1, c]
    ext = np.hypot(T["Ex"][i], T["Ey"][i])
    pred = np.hypot(T["Wx"][i], T["Wy"][i])
    ax.plot(t, ext, color="k", lw=1.0, label="measured")
    ax.plot(t, pred, color=GREY, lw=1.0, ls=":", label="model, along the route")
    axes[1, 0].set_ylabel("slot push [N]")

    ax = axes[2, c]
    ax_ = np.stack([T["ax"][i], T["ay"][i], T["az"][i]], 1)
    tau = np.einsum("ij,ij->i", np.stack([T["Tx"][i], T["Ty"][i], T["Tz"][i]], 1), ax_)
    ax.plot(t, tau, color="k", lw=1.0)
    ax.axhline(0, color=GREY, lw=0.5)
    axes[2, 0].set_ylabel("moment about\npad axis [Nm]")
    ax.set_xlabel("time [s]")
axes[0, 0].legend(loc="lower left")
axes[1, 0].legend(loc="upper left")
fig.legend(handles=[Patch(color=col, label=name) for name, col in SHADE.items()],
           loc="lower center", ncol=3, bbox_to_anchor=(0.5, -0.05))
save(fig, a.out)
