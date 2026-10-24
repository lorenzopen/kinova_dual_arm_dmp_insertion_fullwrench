"""Dellacherie's planned board against the pile PyBullet actually built.

    python fig_boards.py [BOARDS.json GAME.csv]   -> ch10_tetris/planned_vs_pybullet.pdf

Top row: the cells Dellacherie chose, move after move (grid units, the plan).
Bottom row: the same pieces where they really ended in PyBullet after that move
(`shelf_after` of game.py --dump-boards, truth, report only), drawn from the
catalogue columns at their recorded pose. Default game: seed 7025, 7-bag,
today's defaults (figures/data/s7025_on, or runs/cmp_bag/on/s7025 if absent).
"""
import csv
import json
import sys

import numpy as np
from matplotlib.patches import Polygon, Rectangle
from scipy.spatial import ConvexHull

from style import FIG, PIECE, REPO, plt, save

TIGHT = REPO / "lorenzo_data_analysis" / "dmp_realistic_6x6"
sys.path[:0] = [str(TIGHT), str(TIGHT / "tight")]
import geometry as G             # noqa: E402
from prepare import catalogue    # noqa: E402

CELL, SEAM, ROWS, COLS = 0.12, 0.003, 6, 6
PITCH = CELL + SEAM
SHOW = (2, 4, 7)                 # after which moves

data = FIG / "data" / "s7025_on"
old = REPO / "lorenzo_data_analysis" / "tetris_dual_arm" / "tetris" / "runs" / "cmp_bag" / "on"
if len(sys.argv) > 2:
    bpath, cpath = sys.argv[1], sys.argv[2]
elif (data / "boards.json").exists() and (data / "game.csv").exists():
    bpath, cpath = data / "boards.json", data / "game.csv"
else:
    bpath, cpath = old / "s7025.json", old / "s7025.csv"
boards = json.loads(open(bpath, encoding="utf-8").read())
place = {int(r["k"]): r for r in csv.DictReader(open(cpath, encoding="utf-8"))}
objs = catalogue()


def frame(ax):
    w = COLS * PITCH * 1000
    ax.add_patch(Rectangle((-20, -20), 20, ROWS * CELL * 1000 + 20, fc="0.35", lw=0))
    ax.add_patch(Rectangle((w, -20), 20, ROWS * CELL * 1000 + 20, fc="0.35", lw=0))
    ax.add_patch(Rectangle((-20, -20), w + 40, 20, fc="0.35", lw=0))
    ax.set_xlim(-30, w + 30); ax.set_ylim(-30, ROWS * CELL * 1000 + 10)
    ax.set_aspect("equal"); ax.axis("off")


fig, axes = plt.subplots(2, len(SHOW), figsize=(7.2, 4.7), layout="constrained")
for c, k in enumerate(SHOW):
    top, bot = axes[0, c], axes[1, c]
    frame(top); frame(bot)
    for b in boards[:k]:                                   # the plan
        for i, j in b["cells"]:
            top.add_patch(Rectangle((1000 * (b["col"] + i) * PITCH, 1000 * (b["row"] + j) * CELL),
                                    1000 * CELL, 1000 * CELL, fc=PIECE[b["piece"]], ec="k",
                                    lw=0.4, alpha=0.85))
        x, z = np.mean([[(b["col"] + i + 0.5) * PITCH, (b["row"] + j + 0.5) * CELL]
                        for i, j in b["cells"]], axis=0)
        top.text(1000 * x, 1000 * z, str(b["move"]), ha="center", va="center", fontsize=7)
    after = boards[k - 1].get("shelf_after", boards[k - 1]["shelf"])
    for s in after:                                         # what PyBullet built
        for v in G.body_columns(objs[s["piece"]], s["pos"], s["quat"]):
            xz = 1000 * v[:, [0, 2]]
            hull = xz[ConvexHull(xz).vertices]
            bot.add_patch(Polygon(hull, fc=PIECE[s["piece"]], ec="k", lw=0.4, alpha=0.85))
        r = place.get(int(s["move"]))
        if r and r.get("place_mm"):
            bot.text(1000 * s["pos"][0], 1000 * s["pos"][2], f"{float(r['place_mm']):.0f} mm",
                     ha="center", va="center", fontsize=6.5)
    top.set_title(f"after move {k}")
axes[0, 0].text(-90, ROWS * CELL * 500, "planned", rotation=90, ha="center", va="center")
axes[1, 0].text(-90, ROWS * CELL * 500, "PyBullet", rotation=90, ha="center", va="center")
save(fig, "ch10_tetris/planned_vs_pybullet")
print("  from", bpath)
