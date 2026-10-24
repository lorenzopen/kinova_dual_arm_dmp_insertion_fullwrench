"""What a learnt DMP route looks like, at a glance: three narrowed slots.

    python fig_learnt_routes.py   -> ch10_tetris/learnt_routes.pdf

Worlds of the narrow75 set at the real size (100 mm cubes, 70 mm tri-pad), the
routes exported for the Kinova package (kinova_dual_arm_dmp_insertion/cubes100,
routes/narrow75_100/*.json). Drawn as cubes100/plot_routes.py draws them, with
less on the page: the neighbours as settled, the piece at 8 points of the route,
the path of its centre, the slot in red, and the straight descent dashed for
reference. Placement and slip are the route's own PyBullet replay.
"""
import json
import sys

import numpy as np
from matplotlib.patches import Rectangle
from scipy.spatial.transform import Rotation as Rot

from style import PIECE, REPO, plt, save

C100 = REPO / "lorenzo_data_analysis" / "kinova_dual_arm_dmp_insertion" / "cubes100"
sys.path.insert(0, str(C100))
import common as C          # noqa: E402
import push100 as P         # noqa: E402
sys.path[:0] = [str(C.TIGHT.parent)]
import geometry as G        # noqa: E402
from prepare import catalogue  # noqa: E402

SET = "narrow75_100"
WORLDS = ("O_s7094m3", "L_s7098m4", "S_s7103m3")
objs = catalogue()


def cell_boxes(cells, R, c, half):
    return [(q[0] - half, q[2] - half) for q in (c + R.apply(p) for p in cells)]


fig, axes = plt.subplots(1, len(WORLDS), figsize=(7.2, 3.6), layout="constrained", sharey=True)
for ax, world in zip(axes, WORLDS):
    d = json.loads((C100 / "routes" / SET / f"{world}.json").read_text())
    piece = d["piece"]
    cen = np.asarray(d["route"]["centres"], float)
    R = [Rot.from_quat(q) for q in d["route"]["quats"]]
    tgt = np.asarray(d["target"]["position"], float)
    Rt = Rot.from_quat(d["target"]["quat"])
    cells = np.asarray(d["piece_model"]["cells"], float)
    half = float(d["piece_model"]["cell"]) / 2
    for n in d["neighbours"]:
        for v in G.body_columns(objs[n["piece"]], np.asarray(n["position"]) / C.S, n["quat"]):
            v = v * C.S
            ax.add_patch(Rectangle((1000 * v[:, 0].min(), 1000 * v[:, 2].min()),
                                   1000 * np.ptp(v[:, 0]), 1000 * np.ptp(v[:, 2]),
                                   fc=PIECE[n["piece"]], alpha=0.25, ec="0.4", lw=0.3))
    for i in np.linspace(0, len(cen) - 1, 8).astype(int):
        for x0, z0 in cell_boxes(cells, R[i], cen[i], half):
            ax.add_patch(Rectangle((1000 * x0, 1000 * z0), 1000 * 2 * half, 1000 * 2 * half,
                                   fill=False, ec=PIECE[piece], lw=0.6, alpha=0.5))
    for x0, z0 in cell_boxes(cells, Rt, tgt, half):
        ax.add_patch(Rectangle((1000 * x0, 1000 * z0), 1000 * 2 * half, 1000 * 2 * half,
                               fill=False, ec="#c62828", lw=1.3))
    ax.plot([1000 * tgt[0]] * 2, [1000 * cen[0, 2], 1000 * tgt[2]], color="0.4", ls="--", lw=0.8)
    ax.plot(1000 * cen[:, 0], 1000 * cen[:, 2], "k-", lw=1.3)
    sh = d["shelf"]
    for xw in (1000 * sh["inner_faces"][0] - 20, 1000 * sh["inner_faces"][1]):
        ax.add_patch(Rectangle((xw, 0), 20, 620, fc="0.35", lw=0))
    ax.set_xlim(-40, 660)
    ax.set_ylim(-5, 1000 * cen[:, 2].max() + 80)
    ax.set_aspect("equal")
    sim = d["simulated"]
    ax.set_title(f"{piece}: placed {sim['placed_mm']:.1f} mm", color=PIECE[piece] if piece != "O" else "0.2")
    ax.set_xlabel("along the shelf [mm]")
axes[0].set_ylabel("height [mm]")
axes[0].plot([], [], "k-", label="learnt route (centre)")
axes[0].plot([], [], color="0.4", ls="--", label="straight descent")
axes[0].plot([], [], color="#c62828", lw=1.3, label="slot")
fig.legend(loc="lower center", ncol=3, bbox_to_anchor=(0.5, -0.06))
save(fig, "ch10_tetris/learnt_routes")
