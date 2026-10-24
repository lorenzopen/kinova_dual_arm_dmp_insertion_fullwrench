"""The geometry campaign's population, a few bodies of each family.

    python fig_zoo.py   -> ch09_geometry/polytope_zoo.pdf

The bodies are the campaign's own (campaign_geometry/populations, built by
make_populations.py). Nothing is chosen by outcome: the cube family shows the
one-corner pushes and the first pair of the jitter ladder, the polytope row the
first body of each face band. The dashed cube is the perfect 120 mm cube; red
marks where a corner moved.
"""
import sys

import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from style import REPO, plt, save

TRI = REPO / "lorenzo_data_analysis" / "10_tripad_passivo"
sys.path.insert(0, str(TRI))
from lib import population as POP    # noqa: E402

POPS = TRI / "campaign_geometry" / "populations"
H = 0.06
CUBE = np.array([[x, y, z] for x in (-H, H) for y in (-H, H) for z in (-H, H)])
EDGES = [(i, j) for i in range(8) for j in range(i + 1, 8)
         if np.sum(np.sign(CUBE[i]) != np.sign(CUBE[j])) == 1]


def body(fam, idx=None, **match):
    objs = POP.load_objects(str(POPS / fam))
    for o in objs:
        if (idx is None or int(o["idx"]) == idx) and all(o.get(k) == v for k, v in match.items()):
            return o
    raise KeyError(fam, idx, match)


def moved(o):
    if "moved" in o:
        return [int(c) for c in str(o["moved"]).split("-") if c]
    if o.get("family") == "one_corner" and o.get("push_mm", 0):
        return [int(o["corner"])]
    return []


def draw(ax, o, title, colour="#8cbcd6"):
    pc = Poly3DCollection([f["poly"] for f in o["faces"]], alpha=0.7, edgecolor="k", linewidths=0.35)
    pc.set_facecolor(colour)
    ax.add_collection3d(pc)
    if not str(o.get("family", "")).startswith("poly"):
        for i, j in EDGES:
            ax.plot(*zip(CUBE[i], CUBE[j]), ls="--", lw=0.5, color="0.5")
        V = np.asarray(o["V"], float)
        for c in moved(o):
            k = int(np.argmin(np.linalg.norm(V - CUBE[c], axis=1)))
            ax.plot(*zip(CUBE[c], V[k]), color="#c62828", lw=1.3)
            ax.scatter(*V[k], color="#c62828", s=8)
    s = 0.55 * max(float(np.max(o["extent"])), 0.13)
    for d in "xyz":
        getattr(ax, f"set_{d}lim")(-s, s)
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(22, 35)
    ax.axis("off")
    ax.set_title(title, fontsize=8, pad=-4)


row1 = [(body("one_corner", 0), "cube"),
        (body("one_corner", 3), "one corner, 24 mm"),
        (body("jitter_k4_standard", pair=0), "4 corners jittered"),
        (body("jitter_k8_harsh", pair=0), "8 corners, harsh")]
row2 = [(body(f"poly_f{a}_{b}", 0), f"polytope, {a}-{b} faces") for a, b in
        ((10, 20), (20, 50), (50, 100), (100, 200))]
fig = plt.figure(figsize=(7.2, 3.9))
for k, (o, t) in enumerate(row1 + row2):
    ax = fig.add_subplot(2, 4, k + 1, projection="3d")
    draw(ax, o, f"{t}\n{o['n_faces']} faces", "#8cbcd6" if k < 4 else "#b9a6d9")
fig.subplots_adjust(wspace=0.0, hspace=0.05, left=0, right=1, top=0.95, bottom=0)
save(fig, "ch09_geometry/polytope_zoo")
