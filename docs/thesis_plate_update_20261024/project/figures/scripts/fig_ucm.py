"""UCM, session C (K_p,rot = 40): do the two hands co-vary, along the grasp triad?

    python fig_ucm.py   -> ch05_prior/ucm_session_c.pdf

Same index and same data as 07_torsional_ucm/B_ucm/24_ucm_atlas.py (residuals per
run and per gravity cell, grasp drift <= 30 deg, unperturbed runs, phase 2):

    r(n) = 1 - 2 n' S_LR n / (n' S_LL n + n' S_RR n)    in [0, 2]
    r < 1  hands in phase (the internal mode is HELD), r = 1 independent, r > 1 free

Left: the torque about the grasp axis, left hand against right (the axis where
the 3-point contact has no torsional term of its own). Right: r on u, v, w for
forces and torques, 95% CI by bootstrap over RUNS.
"""
import glob
import os
import sys

import h5py
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from style import HELD, FREE, GREY, REPO, plt, save

UCM = REPO / "lorenzo_data_analysis" / "07_torsional_ucm"
sys.path.insert(0, str(UCM))
from lib.datapath import EXP13          # noqa: E402
from lib.sessions import session_of     # noqa: E402

PHASE, DRIFT_OK, G = 2, 30.0, np.array([0.0, 0.0, -9.81])


def triad(pL, pR):
    """u = left->right grasp axis, v = vertical in the grasp plane, w = u x v."""
    u = pR - pL
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    v = np.array([0, 0, 1.0]) - (u @ np.array([0, 0, 1.0]))[:, None] * u
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    return np.stack([u, v, np.cross(u, v)], axis=1)


def cell(gl):
    th = np.degrees(np.arccos(np.clip(-gl[:, 2], -1, 1)))
    ph = np.degrees(np.arctan2(gl[:, 1], gl[:, 0])) % 360
    return np.floor(th / 12).astype(int) * 1000 + np.floor(ph / 30).astype(int)


def residual(L, R, cid):
    L, R = L - L.mean(0), R - R.mean(0)
    ok = np.zeros(len(L), bool)
    for c in np.unique(cid):
        m = cid == c
        if m.sum() >= 25:
            L[m] -= L[m].mean(0)
            R[m] -= R[m].mean(0)
            ok |= m
    return L[ok], R[ok]


runs = []
for path in sorted(glob.glob(os.path.join(EXP13, "exploration_torsional_run_*.h5"))):
    with h5py.File(path, "r") as f:
        d = {k: np.array(f[k]) for k in f.keys()}
    if session_of(int(d["meta_run_id"][0])) != "C" or d["meta_perturb"][0] > 0.5:
        continue
    w = d["phase"] == PHASE
    if w.sum() < 200:
        continue
    Rb = Rot.from_quat(d["box_pose"][w, 3:7])
    rl = Rot.from_quat(d["hand_L_pose"][w, 3:7]).inv() * Rb
    rr = Rot.from_quat(d["hand_R_pose"][w, 3:7]).inv() * Rb
    ang = lambda r: np.degrees(np.linalg.norm(r.as_rotvec(), axis=1))
    keep = 0.5 * (ang(rl[0].inv() * rl) + ang(rr[0].inv() * rr)) <= DRIFT_OK
    if keep.sum() < 200:
        continue
    B = triad(d["hand_L_pose"][w, :3], d["hand_R_pose"][w, :3])[keep]
    gl = Rb.inv().apply(np.tile(G, (int(w.sum()), 1)))[keep]
    gl /= np.linalg.norm(gl, axis=1, keepdims=True)
    rec = {}
    for tag, sl in (("F", slice(0, 3)), ("T", slice(3, 6))):
        L = np.einsum("nij,nj->ni", B, d["wrench_L"][w, sl][keep])
        R = np.einsum("nij,nj->ni", B, d["wrench_R"][w, sl][keep])
        rec[tag] = residual(L, R, cell(gl))
    runs.append(rec)
print(f"  session C: {len(runs)} runs")


def r_index(L, R, j):
    S = np.cov(L[:, j], R[:, j])
    return 1.0 - 2.0 * S[0, 1] / (S[0, 0] + S[1, 1])


rng = np.random.default_rng(7)
res = {}
for tag in ("F", "T"):
    L = np.vstack([r[tag][0] for r in runs])
    R = np.vstack([r[tag][1] for r in runs])
    for j in range(3):
        boot = []
        for _ in range(1500):
            pick = rng.integers(0, len(runs), len(runs))
            boot.append(r_index(np.vstack([runs[i][tag][0] for i in pick]),
                                np.vstack([runs[i][tag][1] for i in pick]), j))
        res[tag, j] = (r_index(L, R, j), *np.percentile(boot, [2.5, 97.5]))
        print(f"  {tag} {'uvw'[j]}: r = {res[tag, j][0]:.2f} [{res[tag, j][1]:.2f}, {res[tag, j][2]:.2f}]")

fig, (a, b) = plt.subplots(1, 2, figsize=(7.2, 3.0), layout="constrained",
                           gridspec_kw=dict(width_ratios=[1, 1.35]))
L = np.vstack([r["T"][0] for r in runs])[:, 0]
R = np.vstack([r["T"][1] for r in runs])[:, 0]
sub = rng.choice(len(L), min(4000, len(L)), replace=False)
a.scatter(L[sub], R[sub], s=2, color=GREY, alpha=0.25, lw=0)
lim = 0.6
a.plot([-lim, lim], [lim, -lim], color=FREE, lw=1.0)
a.plot([-lim, lim], [-lim, lim], color=HELD, lw=1.0, ls=":")
a.text(0.42, -0.52, "internal", color=FREE, ha="center")
a.text(0.42, 0.50, "net", color=HELD, ha="center")
a.set_xlim(-lim, lim); a.set_ylim(-lim, lim); a.set_aspect("equal")
a.set_xlabel(r"left hand $\tau_u$ [Nm]"); a.set_ylabel(r"right hand $\tau_u$ [Nm]")
a.set_title("torque about the grasp axis")

x = np.arange(3)
for k, (tag, name) in enumerate((("F", "force"), ("T", "torque"))):
    val = np.array([res[tag, j][0] for j in range(3)])
    lo = val - np.array([res[tag, j][1] for j in range(3)])
    hi = np.array([res[tag, j][2] for j in range(3)]) - val
    col = [FREE if res[tag, j][1] > 1 else HELD if res[tag, j][2] < 1 else "0.75" for j in range(3)]
    b.bar(x + (k - 0.5) * 0.36, val, 0.34, color=col, alpha=0.85 if tag == "F" else 0.45,
          edgecolor="k", lw=0.5, hatch=None if tag == "F" else "///", label=name)
    b.errorbar(x + (k - 0.5) * 0.36, val, yerr=[lo, hi], fmt="none", ecolor="k", lw=0.8, capsize=2)
b.axhline(1.0, color="k", lw=0.8)
b.set_xticks(x, ["u  grasp axis", "v  vertical", "w  horizontal"])
b.set_ylim(0, 2.05); b.set_ylabel("r  (1 = independent hands)")
b.text(2.55, 1.9, "free", color=FREE, ha="right"); b.text(2.55, 0.08, "held", color=HELD, ha="right")
b.legend(loc="upper center", ncol=2, bbox_to_anchor=(0.45, 1.12))
save(fig, "ch05_prior/ucm_session_c")
