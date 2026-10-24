"""
English, vector versions of the four study figures used in the thesis
(chapter 7). Same data as plots.py / margin_sweep.py.

    python thesis_figures.py OUT_DIR
"""
import csv
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                      # noqa: E402
from matplotlib.lines import Line2D                                  # noqa: E402
from matplotlib.patches import Patch                                 # noqa: E402

from analyze import analyse, RUNS, OUT                               # noqa: E402

C = {"rigid": "#2a78d6", "plate": "#eb6834", "plan": "#8a8985"}
LBL = {"rigid": "fixed tri-pad", "plate": "floating plate"}
FAM = ["cube", "jittered", "deformed", "polytope"]
CMD = "full_nb"
plt.rcParams.update({"font.size": 9, "font.family": "serif",
                     "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.color": "#e4e3df", "grid.linewidth": 0.6,
                     "axes.edgecolor": "#8a8985"})


def cones(out):
    fig, axes = plt.subplots(2, 6, figsize=(13, 4.6), sharex="row", sharey="row")
    for r, arm in enumerate(("rigid", "plate")):
        name = f"pop_cube__000__{arm}__{CMD}"
        c = analyse(os.path.join(RUNS, name + ".npz"),
                    json.load(open(os.path.join(RUNS, name + ".json"))))[1]
        mu = 1.0
        nmax = max(np.nanmax(c["N"]), np.nanmax(c["Np"])) * 1.1
        for k in range(6):
            ax = axes[r, k]
            m = c["pad"] == k
            ax.scatter(c["Np"][m], c["Tp"][m], s=4, color=C["plan"], alpha=.35, rasterized=True,
                       label="SOCP witness")
            ax.scatter(c["N"][m], c["T"][m], s=4, color=C[arm], alpha=.35, rasterized=True,
                       label="realised (PyBullet)")
            rho = float(np.median(c["rho"][m])) if m.any() else 3.0
            n = np.linspace(0, nmax, 50)
            ax.plot(n, mu * n, color="#0b0b0b", lw=1.2, label=r"nominal cone $T=\mu N$")
            ax.plot(n, np.maximum(mu * n - np.sqrt(1 + mu ** 2) * rho, 0), color="#0b0b0b",
                    lw=1, ls="--", label=r"shifted cone $\psi=\rho$")
            sat = np.mean(c["eta"][m] >= .99) if m.any() else np.nan
            ax.set_title(f"{'LR'[k // 3]}{k % 3}: saturated {100 * sat:.0f}%", fontsize=8)
            if k == 0:
                ax.set_ylabel(f"{LBL[arm]}\ntangential $T$ [N]")
            if r == 1:
                ax.set_xlabel("normal $N$ [N]")
    h, l = axes[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=4, frameon=False, markerscale=3)
    fig.tight_layout(rect=(0, 0, 1, .92))
    fig.savefig(os.path.join(out, "cones_cube_example.pdf"), dpi=200)
    plt.close(fig)


def decomposition(out):
    S = np.load(os.path.join(OUT, "samples.npz"))
    get = lambda f, a, n: S[f"{f}|{a}|{CMD}|{n}"] if f"{f}|{a}|{CMD}|{n}" in S.files else np.array([])  # noqa: E731
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.4))
    w = .38
    for r, arm in enumerate(("rigid", "plate")):
        rng_, nul, dfo = [], [], []
        for fam in FAM:
            dr, dn, rho, df = (get(fam, arm, x) for x in ("df_range", "df_null", "rho", "df"))
            rng_.append(np.median(dr) if len(dr) else np.nan)
            nul.append(np.median(dn) if len(dn) else np.nan)
            dfo.append(np.median(df / np.maximum(rho, 1e-9)) if len(df) else np.nan)
        x = np.arange(len(FAM)) + (r - .5) * w
        axes[0].bar(x, nul, w * .95, color=C[arm], label=f"{LBL[arm]}: internal (null space)")
        axes[0].bar(x, rng_, w * .95, bottom=nul, color=C[arm], alpha=.4,
                    label=f"{LBL[arm]}: seen at the wrist")
        axes[1].bar(x, dfo, w * .95, color=C[arm], label=LBL[arm])
    for ax in axes:
        ax.set_xticks(range(len(FAM)))
        ax.set_xticklabels(FAM)
    axes[0].set_ylabel(r"median $|\Delta f_i|$ per pad [N]")
    axes[0].legend(fontsize=7, frameon=False)
    axes[1].axhline(1, color="#0b0b0b", lw=1)
    axes[1].set_ylabel(r"median $|\Delta f_i|/\rho_i$")
    axes[1].legend(fontsize=7, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "error_decomposition.pdf"))
    plt.close(fig)


def wrench_margin(out):
    W = [w for w in csv.DictReader(open(os.path.join(OUT, "wrench_margin.csv")))
         if w["command"] == CMD]
    fig, ax = plt.subplots(figsize=(9, 3.6))
    for r, arm in enumerate(("rigid", "plate")):
        a, b, c = [], [], []
        for fam in FAM:
            rows = [w for w in W if w["family"] == fam and w["arm"] == arm]
            rho = np.array([float(w["rho"]) for w in rows])
            tr = np.array([float(w["t_real"]) for w in rows])
            a.append(np.mean(np.array([float(w["psi_pad_min"]) for w in rows]) < rho - 1e-3))
            b.append(np.mean(tr >= rho - 1e-3))
            c.append(np.mean(tr > 0))
        x = np.arange(len(FAM)) + (r - .5) * .4
        ax.bar(x, a, .38, color=C[arm], alpha=.35)
        ax.scatter(x, b, color=C[arm], marker="o", s=30, zorder=3, edgecolor="white")
        ax.scatter(x, c, color=C[arm], marker="^", s=30, zorder=3, edgecolor="white")
    ax.set_xticks(range(len(FAM)))
    ax.set_xticklabels(FAM)
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("fraction of hand-instants")
    fig.legend([Patch(color="#8a8985", alpha=.35), Line2D([], [], marker="o", ls="", color="#52514e"),
                Line2D([], [], marker="^", ls="", color="#52514e"),
                Patch(color=C["rigid"]), Patch(color=C["plate"])],
               [r"some pad with $\psi<\rho$", r"realised wrench admits $\psi\geq\rho$",
                r"realised wrench admits $\psi>0$", LBL["rigid"], LBL["plate"]],
               fontsize=7, frameon=False, loc="lower center", ncol=5)
    fig.tight_layout(rect=(0, .1, 1, 1))
    fig.savefig(os.path.join(out, "pad_vs_wrench_margin.pdf"))
    plt.close(fig)


def margin_sweep(out):
    here = os.path.dirname(os.path.abspath(__file__))
    t = list(csv.DictReader(open(os.path.join(here, "margin_sweep.csv"))))
    panels = (("N_pad_N", "squeeze: median pad normal [N]"),
              ("df_N", r"per-pad error $|\Delta f_i|$ [N]"),
              ("sat", r"saturated pad samples ($\eta\geq0.99$)"),
              ("slip", "sliding pad samples"))
    fig, axes = plt.subplots(1, 4, figsize=(13, 3.2))
    for ax, (col, title) in zip(axes, panels):
        for arm in ("rigid", "plate"):
            s = [r for r in t if r["arm"] == arm]
            ax.plot([float(r["lambda_m"]) for r in s], [float(r[col]) for r in s], color=C[arm],
                    lw=2, marker="o", ms=8, markeredgecolor="white", label=LBL[arm])
        ax.set_title(title, fontsize=9)
        ax.set_xticks([0.25, 0.5, 0.75])
        ax.set_xlabel(r"$\lambda_m$")
        ax.set_ylim(bottom=0)
    axes[0].legend(fontsize=7, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "margin_sweep.pdf"))
    plt.close(fig)


if __name__ == "__main__":
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    cones(out); decomposition(out); wrench_margin(out); margin_sweep(out)
    print("figures ->", out)
