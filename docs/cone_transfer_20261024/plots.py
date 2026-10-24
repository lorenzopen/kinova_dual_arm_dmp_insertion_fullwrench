"""Figures for REPORT.md, from runs/ and samples.npz (run analyze.py first)."""
import glob
import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                      # noqa: E402

from analyze import analyse, POP_LABEL, RUNS, OUT                               # noqa: E402

FIG = os.path.join(OUT, "figures")
os.makedirs(FIG, exist_ok=True)
C = {"rigid": "#2a78d6", "plate": "#eb6834", "plan": "#8a8985"}
LBL = {"rigid": "tripod fisso", "plate": "floating plate nominale"}
FAM = ["cube", "jittered", "deformed", "polytope"]
MU = float(np.load(sorted(glob.glob(os.path.join(RUNS, "*.npz")))[0])["mu"])
SC = np.sqrt(1 + MU ** 2)
plt.rcParams.update({"font.size": 9, "axes.spines.top": False,
                     "axes.spines.right": False, "axes.grid": True,
                     "grid.color": "#e4e3df", "grid.linewidth": 0.6,
                     "axes.edgecolor": "#8a8985", "savefig.dpi": 160})
S = np.load(os.path.join(OUT, "samples.npz"))
ROWS = json.load(open(os.path.join(OUT, "metrics.json")))


def get(fam, arm, cmd, name):
    k = f"{fam}|{arm}|{cmd}|{name}"
    return S[k] if k in S.files else np.array([])


def run_samples(name):
    meta = json.load(open(os.path.join(RUNS, name + ".json")))
    _, c = analyse(os.path.join(RUNS, name + ".npz"), meta)
    return c


def cone_lines(ax, nmax, rho):
    n = np.linspace(0, nmax, 50)
    ax.plot(n, MU * n, color="#0b0b0b", lw=1.2, label="cono nominale T = μN")
    ax.plot(n, np.maximum(MU * n - SC * rho, 0), color="#0b0b0b", lw=1, ls="--",
            label=f"cono traslato ψ = ρ ({rho:.1f} N)")


# 1 ---- the six cones of the declared example, rigid vs plate
def fig_example(case=None):
    case = case or "pop_cube__000__{}__" + VAR[0][0]
    fig, axes = plt.subplots(2, 6, figsize=(13, 4.6), sharex="row", sharey="row")
    for r, arm in enumerate(("rigid", "plate")):
        c = run_samples(case.format(arm))
        nmax = max(np.nanmax(c["N"]), np.nanmax(c["Np"])) * 1.1
        for k in range(6):
            ax = axes[r, k]
            m = c["pad"] == k
            ax.scatter(c["Np"][m], c["Tp"][m], s=4, color=C["plan"], alpha=.35,
                       rasterized=True, label="piano SOCP")
            ax.scatter(c["N"][m], c["T"][m], s=4, color=C[arm], alpha=.35,
                       rasterized=True, label="realizzato (PyBullet)")
            cone_lines(ax, nmax, float(np.median(c["rho"][m])) if m.any() else 2)
            sat = np.mean(c["eta"][m] >= .99) if m.any() else np.nan
            ax.set_title(f"{'LR'[k // 3]}{k % 3}  saturo {100 * sat:.0f}%", fontsize=8)
            if k == 0:
                ax.set_ylabel(f"{LBL[arm]}\nT tangenziale [N]")
            if r == 1:
                ax.set_xlabel("N normale [N]")
    h, l = axes[0, 0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", ncol=4, frameon=False, markerscale=3)
    fig.tight_layout(rect=(0, 0, 1, .92))
    fig.savefig(os.path.join(FIG, "01_cones_cube_example.png"))
    plt.close(fig)


# 2 ---- time series of eta, same example
def fig_timeseries(case=None):
    case = case or "pop_cube__000__{}__" + VAR[0][0]
    fig, axes = plt.subplots(2, 1, figsize=(10, 4.8), sharex=True, sharey=True)
    for ax, arm in zip(axes, ("rigid", "plate")):
        c = run_samples(case.format(arm))
        t0 = c["t"].min()
        for k in range(6):
            m = c["pad"] == k
            tt = (c["t"][m] - t0) / 1000
            ax.plot(tt, c["etap"][m], color=C["plan"], lw=.8, alpha=.8, zorder=2)
            ax.plot(tt, c["eta"][m], color=C[arm], lw=1.6, alpha=.85, zorder=3)
        ax.axhline(1, color="#0b0b0b", lw=.8, ls=":", zorder=1)
        ax.set_ylabel("η = T/(μN)")
        ax.set_title(f"{LBL[arm]}: grigio = piano SOCP, colore = realizzato (6 pad)",
                     fontsize=9, loc="left")
        ax.set_ylim(0, 1.08)
    axes[-1].set_xlabel("tempo da inizio sollevamento [s]")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "02_eta_time_cube_example.png"))
    plt.close(fig)


# 3 ---- pooled cones per family, normalised so all objects share one picture
def fig_pooled_cones(cmd="full"):
    fig, axes = plt.subplots(2, 4, figsize=(11.5, 5.4), sharex=True, sharey=True)
    for j, fam in enumerate(FAM):
        for r, arm in enumerate(("rigid", "plate")):
            ax = axes[r, j]
            N, T, Np, Tp = (get(fam, arm, cmd, x) for x in ("N", "T", "Np", "Tp"))
            if len(N) == 0:
                ax.set_axis_off()
                continue
            sel = np.random.default_rng(0).choice(len(N), min(len(N), 6000), replace=False)
            ax.scatter(Np[sel], Tp[sel], s=2, color=C["plan"], alpha=.2, rasterized=True)
            ax.scatter(N[sel], T[sel], s=2, color=C[arm], alpha=.2, rasterized=True)
            n = np.linspace(0, 40, 10)
            ax.plot(n, MU * n, color="#0b0b0b", lw=1.1)
            ax.set_xlim(0, 40); ax.set_ylim(0, 40 * MU)
            eta = T / np.maximum(MU * N, 1e-9)
            ax.set_title(f"{fam} · saturi {100 * np.mean(eta >= .99):.0f}%", fontsize=9)
            if j == 0:
                ax.set_ylabel(f"{LBL[arm]}\nT [N]")
            if r == 1:
                ax.set_xlabel("N [N]")
    fig.suptitle(f"Tutti i campioni pad in aria (lift + wielding, comando {cmd}): "
                 "grigio piano, colore realizzato, linea = cono nominale", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, .95))
    fig.savefig(os.path.join(FIG, f"03_pooled_cones_{cmd}.png"))
    plt.close(fig)


# 4 ---- ECDF of cone utilisation
def fig_ecdf(cmd="full"):
    fig, axes = plt.subplots(1, 4, figsize=(11.5, 3.2), sharey=True)
    for ax, fam in zip(axes, FAM):
        for arm in ("rigid", "plate"):
            for name, ls, col in (("etap", ":", C[arm]), ("eta", "-", C[arm])):
                v = np.sort(get(fam, arm, cmd, name))
                if len(v):
                    ax.plot(v, np.arange(1, len(v) + 1) / len(v), ls=ls, color=col, lw=1.5,
                            label=f"{LBL[arm]} {'piano' if name == 'etap' else 'reale'}")
        ax.axvline(1, color="#0b0b0b", lw=.8)
        ax.set_title(fam)
        ax.set_xlabel("η = T/(μN)")
        ax.set_xlim(0, 1.05)
    axes[0].set_ylabel("frazione campioni ≤ η")
    axes[0].legend(fontsize=7, frameon=False, loc="upper left")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, f"04_eta_ecdf_{cmd}.png"))
    plt.close(fig)


# 5 ---- margin surplus psi - rho
def fig_margin(cmd="full"):
    fig, ax = plt.subplots(figsize=(10, 3.6))
    pos, data, cols, ticks = [], [], [], []
    for j, fam in enumerate(FAM):
        for r, arm in enumerate(("rigid", "plate")):
            psi, rho, psip = (get(fam, arm, cmd, x) for x in ("psi", "rho", "psip"))
            if not len(psi):
                continue
            x = 3 * j + r
            data += [psip - rho, psi - rho]
            pos += [x - .18, x + .18]
            cols += [C["plan"], C[arm]]
        ticks.append((3 * j + .5, fam))
    bp = ax.boxplot(data, positions=pos, widths=.3, showfliers=False, patch_artist=True,
                    medianprops=dict(color="#0b0b0b"))
    for b, col in zip(bp["boxes"], cols):
        b.set_facecolor(col); b.set_alpha(.7); b.set_edgecolor(col)
    ax.axhline(0, color="#0b0b0b", lw=1)
    ax.set_xticks([t[0] for t in ticks]); ax.set_xticklabels([t[1] for t in ticks])
    ax.set_ylabel("ψ − ρ  [N]\n(<0: margine perso)")
    from matplotlib.patches import Patch
    ax.legend([Patch(color=C["plan"]), Patch(color=C["rigid"]), Patch(color=C["plate"])],
              ["piano SOCP", "reale, tripod fisso", "reale, floating plate"],
              frameon=False, ncol=3, loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, f"05_margin_surplus_{cmd}.png"))
    plt.close(fig)


# 6 ---- where the error lives: wrist-visible vs internal
def fig_null(cmd="full"):
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.4))
    w = .38
    for r, arm in enumerate(("rigid", "plate")):
        rng_, nul, dfo = [], [], []
        for fam in FAM:
            dr, dn, rho = (get(fam, arm, cmd, x) for x in ("df_range", "df_null", "rho"))
            rng_.append(np.median(dr) if len(dr) else np.nan)
            nul.append(np.median(dn) if len(dn) else np.nan)
            df = get(fam, arm, cmd, "df")
            dfo.append(np.median(df / np.maximum(rho, 1e-9)) if len(df) else np.nan)
        x = np.arange(len(FAM)) + (r - .5) * w
        axes[0].bar(x, nul, w * .95, color=C[arm], label=f"{LBL[arm]}: interna (spazio nullo)")
        axes[0].bar(x, rng_, w * .95, bottom=nul, color=C[arm], alpha=.4,
                    label=f"{LBL[arm]}: visibile al polso")
        axes[1].bar(x, dfo, w * .95, color=C[arm], label=LBL[arm])
    for ax in axes:
        ax.set_xticks(range(len(FAM))); ax.set_xticklabels(FAM)
    axes[0].set_ylabel("mediana |Δf_i| per pad [N]")
    axes[0].legend(fontsize=7, frameon=False)
    axes[1].axhline(1, color="#0b0b0b", lw=1)
    axes[1].set_ylabel("mediana |Δf_i| / ρ_i")
    axes[1].set_title("sopra 1: errore oltre il margine (garanzia Lipschitz nulla)",
                      fontsize=8, loc="left")
    axes[1].legend(fontsize=7, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, f"06_error_decomposition_{cmd}.png"))
    plt.close(fig)


# 7 ---- command and SOCP variant
VAR = (("full", "full"), ("full_nb", "full\nno-bend"), ("scalar", "scalare"))
VAR = tuple(v for v in VAR if any(r.get("command") == v[0] for r in ROWS))


def fig_command():
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.4))
    metrics = (("sat_frac", "pad-campioni saturi (η≥0.99)"),
               ("margin_lost_frac", "pad-campioni con ψ<ρ"),
               ("df_med", "mediana |Δf_i| [N]"))
    for ax, (m, title) in zip(axes, metrics):
        ticks = []
        for i, (cmd, lab) in enumerate(VAR):
            for r, arm in enumerate(("rigid", "plate")):
                v = [x[m] for x in ROWS if x.get("arm") == arm and x.get("command") == cmd
                     and x.get(m) is not None and x.get("n_samples", 0) > 0]
                if not v:
                    continue
                ax.boxplot([v], positions=[3 * i + r], widths=.6, showfliers=False,
                           patch_artist=True,
                           boxprops=dict(facecolor=C[arm], alpha=.75, edgecolor=C[arm]),
                           medianprops=dict(color="#0b0b0b"))
            ticks.append((3 * i + .5, lab))
        ax.set_xticks([t[0] for t in ticks]); ax.set_xticklabels([t[1] for t in ticks])
        ax.set_title(title, fontsize=9)
    from matplotlib.patches import Patch
    axes[0].legend([Patch(color=C["rigid"]), Patch(color=C["plate"])],
                   [LBL["rigid"], LBL["plate"]], frameon=False, fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "07_command_variants.png"))
    plt.close(fig)


# 8 ---- pad level vs wrench level margin
def fig_wrench_margin():
    import csv
    W = list(csv.DictReader(open(os.path.join(OUT, "wrench_margin.csv"))))
    fig, axes = plt.subplots(1, len(VAR), figsize=(max(10, 4.4 * len(VAR) + 1), 3.6), sharey=True, squeeze=False)
    axes = axes[0]
    for ax, (cmd, lab) in zip(axes, VAR):
        for r, arm in enumerate(("rigid", "plate")):
            a, b, c = [], [], []
            for fam in FAM:
                rows = [w for w in W if w["family"] == fam and w["arm"] == arm and w["command"] == cmd]
                if not rows:
                    a.append(np.nan); b.append(np.nan); c.append(np.nan); continue
                rho = np.array([float(w["rho"]) for w in rows])
                a.append(np.mean(np.array([float(w["psi_pad_min"]) for w in rows]) < rho - 1e-3))
                tr = np.array([float(w["t_real"]) for w in rows])
                b.append(np.mean(tr >= rho - 1e-3))
                c.append(np.mean(tr > 0))
            x = np.arange(len(FAM)) + (r - .5) * .4
            ax.bar(x, a, .38, color=C[arm], alpha=.35)
            ax.scatter(x, b, color=C[arm], marker="o", s=30, zorder=3, edgecolor="white")
            ax.scatter(x, c, color=C[arm], marker="^", s=30, zorder=3, edgecolor="white")
        ax.set_xticks(range(len(FAM))); ax.set_xticklabels(FAM)
        ax.set_title(f"comando {lab.replace(chr(10), ' ')}", fontsize=9)
        ax.set_ylim(0, 1.05)
    axes[0].set_ylabel("frazione mani-istanti")
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    fig.legend([Patch(color="#8a8985", alpha=.35),
                    Line2D([], [], marker="o", ls="", color="#52514e"),
                    Line2D([], [], marker="^", ls="", color="#52514e")],
                   ["almeno un pad con ψ<ρ", "wrench reale ridistribuibile con ψ≥ρ",
                    "wrench reale ridistribuibile con ψ>0"], fontsize=7, frameon=False,
                   loc="lower center", ncol=3, bbox_to_anchor=(.5, .04))
    fig.text(.01, .01, "blu = tripod fisso, arancio = floating plate nominale", fontsize=7, color="#52514e")
    fig.tight_layout(rect=(0, .14, 1, 1))
    fig.savefig(os.path.join(FIG, "08_pad_vs_wrench_margin.png"))
    plt.close(fig)


# 9 ---- outcomes
def fig_outcomes():
    stages = (("DONE", "#1baf7a"), ("DROPPED_WIELD", "#eda100"),
              ("DROPPED_LIFT", "#e34948"), ("NO_GRASP", "#8a8985"))
    fig, axes = plt.subplots(1, len(VAR), figsize=(4.4 * len(VAR) + 1, 3.2), sharey=True, squeeze=False)
    axes = axes[0]
    for ax, (cmd, lab) in zip(axes, VAR):
        labels, x = [], 0
        for fam in FAM:
            for arm in ("rigid", "plate"):
                rr = [r for r in ROWS if r.get("family") == fam and r.get("arm") == arm
                      and r.get("command") == cmd]
                tot = max(len(rr), 1)
                bottom = 0
                for st, col in stages:
                    v = sum(r.get("stage") == st for r in rr) / tot
                    ax.bar(x, v, .8, bottom=bottom, color=col, edgecolor="white", lw=1)
                    bottom += v
                labels.append("F" if arm == "rigid" else "P")
                x += 1
            x += .5
        pos = [i + .5 * (i // 2) for i in range(len(labels))]
        ax.set_xticks(pos); ax.set_xticklabels(labels, fontsize=7)
        for j, fam in enumerate(FAM):
            ax.text(pos[2 * j] + .5, -.13, fam, ha="center", va="top", fontsize=8,
                    transform=ax.get_xaxis_transform())
        ax.set_title(f"comando {lab.replace(chr(10), ' ')}", fontsize=9)
    axes[0].set_ylabel("frazione oggetti")
    from matplotlib.patches import Patch
    fig.legend([Patch(color=c) for _, c in stages],
               ["completato", "perso nel wielding", "perso nel sollevamento", "nessuna presa"],
               fontsize=8, frameon=False, loc="lower center", ncol=4)
    fig.text(.01, .01, "F = tripod fisso, P = floating plate nominale", fontsize=7, color="#52514e")
    fig.tight_layout(rect=(0, .08, 1, 1))
    fig.savefig(os.path.join(FIG, "09_outcomes.png"))
    plt.close(fig)


if __name__ == "__main__":
    fig_example(); fig_timeseries()
    for cmd, _ in VAR:
        fig_pooled_cones(cmd); fig_ecdf(cmd); fig_margin(cmd); fig_null(cmd)
    fig_command(); fig_wrench_margin(); fig_outcomes()
    print("figures ->", FIG)
