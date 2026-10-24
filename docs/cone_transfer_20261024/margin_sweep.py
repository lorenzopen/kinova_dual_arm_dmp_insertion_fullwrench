"""
Margin sweep at fixed everything else (feed-forward, no_bending, mu = 1):
lambda_m = 0.25, 0.5, 0.75 in rho = lambda_m ||F_task|| + 1 N.

Reads the per-lambda analyses (run analyze.py / wrench_margin.py with CT_RUNS
and CT_OUT first) and writes margin_sweep.csv and figures/10_margin_sweep.png.
"""
import os

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                      # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SWEEP = ((0.25, "margin_m025"), (0.5, "margin_m050"), (0.75, "margin_m075"))
C = {"rigid": "#2a78d6", "plate": "#eb6834"}
LBL = {"rigid": "tripod fisso", "plate": "floating plate nominale"}


def main():
    rows = []
    for lam, out in SWEEP:
        d = pd.read_csv(os.path.join(HERE, out, "metrics.csv"))
        w = pd.read_csv(os.path.join(HERE, out, "wrench_margin.csv"))
        for arm in ("rigid", "plate"):
            x = d[d.arm == arm]
            y = x[x.n_samples > 0]
            ww = w[w.arm == arm]
            rows.append(dict(
                lambda_m=lam, arm=arm, done=int((x.stage == "DONE").sum()), runs=len(x),
                rho_N=y.rho_med.median(), N_pad_N=y.N_med.median(),
                df_N=y.df_med.median(), df_over_rho=y.df_over_rho_med.median(),
                sat=y.sat_frac.median(), margin_lost=y.margin_lost_frac.median(),
                slip=y.slip_frac.median(), dWF_N=y.dWF_med.median(), dWM_Nm=y.dWM_med.median(),
                wrench_kept=float((ww.t_real >= ww.rho - 1e-3).mean()),
                wrench_feasible=float((ww.t_real > 0).mean()),
                t_over_rho=float((ww.t_real / ww.rho).median())))
    t = pd.DataFrame(rows)
    t.to_csv(os.path.join(HERE, "margin_sweep.csv"), index=False)

    plt.rcParams.update({"font.size": 9, "axes.spines.top": False,
                         "axes.spines.right": False, "axes.grid": True,
                         "grid.color": "#e4e3df", "grid.linewidth": 0.6})
    panels = (("N_pad_N", "squeeze: N mediano per pad [N]"),
              ("df_N", "errore per pad |Δf_i| [N]"),
              ("sat", "pad-campioni saturi (η ≥ 0.99)"),
              ("slip", "pad-campioni in slittamento"))
    fig, axes = plt.subplots(1, 4, figsize=(13, 3.2))
    for ax, (col, title) in zip(axes, panels):
        for arm in ("rigid", "plate"):
            s = t[t.arm == arm]
            ax.plot(s.lambda_m, s[col], color=C[arm], lw=2, marker="o", ms=8,
                    markeredgecolor="white", label=LBL[arm])
        ax.set_title(title, fontsize=9)
        ax.set_xticks([0.25, 0.5, 0.75])
        ax.set_xlabel("λ_m")
        ax.set_ylim(bottom=0)
    axes[0].legend(fontsize=7, frameon=False)
    fig.tight_layout()
    os.makedirs(os.path.join(HERE, "figures"), exist_ok=True)
    fig.savefig(os.path.join(HERE, "figures", "10_margin_sweep.png"), dpi=160)
    print(t.round(3).to_string(index=False))


if __name__ == "__main__":
    main()
