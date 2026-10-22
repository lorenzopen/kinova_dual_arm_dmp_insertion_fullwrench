"""Squeeze force, commanded hand separation and spectrum of the squeeze in the route phase.

    python plot_squeeze.py                          # the four Z runs of 22/10, -> squeeze_gain.png
    python plot_squeeze.py OUT.png "label|log.csv" ...
"""
import os
import sys, numpy as np, pandas as pd, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
COL = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT = [("D 5/1, gain 1 (15:28)", "../20261022_F75_Zt1_s9012_twist_ati/2_1528_riuscita/F75_Zt1_s9012_c100_five_cut800_20261022_152814.csv"),
           ("D 60/7, gain 1 (17:22)", "F75_Zt1_s9012_c100_five_cut800_20261022_172232.csv"),
           ("D 60/7, gain 0.5 (17:34)", "F75_Zt1_s9012_c100_five_cut800_20261022_173458.csv"),
           ("D 60/7, gain 0.25 (17:44)", "F75_Zt1_s9012_c100_five_cut800_20261022_174421.csv")]
if len(sys.argv) > 2:
    out, runs = sys.argv[1], [a.split("|") for a in sys.argv[2:]]
else:
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "squeeze_gain.png")
    runs = [(lab, os.path.join(HERE, f)) for lab, f in DEFAULT]
fig, ax = plt.subplots(3, 1, figsize=(11, 10.5)); fig.patch.set_facecolor("#fcfcfb")
for a in ax:
    a.set_facecolor("#fcfcfb"); [a.spines[s].set_visible(False) for s in ("top", "right")]; a.grid(color="#e6e5e1", lw=0.6)
T = 17.5
for (lab, f), c in zip(runs, COL):
    d = pd.read_csv(f); g = d[d.phase == "route"]; t = g.t_wall.values - g.t_wall.values[0]; m = t <= T
    ax[0].plot(t[m], g.fn_N.values[m], color=c, lw=1.5, label=lab)
    s = (g.yR - g.yL).values[m] * 1000; ax[1].plot(t[m], s - np.median(s), color=c, lw=1.5)
    x = g.fn_N.values[m]; tt = t[m]; x = x - np.polyval(np.polyfit(tt, x, 3), tt)
    fs = 1 / np.median(np.diff(g.t_wall.values)); w = np.hanning(len(x))
    X = np.abs(np.fft.rfft(x * w)) * 2 / w.sum(); fr = np.fft.rfftfreq(len(x), 1 / fs)
    ax[2].plot(fr, X, color=c, lw=1.5)
    k = (fr >= 0.8) & (fr <= 2.5)
    print(f"{lab}: picco {fr[k][np.argmax(X[k])]:.2f} Hz ampiezza {X[k].max():.2f} N; rms stretta {x.std():.2f} N; "
          f"rms separazione {s.std():.2f} mm; stretta mediana {np.median(g.fn_N.values[m]):.1f} N (chiesta {2*np.median(g.target_N.values[m]):.1f}); "
          f"min {g.fn_N.values[m].min():.1f} N; route {t[-1]:.1f} s")
ax[0].set_title("Stretta misurata (somma delle due mani) durante la route", loc="left", fontsize=12)
ax[0].set_ylabel("forza [N]", color="#52514e"); ax[0].set_xlabel("tempo dall'inizio della route [s]", color="#52514e")
fig.legend(*ax[0].get_legend_handles_labels(), loc="upper center", ncol=len(runs), frameon=False, fontsize=10)
ax[1].set_title("Separazione tra le mani comandata dal loop di stretta (senza la media)", loc="left", fontsize=12)
ax[1].set_ylabel("[mm]", color="#52514e"); ax[1].set_xlabel("tempo dall'inizio della route [s]", color="#52514e")
ax[2].set_title("Spettro della stretta: il picco a 1,2-1,5 Hz è la sinusoide", loc="left", fontsize=12)
ax[2].set_xlim(0, 8); ax[2].set_ylabel("ampiezza [N]", color="#52514e"); ax[2].set_xlabel("frequenza [Hz]", color="#52514e")
fig.tight_layout(rect=(0, 0, 1, 0.96)); fig.savefig(out, dpi=110)
