"""Forze della Z offline del 23/10 12:32 contro l'anello chiuso --online gain 0.5 (12:18, stesso giorno).

    python3 plot_offline.py [percorso_csv_online]

Oscillazione = stretta totale nella route, detrend cubico, passa-banda Butterworth 0.8-2.5 Hz
(filtfilt, 1.5 s tolti ai bordi): RMS; picco dello spettro con finestra di Hann.
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import butter, sosfiltfilt

HERE = os.path.dirname(os.path.abspath(__file__))
OFF = os.path.join(HERE, "F75_Zt1_s9012_c100_five_cut800_offline_20261023_123222.csv")
ON = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    HERE, "../../../logs/F75_Zt1_s9012_c100_five_cut800_online_20261023_121841.csv")


def osc(t, x):
    dt = np.median(np.diff(t)); u = np.arange(t[0], t[-1], dt); y = np.interp(u, t, x)
    y = y - np.polyval(np.polyfit(u - u[0], y, 3), u - u[0])
    yb = sosfiltfilt(butter(2, [.8, 2.5], btype="band", fs=1 / dt, output="sos"), y)
    n = int(1.5 / dt)
    w = np.hanning(len(y)); A = 2 * np.abs(np.fft.rfft(y * w)) / w.sum(); f = np.fft.rfftfreq(len(y), dt)
    return u, yb, n, f, A


runs = [(OFF, "offline 12:32 (feedforward, ATI non in feedback)", "C0")]
if os.path.exists(ON):
    runs.append((ON, "online gain 0.5, 12:18 (cart_d 80)", "C1"))
fig, ax = plt.subplots(3, 1, figsize=(13, 12))
for path, lab, c in runs:
    d = pd.read_csv(path)
    t0 = d.t_wall[d.phase == "to_mouth"].iloc[0]
    m = d.phase.isin(["to_mouth", "route", "slow_seat"])
    tt = d.t_wall[m] - t0
    if path == OFF:
        ax[0].plot(tt, d.FyL[m], "C0", lw=.8, label="offline: sinistra FyL")
        ax[0].plot(tt, -d.FyR[m], "C2", lw=.8, label="offline: destra -FyR")
        ax[0].plot(tt, d.target_N[m], "k--", lw=1, label="chiesta per mano")
    else:
        ax[0].plot(tt, d.fn_N[m] / 2, "C1", lw=.6, alpha=.6, label="online: media delle due mani")
    r = d.phase == "route"
    u, yb, n, f, A = osc(d.t_wall[r].values, d.fn_N[r].values)
    ax[1].plot(u[n:-n] - t0, yb[n:-n], c, lw=1, label=f"{lab}: RMS {yb[n:-n].std():.2f} N")
    ax[2].plot(f, A, c, label=lab)
ax[0].set_title("Forza normale per mano nel trasporto (ATI)"); ax[0].set_ylabel("N"); ax[0].set_xlabel("s da inizio to_mouth")
ax[1].set_title("Stretta totale in route, solo 0.8-2.5 Hz"); ax[1].set_ylabel("N"); ax[1].set_xlabel("s da inizio to_mouth")
ax[2].set_title("Spettro della stretta totale in route (detrend cubico)"); ax[2].set_xlim(0, 6); ax[2].set_xlabel("Hz"); ax[2].set_ylabel("N")
for a in ax:
    a.grid(alpha=.3); a.legend(fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(HERE, "forze_offline_vs_online.png"), dpi=85)
