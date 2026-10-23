import sys, numpy as np, pandas as pd, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from scipy.signal import butter, sosfiltfilt
import os
HERE = os.path.dirname(os.path.abspath(__file__))
L = os.path.join(HERE, "F75_Lt1_s9014_c100_five_cut800_")
OLD = os.path.join(HERE, "../20261022_F75_Lt1_s9014_twist_ati")
FFI = sys.argv[2]
REFS = [(os.path.join(HERE, "../20261023_1827_F75_Lt1_s9014_ffi_riuscito/F75_Lt1_s9014_c100_five_cut800_ffi_20261023_182743"), "23/10 18:27 FFI (entrata)"),
        (OLD + "/1_1500_riuscita/F75_Lt1_s9014_c100_five_cut800_20261022_150027", "22/10 15:00 main --twist gain 1 (entrata)"),
        (OLD + "/2_1507_riuscita/F75_Lt1_s9014_c100_five_cut800_20261022_150751", "22/10 15:07 main --twist gain 1 (entrata)"),
        (OLD + "/3_1512_riuscita/F75_Lt1_s9014_c100_five_cut800_20261022_151230", "22/10 15:12 main --twist gain 1 (entrata)")]


def osc(t, x):
    dt = np.median(np.diff(t)); u = np.arange(t[0], t[-1], dt); y = np.interp(u, t, x)
    y = y - np.polyval(np.polyfit(u - u[0], y, 3), u - u[0])
    yb = sosfiltfilt(butter(2, [.8, 2.5], btype="band", fs=1 / dt, output="sos"), y); n = int(1.5 / dt)
    w = np.hanning(len(y)); A = 2 * np.abs(np.fft.rfft(y * w)) / w.sum()
    return u[n:-n], yb[n:-n], np.fft.rfftfreq(len(y), dt), A


d = pd.read_csv(L + FFI + ".csv")
t0 = d.t_wall[d.phase == "to_mouth"].iloc[0]; d["tt"] = d.t_wall - t0
fig, ax = plt.subplots(5, 1, figsize=(13, 19))
c = d.phase.isin(["squeeze", "to_mouth", "route", "slow_seat"])
ax[0].plot(d.tt[c], d.FyL[c], "C0", lw=.8, label="sinistra FyL")
ax[0].plot(d.tt[c], -d.FyR[c], "C2", lw=.8, label="destra -FyR")
ax[0].plot(d.tt[c], d.target_N[c], "k--", lw=1, label="chiesta per mano")
ax[0].set_title(f"FFI {FFI}: forza normale per mano (ATI)"); ax[0].set_ylabel("N")
ax[1].plot(d.tt[c], d.ffi_error_N[c], "C3", label="errore somma (2 target - misurata)"); ax[1].set_ylabel("N"); ax[1].set_title("FFI: errore")
ax[2].plot(d.tt[c], d.ffi_ff_mm[c], label="nominale ff"); ax[2].plot(d.tt[c], d.ffi_i_mm[c], label="trim I")
ax[2].plot(d.tt[c], d.ffi_output_mm[c], "k", lw=1, label="totale")
for col, mk, lab in (("ffi_i_limited", "r|", "I al limite"), ("ffi_rate_limited", "m|", "rate limited")):
    m = c & (d[col] == 1); ax[2].plot(d.tt[m], np.zeros(m.sum()) + (0.2 if "i_" in col else -0.2), mk, ms=6, label=lab)
ax[2].set_ylabel("mm per mano"); ax[2].set_title("FFI: termini (negativo = apre)")
for lab_, tt in d[c].groupby("phase", sort=False).tt:
    for a in ax[:3]: a.axvline(tt.iloc[0], color="k", alpha=.2)
    ax[0].text(tt.iloc[0], ax[0].get_ylim()[0], lab_, fontsize=7, rotation=90, va="bottom")
runs = [(FFI, "FFI (questa)", "k")] + [(r, l, f"C{i}") for i, (r, l) in enumerate(REFS)]
for r, lab, col in runs:
    x = pd.read_csv((r if os.path.isabs(r) else L + r) + ".csv"); q = x[x.phase == "route"]
    u, yb, f, A = osc(q.t_wall.values, q.fn_N.values)
    ax[3].plot(u - u[0], yb, color=col, lw=1.4 if col == "k" else .8, label=f"{lab}: RMS {yb.std():.2f} N")
    ax[4].plot(f, A, color=col, lw=1.4 if col == "k" else .8, label=lab)
    k = (f > .8) & (f < 2.5)
    print(os.path.basename(r), "RMS %.2f N picco %.2f N @%.2f Hz" % (yb.std(), A[k].max(), f[k][A[k].argmax()]))
ax[3].set_title("Stretta totale in route, banda 0.8-2.5 Hz"); ax[3].set_ylabel("N"); ax[3].set_xlabel("s in route")
ax[4].set_title("Spettro della stretta totale in route"); ax[4].set_xlim(0, 6); ax[4].set_xlabel("Hz")
for a in ax[:3]: a.set_xlabel("s da inizio to_mouth")
for a in ax: a.grid(alpha=.3); a.legend(fontsize=8)
fig.tight_layout(); fig.savefig(sys.argv[1], dpi=80)
r = d[d.phase == "route"]
print(" | ".join("%.1f / %.1f" % (2 * p.target_N.median(), p.fn_N.median()) for p in np.array_split(r, 6)))
print("i_limited share %.2f rate_limited %.2f" % (r.ffi_i_limited.mean(), r.ffi_rate_limited.mean()))
