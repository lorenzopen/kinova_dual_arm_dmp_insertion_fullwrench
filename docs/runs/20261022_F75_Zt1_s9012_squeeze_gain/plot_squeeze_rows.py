"""One row per run: the squeeze measured by the ATIs against what the SOCP asked, and its spectrum.

    python plot_squeeze_rows.py      -> squeeze_gain_rows.png (+ the numbers on stdout)

Same four Z runs as plot_squeeze.py (cart_d 5/1 vs 60/7, --squeeze-gain 1 / 0.5 / 0.25), shown
separately so each can be read. Left: sum of the two hands' normal forces (ATI) and twice the SOCP
target per hand, from the mouth of the slot to the end of the seat; the error is measured - asked.
Right: amplitude spectrum of the measured squeeze in the route, trend removed, Hann window.
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402

HERE = Path(__file__).resolve().parent
RUNS = [
    ("15:28  D 5/1, gain 1", HERE.parent / "20261022_F75_Zt1_s9012_twist_ati" / "2_1528_riuscita"
     / "F75_Zt1_s9012_c100_five_cut800_20261022_152814.csv", "entered"),
    ("17:22  D 60/7, gain 1", HERE / "F75_Zt1_s9012_c100_five_cut800_20261022_172232.csv", "stopped (Ctrl-C)"),
    ("17:34  D 60/7, gain 0.5", HERE / "F75_Zt1_s9012_c100_five_cut800_20261022_173458.csv", "entered"),
    ("17:44  D 60/7, gain 0.25", HERE / "F75_Zt1_s9012_c100_five_cut800_20261022_174421.csv", "stopped (Ctrl-C)"),
]
CARRY = ("to_mouth", "route", "slow_seat", "seat_by_feel")
COL = {"to_mouth": "#e8f0f8", "route": "#f8f0e0", "slow_seat": "#e8f4e8", "seat_by_feel": "#d8ecd8"}


def load(csv):
    d = pd.read_csv(csv)
    fn = 0.0
    for s in "LR":
        x = Rot.from_quat(d[[f"mqx{s}", f"mqy{s}", f"mqz{s}", f"mqw{s}"]].values).apply([1.0, 0.0, 0.0])
        fn = fn + np.einsum("ij,ij->i", d[[f"Fx{s}", f"Fy{s}", f"Fz{s}"]].values, x)
    d = d.assign(sq=fn, asked=2.0 * d.target_N)
    d = d[d.phase.isin(CARRY)].copy()
    dt = np.diff(d.t.values, prepend=d.t.values[0])
    d["t"] = np.cumsum(np.clip(dt, 0.0, 0.05))                # waits cut out
    return d


def spectrum(d):
    r = d[d.phase == "route"]
    t = r.t.values
    tu = np.arange(t[0], t[-1], 0.01)
    y = np.interp(tu, t, r.sq.values)
    y = y - np.polyval(np.polyfit(tu, y, 2), tu)
    w = np.hanning(len(y))
    A = 2 * np.abs(np.fft.rfft(y * w)) / w.sum()
    f = np.fft.rfftfreq(len(y), 0.01)
    return f, A


plt.rcParams.update({"font.size": 8, "axes.spines.top": False, "axes.spines.right": False})
fig, axs = plt.subplots(len(RUNS), 2, figsize=(10, 2.0 * len(RUNS)), width_ratios=(3, 1),
                        sharex="col", sharey="col", layout="constrained")
for (label, csv, outcome), (a, b) in zip(RUNS, axs):
    d = load(csv)
    ph, t = d.phase.values, d.t.values
    i0 = 0
    for i in range(1, len(ph) + 1):
        if i == len(ph) or ph[i] != ph[i0]:
            a.axvspan(t[i0], t[i - 1], color=COL[ph[i0]], lw=0)
            i0 = i
    a.plot(t, d.sq, color="#1f6fb2", lw=0.9, label="measured (ATI, both hands)")
    a.plot(t, d.asked, color="k", lw=1.0, ls="--", label="asked (2 x SOCP target)")
    e = (d.sq - d.asked)[d.phase == "route"]
    f, A = spectrum(d)
    band = (f > 0.6) & (f < 3.0)
    k = np.argmax(A * band)
    a.set_title(f"{label}  -  {outcome}:  error in the route rms {np.sqrt(np.mean(e ** 2)):.2f} N, "
                f"min squeeze {d.sq[d.phase == 'route'].min():.1f} N", loc="left", fontsize=8)
    a.set_ylabel("squeeze [N]")
    b.plot(f, A, color="#1f6fb2", lw=0.9)
    b.plot(f[k], A[k], "o", color="#d9622b", ms=4)
    b.text(f[k] + 0.15, A[k], f"{f[k]:.2f} Hz, {A[k]:.2f} N", fontsize=7, va="center")
    b.set_xlim(0, 4)
    b.set_ylabel("amplitude [N]")
    print(f"  {label:26s} {outcome:17s} rms {np.sqrt(np.mean(e ** 2)):.2f} N  peak {f[k]:.2f} Hz {A[k]:.2f} N  "
          f"min {d.sq[d.phase == 'route'].min():.1f} N  mean meas {d.sq[d.phase == 'route'].mean():.1f} "
          f"asked {d.asked[d.phase == 'route'].mean():.1f}")
axs[0, 0].legend(loc="lower left", ncol=2, fontsize=7, frameon=False)
axs[-1, 0].set_xlabel("time from the mouth of the slot [s]  (to mouth blue, route sand, seat green)")
axs[-1, 1].set_xlabel("frequency [Hz] (route)")
fig.savefig(HERE / "squeeze_gain_rows.png", dpi=150)
print("  ->", HERE / "squeeze_gain_rows.png")
