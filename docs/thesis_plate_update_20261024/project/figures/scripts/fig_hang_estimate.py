"""What the wrench of a still hang recovers, move by move (DECLARED regime).

    python fig_hang_estimate.py [--on DIR]   -> ch10_tetris/hang_estimate.pdf

One game (seed 7025, default). For grasp #2 of every move, the three errors of
CLAUDE.md §4.9, from game.csv (truth used for REPORTING only):

  aim    where the pads really landed minus where they aimed: the vision error
         that materialised, i.e. the work the inversion has to do
  obs    the estimate of the grasp lever minus the truth, ACROSS gravity: the
         estimator's own error (tau = r x m g observes these two directions)
  blind  the same, ALONG gravity: filled from the plan, never observed (rank 2)

A curve in time would be flat: the simulated wrench is exact, so the least
squares is right from its first still samples. What matters is the split.
"""
import argparse
import csv

import numpy as np

from style import FIG, GREY, plt, save

ap = argparse.ArgumentParser()
ap.add_argument("--on", default=str(FIG / "data" / "s7025_on"))
ap.add_argument("--out", default="ch10_tetris/hang_estimate")
a = ap.parse_args()

rows = [r for r in csv.DictReader(open(f"{a.on}/game.csv", encoding="utf-8")) if r.get("est_obs")]
k = np.arange(len(rows))
aim = np.array([float(r["aim_err"]) for r in rows])
obs = np.array([float(r["est_obs"]) for r in rows])
blind = np.array([float(r["est_blind"]) for r in rows])

fig, ax = plt.subplots(figsize=(5.2, 2.6), layout="constrained")
w = 0.26
ax.bar(k - w, aim, w, color="0.8", edgecolor="k", lw=0.4, label="vision (aim)")
ax.bar(k, obs, w, color="k", label="estimator, observed")
ax.bar(k + w, blind, w, color=GREY, hatch="///", edgecolor="w", lw=0, label="plan, blind")
ax.set_xticks(k, [f"{r['k']} {r['piece']}" for r in rows])
ax.set_xlabel("move")
ax.set_ylabel("error [mm]")
ax.legend(loc="upper left", ncol=3, bbox_to_anchor=(0, 1.15))
save(fig, a.out)
print("  aim", aim.round(1), "\n  obs", obs.round(2), "\n  blind", blind.round(1))
