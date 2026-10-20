"""Inspect measured CSV samples and timing without re-solving any SOCP.

    python -m kinova_dual_arm_dmp_insertion.inspect_sampling LOG.csv

Writes separate diagnostics; does not alter the input log, plots or controller.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def plot_forces_and_positions(rows, carrying, t, output):
    """Plot only recorded force/position columns, preserving the input times."""
    import matplotlib.pyplot as plt
    time = t[carrying] - t[carrying][0]
    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    for col, side in enumerate("LR"):
        for row, component in enumerate("xyz"):
            F = np.array([float(r[f"F{component}{side}"]) for r in rows])[carrying]
            axes[row, col].plot(time, F, ".-", ms=3, lw=.8)
            axes[row, col].set_ylabel(f"F{component} [N]")
            axes[row, col].grid(alpha=.25)
        axes[0, col].set_title(f"Arm {side}: estimated hand-on-piece force, shelf axes")
        axes[-1, col].set_xlabel("Time since carrying starts [s]")
    fig.suptitle("Successful T run: recorded force components (moments absent from insertion CSV)")
    fig.tight_layout(rect=(0, 0, 1, .96))
    fig.savefig(output / "forces_xyz.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    position_error = {}
    for col, side in enumerate("LR"):
        commanded, measured = [], []
        for row, component in enumerate("xyz"):
            cmd = np.array([float(r[f"{component}{side}"]) for r in rows])[carrying]
            actual = np.array([float(r[f"m{component}{side}"]) for r in rows])[carrying]
            commanded.append(cmd)
            measured.append(actual)
            axes[row, col].plot(time, cmd, "--", lw=1., label="Commanded")
            axes[row, col].plot(time, actual, ".-", ms=2, lw=.7, label="Executed: FK from joint feedback")
            axes[row, col].set_ylabel(f"{component} [m, shelf]")
            axes[row, col].grid(alpha=.25)
        err = 1000 * np.linalg.norm(np.array(measured) - np.array(commanded), axis=0)
        position_error[side] = dict(median_mm=float(np.median(err)), p95_mm=float(np.percentile(err, 95)),
                                    max_mm=float(err.max()))
        axes[0, col].set_title(f"Arm {side}: tool/flange origin (not object pose)")
        axes[0, col].legend(fontsize=8)
        axes[-1, col].set_xlabel("Time since carrying starts [s]")
    fig.suptitle("Successful T run: commanded and executed position (original CSV sampling)")
    fig.tight_layout(rect=(0, 0, 1, .96))
    fig.savefig(output / "trajectory_xyz.png", dpi=150)
    plt.close(fig)
    return position_error


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("log", type=Path)
    ap.add_argument("--output", type=Path, default=Path("docs/analysis/success_T_sampling_20261020"))
    a = ap.parse_args()
    with a.log.open() as f:
        rows = list(csv.DictReader(f))
    a.output.mkdir(parents=True, exist_ok=True)
    t = np.array([float(r["t"]) for r in rows])
    phase = np.array([r["phase"] for r in rows])
    stats = []
    for ph in dict.fromkeys(phase):
        tt = t[phase == ph]
        dt = np.diff(tt)
        stats.append(dict(phase=ph, samples=len(tt), duration_s=float(tt[-1] - tt[0]),
                          median_dt_ms=float(np.median(dt) * 1000) if len(dt) else None,
                          p95_dt_ms=float(np.percentile(dt, 95) * 1000) if len(dt) else None,
                          max_dt_ms=float(dt.max() * 1000) if len(dt) else None,
                          effective_hz=float(len(dt) / (tt[-1] - tt[0])) if len(dt) else None))
    forces = {}
    for side in "LR":
        F = np.array([[float(r[f"F{k}{side}"]) for k in "xyz"] for r in rows])
        q = Rotation.from_quat(np.array([[float(r[f"q{k}{side}"]) for k in "xyzw"] for r in rows]))
        forces[side] = np.einsum("ij,ij->i", F, q.apply([1, 0, 0]))
    target = np.array([float(r["target_N"]) for r in rows])
    logged_normal = np.array([float(r["fn_N"]) for r in rows])
    derived_normal = np.maximum(forces["L"], 0) + np.maximum(forces["R"], 0)
    carrying = np.isin(phase, ["to_mouth", "route", "slow_seat", "seat_by_feel"])
    route = phase == "route"
    t0 = t[carrying][0]
    zoom = route & (t >= t[route][0] + 7) & (t <= t[route][0] + 10)
    report = dict(source=str(a.log.resolve()), source_sha256=hashlib.sha256(a.log.read_bytes()).hexdigest(),
                  total_rows=len(rows), phases=stats,
                  normal_sum_reconstruction_max_difference_N=float(np.max(np.abs(derived_normal - logged_normal))),
                  route_normal_sum_reconstruction_max_difference_N=float(np.max(np.abs(derived_normal[route] - logged_normal[route]))),
                  route_normal_sum_reconstruction_p95_difference_N=float(np.percentile(np.abs(derived_normal[route] - logged_normal[route]), 95)),
                  route_normal_target_correlation={s: float(np.corrcoef(forces[s][route], target[route])[0, 1]) for s in "LR"},
                  zoom_samples=int(zoom.sum()),
                  limitation="CSV times are log-write times; sensor/header times and per-stage durations are absent.")
    (a.output / "sampling.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    with (a.output / "sampling.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(stats[0]))
        w.writeheader()
        w.writerows(stats)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(12, 9))
    # Operator confirmations can leave multi-second gaps BETWEEN phases. Do not
    # let those pauses hide the actual within-phase loop intervals on this axis.
    within_phase = phase[1:] == phase[:-1]
    axes[0].plot(t[1:][within_phase] - t[0], 1000 * np.diff(t)[within_phase], ".", ms=2)
    axes[0].axhline(10, color="C3", ls="--", label="100 Hz configured = 10 ms")
    axes[0].axvspan(t[carrying][0] - t[0], t[carrying][-1] - t[0], color="C1", alpha=.12,
                   label="Carrying: allocation active")
    axes[0].set_ylabel("CSV interval [ms]")
    axes[0].set_xlabel("Time since first CSV row [s]")
    axes[0].set_title("Actual log intervals within phases (operator pauses between phases excluded)")
    for side, color in (("L", "C0"), ("R", "C3")):
        axes[1].plot(t[carrying] - t0, forces[side][carrying], ".-", color=color, ms=3, lw=.8,
                     label=f"{side}: force projected on commanded normal")
        axes[2].plot(t[zoom] - t0, forces[side][zoom], "o-", color=color, ms=4, lw=.8,
                     label=f"{side}: projection, actual CSV samples")
    for ax, mask in ((axes[1], carrying), (axes[2], zoom)):
        ax.plot(t[mask] - t0, logged_normal[mask] / 2, ".-", color=".5", lw=.8, ms=3,
                label="Logged total compression / 2 (hand average)")
        ax.step(t[mask] - t0, target[mask], where="post", color="k", ls=":", label="Logged squeeze target")
        ax.set_ylabel("Normal force [N/hand]")
        ax.set_xlabel("Time since carrying starts [s]")
    axes[1].set_title("CSV force estimates and logged target: no smoothing, no SOCP reconstruction")
    axes[2].set_title(f"Three-second route window: {zoom.sum()} samples (100 Hz would give about 300)")
    for ax in axes:
        ax.grid(alpha=.25)
    axes[0].legend(fontsize=8)
    axes[1].legend(fontsize=8, loc="upper center", ncol=2)
    fig.suptitle("Successful T run, 16 October 2026 12:11: sampling diagnosis")
    fig.tight_layout(rect=(0, 0, 1, .97))
    fig.savefig(a.output / "sampling.png", dpi=150)
    plt.close(fig)
    position_error = plot_forces_and_positions(rows, carrying, t, a.output)
    (a.output / "position_comparison.json").write_text(json.dumps(position_error, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
