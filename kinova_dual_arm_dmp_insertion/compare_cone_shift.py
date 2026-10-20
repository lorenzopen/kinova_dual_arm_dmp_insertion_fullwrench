"""Offline comparison of translated cones using the real insertion controller.

    python -m kinova_dual_arm_dmp_insertion.compare_cone_shift

DryRun has no friction dynamics, no slip, no neighbours and zero measured
moments. This measures allocation and scalar squeeze response, NOT slip
prevention. Never constructs a ROS backend or changes the supplied config.
"""
import argparse
import contextlib
import csv
import hashlib
import io
import json
import platform
import time
from pathlib import Path

import cvxpy
import numpy as np
import scipy

from . import controller
from .dryrun import DryRun
from .main import Log, load_config, socp_kwargs
from .route import Route

DEFAULT_WORLDS = ("F75_It1_s9001", "F75_Tt1_s9007", "F75_Zt1_s9012")


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def run_one(world, arm, cfg, shift, slow, output):
    cfg = dict(cfg, socp=dict(cfg.get("socp") or {}, cone_shift_N=shift))
    route = Route(world, arm, cfg.get("scale", 1.), cfg.get("piece_mass"))
    backend = DryRun(route, period=1. / cfg["rate_hz"])
    tag = f"{world}_shift{shift:g}"
    log = Log(str(output / f"{tag}_ticks.csv"))
    ins = controller.Insertion(route, backend, log=log, confirm=lambda _: None,
                               jam_scale=cfg.get("jam_scale", 1.), **socp_kwargs(cfg))
    ins.motion_speed /= slow
    ins.turn_rate /= slow
    ins.seat_speed /= slow
    allocations = []
    solve_original = ins.socp.solve

    def solve(**kwargs):
        start = time.perf_counter()
        res = solve_original(**kwargs)
        duration = 1000 * (time.perf_counter() - start)
        comp = np.array(res["fn_L"] + res["fn_R"])
        allocations.append(dict(t=backend.now(), phase=ins.phase, ok=res["ok"],
                                reason=res["reason"], fn_L_N=sum(res["fn_L"]),
                                fn_R_N=sum(res["fn_R"]), min_point_N=float(comp.min()),
                                eta_max=res["eta_max"], force_residual_N=res["residual_force"],
                                torque_residual_Nm=res["residual_torque"],
                                cone_violation_N=res.get("cone_violation", float("nan")),
                                min_cone_slack_N=res.get("min_cone_slack_N", float("nan")),
                                solve_ms=duration))
        return res

    ins.socp.solve = solve
    start = time.perf_counter()
    transcript = io.StringIO()
    try:
        with contextlib.redirect_stdout(transcript):
            result = ins.run()
    finally:
        log.close()
    elapsed = time.perf_counter() - start
    (output / f"{tag}_run.txt").write_text(transcript.getvalue(), encoding="utf-8")
    write_csv(output / f"{tag}_allocations.csv", allocations)
    with (output / f"{tag}_ticks.csv").open() as f:
        ticks = list(csv.DictReader(f))
    phases = ("to_mouth", "route", "slow_seat", "seat_by_feel")
    carrying = [t for t in ticks if t["phase"] in phases]
    valid = [r for r in allocations if r["ok"]]
    errors = [abs(float(t["fn_N"]) / 2 - float(t["target_N"])) for t in carrying]
    summary = dict(world=world, shift_N_per_point=shift, reserve_N_per_hand=3 * shift,
                   r_s=ins.socp.r_s, mu=ins.mu, mass_kg=route.mass,
                   full_wrench=ins.full_wrench, hand_wrench=ins.hand_wrench,
                   **result, wall_seconds=elapsed,
                   target_p95_N=float(np.percentile([float(t["target_N"]) for t in carrying], 95)),
                   target_max_N=max(float(t["target_N"]) for t in carrying),
                   measured_half_sum_median_N=float(np.median([float(t["fn_N"]) / 2 for t in carrying])),
                   tracking_half_sum_error_p95_N=float(np.percentile(errors, 95)),
                   abs_ds_max_mm=max(abs(float(t["ds_mm"])) for t in carrying),
                   min_point_N=min((r["min_point_N"] for r in valid), default=float("nan")),
                   eta_max=max((r["eta_max"] for r in valid), default=float("nan")),
                   force_residual_max_N=max((r["force_residual_N"] for r in valid), default=float("nan")),
                   torque_residual_max_Nm=max((r["torque_residual_Nm"] for r in valid), default=float("nan")),
                   cone_violation_max_N=max((r["cone_violation_N"] for r in valid), default=float("nan")),
                   solve_ms_median=float(np.median([r["solve_ms"] for r in allocations])))
    print(f"{world} shift={shift:g} N/point: target median {result['squeeze_median']:.3f} N/hand, "
          f"SOCP {result['socp_ok']}/{result['socp_ticks']}, seat={result['seat_found']}", flush=True)
    return summary, carrying


def plot_traces(output, worlds, shifts, traces):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(worlds), 2, figsize=(12, 3.3 * len(worlds)), squeeze=False)
    for row, world in enumerate(worlds):
        for shift in shifts:
            ticks = traces[world, shift]
            t = np.array([float(r["t"]) for r in ticks])
            t -= t[0]
            label = f"{shift:g} N/point ({3 * shift:g} N/hand reserve)"
            axes[row, 0].plot(t, [float(r["target_N"]) for r in ticks], label=label)
            axes[row, 1].plot(t, [float(r["ds_mm"]) for r in ticks], label=label)
        axes[row, 0].set_title(world + " - squeeze target per hand")
        axes[row, 1].set_title(world + " - scalar squeeze offset")
        for ax, unit in zip(axes[row], ("N", "mm")):
            ax.set_xlabel("Simulated carrying time [s]")
            ax.set_ylabel(unit)
            ax.grid(alpha=.25)
    fig.suptitle("Translated friction cones: dry run (no slip/contact dynamics)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(.5, .96), ncol=2, fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, .90))
    fig.savefig(output / "comparison.png", dpi=160)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config")
    ap.add_argument("--arm", default="c100_five_cut800")
    ap.add_argument("--worlds", nargs="+", default=list(DEFAULT_WORLDS))
    ap.add_argument("--shifts", nargs="+", type=float, default=[0., .25, .5, 1.])
    ap.add_argument("--slow", type=float, default=1.)
    ap.add_argument("--output", type=Path, default=Path("docs/experiments/cone_shift_20261020"))
    a = ap.parse_args()
    if a.slow <= 0 or any(not np.isfinite(s) or s < 0 for s in a.shifts):
        ap.error("slow must be > 0 and shifts must be finite and >= 0")
    cfg, path = load_config(a.config)
    a.output.mkdir(parents=True, exist_ok=True)
    summaries, traces = [], {}
    for world in a.worlds:
        for shift in a.shifts:
            result, trace = run_one(world, a.arm, cfg, shift, a.slow, a.output)
            summaries.append(result)
            traces[world, shift] = trace
    write_csv(a.output / "summary.csv", summaries)
    metadata = dict(python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__,
                    cvxpy=cvxpy.__version__, installed_solvers=cvxpy.installed_solvers(),
                    prior_sha256=hashlib.sha256(Path(controller.PRIOR).read_bytes()).hexdigest(),
                    config_path=str(Path(path).resolve()), config=cfg, arm=a.arm, slow=a.slow,
                    limitation="DryRun has no friction/slip dynamics, neighbours, or measured moments.",
                    runs=summaries)
    (a.output / "summary.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    plot_traces(a.output, a.worlds, a.shifts, traces)


if __name__ == "__main__":
    main()
