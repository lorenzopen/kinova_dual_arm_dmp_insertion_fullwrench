"""Chapter 11: the two Kinova Gen3 arms with wrist ATIs, the cell and the runs of 21-22/10/2026.

    python fig_kinova.py            -> ch11_kinova/*.pdf, *.png, runs_table.tex, kinova_runs_summary.json

Data: the robot package's logs, OUTSIDE this repo (like the GIFs of chapter 10):
    C:/Users/Lorenzo/Desktop/kinova_dual_arm_dmp_insertion_fullwrench   (env KINOVA_REPO overrides)
    docs/runs/<date>_<world>_twist_ati/<n>_<time>_<outcome>/<world>_<set>_<stamp>.csv (+ _meta.json)
One CSV row per 10 ms control tick: commanded and measured flanges (shelf frame), the hand
wrenches the controller used (from the ATIs), the raw ATI and joint-torque readings, the
SOCP target and its solve time. The routes and the cell geometry come from the same package.
The outcomes ("entered", "inserted") are Lorenzo's, written in each run's README.
"""
import json
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from matplotlib.patches import Polygon, Rectangle
from scipy.spatial.transform import Rotation as Rot

from style import FIG, GREY, LEFT, PIECE, RIGHT, plt, save

KIN = Path(os.environ.get("KINOVA_REPO", r"C:/Users/Lorenzo/Desktop/kinova_dual_arm_dmp_insertion_fullwrench"))
sys.path.insert(0, str(KIN))
from kinova_dual_arm_dmp_insertion.main import load_config   # noqa: E402
from kinova_dual_arm_dmp_insertion.route import Route         # noqa: E402

OUT = "ch11_kinova"
CFG, _ = load_config(str(KIN / "config" / "cell.yaml"))
G = 9.81
RUNS = KIN / "docs" / "runs"
# (folder, world, route set, piece, outcome as written in the run README)
SERIES = [
    ("20261021_F75_Tt1_s9007_twist_ati", "F75_Tt1_s9007", "c100_five_cut800", "T",
     ["entered; L not pushed", "inserted; neighbours pushed", "inserted"]),
    ("20261022_S_s7073m3_twist_ati", "S_s7073m3", "c100_narrow75", "S", ["entered", "entered"]),
    ("20261022_F75_Lt1_s9014_twist_ati", "F75_Lt1_s9014", "c100_five_cut800", "L",
     ["inserted", "inserted", "inserted"]),
    ("20261022_F75_Zt1_s9012_twist_ati", "F75_Zt1_s9012", "c100_five_cut800", "Z", ["inserted", "inserted"]),
]
CARRY = ("to_mouth", "route", "slow_seat", "seat_by_feel")


def runs():
    for folder, world, rset, piece, outcomes in SERIES:
        dirs = sorted(p for p in (RUNS / folder).iterdir() if p.is_dir())
        for d, outcome in zip(dirs, outcomes):
            csv = next(d.glob(f"{world}_*.csv"))
            meta = json.loads(csv.with_name(csv.stem + "_meta.json").read_text())
            yield dict(world=world, set=rset, piece=piece, outcome=outcome, csv=csv, meta=meta,
                       time=d.name.split("_")[1], date=folder[6:8] + "/" + folder[4:6])


def route(world, rset):
    return Route(world, rset, CFG.get("scale", 1.0), CFG.get("piece_mass"))


# ---------------------------------------------------------------- piece shapes
def shapes():
    """Cells (piece frame, about the CoM) of every letter, from the route files' own piece models."""
    out = {}
    for d in (KIN / "kinova_dual_arm_dmp_insertion" / "routes").iterdir():
        if not d.name.startswith("c100"):
            continue
        for f in d.glob("*.json"):
            try:
                r = route(f.stem, d.name)
            except Exception:
                continue
            out.setdefault(r.piece, np.asarray(r.cells, float))
    return out


SHAPES = None


def outline(ax, cells, R, c, **kw):
    """Draw each cube of a piece in the shelf x-z plane."""
    cells = np.asarray(cells, float)
    edge = min(np.linalg.norm(a - b) for i, a in enumerate(cells) for b in cells[i + 1:])
    h = 0.5 * edge
    for q in cells:
        sq = np.array([q + [sx * h, sy * h, 0.0] for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
        w = np.asarray(c) + R.apply(sq)
        ax.add_patch(Polygon(1000 * w[:, [0, 2]], closed=True, **kw))


def board(ax, r):
    """Shelf walls and floor, the neighbours as built, the slot, the piece at the start."""
    lo, hi = (1000 * np.asarray(r.shelf["inner_faces"], float) * r.eff_scale)
    ax.add_patch(Rectangle((lo - 20, -15), hi - lo + 40, 15, color="0.35", lw=0))
    for x in (lo - 20, hi):
        ax.add_patch(Rectangle((x, 0), 20, 820, color="0.75", lw=0))
    for n in r.neighbours:
        outline(ax, SHAPES[n["piece"]], Rot.from_quat(n["quat"]), n["position"],
                facecolor=PIECE.get(n["piece"], "0.7"), alpha=0.55, edgecolor="0.2", lw=0.6)
    outline(ax, r.cells, r.R_target, r.target, facecolor="none", edgecolor=PIECE[r.piece], lw=1.2, ls="--")
    outline(ax, r.cells, r.grasp_R, r.grasp_c, facecolor=PIECE[r.piece], alpha=0.30,
            edgecolor=PIECE[r.piece], lw=0.8)


def grasp_path(r):
    """The planned grasp centre (the midpoint of the pads) along the learnt route, shelf frame."""
    off = r.grasp_R.inv().apply(r.centre - r.grasp_c)
    pts = [r.grasp_c + r.grasp_R.apply(off)]
    pts += [c + R.apply(off) for c, R in zip(r.centres, r.rots)]
    pts.append(r.target + r.R_target.apply(off))
    return 1000 * np.array(pts)


def load(csv):
    d = pd.read_csv(csv)
    d["t"] -= d.t.iloc[0]
    for s in "LR":
        x = Rot.from_quat(d[[f"mqx{s}", f"mqy{s}", f"mqz{s}", f"mqw{s}"]].values).apply([1.0, 0.0, 0.0])
        F = d[[f"Fx{s}", f"Fy{s}", f"Fz{s}"]].values
        T = d[[f"Tx{s}", f"Ty{s}", f"Tz{s}"]].values
        d[f"fn{s}"] = np.einsum("ij,ij->i", F, x)                  # hand on piece, along its push axis
        d[f"tn{s}"] = np.einsum("ij,ij->i", T, x)                  # moment about the pad axis
        d[f"rot{s}"] = np.degrees(np.linalg.norm(d[[f"off{s}_rx", f"off{s}_ry", f"off{s}_rz"]].values, axis=1))
        # the joint-torque estimate: external wrench ON the arm, ee axes (ee z = push axis)
        d[f"fn_est{s}"] = -d[f"est{s}_fz"]
    return d.copy()


def active(d):
    """Contact to release, the operator's waits cut out of the time axis (no ticks while waiting)."""
    d = d[d.phase.isin(("approach", "squeeze") + CARRY + ("release",))].copy()
    dt = np.diff(d.t.values, prepend=d.t.values[0])
    d["t"] = np.cumsum(np.clip(dt, 0.0, 0.05))
    return d


# ---------------------------------------------------------------- figure 1: the cell
def fig_cell():
    sep = 0.60
    fig, ax = plt.subplots(figsize=(6.4, 4.4))
    rs = CFG["right"]["shelf_in_base"]["xyz"]
    x0 = 1000 * rs[0]                                   # column 0 (shelf origin), right base frame
    yc = 1000 * rs[1]
    r = route("F75_Tt1_s9007", "c100_five_cut800")
    lo, hi = 1000 * np.asarray(r.shelf["inner_faces"], float) * r.eff_scale
    # shelf: x_shelf runs towards the bases (frame turned 180 deg about z)
    ax.add_patch(Rectangle((x0 - hi - 20, yc - 62.5), hi - lo + 40, 125, facecolor="0.88", edgecolor="0.3", lw=0.8))
    for k in range(7):
        xs = x0 - (lo + k * 102.5)
        ax.plot([xs, xs], [yc - 62.5, yc + 62.5], color="0.6", lw=0.5)
    for k in range(6):
        ax.text(x0 - (lo + (k + 0.5) * 102.5), yc, str(k), ha="center", va="center", fontsize=7, color="0.35")
    for (name, y, col) in (("right arm", 0.0, RIGHT), ("left arm", 1000 * sep, LEFT)):
        ax.add_patch(Rectangle((-60, y - 60), 120, 120, facecolor=col, alpha=0.25, edgecolor=col))
        ax.text(80, y, name, ha="left", va="center", color=col)
    # the tools at the grasp of the T: commanded flange poses at the route start (right base frame)
    a = r.axis / np.linalg.norm(r.axis)
    for side, sign, col, yb in (("right", -1, RIGHT, 0.0), ("left", +1, LEFT, 1000 * sep)):
        sh = CFG[side]["shelf_in_base"]
        R_sb = Rot.from_quat(sh["quat"])
        reach = 0.5 * r.opening + CFG["tool"]["pad_x"] + CFG["tool"]["pad_r"]
        p_shelf = r.centre + (-sign) * reach * a                 # left pushes +axis, from -axis side
        pb = 1000 * (np.asarray(sh["xyz"]) + R_sb.apply(p_shelf))
        tip = 1000 * (np.asarray(sh["xyz"]) + R_sb.apply(r.centre + (-sign) * 0.5 * r.opening * a))
        ax.plot([pb[0], tip[0]], [pb[1] + yb, tip[1] + yb], color=col, lw=3, solid_capstyle="butt")
        ax.plot(pb[0], pb[1] + yb, "s", color=col, ms=6)
    ax.text(x0 - (hi + lo) / 2, yc + 80, "shelf (columns 0-5)", ha="center", fontsize=7, color=GREY)
    ax.set_xlim(-80, 900)
    ax.set_ylim(-150, 750)
    ax.set_aspect("equal")
    ax.set_xlabel("x in the right base [mm]")
    ax.set_ylabel("y in the right base [mm]")
    save(fig, f"{OUT}/cell_top_view")


# ---------------------------------------------------------------- figure 2: planned vs executed
def fig_routes(all_runs):
    fig, axes = plt.subplots(1, 4, figsize=(7.4, 3.9), layout="constrained", sharey=True)
    stats = {}
    for ax, (folder, world, rset, piece, _) in zip(axes, SERIES):
        r = route(world, rset)
        board(ax, r)
        gp = grasp_path(r)
        ax.plot(gp[:, 0], gp[:, 2], color="k", lw=1.0, ls="--", label="planned (route)")
        errs = []
        for k, run in enumerate(x for x in all_runs if x["world"] == world):
            d = load(run["csv"])
            c = d[d.phase.isin(CARRY)]
            cmd = 500 * (c[["xL", "yL", "zL"]].values + c[["xR", "yR", "zR"]].values)
            meas = 500 * (c[["mxL", "myL", "mzL"]].values + c[["mxR", "myR", "mzR"]].values)
            ax.plot(meas[:, 0], meas[:, 2], color=PIECE[piece], lw=0.9, alpha=0.9,
                    label="measured" if k == 0 else None)
            errs.append(np.linalg.norm(meas - cmd, axis=1))
        e = np.concatenate(errs)
        stats[world] = dict(track_median_mm=float(np.median(e)), track_max_mm=float(e.max()))
        ax.set_title(f"{piece}  {world}", fontsize=8)
        ax.set_xlim(-60, 680)
        ax.set_ylim(-20, 1000)
        ax.set_aspect("equal")
        ax.set_xlabel("shelf x [mm]")
    axes[0].set_ylabel("height above the shelf floor [mm]")
    axes[0].legend(loc="upper left", fontsize=7)
    save(fig, f"{OUT}/planned_vs_executed")
    return stats


# ---------------------------------------------------------------- figure 3: one run, the wrench
PHASE_COL = {"approach": "0.93", "squeeze": "0.88", "to_mouth": "#e8f0f8", "route": "#f8f0e0",
             "slow_seat": "#e8f4e8", "seat_by_feel": "#d8ecd8", "release": "0.93"}


def shade(ax, d):
    ph = d.phase.values
    t = d.t.values
    start = 0
    for i in range(1, len(ph) + 1):
        if i == len(ph) or ph[i] != ph[start]:
            if ph[start] in PHASE_COL:
                ax.axvspan(t[start], t[i - 1], color=PHASE_COL[ph[start]], lw=0, zorder=0)
            start = i


def fig_wrench(run):
    r = route(run["world"], run["set"])
    d = active(load(run["csv"]))
    fig, axs = plt.subplots(4, 1, figsize=(6.4, 6.6), sharex=True, layout="constrained")
    # (a) squeeze: per hand, measured by the ATI vs asked by the SOCP
    ax = axs[0]
    shade(ax, d)
    ax.plot(d.t, d.fnL, color=LEFT, lw=0.8, label="left, measured (ATI)")
    ax.plot(d.t, d.fnR, color=RIGHT, lw=0.8, label="right, measured (ATI)")
    ax.plot(d.t, d.target_N, color="k", lw=1.0, ls="--", label="SOCP target per hand")
    ax.set_ylabel("normal force [N]")
    ax.legend(ncol=3, loc="upper right", fontsize=7)
    # (b) what the environment does on the piece, against the jam limit
    ax = axs[1]
    shade(ax, d)
    ext = np.hypot(d.ext_x, d.ext_y)
    mu = float(r.c["mu_eff"])
    share = np.where(d.phase == "route", float(r.c["jam_share"]), 1.0)
    spare = 2.0 * mu * np.maximum(d.target_N.clip(lower=1.0), 1.0) - r.mass * G
    lim = np.maximum(float(CFG.get("jam_scale", 1.0)) * share * spare, float(CFG.get("jam_min", 2.0)))
    ax.plot(d.t, ext, color="0.2", lw=0.8, label="side force on the piece (x, y)")
    ax.plot(d.t, d.ext_z, color="0.55", lw=0.8, label="vertical force on the piece")
    ax.plot(d.t, lim, color="#b22", lw=0.9, ls=":", label="jam limit")
    ax.set_ylim(-1, 16)
    ax.set_ylabel("external force [N]")
    ax.legend(ncol=3, loc="upper left", fontsize=7)
    # (c) the twist channel: the moment each hand applies about the pad axis, and the correction
    ax = axs[2]
    shade(ax, d)
    ax.plot(d.t, d.tnL, color=LEFT, lw=0.8, label="left")
    ax.plot(d.t, d.tnR, color=RIGHT, lw=0.8, label="right")
    ax.set_ylabel("moment about\nthe pad axis [Nm]")
    ax.legend(ncol=2, loc="upper left", fontsize=7)
    ax = axs[3]
    shade(ax, d)
    ax.plot(d.t, d.rotL, color="0.2", lw=0.9)
    ax.set_ylabel("twist correction\n(each hand) [deg]")
    ax.set_xlabel("time from contact [s] (operator waits removed)")
    rt = d.t[d.phase == "route"]
    axs[0].text(rt.mean(), 0.3, "route", ha="center", fontsize=7, color="0.35")
    save(fig, f"{OUT}/wrench_one_run")


# ---------------------------------------------------------------- figure 4: ATI vs the estimate
def fig_ati_vs_estimate(run):
    d = active(load(run["csv"]))
    fig, axs = plt.subplots(2, 1, figsize=(6.4, 3.8), sharex=True, layout="constrained")
    for ax, s, col, name in ((axs[0], "L", LEFT, "left"), (axs[1], "R", RIGHT, "right")):
        shade(ax, d)
        ax.plot(d.t, d[f"fn{s}"], color=col, lw=0.9, label=f"{name}: ATI")
        ax.plot(d.t, d[f"fn_est{s}"], color="0.4", lw=0.8, ls="-", alpha=0.8,
                label=f"{name}: joint-torque estimate")
        ax.set_ylabel("normal force [N]")
        ax.legend(ncol=2, loc="upper right", fontsize=7)
    axs[1].set_xlabel("time from contact [s] (operator waits removed)")
    save(fig, f"{OUT}/ati_vs_estimate")


# ---------------------------------------------------------------- the table
def summary(all_runs, track):
    rows = []
    for run in all_runs:
        d = load(run["csv"])
        c = d[d.phase.isin(CARRY)]
        res = run["meta"].get("result") or {}
        tick = np.diff(c.t.values) * 1000
        tick = tick[tick < 500]
        s = c.socp_ms[c.socp_ms > 0]
        tare = run["meta"]["tares"][0]
        rows.append(dict(
            piece=run["piece"], world=run["world"], date=run["date"], time=run["time"], outcome=run["outcome"],
            route_s=float(d.t[d.phase == "route"].max() - d.t[d.phase == "route"].min()),
            socp_ok=f"{res.get('socp_ok', '')}/{res.get('socp_ticks', '')}",
            asked_N=float(np.median(c.target_N)),
            meas_N=float(np.median(0.5 * (c.fnL + c.fnR))), meas_max_N=float(max(c.fnL.max(), c.fnR.max())),
            ext_max_N=float(np.hypot(c.ext_x, c.ext_y).max()),
            twist_max_deg=float(max(c.rotL.max(), c.rotR.max())),
            socp_ms=float(np.median(s)) if len(s) else float("nan"),
            tick_p95_ms=float(np.percentile(tick, 95)),
            ati_sd_N=float(max(np.max(tare[a]["sd_F"]) for a in ("left", "right"))),
            seat_found=bool(res.get("seat_found", False)), jammed=bool(res.get("jammed", False))))
    (FIG / "data").mkdir(exist_ok=True)
    (FIG / "data" / "kinova_runs_summary.json").write_text(json.dumps(dict(runs=rows, tracking=track), indent=1))
    lines = [r"\begin{tabular}{@{}llrrrrrrrl@{}}", r"\toprule",
             r"Piece & Date, time & Route & SOCP ok & Asked & Measured & Side & Twist & Solve & Outcome \\",
             r" & & [s] & & [N] & [N] & max [N] & max [$^\circ$] & [ms] & \\", r"\midrule"]
    for x in rows:
        lines.append(f"{x['piece']} & {x['date']} {x['time'][:2]}:{x['time'][2:]} & {x['route_s']:.1f} & "
                     f"{x['socp_ok']} & {x['asked_N']:.1f} & {x['meas_N']:.1f} & {x['ext_max_N']:.1f} & "
                     f"{x['twist_max_deg']:.1f} & {x['socp_ms']:.0f} & {x['outcome']} \\\\")
    lines += [r"\bottomrule", r"\end{tabular}"]
    (FIG / OUT).mkdir(exist_ok=True)
    (FIG / OUT / "runs_table.tex").write_text("\n".join(lines) + "\n")
    print("  ->", f"{OUT}/runs_table.tex")
    return rows


if __name__ == "__main__":
    SHAPES = shapes()
    all_runs = list(runs())
    fig_cell()
    track = fig_routes(all_runs)
    one = next(x for x in all_runs if x["world"] == "F75_Lt1_s9014")
    fig_wrench(one)
    fig_ati_vs_estimate(one)
    rows = summary(all_runs, track)
    for x in rows:
        print(f"  {x['piece']} {x['time']}: asked {x['asked_N']:.1f} meas {x['meas_N']:.1f} max {x['meas_max_N']:.1f} "
              f"side {x['ext_max_N']:.1f} twist {x['twist_max_deg']:.1f} solve {x['socp_ms']:.0f} ms "
              f"tick95 {x['tick_p95_ms']:.1f} ati sd {x['ati_sd_N']:.3f}")
    print("  tracking:", {k: {a: round(b, 1) for a, b in v.items()} for k, v in track.items()})
