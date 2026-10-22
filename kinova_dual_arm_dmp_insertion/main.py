"""Command line.

    ros2 run kinova_dual_arm_dmp_insertion list
    ros2 run kinova_dual_arm_dmp_insertion check  L_s7040m6 [--arm scalar]   # no robot: dry run + reach
    ros2 run kinova_dual_arm_dmp_insertion insert L_s7040m6 [--arm scalar] [--config cell.yaml]

The real pair without a shelf calibration (pair.py: left base at +0.60 m y of the
right one, tools facing each other), the route 3 times slower than trained:

    python -m kinova_dual_arm_dmp_insertion.main check  F75_Tt1_s9007 --arm c100_five_cut800 --sep 0.60
    python -m kinova_dual_arm_dmp_insertion.main insert F75_Tt1_s9007 --arm c100_five_cut800 --sep 0.60 --slow 3

`check` and `list` need no ROS at all (python -m kinova_dual_arm_dmp_insertion.main check ...).
"""
import argparse
import csv
import os
import sys
import time

import numpy as np

from . import geometry as GE
from . import pair
from .route import Route, available

HERE = os.path.dirname(os.path.abspath(__file__))


LOCAL = "local.yaml"


def merge(base, over):
    """Nested dict update: `over` wins, key by key."""
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            merge(base[k], v)
        else:
            base[k] = v
    return base


def load_config(path):
    import yaml
    if path is None:
        for p in (os.path.join(HERE, "..", "config", "cell.yaml"),):
            if os.path.isfile(p):
                path = p
        if path is None:   # installed by colcon: share/<pkg>/config
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory("kinova_dual_arm_dmp_insertion"),
                                "config", "cell.yaml")
    with open(path, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    # what was measured on THIS robot PC (ft_check --save): next to the config, not in git
    local = os.path.join(os.path.dirname(os.path.abspath(path)), LOCAL)
    if os.path.isfile(local):
        with open(local, encoding="utf-8") as fh:
            merge(cfg, yaml.safe_load(fh) or {})
        print(f"  local values: {local}")
    t = cfg.get("tool", {})
    GE.set_tool(t.get("pad_x", 0.020), t.get("r_tri", 0.030), t.get("pad_r", 0.010))
    return cfg, path


W6 = ("fx", "fy", "fz", "tx", "ty", "tz")
TELEMETRY_COLS = ([f"{kind}_{s}_{unit}" for s in "LR" for kind, unit in (("joint_age", "ms"), ("ft_age", "ms"))]
                  + [f"{kind}_stamp_{s}" for s in "LR" for kind in ("joint", "ft")]
                  + [f"{kind}{s}_j{j}" for s in "LR" for kind in ("q", "dq", "tau") for j in range(1, 8)])


class Log:
    """One CSV row per control tick: what was commanded, what was measured.
    The first 35 columns are the original ones; then (2026-10-21) the measured
    flange orientations, the moments the controller used, the raw readings of both
    sources (estimated: ee axes; ATI: sensor axes, N / Nm, before bias), the
    full-wrench offsets and the SOCP's desired hand wrenches. Shelf frame
    unless said; F, T = hand ON the piece, T about the tool origin."""
    COLS = ("t phase xL yL zL qxL qyL qzL qwL xR yR zR qxR qyR qzR qwR "
            "mxL myL mzL mxR myR mzR FxL FyL FzL FxR FyR FzR ds_mm target_N "
            "fn_N ext_x ext_y ext_z jammed").split() + (
        "t_wall mqxL mqyL mqzL mqwL mqxR mqyR mqzR mqwR TxL TyL TzL TxR TyR TzR".split()
        + [f"{src}{s}_{c}" for src in ("est", "ft") for s in "LR" for c in W6]
        + [f"off{s}_{c}" for s in "LR" for c in ("x", "y", "z", "rx", "ry", "rz")]
        + [f"wdes{s}_{c}" for s in "LR" for c in W6]
        + ["socp_ms", "ctrl_dt_ms", "ctrl_seq", "socp_ok", "socp_input_age_ms"] + TELEMETRY_COLS)

    def __init__(self, path):
        self.fh = open(path, "w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow(self.COLS)

    def row(self, ins, tL, qL, tR, qR):
        m = ins.meas
        e = ins.external_force()
        raw = getattr(ins.b, "raw", {})
        nan6 = [np.nan] * 6
        wdes = [np.round(np.concatenate(ins.w_des[s]), 4) if s in ins.w_des else nan6 for s in "LR"]
        telemetry = getattr(ins.b, "telemetry", {})
        input_age = (1000 * (ins.b.now() - ins.socp_input_time)
                     if ins.socp_input_time is not None else np.nan)
        self.w.writerow([f"{ins.b.now():.3f}", ins.phase, *np.round(tL, 5), *np.round(qL.as_quat(), 5),
                         *np.round(tR, 5), *np.round(qR.as_quat(), 5), *np.round(m["pL"], 5),
                         *np.round(m["pR"], 5), *np.round(m["FL"], 2), *np.round(m["FR"], 2),
                         f"{1000 * ins.ds:.2f}", f"{ins.target or 0:.2f}", f"{ins.normal_force():.2f}",
                         *np.round(e, 2), int(ins.jammed),
                         f"{time.time():.4f}", *np.round(m["qL"], 5), *np.round(m["qR"], 5),
                         *np.round(m["TL"], 4), *np.round(m["TR"], 4),
                         *np.round(np.concatenate([raw.get(k, nan6) for k in ("estL", "estR", "ftL", "ftR")]), 4),
                         *np.round(ins.off["L"], 5), *np.round(ins.off["R"], 5), *np.concatenate(wdes),
                         f"{ins.socp_ms:.1f}", f"{1000 * ins.control_dt:.3f}", getattr(ins, "control_seq", 0),
                         int(ins.socp_status) if ins.socp_status is not None else "", f"{input_age:.3f}",
                         *(telemetry.get(c, np.nan) for c in TELEMETRY_COLS)])

    def close(self):
        self.fh.close()


def write_meta(path, cfg, cfg_path, a, backend=None, result=None, ins=None):
    """Everything needed to read a run later: the config as used, the command,
    the code version, the ATI units and every tare's biases."""
    import json
    import subprocess
    try:
        rev = subprocess.run(["git", "rev-parse", "HEAD"], cwd=HERE, capture_output=True,
                             text=True, timeout=5, check=True).stdout.strip()
        branch = subprocess.run(["git", "branch", "--show-current"], cwd=HERE, capture_output=True,
                                text=True, timeout=5, check=True).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain"], cwd=HERE, capture_output=True,
                                    text=True, timeout=5, check=True).stdout.strip())
    except Exception:
        rev, branch, dirty = None, None, None
    meta = dict(argv=sys.argv, config_path=os.path.abspath(cfg_path), config=cfg, git=rev, git_branch=branch, git_dirty=dirty,
                started=time.strftime("%Y-%m-%d %H:%M:%S"), result=result)
    meta["log_timing"] = ("poses are commands already sent; measured wrench is read after wait; "
                          "ds/target/wdes are updated after that reading for subsequent commands. "
                          "Source stamps identify the latest message, ages are since local receipt; "
                          "ATI values average the last configured samples.")
    if backend is not None:
        meta["ft_units"] = {arm.name: arm.ft_units for arm in backend.arms.values()}
        meta["wrench_source"] = {arm.name: arm.source for arm in backend.arms.values()}
        meta["tares"] = backend.tares
        meta["controller_parameters"] = getattr(backend, "controller_parameters", {})
        meta["joint_names"] = {s: list(arm.joints.name[:7]) for s, arm in backend.arms.items()}
    if ins is not None:
        from .controller import GAIN, CONTROL_PERIOD
        meta["execution"] = dict(mode=getattr(ins, "execution_mode", "online_socp_ati_feedback"),
                                 squeeze_gain_scale=ins.gain / GAIN,
                                 squeeze_gain_m_per_N_per_update=ins.gain,
                                 nominal_force_update_period_s=CONTROL_PERIOD,
                                 squeeze_feedback=ins.squeeze_feedback, internal_wrench_feedback=ins.hand_wrench,
                                 wrench_channels=ins.channels if ins.full_wrench else "internal" if ins.hand_wrench else "none",
                                 internal_translation_limit_m=ins.force_offset_limit,
                                 k_eff_N_per_m=getattr(ins, "k_eff", None),
                                 offline_profile=getattr(getattr(ins, "profile", None), "path", None),
                                 offline_profile_sha256=getattr(getattr(ins, "profile", None), "sha256", None),
                                 planning_k_eff_N_per_m=getattr(getattr(ins, "profile", None), "k_eff", None),
                                 planned_socp_ok=getattr(getattr(ins, "profile", None), "socp_ok", None),
                                 planned_socp_ticks=getattr(getattr(ins, "profile", None), "socp_ticks", None),
                                 desired_moment_origin="common grasp centre", measured_moment_origin="tool origin")
        meta["socp_last_reason"] = ins.socp_reason
        meta["allocation_failures"] = ins.stats["socp_fail"]
        meta["allocation_result_not_applied"] = ins._socp_job is not None
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(meta, fh, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))


def reach_report(route, cfg, rows):
    """How far each commanded tool pose is from its arm's base (shoulder) - a
    first reachability check; the arm's own IK has the last word."""
    from .geometry import Frame
    out = []
    for side, cols in (("left", slice(1, 4)), ("right", slice(8, 11))):
        f = Frame(**{"t": cfg[side]["shelf_in_base"]["xyz"], "q": cfg[side]["shelf_in_base"]["quat"]})
        p = f.t + f.R.apply(np.asarray(rows)[:, cols].astype(float))
        d = np.linalg.norm(p - np.asarray(cfg[side].get("shoulder_in_base", [0, 0, 0.28])), axis=1)
        out.append(f"{side}: farthest {1000 * d.max():.0f} mm from the shoulder "
                   f"(limit {1000 * cfg['max_reach']:.0f}); tool height in the base frame "
                   f"{1000 * p[:, 2].min():.0f} to {1000 * p[:, 2].max():.0f} mm, "
                   f"x {1000 * p[:, 0].min():.0f} to {1000 * p[:, 0].max():.0f}, "
                   f"y {1000 * p[:, 1].min():.0f} to {1000 * p[:, 1].max():.0f}")
    return out


def placed(a, cfg, r):
    """With --sep: the nominal pair (pair.py) instead of the calibrated shelf_in_base."""
    if a.sep is None:
        return cfg
    cfg = pair.place(cfg, r, a.sep, a.forward, a.lift, a.shelf_x)
    for line in pair.describe(cfg, r):
        print("  " + line)
    return cfg


def slowed(ins, slow):
    """The route, the seat and the approach `slow` times slower than trained
    (quasi-static: the squeeze and the SOCP run on their own clocks)."""
    ins.motion_speed /= slow
    ins.turn_rate /= slow
    ins.seat_speed /= slow
    return ins


def socp_kwargs(cfg):
    """The full-wrench SOCP settings of the cell (config/cell.yaml: socp, full_wrench)."""
    return dict(socp_cfg=cfg.get("socp"), full_wrench=bool(cfg.get("full_wrench", False)),
                full_wrench_channels=cfg.get("full_wrench_channels", "twist"),
                moment_sign=float(cfg.get("full_wrench_moment_sign", -1.0)))


def cmd_check(a, cfg, make=None):
    """Dry run: the whole sequence with no robot, then the commanded route
    against the flange poses the simulated apparatus was commanded.
    `make(a, route, backend, **kw)` builds the insertion (default: Insertion)."""
    from .controller import Insertion
    from .dryrun import DryRun
    make = make or (lambda a_, r_, b_, **kw: Insertion(r_, b_, **kw))
    r = Route(a.world, a.arm, cfg.get('scale', 1.0), cfg.get('piece_mass'))
    cfg = placed(a, cfg, r)
    b = DryRun(r, period=1.0 / cfg["rate_hz"])
    rows = []

    class Keep:
        def row(self, ins, tL, qL, tR, qR):
            rows.append([ins.b.now(), *tL, *qL.as_quat(), *tR, *qR.as_quat(), ins.phase])
    ins = slowed(make(a, r, b, log=Keep(), confirm=lambda msg: print(f"  (dry run) {msg}"),
                      squeeze_gain=a.squeeze_gain, **socp_kwargs(cfg)), a.slow)
    t0 = time.perf_counter()
    ins.run()
    print(f"  dry run: {b.now():.1f} s of motion, solved in {time.perf_counter() - t0:.1f} s; "
          f"SOCP ok {ins.stats['socp_ok']}/{ins.stats['socp_ticks']}")
    # the route phase against the simulator's commanded flanges (same piece frame, same pacing)
    trace = r.path[:-5] + "_trace.npz"
    if r.eff_scale != 1.0:
        print(f"  (the PyBullet trace is at another size: path compared only when the route is not rescaled)")
    elif os.path.isfile(trace):
        tr = np.load(trace)
        sim = tr["rows"][tr["phase"] == "route"]
        mine = np.array([x[:15] for x in rows if x[15] == "route"], float)
        if len(sim) and len(mine):
            # Real flange-to-face depth is 100 mm; this export used ~28 mm.
            # Comparing raw flanges would report this geometric offset as path error.
            sim_depth = float(r.c["pad_x"]) + float(r.c["pad_r"])
            sim_faces = GE.contact_face_centres(sim[:, 1:4], sim[:, 4:8], sim_depth)
            my_faces = GE.contact_face_centres(mine[:, 1:4], mine[:, 4:8], GE.PAD_X + GE.PAD_R)
            dist = [float(np.min(np.linalg.norm(sim_faces - face, axis=1))) for face in my_faces]
            print(f"  route phase: {len(mine)} ticks here, {len(sim)} in PyBullet "
                  f"({mine[-1, 0] - mine[0, 0]:.1f} s vs {sim[-1, 0] - sim[0, 0]:.1f} s); "
                  f"commanded left contact-face path vs PyBullet's: median {1000 * np.median(dist):.1f} mm, "
                  f"max {1000 * np.max(dist):.1f} mm (the squeeze differs: no real contact here)")
    for line in reach_report(r, cfg, rows):
        print("  " + line)
    for line in stiffness_report(r, cfg):
        print("  " + line)


def stiffness_report(route, cfg):
    """The lab controller's cart_k is diagonal in the BASE axes and fixed at
    launch: the torsional 40 Nm/rad goes on the base axis the push axis lies along."""
    from .geometry import Frame
    k = cfg["stiffness"]
    out = []
    a = route.axis / np.linalg.norm(route.axis)
    for side, push in (("left", a), ("right", -a)):
        f = Frame(cfg[side]["shelf_in_base"]["xyz"], cfg[side]["shelf_in_base"]["quat"])
        d = f.R.apply(push)
        i = int(np.argmax(np.abs(d)))
        rot = [k["tilt"]] * 3
        rot[i] = k["torsion"]
        kk = [k["trans"]] * 3 + rot
        line = (f"{side}: push axis in base_link {np.round(d, 3).tolist()} -> "
                f"cart_k:='[{', '.join(f'{v:.1f}' for v in kk)}]'")
        if abs(d[i]) < 0.95:
            line += "  WARNING: not along a base axis, the diagonal stiffness cannot match the sim"
        out.append(line)
    return out


def cmd_insert(a, cfg, make=None):
    from .controller import Abort, Insertion
    from .ros_backend import RosBackend
    make = make or (lambda a_, r_, b_, **kw: Insertion(r_, b_, **kw))
    r = Route(a.world, a.arm, cfg.get('scale', 1.0), cfg.get('piece_mass'))
    if r.hand_wrench != "off" and not cfg.get("allow_hand_wrench", False):
        sys.exit("this route was trained with the hand wrench (hwd): set allow_hand_wrench: true "
                 "in the config once the F/T moments are verified")
    if cfg.get("full_wrench", False) and not cfg.get("allow_hand_wrench", False):
        sys.exit("full_wrench: true regulates the flange MOMENTS: set allow_hand_wrench: true "
                 "in the config once the F/T moments are verified")
    if cfg.get("full_wrench", False) and any(cfg[s].get("wrench_source") != "ft" for s in ("left", "right")):
        sys.exit("full_wrench: true needs the wrist ATI on BOTH arms (wrench_source: ft): "
                 "the joint-torque estimate is not good enough for the moments")
    os.makedirs(cfg["log_dir"], exist_ok=True)
    path = os.path.join(cfg["log_dir"], time.strftime(f"{r.world}_{a.arm}{getattr(a, 'log_tag', '')}_%Y%m%d_%H%M%S.csv"))
    log = Log(path)
    meta = path[:-4] + "_meta.json"
    b = RosBackend(cfg)
    ins = None
    result = None
    try:
        write_meta(meta, cfg, a.cfg_path, a, b)
        ins = slowed(make(a, r, b, log=log, f_abort=cfg["f_abort"], track_abort=cfg["track_abort"],
                          jam_scale=cfg.get("jam_scale", 1.0), jam_min=cfg.get("jam_min", 2.0),
                          async_socp=True, squeeze_gain=a.squeeze_gain, **socp_kwargs(cfg)), a.slow)
        write_meta(meta, cfg, a.cfg_path, a, b, ins=ins)
        result = ins.run()
    except (Abort, KeyboardInterrupt) as e:
        result = dict(stopped=str(e) or "Ctrl-C", phase=ins.phase if ins is not None else "preparation")
        print(f"\n  STOPPED: {e or 'Ctrl-C'} - the arms hold their last target")
    except Exception as e:
        result = dict(stopped=f"{type(e).__name__}: {e}", phase=ins.phase if ins is not None else "preparation")
        raise
    finally:
        log.close()
        try:
            write_meta(meta, cfg, a.cfg_path, a, b, result, ins=ins)
        finally:
            if ins is not None and ins._socp_pool is not None:
                ins._socp_pool.shutdown(wait=True)
            b.close()
        print(f"  log: {path}\n  meta: {meta}")


ARMS = ("scalar", "hwd", "c100_narrow75", "c100_narrow75hs", "c100_five", "c100_five625", "c100_five_cut800", "c100_five_cut700")


def main(argv=None, doc=None, extra=None, make=None):
    """`extra(ap)` adds arguments, `make` builds the insertion (insert_pick.py)."""
    ap = argparse.ArgumentParser(description=doc or __doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=("list", "check", "insert"))
    ap.add_argument("world", nargs="?")
    ap.add_argument("--arm", default="scalar", choices=ARMS,
                    help="scalar: the squeeze alone (thesis design, 24/29 in PyBullet); "
                         "hwd: + the internal hand wrench (25/29). A route runs only with the control it was trained with. "
                         "c100_*: routes TRAINED at the real size (100 mm cubes, 70 mm tri-pad, scalar squeeze; "
                         "cubes100/README.md): c100_narrow75 the 29 narrow75 worlds, c100_five / c100_five625 the "
                         "20-cube kit worlds at 75%% / 62.5%%")
    ap.add_argument("--config", default=None)
    ap.add_argument("--twist", action="store_true",
                    help="full wrench: squeeze + opposite twist about the pad axis, closed on the wrist "
                         "ATIs (= full_wrench: true, full_wrench_channels: twist, allow_hand_wrench: true)")
    ap.add_argument("--wrench-channels", choices=("forces", "twist", "contact", "internal"), default=None,
                    help="ATI feedback channels: forces = squeeze + internal tangential forces, no moment loop; "
                         "twist = squeeze + opposite axial twist; contact = forces + axial twist (pilot); "
                         "internal = all internal wrench components")
    ap.add_argument("--no-bending", action="store_true",
                    help="SOCP: zero bending moment at each pad centre, keeping axial torsion (Wong-like contact model)")
    ap.add_argument("--squeeze-gain", type=float, default=0.5,
                    help="x the squeeze integrator's gain GAIN (default 0.5: 1 gave a ~1.5 Hz sine in the "
                         "squeeze, 0.25 recovers too slowly when the piece meets its neighbours; Z, 22/10)")
    ap.add_argument("--slow", type=float, default=1.0,
                    help="route, seat and turns this many times slower than trained (default 1)")
    pair.add_args(ap)
    if extra:
        extra(ap)
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])
    if not np.isfinite(a.squeeze_gain) or a.squeeze_gain < 0:
        ap.error("--squeeze-gain must be finite and non-negative")
    if not np.isfinite(a.slow) or a.slow <= 0:
        ap.error("--slow must be finite and positive")
    if a.twist and a.wrench_channels not in (None, "twist"):
        ap.error("--twist conflicts with --wrench-channels " + a.wrench_channels)
    if a.command == "list":
        from .route import Route as R
        for arm in ARMS:
            for w in available(arm):
                print("  " + R(w, arm).summary())
        return
    if not a.world:
        ap.error("which world? (list)")
    cfg, path = load_config(a.config)
    a.cfg_path = path
    if a.twist:
        cfg.update(full_wrench=True, full_wrench_channels="twist", allow_hand_wrench=True)
    if a.wrench_channels:
        cfg.update(full_wrench=True, full_wrench_channels=a.wrench_channels, allow_hand_wrench=True)
    if a.no_bending:
        cfg.setdefault("socp", {})["no_bending"] = True
    print(f"  wrench control: {cfg.get('full_wrench_channels') if cfg.get('full_wrench') else 'scalar squeeze only'}"
          f"; wrench source left {cfg['left'].get('wrench_source', 'estimated')}, "
          f"right {cfg['right'].get('wrench_source', 'estimated')}")
    print(f"  squeeze gain x{a.squeeze_gain:g}")
    print(f"  config: {os.path.abspath(path)}")
    (cmd_check if a.command == "check" else cmd_insert)(a, cfg, make)


def list_entry():
    main(["list"])


def check_entry():
    main(["check"] + sys.argv[1:])


def insert_entry():
    main(["insert"] + sys.argv[1:])


if __name__ == "__main__":
    main()
