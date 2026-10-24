"""
One run of the thesis pipeline (blind grasp -> squeeze -> lift -> wielding)
with either the FIXED tri-pad or the NOMINAL floating plate, logging, at every
allocator tick, the six per-pad forces the SOCP planned next to the six forces
PyBullet actually realised at the contacts.

Nothing in TESI_antiG is modified: its `lib/` is imported read-only and
`grasp.ask_socp` is wrapped so that the original runs unchanged and the
recorder only reads. The command is the full-wrench FF+I of `ffi_wrench.py`
(default; insert_ffi extended to 12 components), TESI's `HandWrench` "full"
(all twelve allocated wrench components tracked by a 6-DoF flange bias), or
the scalar squeeze as a reference arm.

    python run_case.py --pop inputs/pop_cube --obj 0 --arm rigid --out runs/x
"""
import argparse
import json
import os
import sys
import time

import numpy as np

TESI = os.path.expanduser(
    "~/Documents/GitHub/TESI_antiG/lorenzo_data_analysis/10_tripad_passivo")
sys.path.insert(0, os.path.join(TESI, "spider_ee"))
sys.path.insert(0, TESI)

import pybullet as p                                                 # noqa: E402
from lib import population as POP, world as W, grasp as G, tool as T  # noqa: E402
from lib import wrist as WR                                          # noqa: E402
from lib.session import Rig                                          # noqa: E402
from lib.inertia import BlindEstimator, OnlineLoad                   # noqa: E402
from lib.audit import LEDGER                                         # noqa: E402
from lib.hand_wrench import HandWrench                               # noqa: E402
from ffi_wrench import HandWrenchFFI                                 # noqa: E402
import floating_plate as FP                                          # noqa: E402

DT = 0.001
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def make_kinova_solver(mu, no_bending=False, task_scale=0.25, floor_N=1.0):
    """The robot's SOCP (kinova_dual_arm_dmp_insertion/socp.py) with the
    settings of config/cell.yaml: signed friction distance psi >= rho_i,
    task-adaptive rho = 0.25 ||F_task|| + 1 N, precision-only TP-GMM prior.
    Loaded by file path so the ROS package (rclpy) is not imported."""
    import importlib.util
    pkg = os.path.join(REPO, "kinova_dual_arm_dmp_insertion")
    spec = importlib.util.spec_from_file_location("kinova_socp", os.path.join(pkg, "socp.py"))
    socp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(socp)
    from lib.prior import HumanPrior
    s = socp.GraspSOCP(prior=HumanPrior(os.path.join(pkg, "prior_torsional_C.npz")),
                       prior_mode="tpgmm", mu=mu, w_force=0.0, no_bending=bool(no_bending),
                       margin_mode="task_adaptive", margin_task_scale=float(task_scale),
                       margin_floor_N=float(floor_N), margin_fixed_N=0.5)
    # read by TESI's lib/audit.check_allocation; this program has no cap and
    # no separate floor (contact maintenance comes from the margin)
    s.f_pad_max, s.f_min, s.r_s = 1e9, 0.0, 0.0     # r_s: newer TESI audit
    return s


# --------------------------------------------------------------- recorder
class Recorder:
    def __init__(self):
        self.rows = []
        self.step = 0
        self.body = None
        self.tools = None

    def pad_links(self, tool):
        if getattr(tool, "plate_id", None) is not None:
            return [(tool.plate_id, i) for i in range(3)]
        return [(tool.id, j) for j in tool.pads]

    def measure(self):
        """Per pad: force ON THE OBJECT (friction sign corrected as in
        `lib.tool.wrench_on_body`), weighted contact point, outward normal,
        number of manifold points, per-point (N, T), tangential slip speed."""
        body = self.body
        pos_o, _ = p.getBasePositionAndOrientation(body)
        v_o, w_o = p.getBaseVelocity(body)
        pos_o, v_o, w_o = (np.asarray(x, float) for x in (pos_o, v_o, w_o))
        out = []
        for tool in self.tools:
            for (bid, link) in self.pad_links(tool):
                pts = p.getContactPoints(bodyA=body, bodyB=bid, linkIndexB=link)
                if not pts:
                    out.append(None)
                    continue
                flip = T._stored_second(body, bid, link)
                F = np.zeros(3); P = np.zeros(3); Nn = np.zeros(3); wsum = 0.0
                per_pt = []
                for c in pts:
                    n_in = np.asarray(c[7], float)      # B(pad) -> A(object)
                    fr = c[10] * np.asarray(c[11], float) + c[12] * np.asarray(c[13], float)
                    if flip:
                        fr = -fr
                    f = c[9] * n_in + fr                # on the object
                    F += f
                    wgt = max(c[9], 1e-6)
                    P += wgt * np.asarray(c[5], float)
                    Nn += wgt * (-n_in)                 # outward from object
                    wsum += wgt
                    per_pt.append((c[9], float(np.linalg.norm(fr))))
                P /= wsum
                n = Nn / max(np.linalg.norm(Nn), 1e-12)
                ls = p.getLinkState(bid, link, computeLinkVelocity=1)
                v_pad = np.asarray(ls[6], float)
                v_obj = v_o + np.cross(w_o, P - pos_o)
                dv = v_pad - v_obj
                dv_t = dv - (dv @ n) * n
                out.append(dict(F=F, P=P, n=n, npts=len(pts), per_pt=per_pt,
                                slip=float(np.linalg.norm(dv_t))))
        return out

    def record(self, result, cL, cR):
        if result is None:
            return
        meas = self.measure()
        pos_o, q_o = p.getBasePositionAndOrientation(self.body)
        tool_ids = {b for t in self.tools for b, _ in self.pad_links(t)}
        tool_ids |= {t.id for t in self.tools}
        ext = [c for c in p.getContactPoints(bodyA=self.body) if c[2] not in tool_ids]
        planned = list(result["f_L"]) + list(result["f_R"])
        contacts = list(cL) + list(cR)
        row = dict(step=self.step, phase=LEDGER.phase, ok=bool(result["ok"]),
                   ok_solver=bool(result.get("ok_solver", result["ok"])),
                   mass_used=float(result.get("mass_used", np.nan)),
                   req=float(result.get("requested_force", np.nan)),
                   rho=list(result.get("required_margin_N", [0.0] * 6)),
                   reason=str(result.get("reason", "")),
                   ext_N=float(sum(c[9] for c in ext)),
                   obj_pos=list(pos_o), obj_quat=list(q_o),
                   tool_pos=[list(t.pose()[0]) for t in self.tools],
                   defl=[(list(t.deflection6()) if hasattr(t, "deflection6") else [0.0] * 6)
                         for t in self.tools],
                   pads=[])
        for k in range(6):
            c = contacts[k]
            m = meas[k]
            row["pads"].append(dict(
                active=bool(c[0]),
                plan=np.asarray(planned[k], float).tolist(),
                plan_n=(np.asarray(c[2], float).tolist() if c[0] else None),
                plan_p=(np.asarray(c[1], float).tolist() if c[0] else None),
                F=None if m is None else m["F"].tolist(),
                P=None if m is None else m["P"].tolist(),
                n=None if m is None else m["n"].tolist(),
                npts=0 if m is None else m["npts"],
                per_pt=[] if m is None else m["per_pt"],
                slip=None if m is None else m["slip"]))
        self.rows.append(row)


REC = Recorder()
_orig_ask = G.ask_socp
_orig_step = p.stepSimulation


def _ask(body, toolL, toolR, solver, load, orientation):
    result, cL, cR = _orig_ask(body, toolL, toolR, solver, load, orientation)
    try:
        REC.record(result, cL, cR)
    except Exception as e:                       # the recorder must never steer
        print("  [recorder]", e)
    return result, cL, cR


def _step():
    REC.step += 1
    return _orig_step()


G.ask_socp = _ask
p.stepSimulation = _step


PHASES = ("squeeze", "lift", "wield")


def pack(rows, mu):
    """Rows -> fixed arrays. Pads that do not touch are NaN. `eta_pt_max` is
    the worst single manifold point of a pad, so a resultant averaged over
    several points cannot hide a saturated one."""
    n = len(rows)
    nan3 = lambda: np.full((n, 6, 3), np.nan, np.float32)        # noqa: E731
    out = dict(step=np.array([r["step"] for r in rows], np.int32),
               phase=np.array([PHASES.index(r["phase"]) if r["phase"] in PHASES
                               else -1 for r in rows], np.int8),
               ok=np.array([r["ok"] for r in rows], bool),
               ext_N=np.array([r["ext_N"] for r in rows], np.float32),
               mass_used=np.array([r["mass_used"] for r in rows], np.float32),
               rho=np.array([r["rho"] for r in rows], np.float32).reshape(n, 6),
               obj_pos=np.array([r["obj_pos"] for r in rows], np.float32),
               obj_quat=np.array([r["obj_quat"] for r in rows], np.float32),
               tool_pos=np.array([r["tool_pos"] for r in rows], np.float32),
               defl=np.array([r["defl"] for r in rows], np.float32),
               active=np.zeros((n, 6), bool), npts=np.zeros((n, 6), np.int8),
               plan=nan3(), plan_n=nan3(), plan_p=nan3(), F=nan3(), P=nan3(),
               n=nan3(), slip=np.full((n, 6), np.nan, np.float32),
               eta_pt_max=np.full((n, 6), np.nan, np.float32), mu=np.float32(mu))
    for t, r in enumerate(rows):
        for k, pd in enumerate(r["pads"]):
            out["active"][t, k] = pd["active"]
            out["plan"][t, k] = pd["plan"]
            if pd["plan_n"] is not None:
                out["plan_n"][t, k] = pd["plan_n"]
                out["plan_p"][t, k] = pd["plan_p"]
            if pd["F"] is not None:
                out["F"][t, k], out["P"][t, k], out["n"][t, k] = pd["F"], pd["P"], pd["n"]
                out["npts"][t, k] = pd["npts"]
                out["slip"][t, k] = pd["slip"]
                out["eta_pt_max"][t, k] = max(
                    (tt / (mu * nn) if nn > 1e-9 else np.inf) for nn, tt in pd["per_pt"])
    return out


# ------------------------------------------------------------------- run
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pop", required=True)
    ap.add_argument("--obj", type=int, required=True)
    ap.add_argument("--arm", choices=["rigid", "plate"], required=True)
    ap.add_argument("--command", choices=["full", "scalar"], default="full")
    ap.add_argument("--wrench-law", choices=["ff", "ff_isq", "ffi", "admittance"], default="ff",
                    help="full command: ff = feed-forward only, ghost = pickup pose + "
                         "K_eff^-1 (W* - W_pick) on all 6 components (default, Wong eq. 48); "
                         "ff_isq = ff + integral on each hand's normal force (insert_ffi I); "
                         "ffi = ff + integral trim on all 6; admittance = TESI HandWrench "
                         "proportional law")
    ap.add_argument("--socp", choices=["kinova", "thesis"], default="kinova",
                    help="kinova: signed-distance (shifted) cone with the "
                         "task-adaptive margin, as on the robot; thesis: the "
                         "older r_s-shrunk cone of TESI_antiG/lib/socp.py")
    ap.add_argument("--no-bending", dest="no_bending", action="store_true", default=True,
                    help="SOCP: no moment across each pad plane, as Wong (default)")
    ap.add_argument("--bending", dest="no_bending", action="store_false",
                    help="SOCP without the no-bending constraint")
    ap.add_argument("--mu", type=float, default=1.0)
    ap.add_argument("--margin-task-scale", type=float, default=0.5,
                    help="lambda_m in rho = lambda_m ||F_task|| + floor. Default 0.5 "
                         "(robot cell.yaml: 0.25), from margin_sweep.py: halves the "
                         "saturated pads, the fixed tri-pad gains nothing beyond it")
    ap.add_argument("--margin-floor", type=float, default=1.0, help="[N] lambda_CoM")
    ap.add_argument("--min-pads", type=int, default=4,
                    help="seated pads (of 6) the blind search needs to accept a grasp")
    ap.add_argument("--amplitude", type=float, default=35.0)
    ap.add_argument("--cycles", type=float, default=2.0)
    ap.add_argument("--lift", type=float, default=0.12)
    ap.add_argument("--fallback-N", type=float, default=21.0)
    ap.add_argument("--mass-prior", type=float, default=3.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    W.set_friction(a.mu)
    FP.PLATE_COLLISION = False          # CONSTRAINT 1: only the six spheres
    obj = POP.load_objects(a.pop, [a.obj])[0]
    t0 = time.time()

    if a.arm == "plate":
        def factory(pos, quat):
            return FP.FloatingPlateTool(pos, quat, FP.PadSprings(),
                                        FP.CentralSpring(), mu=W.MU_BODY,
                                        dt=DT, quiet=True)
        urdf = T.write_urdf(T.scratch(f"_ct_{os.getpid()}.urdf"), None)
        rig = Rig(obj, urdf, None, dt=DT, tool_factory=factory, realign=False)
    else:
        urdf = T.write_urdf(T.scratch(f"_ct_{os.getpid()}.urdf"), None)
        rig = Rig(obj, urdf, None, dt=DT)
    body, toolL, toolR = rig.body, rig.toolL, rig.toolR
    REC.body, REC.tools = body, (toolL, toolR)
    if a.socp == "kinova":
        rig.solver = make_kinova_solver(W.MU_EFF, a.no_bending,
                                        a.margin_task_scale, a.margin_floor)
    P, solver, pace = rig.P, rig.solver, rig.pace

    rec = dict(pop=os.path.basename(a.pop.rstrip("/")), obj=a.obj, arm=a.arm,
               command=a.command, wrench_law=a.wrench_law,
               margin_task_scale=a.margin_task_scale, margin_floor=a.margin_floor, socp=a.socp, min_pads=a.min_pads, amplitude=a.amplitude, no_bending=bool(a.no_bending), mu_eff=W.MU_EFF,
               mass_true=float(obj["mass"]),
               margin_mode=getattr(solver, "margin_mode", None),
               r_s=getattr(solver, "r_s", 0.0))
    state, info = rig.grasp(min_pads=a.min_pads, verbose=False)
    rec.update(attempts=info["attempts"], seated=info["seated_pads"],
               axis=(info["history"][-1]["axis"] if info["history"] else None))
    stage = "NO_GRASP"
    if state is not None:
        if a.command == "full" and a.wrench_law in ("ff", "ff_isq", "ffi"):
            WR.WRENCH_CONTROLLER = HandWrenchFFI(
                (toolL, toolR), P.every * P.dt, kp=WR.KP, kpr=WR.KPR,
                k_tilt=WR.K_TILT_HOLD, fallback_N=a.fallback_N,
                g_int=(1.0 / 0.45 if a.wrench_law == "ffi" else 0.0),
                k_i_squeeze=(1.6e-3 if a.wrench_law == "ff_isq" else 0.0))
            WR.WRENCH_CONTROLLER.enabled = True
        elif a.command == "full":
            WR.WRENCH_CONTROLLER = HandWrench(
                "full", (toolL, toolR), P.every * P.dt, fallback_N=a.fallback_N,
                kp=WR.KP, kpr=WR.KPR, k_tilt=WR.K_TILT_HOLD)
            WR.WRENCH_CONTROLLER.enabled = True
        est = BlindEstimator(m0=a.mass_prior, min_contacts=4)
        load = OnlineLoad(est, mass_band=(T.MASS_MIN, a.mass_prior))
        LEDGER.phase = "squeeze"
        sq = G.squeeze(body, toolL, toolR, state, P, solver, load,
                       fallback_N=a.fallback_N, pace=pace)
        LEDGER.phase = "lift"
        lf = G.lift(body, toolL, toolR, sq, P, solver, load, height=a.lift,
                    fallback_N=a.fallback_N, pace=pace, estimator=est)
        stage = "DROPPED_LIFT"
        rec.update(held_lift=bool(lf["held"]))
        if lf["held"]:
            LEDGER.phase = "wield"
            wl = G.wielding(body, toolL, toolR, lf, P, solver, load,
                            estimator=est, amplitude=a.amplitude,
                            cycles=a.cycles, fallback_N=a.fallback_N, pace=pace)
            held = (toolL.n_contacts(body) > 0 and toolR.n_contacts(body) > 0)
            stage = "DONE" if held else "DROPPED_WIELD"
            v = est.verdict()
            rec.update(mass_est=float(v["mass"]))
        if WR.WRENCH_CONTROLLER is not None:
            rec.update(WR.WRENCH_CONTROLLER.summary())
    rec.update(stage=stage, ticks=len(REC.rows), seconds=round(time.time() - t0, 1))
    p.disconnect()

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out + ".json", "w") as fh:
        json.dump(rec, fh, indent=1, default=float)
    np.savez_compressed(a.out + ".npz", **pack(REC.rows, W.MU_EFF))
    print(json.dumps({k: rec[k] for k in ("pop", "obj", "arm", "command", "stage",
                                          "ticks", "seconds")}))


if __name__ == "__main__":
    main()
