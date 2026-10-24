"""
Planned (SOCP witness) against realised (PyBullet contact) per-pad forces.

Scored samples: allocator ticks in lift or wielding, SOCP accepted, object
off the floor (no non-tool contact), pad touching in both plan and reality.

Per pad-sample, with n the outward normal the plan used and f the force on
the object:  N = -n.f,  T = |f + N n|,  eta = T/(mu N),
psi = (mu N - T)/sqrt(1+mu^2)  (the signed distance the SOCP constrains to
psi >= rho_i).  The error df = f_real - f_plan is split, per hand, into the
part the flange wrench sees (range of H^T) and the internal part (null space
of the 6x9 hand map H), which no wrist F/T can observe.

    python analyze.py          -> metrics.csv, samples.npz, summary.json
"""
import glob
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# Where a campaign lives. Main study: mu_eff = 1 in runs_mu1/, results here.
# The first pass at mu_eff = 0.49 is kept as a sensitivity:
#   CT_RUNS=runs CT_OUT=sens_mu049 python analyze.py
RUNS = os.path.join(HERE, os.environ.get("CT_RUNS", "runs_ff_m050"))
OUT = os.path.join(HERE, os.environ.get("CT_OUT", "."))
POP_LABEL = {"pop_cube_com0": "cube", "pop_cube": "cube", "pop_cube_com40": "cube",
             "pop_jittered_cubes": "jittered", "try_deformed_cubes_jitter005": "deformed",
             "pop_piani_50_100": "polytope"}
# tetris dropped from the study (2026-10-24); its runs stay on disk, unread
EXCLUDED = {"pop_tetris"}
SAT = 0.99          # eta above this = cone saturated (Bullet clamps at 1)
SLIP = 1e-3         # [m/s] tangential relative speed counted as sliding


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def hand_map(pts, base):
    return np.vstack([np.hstack([np.eye(3)] * len(pts)),
                      np.hstack([skew(q - base) for q in pts])])


def analyse(npz_path, meta):
    with np.load(npz_path) as z:          # decompress once, not per access
        d = {k: z[k] for k in z.files}
    mu = float(d["mu"])
    scale = np.sqrt(1 + mu ** 2)
    keep = (d["ok"] & np.isin(d["phase"], (1, 2)) & (d["ext_N"] < 0.05))
    out = dict(cols={k: [] for k in ("t", "phase", "pad", "N", "T", "Np", "Tp", "rho",
                                     "slip", "eta_pt", "df", "df_null", "df_range")},
               hand={k: [] for k in ("dW_F", "dW_M", "null", "range", "Ntot")})
    c = out["cols"]
    for t in np.flatnonzero(keep):
        for h in range(2):
            idx = [k for k in range(3 * h, 3 * h + 3)
                   if d["active"][t, k] and np.isfinite(d["F"][t, k, 0])]
            if len(idx) < 2:
                continue
            base = d["tool_pos"][t, h].astype(float)
            pts = d["plan_p"][t, idx].astype(float)
            Hm = hand_map(pts, base)
            df = (d["F"][t, idx] - d["plan"][t, idx]).astype(float).ravel()
            Pr = np.linalg.pinv(Hm) @ Hm
            d_range, d_null = Pr @ df, df - Pr @ df
            dW = Hm @ df
            out["hand"]["dW_F"].append(np.linalg.norm(dW[:3]))
            out["hand"]["dW_M"].append(np.linalg.norm(dW[3:]))
            out["hand"]["null"].append(np.linalg.norm(d_null))
            out["hand"]["range"].append(np.linalg.norm(d_range))
            Ntot = 0.0
            for j, k in enumerate(idx):
                n = d["plan_n"][t, k].astype(float)
                F, f = d["F"][t, k].astype(float), d["plan"][t, k].astype(float)
                N, Np = -n @ F, -n @ f
                Ntot += N
                c["t"].append(int(d["step"][t])); c["phase"].append(int(d["phase"][t]))
                c["pad"].append(k); c["N"].append(N); c["Np"].append(Np)
                c["T"].append(np.linalg.norm(F + N * n)); c["Tp"].append(np.linalg.norm(f + Np * n))
                c["rho"].append(float(d["rho"][t, k])); c["slip"].append(float(d["slip"][t, k]))
                c["eta_pt"].append(float(d["eta_pt_max"][t, k]))
                c["df"].append(np.linalg.norm(df[3 * j:3 * j + 3]))
                c["df_null"].append(np.linalg.norm(d_null[3 * j:3 * j + 3]))
                c["df_range"].append(np.linalg.norm(d_range[3 * j:3 * j + 3]))
            out["hand"]["Ntot"].append(Ntot)
    c = {k: np.asarray(v, float) for k, v in c.items()}
    hd = {k: np.asarray(v, float) for k, v in out["hand"].items()}
    row = dict(meta)
    row["n_samples"] = int(len(c["N"]))
    if len(c["N"]) == 0:
        return row, c
    eta = c["T"] / np.maximum(mu * c["N"], 1e-9)
    etap = c["Tp"] / np.maximum(mu * c["Np"], 1e-9)
    psi = (mu * c["N"] - c["T"]) / scale
    psip = (mu * c["Np"] - c["Tp"]) / scale
    c.update(eta=eta, etap=etap, psi=psi, psip=psip)
    row.update(
        eta_plan_max=float(etap.max()), eta_med=float(np.median(eta)),
        eta_p95=float(np.percentile(eta, 95)),
        sat_frac=float(np.mean(eta >= SAT)),
        sat_pt_frac=float(np.mean(c["eta_pt"] >= SAT)),
        margin_lost_frac=float(np.mean(psi < c["rho"] - 1e-3)),
        plan_margin_lost_frac=float(np.mean(psip < c["rho"] - 1e-3)),
        psi_neg_frac=float(np.mean(psi < -1e-3)),
        slip_frac=float(np.mean(c["slip"] > SLIP)),
        slip_sat_frac=float(np.mean((c["slip"] > SLIP) & (eta >= SAT))),
        any_pad_sat_frac=float(len(np.unique(c["t"][eta >= SAT])) / len(np.unique(c["t"]))),
        df_med=float(np.median(c["df"])), df_p95=float(np.percentile(c["df"], 95)),
        dN_med=float(np.median(np.abs(c["N"] - c["Np"]))),
        dT_med=float(np.median(np.abs(c["T"] - c["Tp"]))),
        df_over_rho_med=float(np.median(c["df"] / np.maximum(c["rho"], 1e-9))),
        df_gt_rho_frac=float(np.mean(c["df"] > c["rho"])),
        rho_med=float(np.median(c["rho"])),
        N_med=float(np.median(c["N"])),
        hand_null_med=float(np.median(hd["null"])), hand_range_med=float(np.median(hd["range"])),
        null_share=float(np.median(hd["null"] ** 2 / np.maximum(hd["null"] ** 2 + hd["range"] ** 2, 1e-12))),
        dWF_med=float(np.median(hd["dW_F"])), dWM_med=float(np.median(hd["dW_M"])),
        null_over_N=float(np.median(hd["null"] / np.maximum(hd["Ntot"], 1e-9))),
        # the margin that would have kept every realised pad inside the
        # nominal cone, given the planned distance: max(psi_plan - psi_real)
        rho_needed_p95=float(np.percentile(psip - psi, 95)),
        rho_needed_max=float(np.max(psip - psi)),
    )
    return row, c


def main():
    os.makedirs(OUT, exist_ok=True)
    rows, pooled = [], {}
    for js in sorted(glob.glob(os.path.join(RUNS, "*.json"))):
        meta = json.load(open(js))
        npz = js[:-5] + ".npz"
        if meta["pop"] in EXCLUDED:
            continue
        meta["family"] = POP_LABEL.get(meta["pop"], meta["pop"])
        if meta.get("no_bending"):
            meta["command"] += "_nb"
        meta["run"] = os.path.basename(js)[:-5]
        keep = {k: meta.get(k) for k in ("run", "family", "pop", "obj", "arm", "command",
                                         "stage", "mass_true", "mass_est", "attempts",
                                         "seated", "ticks",
                                         "hw_wield_F_error_median_N",
                                         "hw_wield_T_error_median_Nm")}
        if not os.path.exists(npz):
            rows.append(keep)
            continue
        row, c = analyse(npz, keep)
        rows.append(row)
        if len(c.get("N", [])):
            key = (meta["family"], meta["arm"], meta["command"])
            pooled.setdefault(key, []).append(c)
    keys = sorted({k for r in rows for k in r}, key=lambda k: (k not in rows[0], k))
    with open(os.path.join(OUT, "metrics.csv"), "w") as fh:
        fh.write(",".join(keys) + "\n")
        for r in rows:
            fh.write(",".join("" if r.get(k) is None else str(r.get(k)) for k in keys) + "\n")
    arrays = {}
    for (fam, arm, cmd), cs in pooled.items():
        for name in ("N", "T", "Np", "Tp", "rho", "eta", "etap", "psi", "psip",
                     "slip", "df", "df_null", "df_range", "eta_pt", "phase"):
            arrays[f"{fam}|{arm}|{cmd}|{name}"] = np.concatenate([c[name] for c in cs]).astype(np.float32)
    np.savez_compressed(os.path.join(OUT, "samples.npz"), **arrays)
    json.dump(rows, open(os.path.join(OUT, "metrics.json"), "w"), indent=1, default=float)
    print(len(rows), "runs analysed")


if __name__ == "__main__":
    main()
