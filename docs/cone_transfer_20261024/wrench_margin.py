"""
Is the margin lost at the PAD level also lost at the WRENCH level?

For a subsample of scored ticks, take each hand's REALISED flange wrench W_h
(from the measured contact forces, about the wrist) and solve

    t* = max t   s.t.  H_h f = W_h,  psi_mu(f_i) >= t  for the active pads

i.e. the largest uniform signed friction distance any redistribution of that
same wrench over the same contacts could have. t* >= rho means the realised
wrench still carries the designed margin and only its split over the pads
lost it; t* < 0 means no split at all keeps the wrench inside the cones.

    python wrench_margin.py   -> wrench_margin.csv
"""
import glob
import json
import os
from multiprocessing import Pool

import cvxpy as cp
import numpy as np

from analyze import hand_map, POP_LABEL, EXCLUDED

HERE = os.path.dirname(os.path.abspath(__file__))

# Where a campaign lives. Main study: mu_eff = 1 in runs_mu1/, results here.
# The first pass at mu_eff = 0.49 is kept as a sensitivity:
#   CT_RUNS=runs CT_OUT=sens_mu049 python analyze.py
RUNS = os.path.join(HERE, os.environ.get("CT_RUNS", "runs_ff_m050"))
OUT = os.path.join(HERE, os.environ.get("CT_OUT", "."))
EVERY = 8                       # every 8th scored tick (~6 Hz)


def build(npad, mu):
    f = cp.Variable(3 * npad)
    t = cp.Variable()
    H = cp.Parameter((6, 3 * npad))
    W = cp.Parameter(6)
    ns = [cp.Parameter(3) for _ in range(npad)]
    Ps = [cp.Parameter((3, 3)) for _ in range(npad)]
    con = [H @ f == W]
    for i in range(npad):
        fi = f[3 * i:3 * i + 3]
        con += [cp.norm(Ps[i] @ fi, 2) <= mu * (-(ns[i] @ fi)) - np.sqrt(1 + mu ** 2) * t]
    # bounded: t cannot exceed what the normal load allows
    prob = cp.Problem(cp.Maximize(t), con + [t <= 100])
    return prob, f, t, H, W, ns, Ps


def one(js):
    meta = json.load(open(js))
    if meta["pop"] in EXCLUDED:
        return []
    with np.load(js[:-5] + ".npz") as z:
        d = {k: z[k] for k in z.files}
    mu = float(d["mu"])
    progs = {n: build(n, mu) for n in (2, 3)}
    keep = np.flatnonzero(d["ok"] & np.isin(d["phase"], (1, 2)) & (d["ext_N"] < 0.05))[::EVERY]
    out = []
    for t in keep:
        for h in range(2):
            idx = [k for k in range(3 * h, 3 * h + 3)
                   if d["active"][t, k] and np.isfinite(d["F"][t, k, 0])]
            if len(idx) < 2:
                continue
            prob, f, tv, H, W, ns, Ps = progs[len(idx)]
            base = d["tool_pos"][t, h].astype(float)
            pts = d["plan_p"][t, idx].astype(float)
            Hm = hand_map(pts, base)
            H.value = Hm
            Freal = d["F"][t, idx].astype(float).ravel()
            Fplan = d["plan"][t, idx].astype(float).ravel()
            res = {}
            for tag, vec in (("real", Freal), ("plan", Fplan)):
                W.value = Hm @ vec
                for i, k in enumerate(idx):
                    n = d["plan_n"][t, k].astype(float)
                    ns[i].value = n
                    Ps[i].value = np.eye(3) - np.outer(n, n)
                try:
                    prob.solve(solver="CLARABEL")
                    res[tag] = float(tv.value) if prob.status in ("optimal", "optimal_inaccurate") else -np.inf
                except Exception:
                    res[tag] = np.nan
            scale = np.sqrt(1 + mu ** 2)
            psi_real = []
            for k in idx:
                n = d["plan_n"][t, k].astype(float)
                F = d["F"][t, k].astype(float)
                N = -n @ F
                psi_real.append((mu * N - np.linalg.norm(F + N * n)) / scale)
            out.append(dict(run=os.path.basename(js)[:-5], family=POP_LABEL.get(meta["pop"]),
                            arm=meta["arm"], command=meta["command"] + ("_nb" if meta.get("no_bending") else ""), stage=meta["stage"],
                            phase=int(d["phase"][t]), hand=h, npads=len(idx),
                            rho=float(np.mean(d["rho"][t, idx])),
                            psi_pad_min=float(min(psi_real)),
                            t_real=res["real"], t_plan=res["plan"]))
    return out


if __name__ == "__main__":
    files = sorted(glob.glob(os.path.join(RUNS, "*.json")))
    files = [f for f in files if os.path.exists(f[:-5] + ".npz")]
    with Pool(12) as pool:
        rows = [r for rs in pool.map(one, files) for r in rs]
    keys = list(rows[0])
    with open(os.path.join(OUT, "wrench_margin.csv"), "w") as fh:
        fh.write(",".join(keys) + "\n")
        for r in rows:
            fh.write(",".join(str(r[k]) for k in keys) + "\n")
    print(len(rows), "hand-ticks")
