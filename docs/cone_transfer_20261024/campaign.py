"""
The campaign: every object x {rigid, plate} x {full, scalar}, one process per
run, a bounded pool. A finished run (its .json exists) is never redone.

    python campaign.py --workers 12
"""
import argparse
import os
import shlex
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))

# Where a campaign lives. Main study: mu_eff = 1 in runs_mu1/, results here.
# The first pass at mu_eff = 0.49 is kept as a sensitivity:
#   CT_RUNS=runs CT_OUT=sens_mu049 python analyze.py
RUNS = os.path.join(HERE, os.environ.get("CT_RUNS", "runs_ff_m050"))
OUT = os.path.join(HERE, os.environ.get("CT_OUT", "."))
TESI = os.path.expanduser(
    "~/Documents/GitHub/TESI_antiG/lorenzo_data_analysis/10_tripad_passivo")

# (label, population folder, object indices)
POPS = [
    ("cube", os.path.join(HERE, "inputs/pop_cube_com0"), [0]),
    ("cube", os.path.join(HERE, "inputs/pop_cube"), [0]),
    ("cube", os.path.join(HERE, "inputs/pop_cube_com40"), [0]),
    ("jittered", os.path.join(TESI, "pop_jittered_cubes"), range(10)),
    ("deformed", os.path.join(TESI, "try_deformed_cubes_jitter005"), range(10)),
    ("polytope", os.path.join(TESI, "pop_piani_50_100"), range(15)),
]
ARMS = ("rigid", "plate")
# extra run_case arguments for a whole campaign, e.g. CT_EXTRA="--margin-task-scale 0.5"
EXTRA = shlex.split(os.environ.get("CT_EXTRA", ""))
# (command, extra SOCP flag, name suffix); no_bending only with the full wrench
# Default (2026-10-24): full-wrench feed-forward (Wong eq. 48, zero at pickup)
# with the no-bending SOCP and lambda_m = 0.5 (runs_ff_m050/). The margin sweep
# lives in runs_ff/ (0.25) and runs_ff_m075/ (0.75), see margin_sweep.py. The admittance/scalar arms of the earlier pass are
# in runs_mu1/ (CT_RUNS=runs_mu1 with the old VARIANTS to regenerate).
VARIANTS = (("full", [], "_nb"),)


def jobs():
    for label, pop, idx in POPS:
        for i in idx:
            for cmd, extra, suffix in VARIANTS:
                for arm in ARMS:
                    name = f"{os.path.basename(pop)}__{i:03d}__{arm}__{cmd}{suffix}"
                    yield name, ["--pop", pop, "--obj", str(i), "--arm", arm,
                                 "--command", cmd] + extra + EXTRA + [
                                 "--out", os.path.join(RUNS, name)]


def one(job):
    name, args = job
    if os.path.exists(os.path.join(RUNS, name + ".json")):
        return name, "skip"
    log = os.path.join(RUNS, name + ".log")
    with open(log, "w") as fh:
        r = subprocess.run([sys.executable, os.path.join(HERE, "run_case.py")] + args,
                           stdout=fh, stderr=subprocess.STDOUT, cwd=HERE,
                           env=dict(os.environ, OMP_NUM_THREADS="1",
                                    OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1"))
    return name, r.returncode


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=12)
    a = ap.parse_args()
    os.makedirs(RUNS, exist_ok=True)
    J = list(jobs())
    print(len(J), "runs", flush=True)
    with ThreadPoolExecutor(a.workers) as ex:
        for k, (name, rc) in enumerate(ex.map(one, J)):
            print(f"{k + 1}/{len(J)} {name} {rc}", flush=True)
