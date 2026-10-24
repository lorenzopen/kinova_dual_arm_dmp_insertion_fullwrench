"""
CoM from the contact wrench, the simple way, checked on recorded runs.

  mass   Wong et al. eq. 22: m = F_z / g on the level samples after lift-off.
  x, y   Wong et al. eq. 26-29: [m g]x r = s on the level samples. With g
         vertical the matrix has rank 2: the two horizontal components only.
  z      one scalar fit over the wielding roll theta about the grasp axis e_x:
             s(theta) . e_x / (m g) = b_y(theta) r_y + b_z(theta) r_z
         with b(theta) = (-z_world) x (R(theta) R_0 e_k) . e_x, i.e. for a
         pure roll  s_x/(m g) = r_y cos(theta) + r_z sin(theta)  (up to sign).
         r_x is taken from the level pose; r_y, r_z from the torsion curve.
         This is the smallest case of the static load identification of
         Atkeson, An & Hollerbach (1986): two gravity directions suffice.

Frame: pole = midpoint of the wrists at the last level tick, e_x = grasp
axis (left -> right), e_z = world up orthogonalised, e_y = e_z x e_x.

CHECK ONLY: theta is read here from the object's attitude, i.e. ground truth
the robot does not have. In the pipeline it has to come from the hand roll
(plate attitude for the floating plate). The true CoM is used only to score.

    python com_simple.py [runs_dir]
"""
import glob
import json
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot

G = np.array([0.0, 0.0, -9.81])
DOWN = np.array([0.0, 0.0, -1.0])


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def wrench(z, t):
    """Net force on the object and its moment about the wrist midpoint."""
    pole = z["tool_pos"][t].mean(0)
    F, P = np.nan_to_num(z["F"][t]), np.nan_to_num(z["P"][t])
    return F.sum(0), np.cross(P - pole, F).sum(0), pole


def estimate(z, level, wield, theta):
    t0 = level[-1]
    tp = z["tool_pos"][t0]
    ex = (tp[1] - tp[0]) / np.linalg.norm(tp[1] - tp[0])
    ez = np.array([0, 0, 1.0]) - ex[2] * ex
    ez /= np.linalg.norm(ez)
    R0 = np.c_[ex, np.cross(ez, ex), ez]
    F = np.array([wrench(z, t)[0] for t in level])
    m = float(F[:, 2].mean() / 9.81)                                  # eq. 22
    S = np.concatenate([wrench(z, t)[1] for t in level])
    r_w = np.linalg.lstsq(np.vstack([skew(m * G)] * len(level)), S, rcond=None)[0]
    r_x, r_y0 = (R0.T @ r_w)[:2]                                      # eq. 29
    sx = np.array([wrench(z, t)[1] @ ex for t in wield]) / (m * 9.81)
    B = np.array([[np.cross(DOWN, (Rot.from_rotvec(a * ex).as_matrix() @ R0)[:, k]) @ ex
                   for k in range(3)] for a in theta])
    r_y, r_z = np.linalg.lstsq(B[:, 1:], sx - B[:, 0] * r_x, rcond=None)[0]
    return m, np.array([r_x, r_y0, r_z]), np.array([r_x, r_y, r_z]), R0, wrench(z, t0)[2]


def main(runs):
    rows = []
    for f in sorted(glob.glob(os.path.join(runs, "*.npz"))):
        meta = json.load(open(f[:-4] + ".json"))
        if meta["stage"] != "DONE":
            continue
        with np.load(f) as zz:
            z = {k: zz[k] for k in zz.files}
        air = (z["ext_N"] < 0.05)
        level = np.flatnonzero(air & (z["phase"] == 1))[-20:]
        wield = np.flatnonzero(air & (z["phase"] == 2))
        if len(level) < 5 or len(wield) < 50:
            continue
        q0 = Rot.from_quat(z["obj_quat"][level[-1]])
        tp = z["tool_pos"][level[-1]]
        ex = (tp[1] - tp[0]) / np.linalg.norm(tp[1] - tp[0])
        theta = np.array([(Rot.from_quat(z["obj_quat"][t]) * q0.inv()).as_rotvec() @ ex
                          for t in wield])
        m, r_lvl, r, R0, pole = estimate(z, level, wield, theta)
        true = R0.T @ (z["obj_pos"][level[-1]] - pole)
        rows.append(dict(run=os.path.basename(f)[:-4], arm=meta["arm"],
                         m_err=abs(m - meta["mass_true"]) / meta["mass_true"],
                         err=1000 * (r - true), swing=np.degrees(np.ptp(theta))))
    err = np.array([r["err"] for r in rows])
    print(f"{len(rows)} completed runs; |error| [mm] x, y, z")
    print("  median", np.round(np.median(np.abs(err), 0), 1),
          " p90", np.round(np.percentile(np.abs(err), 90, 0), 1),
          f"  mass error median {100 * np.median([r['m_err'] for r in rows]):.1f}%")
    for arm in ("rigid", "plate"):
        e = np.array([r["err"] for r in rows if r["arm"] == arm])
        if len(e):
            print(f"  {arm:6s} n={len(e)} median", np.round(np.median(np.abs(e), 0), 1))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else
         os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs_ff_m050"))
