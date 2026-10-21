"""The wrist ATIs, checked through the SAME code `insert` uses (ros_backend). NO motion.

    python -m kinova_dual_arm_dmp_insertion.ft_check                     # live readings
    python -m kinova_dual_arm_dmp_insertion.ft_check --calibrate --save  # find ft_in_ee, ft_sign
    python -m kinova_dual_arm_dmp_insertion.ft_check --tool-mass 0.080 0.082 --pad-x 0.070 --save

--save writes what was measured on THIS robot PC to config/local.yaml (not in git,
read on top of cell.yaml by every command): ft_in_ee / ft_sign from --calibrate,
the weighed tool mass per arm (what hangs on each sensor), the tool's pad_x.

Both arms in home, still, both kinova_controller and both ft_raw running. It reads
the NetBox units (lbf -> N), tares with nothing touching and prints the noise.

--calibrate, per arm, three short holds by hand (each averaged over 1 s):
    nothing touching | press the tri-pad face towards its flange | press the
    tri-pad DOWN (towards the floor), at its front, away from the sensor.
From the two forces it solves the sensor's rotation in end_effector_link
(`ft_in_ee`), from the moment of the down press (its lever) the sign
(`ft_sign`); it also says if the two sensors are swapped. Run it again without --calibrate: live, each press must
read on its own arm, with the signs below.

Then live, twist the tri-pad CLOCKWISE as seen from its face: Tn must go positive.

Live, per arm (hand ON the piece, as the controller sees it):
    fn   along the push axis; pressing the pad face gives fn > 0, about the push
    Tn   moment about the push axis; a clockwise twist seen from the face gives Tn > 0
    F    shelf frame [x, y, z]; pressing the tool down gives F z > 0
"""
import argparse
import os
import time

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from .main import LOCAL, load_config, merge

SIDES = (("L", "left"), ("R", "right"))


def triad(a, b):
    """Orthonormal frame (columns) from two non-parallel vectors."""
    t1 = a / np.linalg.norm(a)
    t2 = np.cross(a, b)
    t2 /= np.linalg.norm(t2)
    return np.column_stack([t1, t2, np.cross(t1, t2)])


def hold(b, msg, seconds=1.0):
    input(f"  >>> {msg}, keep it, [Enter] ")
    time.sleep(0.3)
    n = int(seconds * 1000)
    time.sleep(seconds)
    return {s: b.arms[s].ft_last(n).mean(0) for s, _ in SIDES}


def calibrate(b):
    out = {}
    for s, name in SIDES:
        arm = b.arms[s]
        other = "R" if s == "L" else "L"
        print(f"\n  --- {name} arm ---")
        z = hold(b, f"{name}: NOTHING touching either tool")
        pr = hold(b, f"{name}: PRESS the tri-pad face towards its flange (5-10 N)")
        dn = hold(b, f"{name}: PRESS the tri-pad DOWN, towards the floor, at its front (5-10 N)")
        d_pr, d_dn = pr[s] - z[s], dn[s] - z[s]
        if np.linalg.norm(pr[other] - z[other]) > np.linalg.norm(d_pr[:3]):
            print(f"  !!! the press read MORE on the {dict(SIDES)[other]} sensor: the two ATIs are "
                  f"SWAPPED (ft_topic / ft_raw ip of the two arms)")
            return None
        for what, d in (("press", d_pr), ("down", d_dn)):
            if np.linalg.norm(d[:3]) < 2.0:
                print(f"  !!! {what}: only {np.linalg.norm(d[:3]):.1f} N measured, do it again harder")
                return None
        pe, qe = b._ee_pose(arm)
        e_pr = arm.tool_in_ee.R.apply([-1.0, 0.0, 0.0])          # finger force on the tool, ee axes
        e_dn = qe.inv().apply([0.0, 0.0, -1.0])
        # the down press acts ahead of the sensor (along x_tool): its moment is x_tool x down.
        # A twist about the pad axis could not tell the sign: a flipped sensor and a frame
        # turned 180 deg about (press x down) read the same twist.
        e_mt = np.cross(arm.tool_in_ee.R.apply([1.0, 0.0, 0.0]), e_dn)
        R = triad(e_pr, e_dn) @ triad(d_pr[:3], d_dn[:3]).T
        lever = (R @ d_dn[3:]) @ e_mt / np.linalg.norm(d_dn[:3])
        sign = 1.0 if lever > 0 else -1.0
        if sign < 0:
            R = triad(e_pr, e_dn) @ triad(-d_pr[:3], -d_dn[:3]).T
        if abs(lever) < 0.02:
            print(f"  !!! the down press has a lever of only {1000 * abs(lever):.0f} mm: press at the FRONT "
                  f"of the tri-pad and do it again")
            return None
        fit = [np.degrees(np.arccos(np.clip((R @ (sign * d[:3])) @ e / np.linalg.norm(d[:3]), -1, 1)))
               for d, e in ((d_pr, e_pr), (d_dn, e_dn))]
        q = Rot.from_matrix(R)
        ax = q.as_rotvec()
        print(f"  {name}: press {np.linalg.norm(d_pr[:3]):.1f} N, down {np.linalg.norm(d_dn[:3]):.1f} N "
              f"at {1000 * abs(lever):.0f} mm ahead of the sensor")
        print(f"  {name}: sensor in end_effector_link = {np.degrees(np.linalg.norm(ax)):.1f} deg about "
              f"{np.round(ax / max(np.linalg.norm(ax), 1e-9), 3).tolist()}; the down press is "
              f"{fit[1]:.1f} deg off the solved frame (a few deg: hand pushes; > 15: do it again)")
        out[name] = dict(quat=np.round(q.as_quat(), 4).tolist(), sign=sign)
    print("\n  measured:")
    for name, v in out.items():
        print(f"    {name}:  ft_in_ee quat {v['quat']}   ft_sign {v['sign']:.0f}")
    return out


def save(path, cfg, values):
    """Merge `values` into config/local.yaml next to the config."""
    import yaml
    local = os.path.join(os.path.dirname(os.path.abspath(path)), LOCAL)
    old = {}
    if os.path.isfile(local):
        with open(local, encoding="utf-8") as fh:
            old = yaml.safe_load(fh) or {}
    merge(old, values)
    with open(local, "w", encoding="utf-8") as fh:
        fh.write(f"# measured on this robot PC ({time.strftime('%Y-%m-%d %H:%M')}), read on top of "
                 f"cell.yaml; not in git\n")
        yaml.safe_dump(old, fh, sort_keys=False)
    print(f"  saved in {local}")


def live(b):
    print("  live, Ctrl-C to stop (hand ON the piece: press face -> fn > 0, clockwise twist -> Tn > 0,"
          " press down -> F z > 0)")
    try:
        while True:
            m = b.read()
            line = []
            for s, name in SIDES:
                x = Rot.from_quat(m["q" + s]).apply([1.0, 0.0, 0.0])
                F, T = m["F" + s], m["T" + s]
                line.append(f"{name}: fn {F @ x:+6.2f} N  Tn {T @ x:+6.3f} Nm  F {np.round(F, 2).tolist()}")
            print("  " + " | ".join(line), flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--calibrate", action="store_true", help="solve ft_in_ee and ft_sign from three holds")
    ap.add_argument("--tool-mass", nargs=2, type=float, metavar=("LEFT", "RIGHT"),
                    help="[kg] weighed: what hangs on each sensor (adapter + spring + tri-pad)")
    ap.add_argument("--pad-x", type=float,
                    help="[m] flange to the plane of the sphere centres = flange to the front of the "
                         "spheres minus pad_r (0.015)")
    ap.add_argument("--save", action="store_true", help="write the values to config/local.yaml")
    a = ap.parse_args()
    cfg, path = load_config(a.config)
    print(f"  config: {path}")
    values = {}
    if a.tool_mass:
        values.update(left={"tool_mass": a.tool_mass[0]}, right={"tool_mass": a.tool_mass[1]})
    if a.pad_x is not None:
        values["tool"] = {"pad_x": a.pad_x}
    merge(cfg, values)
    for _, name in SIDES:
        print(f"  {name}: tool_mass {cfg[name].get('tool_mass', 0.0):.3f} kg, ft_in_ee quat "
              f"{cfg[name]['ft_in_ee']['quat']}, ft_sign {cfg[name].get('ft_sign', 1.0):.0f}")
    print(f"  tool: pad_x {cfg['tool']['pad_x']:.4f} m")
    if a.save and values and not a.calibrate:
        save(path, cfg, values)
        return
    for _, name in SIDES:
        if cfg[name].get("wrench_source") != "ft":
            raise SystemExit(f"  {name}: wrench_source is not ft in {path}")
    from .ros_backend import RosBackend
    b = RosBackend(cfg)
    try:
        input("  >>> both arms still, NOTHING touching the tools: [Enter] to tare ")
        b.tare()
        if a.calibrate:
            res = calibrate(b)
            if res and a.save:
                for name, v in res.items():
                    values.setdefault(name, {}).update(ft_sign=v["sign"], ft_in_ee={
                        "xyz": cfg[name]["ft_in_ee"]["xyz"], "quat": v["quat"]})
                save(path, cfg, values)
            elif res:
                print("  not saved: run again with --save")
        else:
            live(b)
    finally:
        b.close()


if __name__ == "__main__":
    main()
