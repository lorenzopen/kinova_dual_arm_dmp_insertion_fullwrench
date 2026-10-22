"""Reachability of commanded flange poses by inverse kinematics on the controller's URDF.
numpy only (no pinocchio): the chain base_link -> ee_frame is read from the URDF.

It answers ONE question: is there a joint configuration within the joint limits that
puts the ee frame at each pose? It does not check collisions (arm-arm, arm-table) and
says nothing of the elbow the impedance controller will choose (no null-space
stiffness): it reports the lowest joint height so a wrist or elbow near the table shows.
"""
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation as Rot

# Kinova Gen3 "Home" (deg): the seed of the first pose
HOME_DEG = [0.0, 15.0, 180.0, 230.0, 0.0, 55.0, 90.0]
# The real Gen3 7-DoF: joints 1, 3, 5, 7 turn without limit; 2, 4, 6 are limited (Kinova
# user guide). Some lab URDFs put +-2.24 rad on every joint: that would fail poses the
# robot reaches (the 2026-10-16 route), so these override the URDF by joint name.
GEN3_LIMITS_DEG = {"joint_1": None, "joint_2": 128.9, "joint_3": None, "joint_4": 147.8,
                   "joint_5": None, "joint_6": 120.3, "joint_7": None}


def _T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = Rot.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = xyz
    return T


class Chain:
    def __init__(self, urdf, base="base_link", tip="tool_frame"):
        root = ET.parse(urdf).getroot()
        by_child = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0").split() if o is not None else [0, 0, 0])]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0").split() if o is not None else [0, 0, 0])]
            ax = j.find("axis")
            lim = j.find("limit")
            name, kind = j.get("name"), j.get("type")
            lo = float(lim.get("lower")) if lim is not None and lim.get("lower") else -np.inf
            hi = float(lim.get("upper")) if lim is not None and lim.get("upper") else np.inf
            if name in GEN3_LIMITS_DEG:
                d = GEN3_LIMITS_DEG[name]
                lo, hi = (-np.inf, np.inf) if d is None else (-np.radians(d), np.radians(d))
                kind = "continuous" if d is None else "revolute"
            by_child[j.find("child").get("link")] = dict(
                parent=j.find("parent").get("link"), type=kind, T=_T(xyz, rpy),
                axis=np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else None,
                lo=lo, hi=hi)
        chain, link = [], tip
        while link != base:
            if link not in by_child:
                raise ValueError(f"{tip} is not below {base} in {urdf}")
            chain.append(by_child[link])
            link = by_child[link]["parent"]
        self.joints = chain[::-1]
        self.act = [j for j in self.joints if j["type"] in ("revolute", "continuous")]
        self.lo = np.array([j["lo"] if j["type"] == "revolute" else -np.inf for j in self.act])
        self.hi = np.array([j["hi"] if j["type"] == "revolute" else np.inf for j in self.act])

    def fk(self, q):
        """(4x4 of the tip, [positions of every joint origin])."""
        T, k, pts = np.eye(4), 0, []
        for j in self.joints:
            T = T @ j["T"]
            if j["type"] in ("revolute", "continuous"):
                R = np.eye(4)
                R[:3, :3] = Rot.from_rotvec(j["axis"] * q[k]).as_matrix()
                T = T @ R
                k += 1
            pts.append(T[:3, 3].copy())
        return T, np.array(pts)

    def jac(self, q, eps=1e-6):
        T0, _ = self.fk(q)
        J = np.zeros((6, len(q)))
        for i in range(len(q)):
            dq = q.copy()
            dq[i] += eps
            T1, _ = self.fk(dq)
            J[:3, i] = (T1[:3, 3] - T0[:3, 3]) / eps
            J[3:, i] = Rot.from_matrix(T1[:3, :3] @ T0[:3, :3].T).as_rotvec() / eps
        return J

    def ik(self, p, R, q0, iters=200, tol_p=1e-4, tol_r=1e-3):
        """Bounded least squares on (position, 0.2 m x rotation vector)."""
        from scipy.optimize import least_squares
        lo = np.where(np.isfinite(self.lo), self.lo, -np.inf)
        hi = np.where(np.isfinite(self.hi), self.hi, np.inf)
        q = np.clip((np.array(q0, float) + np.pi) % (2 * np.pi) - np.pi, lo + 1e-6, hi - 1e-6)

        def res(x):
            T, _ = self.fk(x)
            return np.concatenate([p - T[:3, 3], 0.2 * Rot.from_matrix(R @ T[:3, :3].T).as_rotvec()])
        q = least_squares(res, q, bounds=(lo, hi), max_nfev=iters, xtol=1e-10, ftol=1e-12).x
        T, pts = self.fk(q)
        err_p = float(np.linalg.norm(p - T[:3, 3]))
        err_r = float(np.linalg.norm(Rot.from_matrix(R @ T[:3, :3].T).as_rotvec()))
        return q, err_p, err_r, pts


def check_path(chain, poses, tol_p=0.002, tol_r=0.02, seeds=4, rng=None):
    """poses: [(p, R 3x3)] in the base frame, in order. Continuation along the path,
    a few random restarts where it fails. Per pose: (position error [m], rotation
    error [rad], lowest joint height [m] or nan). A position error is how far the
    flange cannot go: the impedance then holds it that far from its target."""
    rng = rng or np.random.default_rng(0)
    q = np.radians(HOME_DEG)
    lo = np.where(np.isfinite(chain.lo), chain.lo, -np.pi)
    hi = np.where(np.isfinite(chain.hi), chain.hi, np.pi)
    out = []
    for p, R in poses:
        best = chain.ik(p, R, q)
        k = 0
        while (best[1] > tol_p or best[2] > tol_r) and k < seeds:
            cand = chain.ik(p, R, rng.uniform(lo, hi), iters=300)
            if cand[1] + 0.2 * cand[2] < best[1] + 0.2 * best[2]:
                best = cand
            k += 1
        q1, ep, er, pts = best
        q = q1
        out.append((ep, er, float(pts[2:-1, 2].min())))
    return out
