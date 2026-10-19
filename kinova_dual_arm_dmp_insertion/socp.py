"""
The convex program: which pair of FLANGE wrenches to ask for (full-wrench SOCP,
2026-10-19). Same mathematics as `lorenzo_data_analysis/full_wrench_socp/lib/socp.py`,
written so cvxpy compiles it ONCE per set of active pads (DPP).

    w = H f                         the two flange wrenches (what the F/T reads), 12
    min_f  1/2 w' D' Sigma^-1 D w   human precision only (Vitus' cost, on the flanges)
         + k_f |f|^2                optional effort term, k_f = w_force / (m g)^2 (0 = off)
    s.t.   sum f_i + m g = 0, sum (p_i - c) x f_i = 0          equilibrium about the believed CoM
           |(I - n n') f_i| <= (1 - r_s) mu (-n' f_i)          Coulomb cone per point
           [no_bending] no moment across each hand's pad plane

f (18 = three point forces per hand) is ONLY the exact friction model of the
tri-pad: a point carries no moment, so the flange moment comes from the lever
arms. The forces per pad are not measured and not commanded - what goes to the
robot is the flange wrench. No force floor, no cap, no equilibrium slack: an
impossible grasp returns ok=False and the controller's fallback decides.

Why the prior alone is bounded: Sigma^-1 is full rank on all 12 flange
components, the squeeze between the hands included, and with w fixed the pad
forces are bounded by the cones. Measured on the tutorial box and two 2 kg
cases: the flange wrenches equal the effort+prior version to 0.5%.

Interface kept from the old `socp.py`, so `controller.py` changes little:
`solve(...)` returns f_L, f_R, fn_L, fn_R, peak, ok, residuals, eta_max; the
flange wrenches are added as wrench_L / wrench_R. `f_min` is accepted and
ignored (there is no floor).
"""
import numpy as np
import cvxpy as cp

FORCE_TOL = 1e-4     # [N] numerical feasibility, not sensor noise
TORQUE_TOL = 1e-5    # [Nm]
CONTACT_TOL = 1e-4   # [N] cone / compression residuals


def _skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def wrench_map(points, pole):
    """[sum f; sum (p - pole) x f] for point forces stacked [f_1, f_2, ...]."""
    return np.vstack((np.tile(np.eye(3), (1, len(points))),
                      np.hstack([_skew(np.asarray(p, float) - pole) for p in points])))


class GraspSOCP:
    def __init__(self, prior=None, prior_mode="tpgmm", mu=1.0, r_s=0.30,
                 w_force=0.0, no_bending=False, solver_order=("CLARABEL", "ECOS", "SCS"),
                 **_legacy):
        # `_legacy` swallows the old f_pad_max / f_min / r_char / w_slack arguments
        if prior_mode not in ("tpgmm", "identity", "none"):
            raise ValueError("prior_mode must be tpgmm, identity or none")
        if prior_mode == "none" and w_force <= 0:
            raise ValueError("with no prior the effort term is the whole cost: w_force must be > 0")
        if prior is None and prior_mode != "none":
            raise ValueError("a prior is required unless prior_mode is 'none'")
        self.prior, self.prior_mode = prior, prior_mode
        self.mu, self.r_s, self.w_force = float(mu), float(r_s), float(w_force)
        self.no_bending = bool(no_bending)
        self.solver_order = tuple(solver_order)
        self.std_f = float(getattr(prior, "std_f", 12.0))
        self.std_tau = float(getattr(prior, "std_tau", 0.7))
        self.S_inv = np.diag(1.0 / np.array([self.std_f] * 3 + [self.std_tau] * 3
                                             + [self.std_f] * 3 + [self.std_tau] * 3))
        self._cache = {}                    # GMR query per attitude (expensive)
        self._programs = {}                 # compiled program per set of active pads
        self._installed = set(cp.installed_solvers())

    # ------------------------------------------------------------ the prior
    def _root(self, orientation):
        """L with L L' = S Sigma^-1 S (dimensionless), so the prior term is |L' S^-1 D w|^2 / 2."""
        if self.prior is None or self.prior_mode == "none":
            return None
        key = np.round(np.asarray(orientation, float), 3).tobytes()
        if key not in self._cache:
            if len(self._cache) > 512:
                self._cache.clear()
            _mean, _cov, Q = self.prior.query(orientation)      # the mean is NOT used
            S = np.linalg.inv(self.S_inv)
            Q_n = S @ np.asarray(Q, float) @ S
            Q_n = 0.5 * (Q_n + Q_n.T)
            w, V = np.linalg.eigh(Q_n)
            w = np.clip(w, 0.0, None)
            if self.prior_mode == "identity":                   # same total weight, no shape
                L = np.eye(12) * np.sqrt(float(w.sum()) / 12.0)
            else:
                L = V * np.sqrt(w)
            self._cache[key] = L
        return self._cache[key]

    # ------------------------------------------------------------ the program, built once
    def _program(self, aL, aR):
        key = (aL, aR)
        if key in self._programs:
            return self._programs[key]
        active = aL + aR
        f = cp.Variable(18, name="pad_forces")
        P = dict(G=cp.Parameter((6, 18)), target=cp.Parameter(6),
                 n=[cp.Parameter(3) for _ in range(6)],
                 proj=[cp.Parameter((3, 3)) for _ in range(6)])
        con = [P["G"] @ f == P["target"]]
        k = (1.0 - self.r_s) * self.mu
        for i in range(6):
            fi = f[3 * i:3 * i + 3]
            if active[i]:
                comp = -(P["n"][i] @ fi)                        # compression, >= 0
                con += [comp >= 0, cp.norm(P["proj"][i] @ fi, 2) <= k * comp]
            else:
                con += [fi == 0]
        if self.no_bending:
            P["bend"] = [cp.Parameter((3, 9)) for _ in range(2)]
            con += [P["bend"][0] @ f[:9] == 0, P["bend"][1] @ f[9:] == 0]
        cost = 0
        if self.prior is not None and self.prior_mode != "none":
            P["A"] = cp.Parameter((12, 18))
            cost += 0.5 * cp.sum_squares(P["A"] @ f)
        if self.w_force > 0:
            P["kf"] = cp.Parameter(nonneg=True)
            cost += P["kf"] * cp.sum_squares(f)
        prob = cp.Problem(cp.Minimize(cost), con)
        if not prob.is_dcp(dpp=True):
            raise RuntimeError("the SOCP is not DPP: cvxpy would recompile it on every call")
        P.update(prob=prob, f=f)
        self._programs[key] = P
        return P

    # ------------------------------------------------------------ one call
    def solve(self, posL, posR, baseL, baseR, nrmL, nrmR, m_hat, p_com, orientation,
              active_L=None, active_R=None, f_min=None, w_des=None):
        aL = tuple([True] * 3 if active_L is None else [bool(x) for x in active_L])
        aR = tuple([True] * 3 if active_R is None else [bool(x) for x in active_R])
        if not any(aL) or not any(aR):
            return _empty("not a bimanual grasp: one hand is not touching")
        pts = np.asarray([*posL, *posR], float)
        nrm = np.asarray([*nrmL, *nrmR], float)
        bases = np.asarray([baseL, baseR], float)
        pole = np.asarray(p_com, float)
        R = np.asarray(orientation, float)
        m = float(m_hat)
        if pts.shape != (6, 3) or nrm.shape != (6, 3) or m <= 0 or not np.all(np.isfinite(pts)):
            return _empty("invalid geometry or load")
        target = (np.array([0.0, 0.0, 9.81 * m, 0.0, 0.0, 0.0]) if w_des is None
                  else np.asarray(w_des, float))

        H = np.zeros((12, 18))
        H[:6, :9] = wrench_map(pts[:3], bases[0])
        H[6:, 9:] = wrench_map(pts[3:], bases[1])
        P = self._program(aL, aR)
        P["G"].value = wrench_map(pts, pole)
        P["target"].value = target
        for i in range(6):
            n = nrm[i] / max(np.linalg.norm(nrm[i]), 1e-12)
            P["n"][i].value = n
            P["proj"][i].value = np.eye(3) - np.outer(n, n)
        if self.no_bending:
            for h, idx in enumerate(((0, 1, 2), (3, 4, 5))):
                act = [i for i in idx if (aL + aR)[i]]
                centre = pts[act].mean(0)
                nu = nrm[act].mean(0)
                nu /= np.linalg.norm(nu)
                B = np.hstack([_skew(pts[i] - centre) if (aL + aR)[i] else np.zeros((3, 3)) for i in idx])
                P["bend"][h].value = (np.eye(3) - np.outer(nu, nu)) @ B
        L = self._root(R)
        if "A" in P:
            D = np.kron(np.eye(4), R.T)                          # world -> grasp frame
            P["A"].value = L.T @ self.S_inv @ D @ H
        if "kf" in P:
            P["kf"].value = self.w_force / (9.81 * m) ** 2

        errors = []
        for s in self.solver_order:
            if s not in self._installed:
                continue
            try:
                kw = dict(max_iters=20000, eps=1e-6) if s == "SCS" else {}
                P["prob"].solve(solver=s, **kw)
            except cp.error.SolverError as e:
                errors.append(f"{s}: {e}")
                continue
            if P["prob"].status in (cp.INFEASIBLE, cp.INFEASIBLE_INACCURATE):
                return _empty("equilibrium and contact constraints are incompatible", P["prob"].status, s)
            if P["f"].value is None:
                continue
            r = self._check(P["f"].value, pts, nrm, aL + aR, pole, target, H, P["prob"], s)
            if r["ok"]:
                return r
            errors.append(f"{s}: {r['reason']}")
        return _empty("no solver succeeded: " + "; ".join(errors))

    def _check(self, vec, pts, nrm, active, pole, target, H, prob, solver):
        """Recompute every hard constraint from the forces, whatever the status says."""
        F = np.asarray(vec, float).reshape(6, 3)
        active = np.asarray(active, bool)
        res = wrench_map(pts, pole) @ F.ravel() - target
        comp = -np.sum(nrm * F, axis=1)
        tang = np.linalg.norm(F + comp[:, None] * nrm, axis=1)
        k = (1.0 - self.r_s) * self.mu
        cone_err = max(0.0, float(np.max(tang[active] - k * comp[active])))
        tension_err = max(0.0, float(-comp[active].min()))
        rF, rT = float(np.linalg.norm(res[:3])), float(np.linalg.norm(res[3:]))
        ok = bool(prob.status in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE)
                  and rF <= FORCE_TOL and rT <= TORQUE_TOL
                  and max(cone_err, tension_err) <= CONTACT_TOL)
        fn = np.maximum(comp, 0.0)
        w = H @ F.ravel()
        eta = float(np.max(tang[active] / np.maximum(self.mu * comp[active], 1e-12)))
        return dict(status=prob.status, solver=solver,
                    f_L=[F[i] for i in range(3)], f_R=[F[i] for i in range(3, 6)],
                    fn_L=fn[:3].tolist(), fn_R=fn[3:].tolist(),
                    peak=float(max(fn[:3].sum(), fn[3:].sum())),
                    residual_force=rF, residual_torque=rT, eta_max=eta,
                    cone_violation=cone_err, cost=float(prob.value), ok=ok,
                    wrench_L=w[:6], wrench_R=w[6:],
                    reason="" if ok else "returned allocation violates hard constraints")


def _empty(reason, status="not_solved", solver=None):
    z = [np.zeros(3)] * 3
    return dict(status=status, solver=solver, f_L=z, f_R=z, fn_L=[0.0] * 3, fn_R=[0.0] * 3,
                peak=0.0, residual_force=float("nan"), residual_torque=float("nan"),
                eta_max=float("nan"), cost=float("nan"), ok=False, reason=reason)
