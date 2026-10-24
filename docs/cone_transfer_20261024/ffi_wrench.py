"""
Full-wrench feed-forward (default, `g_int=0`): Wong et al. eq. 48 with the
zero-force pose taken from the measured pickup, as `insert_ffi` does.
With `g_int > 0` it adds an integral trim on all six components (FF+I); with
`k_i_squeeze > 0` only on each hand's normal force, along the push axis
(FF + squeeze I, as insert_ffi / Wong's squeeze PID).

Per hand k, with W the wrench ON THE OBJECT about that wrist (6-vector,
world frame) and K_eff the 6x6 stiffness between commanded wrist pose and
contact:

    u_ff  = K_eff^-1 (W*_k - W_pick,k)       nominal, relative to the pickup
    i    += g K_eff^-1 (W*_k - W_k) dt        integral trim, |i| bounded
    ghost = commanded pose + u_ff + i

  * W_pick is the wrench measured at the first tick with both hands in
    contact, as `insert_ffi` records N_pick: the pose that produced it is the
    reference, so only the CHANGE asked by the SOCP goes through K_eff^-1
    (docs/FFI_CONTROLLER.md). It is stored in the hand frame and carried with
    the grasp.
  * K_eff = wrist virtual coupling (kp, kpr, k_tilt) for the fixed tri-pad;
    for the floating plate it is that coupling in SERIES with the plate's own
    6x6 (floating_plate.stiffness_6x6, transported from the plate centre to
    the wrist), the 6-D analogue of k_eff = (1/K + 1/k_spring)^-1.
  * g = 1/0.45 s: the closed-loop time constant `insert_ffi` was tuned to
    (k_L K_I tau = 0.5). The integral trim is limited to 3 mm / 0.05 rad, the
    robot's +-3 mm trim; the total bias is bounded at 50 mm / 0.35 rad with
    HandWrench's rates (10 mm/s, 0.1 rad/s). After the bounds the integral state is
    advanced only by the fraction of the increment actually applied, while
    the feed-forward keeps its target (anti-windup, as in insert_ffi).
  * A rejected allocation, or a hand that lost all contact, freezes both
    terms: no target is streamed, as on the robot.

Everything else (where it is called, phases, tracing) is TESI's HandWrench.
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from lib.hand_wrench import HandWrench, shift, wrist_stiffness, bounded_increment
from lib.tool import wrench_on_body
import floating_plate as FP

G_INT = 1.0 / 0.45          # [1/s]
I_LIM = (0.003, 0.05)       # [m], [rad] integral trim per hand
# Total bias per hand. Wong's eq. 48 bounds nothing; TESI's HandWrench caps at
# 20 mm / 0.15 rad, and the feed-forward sat on the 0.15 rad cap for both tools
# (wanted 8.8 deg fixed, 12.6 deg plate, cube). Wider caps keep it a safety
# bound rather than the operating point; the rate limits are unchanged.
TOTAL_LIM = (0.050, 0.35)   # [m], [rad]


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def _clip(v, lim):
    n = float(np.linalg.norm(v))
    return v * (lim / n) if n > lim else v


class HandWrenchFFI(HandWrench):
    def __init__(self, tools, dt, kp, kpr, k_tilt, fallback_N=21.0, g_int=0.0,
                 k_i_squeeze=0.0):
        super().__init__("full", tools, dt, fallback_N=fallback_N, kp=kp,
                         kpr=kpr, k_tilt=k_tilt, law="integral")
        self.law = ("ffi" if g_int > 0 else
                    "ff_isqueeze" if k_i_squeeze > 0 else "ff")
        self.g_int = float(g_int)
        # [m/(N s)] integral on each hand's summed normal force, along the
        # push axis only: insert_ffi's I (1.6 mm/(N s), +-3 mm), per hand
        self.k_i_squeeze = float(k_i_squeeze)
        self.pick_local = None               # (2,6) hand frame
        self.i_local = np.zeros((2, 6))      # integral trim, hand frame
        self.stats = dict(ticks=0, accepted=0, frozen=0, i_limited=0,
                          limited=0, F=[], T=[], trans=[], rot=[], ff_rot=[],
                          N=[], i_sq=[])

    # ------------------------------------------------------------ stiffness
    def k_eff(self, tool, R, base):
        Kw = wrist_stiffness(tool, R, self.kp, self.kpr, self.k_tilt)
        if getattr(tool, "plate_id", None) is None:
            return Kw
        # plate 6x6 in the tool frame about the plate centre -> world, wrist
        Kp_local = FP.stiffness_6x6(tool.pads_spring, tool.centre)
        B = np.zeros((6, 6)); B[:3, :3] = B[3:, 3:] = R
        Cp = B @ np.linalg.inv(Kp_local) @ B.T
        c = base + R @ np.array([FP.PLATE_X, 0.0, 0.0])
        A = np.eye(6); A[3:, :3] = _skew(base - c)      # W_c = A W_base
        Cp_base = A.T @ Cp @ A
        return np.linalg.inv(np.linalg.inv(Kw) + Cp_base)

    @staticmethod
    def _to_world(R, v):
        return np.concatenate([R @ v[:3], R @ v[3:]])

    @staticmethod
    def _to_local(R, v):
        return np.concatenate([R.T @ v[:3], R.T @ v[3:]])

    # ------------------------------------------------------------ update
    def update(self, body, result, contacts, phase, orientation):
        if not self.enabled:
            return
        self.advect(orientation)
        poses = [t.pose() for t in self.tools]
        bases = [np.asarray(q[0], float) for q in poses]
        Rs = [Rot.from_quat(q[1]).as_matrix() for q in poses]
        measured = [np.concatenate(wrench_on_body(body, [t], b))
                    for t, b in zip(self.tools, bases)]
        both = all(any(bool(c[0]) for c in hand) for hand in contacts)
        accepted = result is not None and bool(result["ok"])
        s = self.stats
        s["ticks"] += 1
        if not both:
            s["frozen"] += 1
            return
        if self.pick_local is None:
            self.pick_local = np.array([self._to_local(R, m) for R, m in zip(Rs, measured)])
        if not accepted:
            s["frozen"] += 1
            return
        s["accepted"] += 1
        target = np.zeros((2, 6))
        for k, key in enumerate(("f_L", "f_R")):
            for f, c in zip(result[key], contacts[k]):
                f = np.asarray(f, float)
                at = np.asarray(c[1] if c[0] else bases[k], float)
                target[k, :3] += f
                target[k, 3:] += np.cross(at - bases[k], f)
        new = np.zeros((2, 6))
        ff_w = np.zeros((2, 6))
        for k, tool in enumerate(self.tools):
            R = Rs[k]
            K = self.k_eff(tool, R, bases[k])
            Kinv = np.linalg.inv(K)
            pick = self._to_world(R, self.pick_local[k])
            ff_w[k] = Kinv @ (target[k] - pick)
            err = target[k] - measured[k]
            s["F"].append(float(np.linalg.norm(err[:3])))
            s["T"].append(float(np.linalg.norm(err[3:])))
            di = self.dt * self.g_int * (Kinv @ err)
            if self.k_i_squeeze > 0:
                n_err = float(sum(result["fn_L" if k == 0 else "fn_R"])) - tool.normal_force(body)
                s["N"].append(abs(n_err))
                di = di + self._to_world(R, np.r_[self.dt * self.k_i_squeeze * n_err, np.zeros(5)])
            i_w = self._to_world(R, self.i_local[k]) + di
            i_c = np.concatenate([_clip(i_w[:3], I_LIM[0]), _clip(i_w[3:], I_LIM[1])])
            s["i_limited"] += int(not np.allclose(i_c, i_w))
            new[k] = ff_w[k] + i_c
        previous = self.offset.copy()
        self.offset, scale = bounded_increment(self.offset, new - previous, self.dt,
                                               max_translation=TOTAL_LIM[0],
                                               max_rotation=TOTAL_LIM[1])
        s["limited"] += int(scale < 1.0 - 1e-12)
        s["trans"].append(float(np.linalg.norm(self.offset[:, :3], axis=1).max()))
        s["rot"].append(float(np.linalg.norm(self.offset[:, 3:], axis=1).max()))
        s["ff_rot"].append(float(np.linalg.norm(ff_w[:, 3:], axis=1).max()))
        # anti-windup: the feed-forward keeps its target across rate limits;
        # only the integral is advanced by the fraction actually applied
        for k in range(2):
            i_prev = self._to_world(Rs[k], self.i_local[k])
            i_now = i_prev + scale * (new[k] - ff_w[k] - i_prev)
            self.i_local[k] = self._to_local(Rs[k], i_now)
        s["i_sq"].append(float(np.abs(self.i_local[:, 0]).max()))
        self.tick += 1

    def summary(self):
        s = self.stats
        med = lambda v: float(np.median(v)) if v else None      # noqa: E731
        return dict(hand_wrench_mode="full", hand_wrench_law=self.law,
                    ffi_ticks=s["ticks"], ffi_accepted=s["accepted"],
                    ffi_frozen=s["frozen"], ffi_limited=s["limited"],
                    ffi_i_limited=s["i_limited"],
                    ffi_F_error_median_N=med(s["F"]),
                    ffi_T_error_median_Nm=med(s["T"]),
                    ffi_N_error_median_N=med(s["N"]),
                    ffi_i_squeeze_median_mm=1000 * (med(s["i_sq"]) or 0.0),
                    ffi_trans_median_mm=1000 * (med(s["trans"]) or 0.0),
                    ffi_rot_median_deg=float(np.degrees(med(s["rot"]) or 0.0)),
                    ffi_ff_rot_wanted_median_deg=float(np.degrees(med(s["ff_rot"]) or 0.0)))
