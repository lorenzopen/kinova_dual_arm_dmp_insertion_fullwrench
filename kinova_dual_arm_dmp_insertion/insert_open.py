"""Separate force planning from execution, on the same route and piece:

    python -m kinova_dual_arm_dmp_insertion.insert_open insert W --arm R --slow 3 --offline   # as the paper
    python -m kinova_dual_arm_dmp_insertion.insert_open insert W --arm R --slow 3             # feedforward, SOCP online
    python -m kinova_dual_arm_dmp_insertion.insert_open insert W --arm R --slow 3 --online    # closed loop, gain 0.1
    python -m kinova_dual_arm_dmp_insertion.insert_open insert W --arm R --slow 3 --offline --online --squeeze-gain 0.25
    (`check` instead of `insert`: the same with no robot)

--offline   AS IN THE FRICTION-MARGIN PAPER: the SOCP is solved BEFORE the run, along the whole
            commanded sequence (a dry run, the flanges exactly where commanded), and saved as a
            profile f*(phase, time in phase). During the run nothing is solved: the squeeze is an
            equilibrium-point offset of the two flange targets along the pad axis,
                ds = f* / k_eff           (each hand, beyond the nominal contact with the faces)
            The wrist ATIs are logged, not fed back.
(default)   the same feedforward ds = f*/k_eff, with the SOCP re-solved online every 100 ms on the
            measured flange poses (still no force feedback).
--online    the usual closed loop: the squeeze integrated on the ATI force towards the SOCP target,
            with --squeeze-gain 0.1 unless given (main insert's default is 0.5).
Both flags  SOCP computed offline, ATI feedback online. The offline file retains both complete hand
            wrenches; --wrench-channels forces also corrects their INTERNAL tangential force split.
            These are feedback experiments, not the paper's pure feedforward execution.

k_eff is the stiffness flange -> pad face per hand: the arm's impedance in series with the tool
spring, 1/(1/1000 + 1/2700) = 730 N/m by default (--k-eff). In the feedforward modes the force is
only as right as k_eff and the piece's width (2 mm of width error = 0.7 N per hand); at the end
the median measured/asked is printed with the k_eff that would match it.

In every mode the sensors still trigger EVENTS, not regulation: the safety stops (60 N, tracking),
the jam sentry and the seat by feel. In the feedforward modes the approach does not wait for a
measured touch: the tools close to the nominal faces (the route's opening), then press by
f_pickup / k_eff. --twist only with --online (it is a moment loop).
"""
import contextlib
import csv
import hashlib
import io
import os
import sys
import time

import numpy as np

from . import main as M
from .controller import DS_MAX, PICKUP_N, RELEASE, S_MAX, V_APP, Insertion, Abort

SPRING_K = 2700.0          # [N/m] the tool spring, axial (Lorenzo, 2026-10-21)
WRENCH_COLS = [f"wdes{s}_{c}" for s in "LR" for c in M.W6]


def series_stiffness(arm_k, spring_k=SPRING_K):
    """Axial stiffness only. Lateral and rotational stiffness are not inferred from this."""
    if not np.isfinite(arm_k) or not np.isfinite(spring_k) or min(arm_k, spring_k) <= 0:
        raise ValueError("arm and spring stiffness must be finite and positive")
    return 1.0 / (1.0 / arm_k + 1.0 / spring_k)


def effective_stiffness(cfg, backend, override=None):
    if override is not None:
        if not np.isfinite(override) or override <= 0:
            raise ValueError("--k-eff must be finite and positive")
        return float(override)
    spring_k = float(cfg.get("tool", {}).get("spring_axial_N_per_m", SPRING_K))
    # On hardware use the stiffness actually read from both controllers, not a YAML
    # label that cannot change their launch parameters. A scalar model requires the
    # same isotropic translational stiffness on the two arms.
    actual = getattr(backend, "controller_parameters", {})
    values = [actual.get(s, {}).get("cart_k") for s in ("L", "R")]
    if all(v is not None for v in values):
        trans = np.array([v[:3] for v in values], float)
        if not np.allclose(trans, trans[0, 0], rtol=1e-4, atol=1e-6):
            raise ValueError("feedforward needs equal isotropic arm stiffness; specify a calibrated --k-eff")
        return series_stiffness(float(trans[0, 0]), spring_k)
    if actual:
        raise ValueError("cannot read both arm stiffnesses: use an explicitly calibrated --k-eff")
    if hasattr(backend, "arms"):
        raise ValueError("arm stiffness unavailable: use an explicitly calibrated --k-eff")
    return series_stiffness(float(cfg["stiffness"]["trans"]), spring_k)


def wrench_values(ins):
    return np.concatenate([np.concatenate(ins.w_des[s]) if s in ins.w_des else np.full(6, np.nan)
                           for s in "LR"])


class OpenInsertion(Insertion):
    """Feedforward squeeze; `profile` = None: the SOCP online, else the offline f*(phase, t)."""

    def __init__(self, route, backend, k_eff=730.0, profile=None, **kw):
        kw["full_wrench"] = False                       # the twist channel is a moment loop
        if profile is not None:
            kw["async_socp"] = False
        super().__init__(route, backend, **kw)
        self.hand_wrench = False                        # also for historical hwd routes: pure feedforward
        self.squeeze_feedback = False
        self.k_eff = float(k_eff)
        if not np.isfinite(self.k_eff) or self.k_eff <= 0:
            raise ValueError("k_eff must be finite and positive")
        self.s_pick = 0.0
        self.profile = profile
        self.ratio = []                                 # measured / asked, while carrying
        self.trace = []                                 # phase, phase time, normal target, full hand wrenches
        self.execution_mode = "offline_feedforward" if profile is not None else "online_socp_feedforward"

    def approach(self, seconds=None):
        """To the nominal contact: the pad fronts on the piece's faces (the route's opening)."""
        self.phase = "approach"
        d = float(self.r.c["open_clear"])
        secs = max(d / V_APP, 0.5)
        t0 = self.b.now()
        while True:
            u = min(1.0, (self.b.now() - t0) / secs)
            self.tick(self.cL + u * d * self.axis, self.qL, self.cR - u * d * self.axis, self.qR, 3.0)
            if u >= 1.0:
                break
        self.cL, self.cR = self.cL + d * self.axis, self.cR - d * self.axis

    def squeeze(self, force_n, seconds=1.0):
        """Press by force_n / k_eff per hand, ramped; the ATI only watches."""
        self.phase = "squeeze"
        s = float(force_n) / self.k_eff
        t0 = self.b.now()
        while True:
            u = min(1.0, (self.b.now() - t0) / seconds)
            self.tick(self.cL + u * s * self.axis, self.qL, self.cR - u * s * self.axis, self.qR, 8.0)
            if u >= 1.0:
                break
        self.cL, self.cR = self.cL + s * self.axis, self.cR - s * self.axis
        self.s_pick = s

    def _prepare_control(self):
        if self.profile is None:
            super()._prepare_control()
        else:
            self.target, self.w_des = self.profile.sample("to_mouth", 0.)
            self.socp_status = True
            self.t_ctrl = self.b.now()

    def _control(self, now):
        if self.profile is None:                         # the SOCP online, as `insert`
            super()._control(now)
            self.trace.append((self.phase, self._phase_time(now), float(self.target), *wrench_values(self)))
            return
        # offline: the target from the profile, nothing solved now
        self.target, self.w_des = self.profile.sample(self.phase, self._phase_time(now))
        self.socp_status = True
        self._squeeze_update()
        self.stats["asked"].append(self.target)

    def _squeeze_update(self):
        """ds = f*/k_eff (beyond the pickup press already in the frame), slew-limited."""
        want = float(np.clip(self.target / self.k_eff - self.s_pick, -RELEASE, S_MAX))
        self.ds = float(np.clip(self.ds + np.clip(want - self.ds, -DS_MAX, DS_MAX), -RELEASE, S_MAX))
        if self.phase in ("route", "to_mouth", "slow_seat") and self.target > 0.5:
            self.ratio.append(self.normal_force() / (2.0 * self.target))

    def run(self, from_mouth=False):
        out = super().run(from_mouth)
        if self.ratio:
            r = float(np.median(self.ratio))
            print(f"  FEEDFORWARD: measured / asked squeeze, median {r:.2f} while carrying "
                  f"(model k_eff {self.k_eff:.0f} N/m; this ratio does not identify stiffness "
                  "when contact geometry or piece width changes)")
        return out


class NominalPlanner(OpenInsertion):
    """Use the toy run for pacing, but place the SOCP contacts on the nominal faces.

    Equilibrium-point closure includes virtual arm deflection and spring preload;
    it is not a measured contact position. Removing it avoids planning forces on
    points artificially driven inside the object by the previous force command.
    """
    def _socp_inputs(self):
        axis = self.R_cmd.apply(self.frame["ax"])
        closure = (self.s_pick + self.ds) * axis
        geometry = dict(self.meas, pL=np.asarray(self.meas["pL"]) - closure,
                        pR=np.asarray(self.meas["pR"]) + closure)
        return super()._socp_inputs(geometry)


class Profile:
    """Complete hand wrench plan, linear within each phase and held at its endpoints.
    A missing phase is an invalid plan, not permission to apply a different phase's wrench.
    """
    ORDER = ("to_mouth", "route", "slow_seat", "seat_by_feel", "release")

    def __init__(self, trace, path=None):
        self.path = path
        self.tab = {}
        for ph in self.ORDER:
            pts = [row[1:] for row in trace if row[0] == ph]
            if pts:
                pts = np.array(pts, float)
                if pts.shape[1] != 14 or not np.all(np.isfinite(pts)):
                    raise ValueError(f"offline {ph}: missing or non-finite full wrench")
                if np.any(np.diff(pts[:, 0]) <= 0):
                    raise ValueError(f"offline {ph}: timestamps must increase")
                self.tab[ph] = (pts[:, 0], pts[:, 1:])
        if not self.tab:
            raise ValueError("empty offline profile")

    def __call__(self, phase, t):
        return self.sample(phase, t)[0]

    def require_phases(self, phases):
        missing = [phase for phase in phases if phase not in self.tab]
        if missing:
            raise Abort("offline profile lacks required motion phases: " + ", ".join(missing))

    def sample(self, phase, t):
        if phase not in self.tab:
            raise Abort(f"offline profile has no {phase!r} phase: refusing to reuse an unrelated wrench")
        ts, values = self.tab[phase]
        v = np.array([np.interp(t, ts, col) for col in values.T])
        return float(v[0]), {"L": (v[1:4], v[4:7]), "R": (v[7:10], v[10:13])}


class ProfileInsertion(Insertion):
    """Fixed offline force plan, existing ATI feedback and measured-touch pickup.

    The normal loop and selected internal wrench channels are unchanged. No unknown
    lateral spring stiffness is used to turn tangential force into feedforward pose.
    """
    def __init__(self, route, backend, profile, **kw):
        kw["async_socp"] = False
        super().__init__(route, backend, **kw)
        self.profile = profile
        self.execution_mode = "offline_socp_ati_feedback"

    def _prepare_control(self):
        self.target, self.w_des = self.profile.sample("to_mouth", 0.)
        self.socp_status = True
        self.t_ctrl = self.b.now()

    def _control(self, now):
        self.target, self.w_des = self.profile.sample(self.phase, self._phase_time(now))
        self.socp_status = True
        self._squeeze_update()
        self.stats["asked"].append(self.target)
        if self.hand_wrench and self.w_des:
            self._chase_wrench()


def offline_profile(a, r, k_eff, cfg):
    """SOCP on nominal contact faces along the sequence, saved to logs/.
    The toy backend supplies pacing/events; virtual preload is removed from contact geometry.
    """
    from .dryrun import DryRun
    b = DryRun(r, period=1.0 / cfg["rate_hz"], k_face=k_eff)
    ins = M.slowed(NominalPlanner(r, b, k_eff=k_eff, confirm=lambda msg: None,
                                 **M.socp_kwargs(cfg)), a.slow)
    t0 = time.perf_counter()
    with contextlib.redirect_stdout(io.StringIO()):     # its own RESULT is not the run's
        ins.run()
    if ins.stats["socp_ok"] != ins.stats["socp_ticks"]:
        raise Abort(f"offline planning failed: {ins.stats['socp_ok']}/{ins.stats['socp_ticks']} feasible solves; "
                    "no profile will be executed")
    prof = Profile(ins.trace)
    prof.require_phases(("to_mouth", "route", "slow_seat", "seat_by_feel"))
    os.makedirs(cfg["log_dir"], exist_ok=True)
    path = os.path.join(cfg["log_dir"], time.strftime(f"{r.world}_{a.arm}_offline_profile_%Y%m%d_%H%M%S.csv"))
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["phase", "t_in_phase", "target_N", *WRENCH_COLS])
        w.writerows((row[0], *(f"{x:.6f}" for x in row[1:])) for row in ins.trace)
    prof.path = os.path.abspath(path)
    prof.k_eff = k_eff
    prof.socp_ok, prof.socp_ticks = ins.stats["socp_ok"], ins.stats["socp_ticks"]
    with open(path, "rb") as fh:
        prof.sha256 = hashlib.sha256(fh.read()).hexdigest()
    route = prof.tab.get("route", (np.zeros(1), np.zeros((1, 13))))[1][:, 0]
    print(f"  OFFLINE PROFILE: {ins.stats['socp_ok']}/{ins.stats['socp_ticks']} feasible allocations, "
          f"{len(ins.trace)} profile samples, in {time.perf_counter() - t0:.1f} s "
          f"before the run (route: {route.min():.1f}..{route.max():.1f} N/hand) -> {path}")
    return prof


def main():
    mode = ("offline_online" if "--offline" in sys.argv and "--online" in sys.argv else
            "offline" if "--offline" in sys.argv else "online" if "--online" in sys.argv else "open")

    def extra(ap):
        ap.add_argument("--offline", action="store_true", help="SOCP solved before the run (complete hand wrenches)")
        ap.add_argument("--online", action="store_true", help="ATI feedback; combine with --offline for a fixed force plan")
        ap.add_argument("--k-eff", type=float, default=None,
                        help="[N/m] calibrated axial stiffness per hand (default: actual arm cart_k on hardware, "
                             f"cell stiffness.trans in dry run, in series with the {SPRING_K:.0f} N/m tool spring)")
        ap.set_defaults(log_tag=f"_{mode}", squeeze_gain=0.1)

    def make(a, r, b, **kw):
        cfg = M.load_config(a.config)[0]
        if a.no_bending:
            cfg.setdefault("socp", {})["no_bending"] = True
        if a.online:
            print(f"  SQUEEZE CLOSED ON THE ATI (online), gain x{kw.get('squeeze_gain', a.squeeze_gain):g}")
            if a.offline:
                # k_eff drives the precomputation's toy pickup/execution, not its contact
                # geometry. Real execution uses measured touch and ATI feedback, not f/k_eff.
                k = effective_stiffness(cfg, b, a.k_eff)
                prof = offline_profile(a, r, k, cfg)
                return ProfileInsertion(r, b, profile=prof, **kw)
            ins = Insertion(r, b, **kw)
            ins.execution_mode = "online_socp_ati_feedback"
            return ins
        if kw.get("full_wrench") or r.hand_wrench != "off":
            print("  pure feedforward: all ATI wrench regulation disabled; combine with --online for feedback")
        k = effective_stiffness(cfg, b, a.k_eff)
        pick = min(r.derived_N, PICKUP_N)
        print(f"  SQUEEZE IN FEEDFORWARD ({'SOCP offline, as the paper' if a.offline else 'SOCP online'}): "
              f"k_eff {k:.0f} N/m; pickup {pick:.1f} N/hand = {1000 * pick / k:.1f} mm past the nominal "
              f"contact; the ATIs are logged, not fed back")
        prof = offline_profile(a, r, k, cfg) if a.offline else None
        return OpenInsertion(r, b, k_eff=k, profile=prof, **kw)

    M.main(sys.argv[1:], doc=__doc__, extra=extra, make=make)


if __name__ == "__main__":
    main()
