"""Three ways to realize the squeeze, to compare on the same world (logs `_offline`, `_open`, `_online`):

    python -m kinova_dual_arm_dmp_insertion.insert_open insert W --arm R --slow 3 --offline   # as the paper
    python -m kinova_dual_arm_dmp_insertion.insert_open insert W --arm R --slow 3             # feedforward, SOCP online
    python -m kinova_dual_arm_dmp_insertion.insert_open insert W --arm R --slow 3 --online    # closed loop, gain 0.1
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
import io
import os
import sys
import time

import numpy as np

from . import main as M
from .controller import DS_MAX, PICKUP_N, RELEASE, S_MAX, V_APP, Insertion

SPRING_K = 2700.0          # [N/m] the tool spring, axial (Lorenzo, 2026-10-21)


class OpenInsertion(Insertion):
    """Feedforward squeeze; `profile` = None: the SOCP online, else the offline f*(phase, t)."""

    def __init__(self, route, backend, k_eff=730.0, profile=None, **kw):
        kw["full_wrench"] = False                       # the twist channel is a moment loop
        super().__init__(route, backend, **kw)
        self.k_eff = float(k_eff)
        self.s_pick = 0.0
        self.profile = profile
        self._ph, self._t_ph = None, 0.0
        self.ratio = []                                 # measured / asked, while carrying
        self.trace = []                                 # (phase, time in phase, target): the profile

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

    def _phase_time(self, now):
        if self.phase != self._ph:
            self._ph, self._t_ph = self.phase, now
        return now - self._t_ph

    def _control(self, now):
        if self.profile is None:                         # the SOCP online, as `insert`
            super()._control(now)
            self.trace.append((self.phase, self._phase_time(now), float(self.target)))
            return
        # offline: the target from the profile, nothing solved now
        self.target = self.profile(self.phase, self._phase_time(now))
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
                  f"(k_eff {self.k_eff:.0f} N/m -> a matching k_eff would be ~{r * self.k_eff:.0f} N/m)")
        return out


class Profile:
    """f*(phase, time in phase) from the offline pass: linear in time within a phase, held at
    the ends; a phase the offline pass never reached holds the last value of the previous one."""
    ORDER = ("to_mouth", "route", "slow_seat", "seat_by_feel", "release")

    def __init__(self, trace):
        self.tab = {}
        for ph in self.ORDER:
            pts = [(t, f) for p, t, f in trace if p == ph]
            if pts:
                t, f = np.array(pts).T
                self.tab[ph] = (t, f)
        self.first = next(iter(self.tab.values()))[1][0]

    def __call__(self, phase, t):
        if phase in self.tab:
            ts, fs = self.tab[phase]
            return float(np.interp(t, ts, fs))
        prev = [p for p in self.ORDER[:self.ORDER.index(phase)] if p in self.tab] if phase in self.ORDER else []
        return float(self.tab[prev[-1]][1][-1]) if prev else float(self.first)


def offline_profile(a, r, k_eff, cfg):
    """The SOCP along the whole commanded sequence, with no robot (the dry-run backend: the flanges
    exactly where commanded), at the same pace as the run. Saved to logs/ for the record."""
    from .dryrun import DryRun
    b = DryRun(r, period=1.0 / cfg["rate_hz"])
    ins = M.slowed(OpenInsertion(r, b, k_eff=k_eff, confirm=lambda msg: None,
                                 **M.socp_kwargs(cfg)), a.slow)
    t0 = time.perf_counter()
    with contextlib.redirect_stdout(io.StringIO()):     # its own RESULT is not the run's
        ins.run()
    prof = Profile(ins.trace)
    os.makedirs(cfg["log_dir"], exist_ok=True)
    path = os.path.join(cfg["log_dir"], time.strftime(f"{r.world}_{a.arm}_offline_profile_%Y%m%d_%H%M%S.csv"))
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["phase", "t_in_phase", "target_N"])
        w.writerows((p, f"{t:.3f}", f"{f:.3f}") for p, t, f in ins.trace)
    route = prof.tab.get("route", (np.zeros(1), np.zeros(1)))[1]
    print(f"  OFFLINE PROFILE: {len(ins.trace)} SOCP targets solved in {time.perf_counter() - t0:.1f} s "
          f"before the run (route: {route.min():.1f}..{route.max():.1f} N/hand) -> {path}")
    return prof


def main():
    mode = "offline" if "--offline" in sys.argv else "online" if "--online" in sys.argv else "open"

    def extra(ap):
        g = ap.add_mutually_exclusive_group()
        g.add_argument("--offline", action="store_true", help="SOCP solved before the run, feedforward squeeze")
        g.add_argument("--online", action="store_true", help="closed loop on the ATI, --squeeze-gain 0.1 by default")
        ap.add_argument("--k-eff", type=float, default=None,
                        help="[N/m] flange -> pad stiffness per hand (default: cell stiffness.trans in series "
                             f"with the {SPRING_K:.0f} N/m tool spring)")
        ap.set_defaults(log_tag=f"_{mode}")

    def make(a, r, b, **kw):
        cfg = M.load_config(a.config)[0]
        if a.online:
            if "--squeeze-gain" not in sys.argv:
                kw["squeeze_gain"] = 0.1
            print(f"  SQUEEZE CLOSED ON THE ATI (online), gain x{kw.get('squeeze_gain', a.squeeze_gain):g}")
            return Insertion(r, b, **kw)
        if "--twist" in sys.argv:
            print("  --twist ignored: the feedforward runs have no moment loop")
        k = a.k_eff or 1.0 / (1.0 / float(cfg["stiffness"]["trans"]) + 1.0 / SPRING_K)
        pick = min(r.derived_N, PICKUP_N)
        print(f"  SQUEEZE IN FEEDFORWARD ({'SOCP offline, as the paper' if a.offline else 'SOCP online'}): "
              f"k_eff {k:.0f} N/m; pickup {pick:.1f} N/hand = {1000 * pick / k:.1f} mm past the nominal "
              f"contact; the ATIs are logged, not fed back")
        prof = offline_profile(a, r, k, cfg) if a.offline else None
        return OpenInsertion(r, b, k_eff=k, profile=prof, **kw)

    M.main(sys.argv[1:], doc=__doc__, extra=extra, make=make)


if __name__ == "__main__":
    main()
