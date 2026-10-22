"""`insert` with the squeeze in FEEDFORWARD: the wrist ATIs are recorded, not fed back to the squeeze.

    python -m kinova_dual_arm_dmp_insertion.insert_open check  W --arm R --slow 3
    python -m kinova_dual_arm_dmp_insertion.insert_open insert W --arm R --slow 3 [--k-eff 730]

As in the friction-margin paper's execution (an equilibrium-point offset u = p + K^-1 f*): the
force the SOCP asks per hand becomes a closure of the two flange targets along the pad axis,

    ds = f* / k_eff          (each hand, beyond the nominal contact with the piece's faces)

with k_eff the stiffness flange -> pad face per hand: the arm's impedance in series with the tool
spring, 1/(1/1000 + 1/2700) = 730 N/m by default. Nothing integrates the measured force, so there
is no squeeze loop to oscillate; the price is that the force is only as right as k_eff and the
piece's width (an error e in the width gives k_eff * e / 2 per hand: 730 N/m x 2 mm = 0.7 N).

What still uses the sensors, as events and not as regulation: the safety stops (60 N, tracking),
the jam sentry and the seat by feel. The approach does NOT: the tools close to the nominal faces of
the piece (route opening), then press by f_pickup / k_eff. No --twist here (it is a moment loop).

The log is `insert`'s, with `_open` in its name: the measured squeeze (ATI) beside the asked one
shows how good the feedforward was; at the end the ratio measured/asked gives a corrected k_eff.
"""
import sys

import numpy as np

from . import geometry as GE
from . import main as M
from .controller import DS_MAX, PICKUP_N, RELEASE, S_MAX, V_APP, Insertion

SPRING_K = 2700.0          # [N/m] the tool spring, axial (Lorenzo, 2026-10-21)


class OpenInsertion(Insertion):
    def __init__(self, route, backend, k_eff=730.0, **kw):
        kw["full_wrench"] = False                       # the twist channel is a moment loop
        super().__init__(route, backend, **kw)
        self.k_eff = float(k_eff)
        self.s_pick = 0.0
        self.ratio = []                                 # measured / asked, while carrying

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


def main():
    def extra(ap):
        ap.add_argument("--k-eff", type=float, default=None,
                        help="[N/m] flange -> pad stiffness per hand (default: cell stiffness.trans in series "
                             f"with the {SPRING_K:.0f} N/m tool spring)")
        ap.set_defaults(log_tag="_open")

    def make(a, r, b, **kw):
        if "--twist" in sys.argv:
            print("  --twist ignored: the feedforward run has no moment loop")
        k = a.k_eff
        if k is None:
            arm = float(M.load_config(a.config)[0]["stiffness"]["trans"])
            k = 1.0 / (1.0 / arm + 1.0 / SPRING_K)
        pick = min(r.derived_N, PICKUP_N)
        print(f"  SQUEEZE IN FEEDFORWARD: k_eff {k:.0f} N/m; pickup {pick:.1f} N/hand = "
              f"{1000 * pick / k:.1f} mm past the nominal contact; the ATIs are logged, not fed back")
        return OpenInsertion(r, b, k_eff=k, **kw)

    M.main(sys.argv[1:], doc=__doc__, extra=extra, make=make)


if __name__ == "__main__":
    main()
