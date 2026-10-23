"""Separate PI squeeze experiment; historical entry points keep their original law.

    python -m kinova_dual_arm_dmp_insertion.insert_pi check F75_Zt1_s9012 --arm c100_five_cut800 --slow 3
    python -m kinova_dual_arm_dmp_insertion.insert_pi insert F75_Zt1_s9012 --arm c100_five_cut800 --slow 3

--offline precomputes the same full wrench plan as insert_open, then executes it
with PI force feedback. No additional --online flag is needed in this module.
PI regulates SUMMED normal force by symmetric per-hand position correction.
Tangential/moment feedback, if selected, retains the existing independent law.
"""
from dataclasses import asdict, dataclass

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from . import main as M
from .controller import (Abort, CONTROL_PERIOD, GAIN, Insertion,
                         K_TILT_HOLD, RELEASE)
from .insert_open import ProfileInsertion, effective_stiffness, offline_profile


@dataclass(frozen=True)
class PISettings:
    # Physical units: error is the SUM of the two normal forces, not force/hand.
    kp_m_per_N: float = 0.0001
    ki_m_per_N_s: float = 0.001
    slew_m_per_s: float = 0.002
    correction_limit_m: float = 0.006
    pickup_limit_m: float = 0.015
    max_dt_s: float = 0.100

    def __post_init__(self):
        vals = asdict(self)
        if not all(np.isfinite(v) for v in vals.values()):
            raise ValueError("PI settings must be finite")
        if self.kp_m_per_N < 0 or self.ki_m_per_N_s <= 0:
            raise ValueError("PI requires kp >= 0 and ki > 0")
        if min(self.slew_m_per_s, self.correction_limit_m, self.pickup_limit_m) <= 0:
            raise ValueError("PI speed and travel limits must be positive")
        if self.slew_m_per_s > 0.010:
            raise ValueError("PI speed exceeds the historical nominal 10 mm/s envelope")
        if not CONTROL_PERIOD <= self.max_dt_s <= 0.100:
            raise ValueError("PI max_dt must be between the nominal period and 0.100 s")
        if self.correction_limit_m > RELEASE or self.pickup_limit_m > 0.060:
            raise ValueError("PI travel limits exceed the historical squeeze envelope")


class SqueezePI:
    """Position-form PI, actual dt, bumpless entry and actuator-tracking anti-windup.

    p = kp * (target_total - measured_total)
    i_trial = i + ki * error * dt
    output = clip(p + i_trial, position bounds AND previous output +/- speed*dt)
    i = output - p

    Tracking the attainable output prevents hidden integral accumulation at both
    position and rate saturation. The first sample tracks the current correction
    without moving. P is an absolute term; it is NEVER added repeatedly to output.
    No derivative, force filter or feedforward preload is added implicitly.
    """
    def __init__(self, settings):
        self.s = settings
        self.time = None
        self.integral = 0.0
        self.output = 0.0
        self.sequence = 0
        self.diag = {}

    def reset(self):
        self.time = None
        self.diag = {}

    def update(self, target_total, measured_total, now, current, lower, upper):
        args = (target_total, measured_total, now, current, lower, upper)
        if not all(np.isfinite(v) for v in args) or lower >= upper:
            raise Abort("PI received non-finite data or invalid bounds")
        if not lower - 1e-12 <= current <= upper + 1e-12:
            raise Abort("PI correction outside configured limits")
        e = float(target_total - measured_total)
        p = self.s.kp_m_per_N * e
        dt, rate_limited, position_limited = 0.0, False, False
        if self.time is None:
            out = float(current)
            self.integral = out - p
        else:
            dt = float(now - self.time)
            if dt <= 0 or dt > self.s.max_dt_s + 1e-9:
                raise Abort(f"PI update interval {1000*dt:.1f} ms outside (0, {1000*self.s.max_dt_s:g}] ms")
            if abs(current - self.output) > 1e-9:
                raise Abort("PI output changed externally without a reset")
            trial = p + self.integral + self.s.ki_m_per_N_s * e * dt
            step = self.s.slew_m_per_s * dt
            lo, hi = max(lower, current - step), min(upper, current + step)
            out = float(np.clip(trial, lo, hi))
            rate_limited = trial < current - step - 1e-12 or trial > current + step + 1e-12
            position_limited = trial < lower - 1e-12 or trial > upper + 1e-12
            self.integral = out - p
        self.time, self.output = float(now), out
        self.sequence += 1
        self.diag = dict(pi_seq=self.sequence, pi_dt_ms=1000*dt, pi_error_N=e,
                         pi_p_mm=1000*p, pi_i_mm=1000*self.integral,
                         pi_output_mm=1000*out, pi_rate_limited=int(rate_limited),
                         pi_position_limited=int(position_limited))
        return out


class PISqueezeMixin:
    def __init__(self, *args, pi_settings=None, **kw):
        super().__init__(*args, **kw)
        self.pi_settings = pi_settings or PISettings()
        self.squeeze_pi = SqueezePI(self.pi_settings)
        self.execution_mode = ("offline_socp_ati_pi" if getattr(self, "profile", None) is not None
                               else "online_socp_ati_pi")

    def _squeeze_update(self):
        lim = self.pi_settings.correction_limit_m
        self.ds = self.squeeze_pi.update(2*self.target, self.normal_force(), self.b.now(),
                                         self.ds, -lim, lim)

    def _prepare_control(self):
        super()._prepare_control()
        # Pickup displacement is now in the frozen hand frame, not in ds.
        # Track ds=0 at the first force-loop sample; do not carry pickup I across.
        self.squeeze_pi.reset()

    def squeeze(self, force_n, seconds=6.0):
        """Same PI for pickup; require sustained measured force before operator release."""
        if not np.isfinite(force_n) or force_n <= 0 or not np.isfinite(seconds) or seconds <= 0:
            raise Abort('PI pickup requires positive finite force and duration')
        self.phase = "squeeze"
        self.squeeze_pi.reset()
        start = self.b.now()
        last, settled = -np.inf, None
        s = 0.0
        while self.b.now() - start < seconds:
            now = self.b.now()
            if now - last >= CONTROL_PERIOD - 1e-9:
                last = now
                f = self.normal_force()
                s = self.squeeze_pi.update(2*force_n, f, now, s, 0.0,
                                           self.pi_settings.pickup_limit_m)
                self.ds = s  # pickup command and PI diagnostics visible in the CSV
                normals = [float(self.meas['F'+side] @
                                 Rot.from_quat(self.meas['q'+side]).apply([1.,0.,0.]))
                           for side in 'LR']
                ready = (abs(f - 2*force_n) <= max(0.3, 0.05*2*force_n)
                         and min(normals) >= 0.5*force_n)
                settled = (now if settled is None else settled) if ready else None
            self.tick(self.cL + s*self.axis, self.qL,
                      self.cR - s*self.axis, self.qR, K_TILT_HOLD)
            if settled is not None and now - settled >= 0.2:
                break
        else:
            raise Abort(f"PI pickup did not settle in {seconds:g} s: "
                        f"{self.normal_force():.2f}/{2*force_n:.2f} N total; keep supporting the piece")
        self.cL, self.cR = self.cL + s*self.axis, self.cR - s*self.axis
        self.ds = 0.0
        self.squeeze_pi.reset()

    def control_metadata(self):
        return dict(squeeze_controller="position_PI", squeeze_gain_scale=None,
                    squeeze_gain_m_per_N_per_update=None,
                    legacy_internal_translation_gain_scale=self.gain / GAIN,
                    pi=dict(**asdict(self.pi_settings), error="summed desired minus measured normal force [N]",
                            output="symmetric per-hand position correction [m]",
                            anti_windup="track rate- and position-limited output",
                            entry="bumpless; reset after pickup and before motion",
                            hardware_tuned=False), pi_last=dict(self.squeeze_pi.diag))


class PIInsertion(PISqueezeMixin, Insertion):
    pass


class PIProfileInsertion(PISqueezeMixin, ProfileInsertion):
    pass


def main(argv=None):
    def nonnegative(value):
        v = float(value)
        if not np.isfinite(v) or v < 0:
            raise ValueError('must be finite and non-negative')
        return v

    def positive(value):
        v = nonnegative(value)
        if v <= 0:
            raise ValueError('must be positive')
        return v

    def speed(value):
        v = positive(value)
        if v > 10:
            raise ValueError('maximum speed is 10 mm/s')
        return v

    def travel(value):
        v = positive(value)
        if v > 12:
            raise ValueError('maximum correction is 12 mm/hand')
        return v

    def extra(ap):
        ap.add_argument('--offline', action='store_true', help='precompute SOCP; PI still uses ATI during execution')
        ap.add_argument('--k-eff', type=float, default=None, help='axial stiffness for offline planning only [N/m]')
        ap.add_argument('--pi-kp', type=nonnegative, default=0.0001, help='P [m/N], error on SUMMED normal force')
        ap.add_argument('--pi-ki', type=positive, default=0.001, help='I [m/(N s)], error on SUMMED normal force')
        ap.add_argument('--pi-speed-mm-s', type=speed, default=2.0, help='maximum PI correction speed per hand [mm/s]')
        ap.add_argument('--pi-limit-mm', type=travel, default=6.0, help='extra correction travel each way after pickup [mm/hand]')
        ap.set_defaults(log_tag='_pi', squeeze_gain=0.25, squeeze_controller='PI')
        for action in ap._actions:
            if action.dest == 'squeeze_gain':
                action.help = 'PI module: scales ONLY optional legacy tangential feedback; use --pi-kp/--pi-ki for normal force'

    def make(a, r, b, **kw):
        settings = PISettings(kp_m_per_N=a.pi_kp, ki_m_per_N_s=a.pi_ki,
                              slew_m_per_s=a.pi_speed_mm_s/1000,
                              correction_limit_m=a.pi_limit_mm/1000)
        if hasattr(b, 'arms') and any(arm.source != 'ft' for arm in b.arms.values()):
            raise ValueError('PI experiment requires ATI feedback on BOTH arms')
        print(f'  PI SQUEEZE: kp={settings.kp_m_per_N:g} m/N, ki={settings.ki_m_per_N_s:g} m/(N s); '
              f'error is SUMMED force; speed {settings.slew_m_per_s*1000:g} mm/s/hand, '
              f'extra travel +/-{settings.correction_limit_m*1000:g} mm/hand')
        if a.offline:
            cfg = M.load_config(a.config)[0]
            if a.no_bending:
                cfg.setdefault('socp', {})['no_bending'] = True
            k = effective_stiffness(cfg, b, a.k_eff)
            profile = offline_profile(a, r, k, cfg)
            return PIProfileInsertion(r, b, profile=profile, pi_settings=settings, **kw)
        if a.k_eff is not None:
            raise ValueError('--k-eff is only used with --offline in the PI module')
        return PIInsertion(r, b, pi_settings=settings, **kw)

    M.main(argv, doc=__doc__, extra=extra, make=make)


if __name__ == '__main__':
    main()
