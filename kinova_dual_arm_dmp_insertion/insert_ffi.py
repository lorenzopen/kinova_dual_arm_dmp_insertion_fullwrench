"""SOCP normal-force feedforward + small ATI integral trim; separate experiment.

    python -m kinova_dual_arm_dmp_insertion.insert_ffi check WORLD --arm c100_five_cut800 --slow 3 --offline
    python -m kinova_dual_arm_dmp_insertion.insert_ffi insert WORLD --arm c100_five_cut800 --slow 3 --offline

Same friction margins/SOCP as other modules. --offline fixes the full wrench plan;
without it SOCP updates online. ATI feedback is ALWAYS active in this module.
Only the normal squeeze gets feedforward + I. No new moment/lateral compliance model.
"""
from dataclasses import asdict, dataclass

import numpy as np

from . import main as M
from .controller import Abort, GAIN, Insertion
from .insert_open import ProfileInsertion, effective_stiffness, offline_profile
from .insert_pi import PISettings, PISqueezeMixin


@dataclass(frozen=True)
class FFISettings:
    # Delay rule (docs/FFI_CONTROLLER.md): ki = 0.5/(k_loop*tau), k_loop = 2*680 N/m,
    # tau = 0.23 s worst measured command->flange lag -> ~60 deg phase margin.
    ki_m_per_N_s: float = 0.0016  # error is the SUM of both hand normal forces
    trim_limit_m: float = 0.003   # integral correction per hand
    trim_speed_m_per_s: float = 0.002
    total_limit_m: float = 0.012  # total extra motion relative to frozen pickup frame
    total_speed_m_per_s: float = 0.002
    max_dt_s: float = 0.100       # longer intervals integrate at most this much
    stall_dt_s: float = 0.300     # beyond this the loop is considered stalled: Abort

    def __post_init__(self):
        if not all(np.isfinite(v) and v > 0 for v in asdict(self).values()):
            raise ValueError('FFI settings must be finite and positive')
        if not self.trim_limit_m < self.total_limit_m <= 0.012:
            raise ValueError('FFI requires 0 < trim limit < total limit <= 12 mm')
        if not self.trim_speed_m_per_s <= self.total_speed_m_per_s <= 0.010:
            raise ValueError('FFI requires trim speed <= total speed <= 10 mm/s')
        if not .020 <= self.max_dt_s <= .100:
            raise ValueError('FFI max dt must be between 20 and 100 ms')
        if not self.max_dt_s < self.stall_dt_s <= .500:
            raise ValueError('FFI stall dt must be above max dt and at most 500 ms')


class NominalIntegral:
    """Bumpless feedforward referenced to measured pickup preload, plus bounded I.

    ff_want = (N_target_per_hand - N_pickup_per_hand) / k_eff
    trim_trial = trim + ki * (2*N_target - N_measured_sum) * actual_dt
    ds = ff + trim

    Feedforward ramps with priority; feasible intervals jointly enforce trim travel,
    trim speed, total travel AND total speed. I state is the attainable trim, so it
    cannot wind up against any of these limits. No P, D or force filter is added.
    """
    def __init__(self, settings, k_eff, pickup_N):
        if not np.isfinite(k_eff) or k_eff <= 0 or not np.isfinite(pickup_N) or pickup_N <= 0:
            raise ValueError('FFI needs positive finite stiffness and measured pickup force')
        self.s, self.k_eff, self.pickup_N = settings, float(k_eff), float(pickup_N)
        self.time = None
        self.ff = self.trim = self.output = 0.
        self.sequence = 0
        self.diag = {}

    def update(self, target_N, measured_sum_N, now, current):
        if not all(np.isfinite(v) for v in (target_N, measured_sum_N, now, current)) or target_N < 0:
            raise Abort('FFI received invalid target/measurement/time/output')
        if abs(current-self.output) > 1e-9:
            raise Abort('FFI total correction changed externally')
        s = self.s
        want = (float(target_N)-self.pickup_N)/self.k_eff
        error = 2*float(target_N)-float(measured_sum_N)
        dt = 0.
        feed_clipped = trim_clipped = total_rate = total_position = dt_clamped = False
        ff, trim = self.ff, self.trim
        if self.time is not None:
            dt = float(now-self.time)
            if dt <= 0 or dt > s.stall_dt_s+1e-9:
                raise Abort(f'FFI update interval {1000*dt:.1f} ms outside (0, {1000*s.stall_dt_s:g}] ms')
            # Real-PC hiccups (ctrl_dt up to ~126 ms on 23/10): do not abort, but
            # never integrate or move more than one max_dt worth in a single tick.
            dt_clamped = dt > s.max_dt_s
            dt = min(dt, s.max_dt_s)
            total_step, trim_step = s.total_speed_m_per_s*dt, s.trim_speed_m_per_s*dt
            total_lo = max(-s.total_limit_m, current-total_step)
            total_hi = min(s.total_limit_m, current+total_step)
            trim_lo = max(-s.trim_limit_m, trim-trim_step)
            trim_hi = min(s.trim_limit_m, trim+trim_step)
            # Reserve room for the integral range; ff cannot violate trim speed
            # just because the combined output is rate-limited.
            ff_limit = s.total_limit_m-s.trim_limit_m
            ff_lo = max(-ff_limit, ff-total_step, total_lo-trim_hi)
            ff_hi = min(ff_limit, ff+total_step, total_hi-trim_lo)
            ff = float(np.clip(want, ff_lo, ff_hi))
            trim_trial = trim+s.ki_m_per_N_s*error*dt
            attainable_lo, attainable_hi = max(trim_lo,total_lo-ff), min(trim_hi,total_hi-ff)
            trim = float(np.clip(trim_trial,attainable_lo,attainable_hi))
            raw_total = ff+trim_trial
            feed_clipped = abs(ff-want) > 1e-12
            trim_clipped = abs(trim-trim_trial) > 1e-12
            total_rate = raw_total < current-total_step-1e-12 or raw_total > current+total_step+1e-12
            total_position = raw_total < -s.total_limit_m-1e-12 or raw_total > s.total_limit_m+1e-12
        out = ff+trim
        self.ff, self.trim, self.output, self.time = ff, trim, out, float(now)
        self.sequence += 1
        self.diag = dict(ffi_seq=self.sequence, ffi_dt_ms=1000*dt, ffi_dt_clamped=int(dt_clamped),
                         ffi_error_N=error,
                         ffi_ff_want_mm=1000*want, ffi_ff_mm=1000*ff, ffi_i_mm=1000*trim,
                         ffi_output_mm=1000*out, ffi_ff_limited=int(feed_clipped),
                         ffi_i_limited=int(trim_clipped), ffi_rate_limited=int(total_rate),
                         ffi_position_limited=int(total_position))
        return out


class FFIMixin(PISqueezeMixin):
    """Reuse measured-touch pickup with a pure I; use a separate FFI during motion."""
    def __init__(self, *args, k_eff, ffi_settings=None, **kw):
        # Shared pickup implementation, P explicitly zero. This is NOT the runtime trim gain.
        kw['pi_settings'] = PISettings(kp_m_per_N=0., ki_m_per_N_s=.001)
        super().__init__(*args, **kw)
        if not np.isfinite(k_eff) or k_eff <= 0:
            raise ValueError('FFI k_eff must be positive and finite')
        self.k_eff = float(k_eff)
        self.ffi_settings = ffi_settings or FFISettings()
        self.normal_ffi = None
        self.execution_mode = ('offline_socp_ati_feedforward_I' if getattr(self,'profile',None) is not None
                               else 'online_socp_ati_feedforward_I')

    def squeeze(self, force_n, seconds=6.):
        super().squeeze(force_n, seconds)
        # Baseline normal force measured at pickup; its commanded pose will be
        # absorbed in frame.oL/oR. Never add the whole N*/k on top of that pose.
        self.pickup_normal_N = .5*self.normal_force()

    def _prepare_control(self):
        super()._prepare_control()
        if not hasattr(self,'pickup_normal_N'):
            raise Abort('FFI requires completed measured-force pickup')
        if abs(self.ds) > 1e-12:
            raise Abort('FFI expects zero extra correction at frozen pickup frame')
        self.normal_ffi = NominalIntegral(self.ffi_settings,self.k_eff,self.pickup_normal_N)

    def _squeeze_update(self):
        if self.normal_ffi is None:
            raise Abort('FFI normal controller has not been prepared')
        self.ds = self.normal_ffi.update(self.target,self.normal_force(),self.b.now(),self.ds)

    def control_metadata(self):
        return dict(squeeze_controller='normal_feedforward_plus_I', squeeze_gain_scale=None,
                    squeeze_gain_m_per_N_per_update=None,
                    legacy_internal_translation_gain_scale=self.gain/GAIN,
                    ffi=dict(**asdict(self.ffi_settings), k_eff_N_per_m=self.k_eff,
                             pickup_measured_N_per_hand=getattr(self,'pickup_normal_N',None),
                             nominal='(target_N_per_hand - pickup_measured_N_per_hand) / k_eff',
                             normal_error='2*target_N_per_hand - measured_normal_sum_N',
                             entry='zero ff and trim at frozen pickup pose; feedforward ramps',
                             anti_windup='track feasible integral after trim and combined command limits',
                             hardware_tuned=False),
                    pickup_controller=dict(type='I', **asdict(self.pi_settings)),
                    ffi_last=dict(self.normal_ffi.diag) if self.normal_ffi is not None else {})


class FFIInsertion(FFIMixin, Insertion):
    pass


class FFIProfileInsertion(FFIMixin, ProfileInsertion):
    pass


def positive(value):
    v = float(value)
    if not np.isfinite(v) or v <= 0:
        raise ValueError('must be finite and positive')
    return v


def trim_mm(value):
    v = positive(value)
    if v >= 12:
        raise ValueError('trim limit must be below 12 mm/hand')
    return v


def main(argv=None):
    def extra(ap):
        ap.add_argument('--offline',action='store_true',help='fixed SOCP plan; ATI integral trim remains active')
        ap.add_argument('--k-eff',type=positive,default=None,help='axial feedforward stiffness per hand [N/m]')
        ap.add_argument('--i-ki',type=positive,default=FFISettings.ki_m_per_N_s,
                        help='runtime trim I [m/(N s)], SUMMED force error')
        ap.add_argument('--i-limit-mm',type=trim_mm,default=3.,help='maximum integral trim each way [mm/hand]')
        ap.set_defaults(log_tag='_ffi',squeeze_gain=.1,squeeze_controller='FFI')
        for action in ap._actions:
            if action.dest=='squeeze_gain':
                action.help='FFI: scales only optional legacy tangential feedback; normal trim uses --i-ki'

    def make(a,r,b,**kw):
        cfg=M.load_config(a.config)[0]
        if a.no_bending:
            cfg.setdefault('socp',{})['no_bending']=True
        if hasattr(b,'arms') and any(arm.source!='ft' for arm in b.arms.values()):
            raise ValueError('FFI requires ATI feedback on BOTH arms')
        k=effective_stiffness(cfg,b,a.k_eff)
        settings=FFISettings(ki_m_per_N_s=a.i_ki,trim_limit_m=a.i_limit_mm/1000)
        print(f'  NORMAL FEEDFORWARD + I: k_eff={k:.1f} N/m/hand, ki={a.i_ki:g} m/(N s), '
              f'trim +/-{a.i_limit_mm:g} mm/hand; trim and total speed 2 mm/s/hand')
        if a.offline:
            profile=offline_profile(a,r,k,cfg)
            return FFIProfileInsertion(r,b,k_eff=k,ffi_settings=settings,profile=profile,**kw)
        return FFIInsertion(r,b,k_eff=k,ffi_settings=settings,**kw)

    M.main(argv,doc=__doc__,extra=extra,make=make)


if __name__=='__main__':
    main()
