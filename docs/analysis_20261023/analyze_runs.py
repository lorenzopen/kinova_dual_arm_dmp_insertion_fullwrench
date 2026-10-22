"""Read-only audit of the four recorded Kinova Z trials; no ROS or robot access."""
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import detrend
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
BASE = ROOT / 'docs/runs'
RUNS = [
 ('D 5/1, gain 1', '20261022_F75_Zt1_s9012_twist_ati/2_1528_riuscita/F75_Zt1_s9012_c100_five_cut800_20261022_152814.csv'),
 ('D 60/7, gain 1', '20261022_F75_Zt1_s9012_squeeze_gain/F75_Zt1_s9012_c100_five_cut800_20261022_172232.csv'),
 ('D 60/7, gain 0.5', '20261022_F75_Zt1_s9012_squeeze_gain/F75_Zt1_s9012_c100_five_cut800_20261022_173458.csv'),
 ('D 60/7, gain 0.25', '20261022_F75_Zt1_s9012_squeeze_gain/F75_Zt1_s9012_c100_five_cut800_20261022_174421.csv'),
]

def load(path):
    with path.open() as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([r[k] for r in rows], dtype=str if k == 'phase' else float) for k in rows[0]}

def remove_trend(t, x):
    return x - np.polyval(np.polyfit(t, x, 3), t)

def harmonic(t, x, freq):
    A = np.column_stack([np.cos(2*np.pi*freq*t), np.sin(2*np.pi*freq*t), np.ones(len(t)), t, t*t, t*t*t])
    c = np.linalg.lstsq(A, x, rcond=None)[0]
    return float(np.hypot(c[0], c[1])), float(np.arctan2(-c[1], c[0]))

fig, axes = plt.subplots(4, 3, figsize=(14, 12), constrained_layout=True)
results = []
for row, (label, rel) in enumerate(RUNS):
    path = BASE / rel
    d = load(path)
    meta = json.loads(path.with_name(path.stem + '_meta.json').read_text())
    route = d['phase'] == 'route'
    g = {k:v[route] for k,v in d.items()}
    t = g['t_wall'] - g['t_wall'][0]
    dt = np.diff(g['t_wall'])
    force_hand = []
    hand_tangent = []
    hand_eta = []
    hand_moments = []
    for side in 'LR':
        quat = np.column_stack([g['mq'+q+side] for q in ('x','y','z','w')])
        axis = Rotation.from_quat(quat).apply(np.tile([1.,0,0], (len(t),1)))
        F = np.column_stack([g['F'+q+side] for q in ('x','y','z')])
        normal = np.maximum(0, np.einsum('ij,ij->i', F, axis))
        tangent = np.linalg.norm(F - np.einsum('ij,ij->i', F, axis)[:,None]*axis, axis=1)
        force_hand.append(normal)
        hand_tangent.append(tangent)
        hand_eta.append(tangent/np.maximum(normal,.01))
        desired_F = np.column_stack([g['wdes'+side+'_f'+q] for q in 'xyz'])
        desired_T = np.column_stack([g['wdes'+side+'_t'+q] for q in 'xyz'])
        T = np.column_stack([g['T'+q+side] for q in 'xyz'])
        p = np.column_stack([g['m'+q+side] for q in 'xyz'])
        c = .5*np.column_stack([g['m'+q+'L']+g['m'+q+'R'] for q in 'xyz'])
        desired_pad_T = desired_T - np.cross(p+.100*axis-c, desired_F)
        actual_pad_T = T - np.cross(.100*axis,F)
        twist_des = np.einsum('ij,ij->i',desired_pad_T,axis)
        twist_actual = np.einsum('ij,ij->i',actual_pad_T,axis)
        bend_des = desired_pad_T - twist_des[:,None]*axis
        hand_moments.append(dict(side=side, desired_twist_abs_median_Nm=float(np.nanmedian(np.abs(twist_des))),
            desired_twist_abs_max_Nm=float(np.nanmax(np.abs(twist_des))),
            measured_twist_abs_median_Nm=float(np.nanmedian(np.abs(twist_actual))),
            desired_pad_bending_norm_median_Nm=float(np.nanmedian(np.linalg.norm(bend_des,axis=1))),
            twist_tracking_rms_Nm=float(np.sqrt(np.nanmean((twist_actual-twist_des)**2)))))
    cmd = np.column_stack([g[q+'R']-g[q+'L'] for q in 'xyz'])
    measured = np.column_stack([g['m'+q+'R']-g['m'+q+'L'] for q in 'xyz'])
    qcmd = np.column_stack([g[q+'L'] for q in ('qx','qy','qz','qw')])
    axis = Rotation.from_quat(qcmd).apply(np.tile([1.,0,0], (len(t),1)))
    closure_cmd = -1000 * np.einsum('ij,ij->i', cmd, axis)
    closure_meas = -1000 * np.einsum('ij,ij->i', measured, axis)
    m = t <= 17.5
    tt = t[m]
    uniform = np.arange(tt[0], tt[-1], .01)
    x = np.interp(uniform, tt, g['fn_N'][m])
    y = remove_trend(uniform, x)
    win = np.hanning(len(y))
    spectrum = 2*np.abs(np.fft.rfft(win*y))/win.sum()
    frequencies = np.fft.rfftfreq(len(y), .01)
    band = (frequencies >= .8) & (frequencies <= 2.5)
    peak = float(frequencies[band][np.argmax(spectrum[band])])
    hforce, phase_f = harmonic(tt, g['fn_N'][m], peak)
    hcmd, phase_cmd = harmonic(tt, closure_cmd[m], peak)
    hmeas, phase_meas = harmonic(tt, closure_meas[m], peak)
    lag = float(np.angle(np.exp(1j*(phase_cmd-phase_f))))
    # Frequency fit on the actual timestamps, independent of interpolation/FFT bins.
    fine = np.linspace(.8, 2.5, 851)
    fine_amp = np.array([harmonic(tt, g['fn_N'][m], f)[0] for f in fine])
    fine_peak = float(fine[np.argmax(fine_amp)])
    finite_w = np.isfinite(g['wdesL_fx'])
    fdes_sum = np.column_stack([g['wdesL_f'+q]+g['wdesR_f'+q] for q in 'xyz'])
    fsum = np.column_stack([g['F'+q+'L']+g['F'+q+'R'] for q in 'xyz'])
    rem = fsum - fdes_sum
    late = t >= max(0.,t[-1]-3.)
    result = dict(label=label, path=str(path), git=meta.get('git'), result=meta.get('result'),
      route_s=float(t[-1]), rows=len(t), actual_rate_Hz=float((len(t)-1)/t[-1]),
      dt_ms_p50_p95_p99_max=(1000*np.percentile(dt,[50,95,99,100])).tolist(), gaps_over_30ms=int((dt>.03).sum()),
      peak_Hz_resampled=peak, peak_Hz_actual_timestamps=fine_peak, peak_amplitude_N=float(spectrum[band].max()),
      detrended_std_N=float(y.std()), target_peak_to_peak_N=float(np.ptp(g['target_N'][m])),
      force_median_N=float(np.median(g['fn_N'][m])), target_sum_median_N=float(2*np.median(g['target_N'][m])),
      min_force_N=float(g['fn_N'][m].min()), late_force_median_N=float(np.median(g['fn_N'][late])),
      late_target_sum_median_N=float(2*np.median(g['target_N'][late])),
      final_half_second_force_median_N=float(np.median(g['fn_N'][t>=t[-1]-.5])),
      final_half_second_tangent_median_N=[float(np.median(x[t>=t[-1]-.5])) for x in hand_tangent],
      gross_hand_friction_usage_max=[float(x.max()) for x in hand_eta],
      mean_actual_sum_force_N=np.mean(fsum, axis=0).tolist(),
      max_actual_horizontal_sum_force_N=float(np.linalg.norm(fsum[:,:2],axis=1).max()),
      max_predicted_horizontal_sum_force_N=float(np.linalg.norm(fdes_sum[finite_w,:2],axis=1).max()),
      pad_moments=hand_moments,
      harmonic_force_N=hforce, harmonic_command_closure_mm=hcmd, harmonic_measured_closure_mm=hmeas,
      command_to_force_phase_deg=float(np.degrees(lag)), equivalent_phase_lag_ms=float(1000*lag/(2*np.pi*peak)),
      per_hand_correlation=float(np.corrcoef(remove_trend(tt, force_hand[0][m]), remove_trend(tt, force_hand[1][m]))[0,1]),
      fn_reconstruction_max_error_N=float(np.max(np.abs(sum(force_hand)-g['fn_N']))),
      target_max_all_phases_N=float(d['target_N'].max()),
      actual_total_force_residual_rms_N=float(np.sqrt(np.mean(np.sum(rem[finite_w]**2,axis=1)))) if finite_w.any() else None,
      fullwrench_max_offset_rad=float(max(np.max(np.sqrt(sum(g['off'+s+'_'+q]**2 for q in ('rx','ry','rz')))) for s in 'LR')),
      source=meta.get('wrench_source'))
    results.append(result)
    a=axes[row,0]
    a.plot(t, g['fn_N'], label='ATI: sum of normal forces', lw=1)
    a.plot(t, 2*g['target_N'], label='2 x SOCP target', lw=1)
    a.set_title(label); a.set_xlabel('time in route [s]'); a.set_ylabel('force [N]')
    a=axes[row,1]
    a.plot(tt, closure_cmd[m]-np.median(closure_cmd[m]), label='commanded closure', lw=1)
    a.plot(tt, closure_meas[m]-np.median(closure_meas[m]), label='measured closure', lw=1)
    a.set_xlabel('time in route [s]'); a.set_ylabel('closure variation [mm]')
    a=axes[row,2]
    a.plot(frequencies,spectrum,lw=1)
    a.set_xlim(0,4); a.set_xlabel('frequency [Hz]'); a.set_ylabel('amplitude [N]')
    a.set_title(f'Actual-time resampling: {peak:.2f} Hz')
    for a in axes[row]:
        a.grid(alpha=.25)
for a in axes[0,:2]: a.legend(fontsize=8)
fig.savefig(OUT/'kinova_force_audit.png',dpi=150)
(OUT/'metrics.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))
