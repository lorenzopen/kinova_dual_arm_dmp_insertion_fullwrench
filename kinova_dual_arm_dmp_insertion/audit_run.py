"""Inspect a recorded insertion, without ROS, pandas or any robot connection.

python -m kinova_dual_arm_dmp_insertion.audit_run logs/RUN.csv [--no-plots]

Desired moments in the CSV are about the common grasp centre; measured moments
are moved there before comparison. Spectra use actual timestamps, with no removal
or compression of gaps. This checks records, not object slip or placement accuracy.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def load(path):
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise ValueError("empty run CSV")
    return {key: np.array([r[key] if key == 'phase' else float(r[key]) if r[key].strip() else np.nan
                          for r in rows], dtype=str if key == "phase" else float)
            for key in rows[0]}


def measured_common_wrench(d, side):
    centre = .5*np.column_stack([d['m'+q+'L']+d['m'+q+'R'] for q in 'xyz'])
    force = np.column_stack([d['F'+q+side] for q in 'xyz'])
    moment = np.column_stack([d['T'+q+side] for q in 'xyz'])
    position = np.column_stack([d['m'+q+side] for q in 'xyz'])
    return np.column_stack([force,moment+np.cross(position-centre,force)])


def spectrum(t, force):
    """Hann amplitude spectrum after resampling to the median actual interval.

    Large gaps are a data-quality finding in the report, not hidden by clipping time.
    """
    if len(t) < 8 or np.any(np.diff(t) <= 0):
        return np.array([]), np.array([])
    dt = float(np.median(np.diff(t)))
    uniform = np.arange(t[0], t[-1], dt)
    x = np.interp(uniform, t, force)
    x -= np.polyval(np.polyfit(uniform-uniform[0], x, min(3, len(x)-1)), uniform-uniform[0])
    w = np.hanning(len(x))
    return np.fft.rfftfreq(len(x), dt), 2*np.abs(np.fft.rfft(x*w))/w.sum()


def inspect(path, plots=True, out=None):
    path = Path(path)
    d = load(path)
    if not all(k in d for k in ('t','phase','fn_N','target_N','mxL','mxR')):
        raise ValueError("expected an insertion run CSV, not an offline force profile")
    out = Path(out) if out else path.with_name(path.stem + "_analysis")
    out.mkdir(parents=True, exist_ok=True)
    clock = "t_wall" if "t_wall" in d else "t"
    t = d[clock] - d[clock][0]
    route = d['phase'] == 'route'
    dt = np.diff(t[route])
    mpath = path.with_name(path.stem + '_meta.json')
    meta = json.loads(mpath.read_text(encoding='utf-8')) if mpath.exists() else {}
    report = dict(csv=str(path.resolve()), csv_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  clock=clock, rows=len(t), result=meta.get('result'),
                  execution=meta.get('execution'), controller_parameters=meta.get('controller_parameters'),
                  warnings=[], physical_outcome='measure object slip, neighbour motion and placement independently')
    report['allocation_failures'] = meta.get('allocation_failures')
    report['allocation_result_not_applied'] = meta.get('allocation_result_not_applied')
    if len(dt) and np.all(dt > 0):
        report['route'] = dict(duration_s=float(t[route][-1]-t[route][0]),
                               actual_csv_rate_Hz=float(len(dt)/np.sum(dt)),
                               dt_ms_p50_p95_p99_max=(1000*np.percentile(dt,[50,95,99,100])).tolist(),
                               gaps_over_30ms=int((dt > .03).sum()),
                               squeeze_error_rms_N=float(np.sqrt(np.mean((d['fn_N'][route]-2*d['target_N'][route])**2))),
                               minimum_squeeze_N=float(d['fn_N'][route].min()))
        f, amp = spectrum(t[route], d['fn_N'][route])
        band = (f >= .8) & (f <= 2.5)
        if band.any():
            k = np.flatnonzero(band)[np.argmax(amp[band])]
            report['route'].update(peak_0p8_to_2p5_Hz=float(f[k]), peak_amplitude_N=float(amp[k]))
        if np.max(dt) > .03:
            report['warnings'].append('CSV gaps >30 ms: inspect timing before interpreting high-frequency spectra')
    else:
        f, amp = np.array([]), np.array([])
        report['warnings'].append('no usable route or non-increasing timestamps; spectrum omitted')
    carry = np.isin(d['phase'], ['to_mouth','route','slow_seat','seat_by_feel'])
    if not carry.any():
        carry = np.ones(len(t), dtype=bool)
    if 'socp_ok' in d:
        report['carrying_rows_with_failed_last_allocation'] = int(np.sum(carry & (d['socp_ok'] == 0)))
    for key in ('joint_age_L_ms','joint_age_R_ms','ft_age_L_ms','ft_age_R_ms','ctrl_dt_ms','socp_input_age_ms'):
        if key in d:
            valid = carry & np.isfinite(d[key])
            if key == 'ctrl_dt_ms' and 'ctrl_seq' in d:
                valid &= np.r_[False,np.diff(d['ctrl_seq'])>0]
            values = d[key][valid]
            if len(values):
                report[key + '_p50_p99_max'] = np.percentile(values,[50,99,100]).tolist()
    params = meta.get('controller_parameters') or {}
    if not all(params.get(s,{}).get('cart_k') and params.get(s,{}).get('cart_d') for s in 'LR'):
        report['warnings'].append('actual controller parameters were not recorded; YAML does not prove launched stiffness')
    if not meta.get('tares'):
        report['warnings'].append('no tare record available')
    if meta.get('git_dirty'):
        report['warnings'].append('run code had uncommitted changes; preserve its source diff with the data')
    limit = (meta.get('execution') or {}).get('internal_translation_limit_m')
    if limit:
        for side in 'LR':
            keys = [f'off{side}_{q}' for q in 'xyz']
            if all(k in d for k in keys):
                norms = np.linalg.norm(np.column_stack([d[k] for k in keys]),axis=1)
                share = float(np.mean(norms[carry] >= limit - 1e-5))
                report[f'internal_translation_{side}_at_limit_share'] = share
                if share > 0:
                    report['warnings'].append(f'{side} internal correction saturated: desired tangential sharing may be unattainable')
    if plots:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3,1,figsize=(11,9),layout='constrained')
        ax[0].plot(t[carry],d['fn_N'][carry],label='ATI normal force sum')
        ax[0].plot(t[carry],2*d['target_N'][carry],label='2 x conservative normal target')
        ax[0].set_ylabel('force [N]'); ax[0].legend()
        for side in 'LR':
            p = np.column_stack([d['m'+q+side] for q in 'xyz'])
            command = np.column_stack([d[q+side] for q in 'xyz'])
            ax[1].plot(t[carry],1000*np.linalg.norm(p-command,axis=1)[carry],label=side)
        ax[1].set_ylabel('position tracking error [mm]'); ax[1].legend()
        ax[2].plot(f,amp); ax[2].set_xlim(0, min(8.,f[-1]) if len(f) else 8.)
        ax[2].set_xlabel('frequency [Hz]'); ax[2].set_ylabel('squeeze amplitude [N]')
        for a in ax[:2]: a.set_xlabel('elapsed wall time [s]')
        for a in ax: a.grid(alpha=.25)
        fig.savefig(out/'tracking.png',dpi=140); plt.close(fig)
        required = [f'mq{q}{s}' for s in 'LR' for q in ('x','y','z','w')]
        if all(k in d for k in required) and all('T'+q+s in d for s in 'LR' for q in 'xyz'):
            fig, axs = plt.subplots(6,2,figsize=(13,14),sharex=True,layout='constrained')
            for col, side in enumerate('LR'):
                measured = measured_common_wrench(d,side)
                for row, component in enumerate(('fx','fy','fz','tx','ty','tz')):
                    a = axs[row,col]
                    a.plot(t[carry],measured[carry,row],label='ATI')
                    key = f'wdes{side}_{component}'
                    if key in d and np.isfinite(d[key][carry]).any():
                        a.plot(t[carry],d[key][carry],label='SOCP',lw=1)
                    a.set_ylabel(f'{component} [{"N" if row<3 else "Nm"}]'); a.grid(alpha=.25)
                axs[0,col].set_title(f'{side}: shelf axes, moments about common grasp centre')
                axs[0,col].legend()
                axs[-1,col].set_xlabel('elapsed wall time [s]')
            fig.savefig(out/'wrenches.png',dpi=140); plt.close(fig)
        else:
            report['warnings'].append('full moment comparison unavailable in this older CSV schema')
        qcols = [f'q{s}_j{j}' for s in 'LR' for j in range(1,8)]
        if all(k in d and np.isfinite(d[k]).any() for k in qcols):
            fig, axs = plt.subplots(3,2,figsize=(13,9),sharex=True,layout='constrained')
            for col, side in enumerate('LR'):
                for row, kind in enumerate(('q','dq','tau')):
                    for j in range(1,8):
                        axs[row,col].plot(t,d[f'{kind}{side}_j{j}'],label=str(j),lw=.8)
                    axs[row,col].set_ylabel(('position [rad]','velocity [rad/s]','effort [Nm]')[row])
                    axs[row,col].grid(alpha=.25)
                axs[0,col].set_title(f'{side} joint order: see meta joint_names')
                axs[0,col].legend(ncol=7,fontsize=7)
                axs[-1,col].set_xlabel('elapsed wall time [s]')
            fig.savefig(out/'joints.png',dpi=140); plt.close(fig)
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    return report, out


def main():
    ap = argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv')
    ap.add_argument('--no-plots',action='store_true')
    ap.add_argument('--out')
    a = ap.parse_args()
    report, out = inspect(a.csv,plots=not a.no_plots,out=a.out)
    print(json.dumps(report,indent=2,allow_nan=False))
    print(f'analysis saved to {out}')


if __name__ == '__main__':
    main()
