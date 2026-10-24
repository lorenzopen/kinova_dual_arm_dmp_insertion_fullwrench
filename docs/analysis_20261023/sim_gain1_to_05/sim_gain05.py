"""Counterfactual replay: successful squeeze-gain-1 runs re-simulated at squeeze-gain 0.5.

SIMULATED DATA. Nothing here was measured at gain 0.5; the robot was not commanded.

Model per run, identified from that run's own log (route phase):
    fn(t) = k * ds(t - tau) + d(t)
k [N/m on the SUM of normals per m of ds per hand] and tau [s] by least squares on the
0.5-3 Hz band (where the run's own loop excites the arm); d(t) = everything else
(target changes, route/twist motion, neighbours, ATI noise), recovered exactly from the log.
Controller replayed with the logged update instants and the code's law
(controller.py: ds += clip(gain*GAIN*(2*target - fn), +-DS_MAX)).
Check 1: replay at gain 1 must reproduce the log.  Check 2: the same method applied
to the real gain-0.5 run 17:34 predicts gain 1, compared with the real gain-1 runs.

    python docs/analysis_20261023/sim_gain1_to_05/sim_gain05.py --archive E:/TESI/NVIDIA_LOG_RUNS
"""
import argparse, json, os
import numpy as np, pandas as pd
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
GAIN, DS_MAX, S_MAX, RELEASE = 1/(4*1500.), 2e-4, .060, .012
PREFIX = 'F75_Zt1_s9012_c100_five_cut800_'


def runs(archive):
    tw = os.path.join(ROOT, 'docs/runs/20261022_F75_Zt1_s9012_twist_ati')
    sg = os.path.join(ROOT, 'docs/runs/20261022_F75_Zt1_s9012_squeeze_gain')
    return {  # name: (csv, logged gain, cart_d)
        '20261022_152007': (f'{tw}/1_1520_riuscita/{PREFIX}20261022_152007.csv', 1., '5/1'),
        '20261022_152814': (f'{tw}/2_1528_riuscita/{PREFIX}20261022_152814.csv', 1., '5/1'),
        '20261022_184327': (f'{archive}/{PREFIX}20261022_184327.csv', 1., '60/7'),
        '20261022_173458': (f'{sg}/{PREFIX}20261022_173458.csv', .5, '60/7'),  # real gain 0.5, validation
    }


def load(path):
    d = pd.read_csv(path, low_memory=False)
    r = d[d.phase == 'route'].reset_index(drop=True)
    t = r.t.values.astype(float)
    ds = r.ds_mm.values/1000
    upd = np.flatnonzero(np.r_[False, np.diff(ds) != 0])        # rows logged right after an update
    return r, t, ds, upd


def zoh(t_q, t, x):
    i = np.clip(np.searchsorted(t, t_q, side='right')-1, 0, len(t)-1)
    return x[i]


def identify(t, ds, fn):
    fs = 100.
    tt = np.arange(t[0], t[-1], 1/fs)
    sos = signal.butter(2, [.5, 3.], 'band', fs=fs, output='sos')
    F = signal.sosfiltfilt(sos, np.interp(tt, t, fn))
    best = None
    for tau in np.arange(0., .40, .005):
        X = signal.sosfiltfilt(sos, zoh(tt-tau, t, ds))
        k = X @ F / (X @ X)
        res = np.mean((F-k*X)**2)
        if best is None or res < best[2]:
            best = (k, tau, res, 1-res/np.mean(F**2))
    return best


def replay(t, ds, fn, target, upd, k, tau, gain):
    """Re-run the integrator on the logged clock with fn = k*ds(t-tau) + d."""
    d = fn - k*zoh(t-tau, t, ds)                   # exact on the log rows
    ds_hist_t, ds_hist = [t[0]-1.], [ds[0]]         # piecewise-constant command history
    ds_sim = np.empty_like(ds); fn_sim = np.empty_like(fn)
    cur, ui = ds[0], set(upd.tolist())
    for j in range(len(t)):
        past = np.searchsorted(ds_hist_t, t[j]-tau, side='right')-1
        fn_sim[j] = k*ds_hist[max(past, 0)] + d[j]
        if j in ui:
            step = float(np.clip(gain*GAIN*(2*target[j]-fn_sim[j]), -DS_MAX, DS_MAX))
            cur = float(np.clip(cur+step, -RELEASE, S_MAX))
            ds_hist_t.append(t[j]); ds_hist.append(cur)
        ds_sim[j] = cur
    return ds_sim, fn_sim, d


def metrics(t, fn, target):
    fs = 100.
    tt = np.arange(t[0], t[-1], 1/fs)
    f = np.interp(tt, t, fn); g = np.interp(tt, t, 2*target)
    e = f-g
    m = tt-tt[0] > 1.
    sos = signal.butter(2, [.8, 2.5], 'band', fs=fs, output='sos')
    band = signal.sosfiltfilt(sos, signal.detrend(f, type='cubic' if False else 'linear'))[150:-150]
    fq, P = signal.welch(signal.detrend(f), fs=fs, nperseg=512)
    w = (fq > .5) & (fq < 3)
    k = np.argmax(np.where(w, P, 0))
    return dict(err_rms_N=float(np.sqrt(np.mean(e[m]**2))), min_N=float(f[m].min()),
                band_rms_N=float(band.std()), peak_Hz=float(fq[k]),
                peak_amp_N=float(np.sqrt(2*P[k]*(fq[1]-fq[0])*1.5)),
                ratio_median=float(np.median(f[m]/np.maximum(g[m], 1e-3))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--archive', default='E:/TESI/NVIDIA_LOG_RUNS')
    a = ap.parse_args()
    summary = {}
    figs = []
    for name, (path, g_log, cart_d) in runs(a.archive).items():
        r, t, ds, upd = load(path)
        fn, target = r.fn_N.values.astype(float), r.target_N.values.astype(float)
        k, tau, _, r2 = identify(t, ds, fn)
        ds1, fn1, _ = replay(t, ds, fn, target, upd, k, tau, g_log)          # check 1
        g_new = .5 if g_log == 1. else 1.
        ds2, fn2, dist = replay(t, ds, fn, target, upd, k, tau, g_new)
        summary[name] = dict(source=os.path.relpath(path, ROOT) if path.startswith(ROOT) else path,
                             cart_d=cart_d, logged_gain=g_log, simulated_gain=g_new,
                             model=dict(k_sum_N_per_m=float(k), tau_s=float(tau), band_fit_R2=float(r2)),
                             replay_check_max_abs_N=float(np.abs(fn1-fn).max()),
                             measured=metrics(t, fn, target), simulated=metrics(t, fn2, target))
        out = pd.DataFrame(dict(t=r.t, phase=r.phase, target_N=target,
                                ds_mm_measured=1000*ds, fn_N_measured=fn,
                                ds_mm_SIMULATED=1000*ds2, fn_N_SIMULATED=fn2, disturbance_N=dist))
        tag = f'{name}_gain{g_log:g}_SIMULATED_gain{g_new:g}'
        out.to_csv(os.path.join(HERE, tag+'.csv'), index=False, float_format='%.5f')
        figs.append((name, cart_d, g_log, g_new, t-t[0], target, fn, fn2, ds, ds2))
        print(f'{name} cart_d {cart_d}: k={k:.0f} N/m tau={1000*tau:.0f} ms R2={r2:.2f} '
              f'replay err {summary[name]["replay_check_max_abs_N"]:.1e} N')
        for kk in ('measured', 'simulated'):
            print('   %-9s' % kk, {x: round(v, 3) for x, v in summary[name][kk].items()})
    json.dump(dict(note='SIMULATED counterfactual, not robot data; see module docstring', runs=summary),
              open(os.path.join(HERE, 'summary.json'), 'w'), indent=1)

    # Plots: one row per run, force (left), command (middle), spectrum (right)
    fig, ax = plt.subplots(len(figs), 3, figsize=(16, 3.3*len(figs)), gridspec_kw=dict(width_ratios=[3, 2, 1.3]))
    for i, (name, cd, g0, g1, tt, tg, fn, fn2, ds, ds2) in enumerate(figs):
        lab0, lab1 = f'misurata, gain {g0:g}', f'SIMULATA, gain {g1:g}'
        ax[i, 0].plot(tt, 2*tg, 'k--', lw=1, label='richiesta (somma)')
        ax[i, 0].plot(tt, fn, color='0.6', lw=1, label=lab0)
        ax[i, 0].plot(tt, fn2, color='C3' if g1 < g0 else 'C0', lw=1.2, label=lab1)
        ax[i, 0].set_ylabel('forza normale somma [N]')
        ax[i, 0].set_title(f'{name}  (cart_d {cd})', loc='left', fontsize=10)
        ax[i, 0].legend(fontsize=8, loc='lower left')
        ax[i, 1].plot(tt, 1000*ds, color='0.6', lw=1, label=lab0)
        ax[i, 1].plot(tt, 1000*ds2, color='C3' if g1 < g0 else 'C0', lw=1.2, label=lab1)
        ax[i, 1].set_ylabel('ds per mano [mm]')
        for y, lab, c in ((fn, lab0, '0.6'), (fn2, lab1, 'C3' if g1 < g0 else 'C0')):
            f = np.interp(np.arange(tt[0], tt[-1], .01), tt, y)
            fq, P = signal.welch(signal.detrend(f), fs=100, nperseg=512)
            ax[i, 2].plot(fq, np.sqrt(2*P*(fq[1]-fq[0])*1.5), color=c, label=lab)
        ax[i, 2].set_xlim(0, 3); ax[i, 2].set_ylabel('ampiezza [N]')
    for j in range(3):
        ax[-1, j].set_xlabel(['tempo nella route [s]', 'tempo nella route [s]', 'frequenza [Hz]'][j])
    fig.suptitle('SIMULAZIONE controfattuale: run riuscite a gain 1 rigiocate a gain 0.5 '
                 '(ultima riga: validazione, run reale a 0.5 rigiocata a 1)', fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'sim_gain1_to_05.png'), dpi=130)
    print('written', HERE)


if __name__ == '__main__':
    main()
