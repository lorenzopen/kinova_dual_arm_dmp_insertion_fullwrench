"""Plot real force runs, distinguishing recorded forces from band RMS.

Run with Python plus numpy, scipy and matplotlib; optional argument: online_2020.
No ROS or robot connection.
"""
import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
# Optional local dependencies for the bundled desktop Python.
LOCAL_DEPS = ROOT / 'tmp/force_plot_deps'
if LOCAL_DEPS.exists():
    sys.path.insert(0, str(LOCAL_DEPS))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import butter, sosfiltfilt
from scipy.spatial.transform import Rotation

OUT = Path(__file__).resolve().parent
RUNS = {
    'feedforward': {
        'source': 'docs/runs/20261023_F75_Zt1_s9012_offline/'
                  'F75_Zt1_s9012_c100_five_cut800_offline_20261023_123222.csv',
        'title': 'Feedforward puro: quanto variano le forze?',
        'subtitle': 'Run reale Z · 23/10/2026, ore 12:32 · sola fase route · tempo reale dal log',
        'output': 'feedforward_forze_chiare',
    },
    'online_2020': {
        'source': 'docs/runs/20261022_F75_Zt1_s9012_offline/4_2020_K1500_online_gain05/'
                  'F75_Zt1_s9012_c100_five_cut800_online_20261022_202036.csv',
        'title': 'Online, gain 0,5: quanto variano le forze?',
        'subtitle': 'Run reale Z · 22/10/2026, ore 20:20 · sola fase route · feedback ATI attivo',
        'output': 'online_2020_forze_chiare',
    },
}


def main(run='feedforward'):
    spec = RUNS[run]
    source = ROOT / spec['source']
    with source.open(newline='', encoding='utf-8') as fh:
        rows = [r for r in csv.DictReader(fh) if r['phase'] == 'route']
    def col(key):
        return np.array([float(r[key]) for r in rows])
    t = col('t_wall')
    t -= t[0]
    assert np.all(np.diff(t) > 0)
    target, total = col('target_N'), col('fn_N')
    normals = []
    for side in 'LR':
        quat = np.column_stack([col('mq' + q + side) for q in 'xyzw'])
        axis = Rotation.from_quat(quat).apply(np.tile([1., 0., 0.], (len(t), 1)))
        force = np.column_stack([col('F' + q + side) for q in 'xyz'])
        normals.append(np.maximum(np.einsum('ij,ij->i', force, axis), 0.))
    projection_error = float(np.max(np.abs(sum(normals) - total)))
    assert projection_error < .02
    dt = np.median(np.diff(t))
    u = np.arange(0., t[-1], dt)
    valid = (u >= 1.5) & (u <= u[-1] - 1.5)
    sos = butter(2, [.8, 2.5], btype='band', fs=1/dt, output='sos')
    filtered = []
    stats = {}
    for name, y in zip(['sinistra', 'destra', 'somma'], [*normals, total]):
        uniform = np.interp(u, t, y)
        residual = uniform - np.polyval(np.polyfit(u, uniform, 3), u)
        band = sosfiltfilt(sos, residual)
        filtered.append(band)
        stats[name] = {
            'band_rms_N': float(np.std(band[valid])),
            'band_peak_to_peak_N': float(np.ptp(band[valid])),
            'raw_min_N': float(y.min()), 'raw_max_N': float(y.max()),
            'raw_first_sixth_median_N': float(np.median(y[t < t[-1]/6])),
            'raw_last_sixth_median_N': float(np.median(y[t >= 5*t[-1]/6])),
            'zoom_5_to_10_s_peak_to_peak_N': float(np.ptp(y[(t >= 5) & (t <= 10)])),
        }
    def it(x, digits=2):
        return f'{x:.{digits}f}'.replace('.', ',')

    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.titleweight': 'bold', 'axes.titlesize': 13})
    fig, axes = plt.subplots(3, 1, figsize=(12, 11.4),
                             gridspec_kw={'height_ratios': [1.2, 1, 1]})
    fig.subplots_adjust(left=.085, right=.975, top=.88, bottom=.16, hspace=.65)
    fig.suptitle(spec['title'], x=.085, ha='left',
                 y=.978, fontsize=21, weight='bold', color='#172b40')
    fig.text(.085, .941, spec['subtitle'],
             fontsize=11, color='#526173')
    fig.text(.085, .909,
             f'Oscillazione RMS: sinistra {it(stats["sinistra"]["band_rms_N"])} N  |  '
             f'destra {it(stats["destra"]["band_rms_N"])} N  |  '
             f'somma {it(stats["somma"]["band_rms_N"])} N',
             fontsize=12, weight='bold', color='#172b40')
    colors = ['#1675bb', '#df711c', '#8556a3']
    labels = ['Mano sinistra', 'Mano destra']
    a = axes[0]
    a.plot(t, target, color='#2a3038', ls='--', lw=1.5, label='Target SOCP per mano')
    for y, color, label in zip(normals, colors, labels):
        a.plot(t, y, color=color, lw=.95, alpha=.9, label=label)
    a.axvspan(5, 10, color='#c5d6e5', alpha=.32, label='Intervallo dello zoom sotto')
    force_max = max(float(target.max()), *(float(y.max()) for y in normals))
    a.set(title='1  |  Forze effettive per mano e target richiesto', ylabel='Forza normale [N]',
          xlim=(0, t[-1]), ylim=(0, max(6, np.ceil(force_max+.15))))
    a.legend(loc='upper right', fontsize=9, ncol=2, framealpha=.95)
    a = axes[1]
    zoom = (t >= 5) & (t <= 10)
    for y, color, label in zip(normals, colors, labels):
        a.plot(t[zoom], y[zoom], color=color, lw=1.15, label=label)
    ymin = np.floor(min(y[zoom].min() for y in normals)*5)/5 - .1
    ymax = np.ceil(max(y[zoom].max() for y in normals)*5)/5 + .1
    a.set(title='2  |  Zoom di 5 secondi: segnali registrati, senza filtri aggiuntivi', ylabel='Forza normale [N]',
          xlim=(5, 10), ylim=(ymin, ymax))
    ybar = ymax - .32
    a.annotate('', xy=(9.73, ybar+.2), xytext=(9.73, ybar),
               arrowprops={'arrowstyle': '|-|', 'lw': 1.8, 'color': '#263746'})
    a.text(9.57, ybar+.1, '0,20 N', ha='right', va='center', fontsize=10,
           bbox={'facecolor': 'white', 'edgecolor': 'none', 'alpha': .9})
    a.legend(loc='upper left', ncol=2, fontsize=9)
    a = axes[2]
    for band, color, label in zip(filtered, colors, [*labels, 'Somma delle mani']):
        a.plot(u[valid], band[valid], color=color,
               lw=1.3 if label.startswith('Somma') else 1.,
               alpha=.85, label=label)
    a.axhline(0, color='#77818b', lw=.8)
    a.axhline(.2, color='#a2a8ae', ls=':', lw=.9)
    a.axhline(-.2, color='#a2a8ae', ls=':', lw=.9)
    band_min = min(float(band[valid].min()) for band in filtered)
    band_max = max(float(band[valid].max()) for band in filtered)
    band_limits = (min(-.75, np.floor((band_min-.1)*4)/4),
                   max(.95, np.ceil((band_max+.3)*4)/4))
    a.set(title='3  |  Solo oscillazioni nella banda 0,8–2,5 Hz',
          ylabel='Variazione filtrata [N]', xlim=(0, t[-1]), ylim=band_limits)
    a.legend(loc='upper right', fontsize=9, ncol=3)
    for a in axes:
        a.set_xlabel('Tempo da inizio route [s]')
        a.grid(alpha=.2)
    fig.text(.085, .035,
             'RMS = ampiezza efficace della componente filtrata; non è il massimo né il picco-picco.\n'
             'Pannello 3: ricampionamento al dt mediano, detrend cubico, Butterworth ordine 2 avanti/indietro;\n'
             'esclusi 1,5 s a ciascun bordo. Forze per mano proiettate sugli assi delle flange misurate.',
             fontsize=9, color='#526173', linespacing=1.5)
    fig.savefig(OUT / (spec['output'] + '.png'), dpi=160, facecolor='white')
    report = {'source': str(source.relative_to(ROOT)),
              'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'route_duration_s': float(t[-1]), 'samples': len(t),
              'projection_vs_logged_sum_max_difference_N': projection_error,
              'band_Hz': [.8, 2.5], 'excluded_edge_s': 1.5, 'forces': stats}
    (OUT / (spec['output'] + '.json')).write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', nargs='?', default='feedforward', choices=RUNS)
    main(parser.parse_args().run)
