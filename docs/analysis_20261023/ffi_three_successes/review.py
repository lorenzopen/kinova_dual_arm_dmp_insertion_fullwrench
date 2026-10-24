"""Reproducible analysis of three real FFI runs and two archived references.

Python: numpy, scipy, matplotlib. Reads CSVs only; no ROS or robot commands.
All force errors are measured minus target. Filter metrics describe a selected
frequency band, not full tracking quality or independently measured object slip.
"""
import csv
import hashlib
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'tmp/force_plot_deps'))
os.environ.setdefault('MPLCONFIGDIR', str(ROOT / 'tmp/mplconfig'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import butter, sosfiltfilt
from scipy.spatial.transform import Rotation

BASE = ROOT / 'docs/runs'
SOURCES = [
    ('FF puro 23/10 12:32', '20261023_F75_Zt1_s9012_offline', '*offline_202*.csv'),
    ('Online 22/10 20:20', '20261022_F75_Zt1_s9012_offline/4_2020_K1500_online_gain05', '*online_202*.csv'),
    ('FFI 17:43', '20261023_1743_F75_Zt1_s9012_ffi_riuscito', '*ffi_202*.csv'),
    ('FFI 17:51', '20261023_1751_F75_Zt1_s9012_ffi_riuscito', '*ffi_202*.csv'),
    ('FFI 17:56', '20261023_1756_F75_Zt1_s9012_ffi_riuscito', '*ffi_202*.csv'),
]
COLORS = ['#8a9199', '#9b62ae', '#1675bb', '#df711c', '#23845a']


def load(path):
    with path.open(newline='', encoding='utf-8') as fh:
        rows = list(csv.DictReader(fh))
    return {k: np.array([r[k] if k == 'phase' else float(r[k]) if r[k].strip() else np.nan
                         for r in rows], dtype=str if k == 'phase' else float) for k in rows[0]}


def rms(x):
    return float(np.sqrt(np.mean(x*x)))


def bandpass(t, x, limits):
    dt = np.median(np.diff(t))
    u = np.arange(t[0], t[-1], dt)
    y = np.interp(u, t, x)
    y -= np.polyval(np.polyfit(u-u[0], y, 3), u-u[0])
    b = sosfiltfilt(butter(2, limits, btype='band', fs=1/dt, output='sos'), y)
    keep = (u >= u[0]+1.5) & (u <= u[-1]-1.5)
    return u, b, keep


def spectrum(t, x):
    dt = np.median(np.diff(t))
    u = np.arange(t[0], t[-1], dt)
    y = np.interp(u, t, x)
    y -= np.polyval(np.polyfit(u-u[0], y, 3), u-u[0])
    w = np.hanning(len(y))
    return np.fft.rfftfreq(len(y), dt), 2*np.abs(np.fft.rfft(y*w))/w.sum()


def summary(y, target):
    e = y-target
    return {'mean_N': float(y.mean()), 'min_N': float(y.min()), 'max_N': float(y.max()),
            'target_mean_N': float(target.mean()), 'error_mean_N': float(e.mean()),
            'error_rms_N': rms(e), 'error_abs_p95_N': float(np.percentile(abs(e), 95)),
            'error_abs_max_N': float(abs(e).max()),
            'median_measured_over_target': float(np.median(y/target)),
            'below_80pct_target_sample_fraction': float(np.mean(y < .8*target))}


def analyse(label, folder, pattern):
    paths = list((BASE/folder).glob(pattern))
    assert len(paths) == 1, paths
    path = paths[0]
    d = load(path)
    meta = json.loads(path.with_name(path.stem+'_meta.json').read_text(encoding='utf-8'))
    route = d['phase'] == 'route'
    t = d['t_wall'] - d['t_wall'][route][0]
    assert np.all(np.diff(t) > 0)
    tr = t[route]
    normal, tangent = [], []
    for side in 'LR':
        q = np.column_stack([d['mq'+c+side] for c in 'xyzw'])
        a = Rotation.from_quat(q).apply(np.tile([1., 0., 0.], (len(t), 1)))
        f = np.column_stack([d['F'+c+side] for c in 'xyz'])
        n = np.einsum('ij,ij->i', f, a)
        normal.append(np.maximum(n, 0.))
        tangent.append(np.linalg.norm(f-n[:, None]*a, axis=1))
    check = float(np.max(abs(d['fn_N'][route]-sum(normal)[route])))
    assert check < .025, check
    target = d['target_N'][route]
    total = d['fn_N'][route]
    duration = float(tr[-1])
    m = {'label': label, 'source': str(path.relative_to(ROOT)),
         'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
         'argv': meta.get('argv'), 'result': meta.get('result'),
         'execution': meta.get('execution'), 'controller_parameters': meta.get('controller_parameters'),
         'duration_s': duration, 'projection_check_max_N': check,
         'csv_dt_ms_p50_p99_max': (1000*np.percentile(np.diff(tr), [50, 99, 100])).tolist(),
         'route_sum': summary(total, 2*target),
         'route_average_hand': summary(total/2, target),
         'per_hand': {}, 'phase_sum': {}, 'sixths': [], 'bands': {}}
    for j, name in enumerate(['left', 'right']):
        y = normal[j][route]
        u, b, keep = bandpass(tr, y, [.8, 2.5])
        z = (tr >= 5) & (tr <= 10)
        m['per_hand'][name] = dict(**summary(y, target), band_rms_N=float(np.std(b[keep])),
                                  band_peak_to_peak_N=float(np.ptp(b[keep])),
                                  raw_zoom_5_10_min_N=float(y[z].min()),
                                  raw_zoom_5_10_max_N=float(y[z].max()),
                                  tangential_over_normal_p95=float(np.percentile(
                                      tangent[j][route]/np.maximum(y, .01), 95)))
    for limits in ([.2, .8], [.8, 2.5], [2.5, 10.]):
        u, b, keep = bandpass(tr, total, limits)
        m['bands'][str(limits)] = {'rms_sum_N': float(np.std(b[keep])),
                                  'percent_mean_force': float(100*np.std(b[keep])/total.mean()),
                                  'peak_to_peak_sum_N': float(np.ptp(b[keep]))}
    f, amp = spectrum(tr, total)
    band = (f >= .8) & (f <= 2.5)
    p = np.flatnonzero(band)[np.argmax(amp[band])]
    m['spectrum_peak'] = {'Hz': float(f[p]), 'amplitude_sum_N': float(amp[p])}
    for phase in ['to_mouth', 'route', 'slow_seat']:
        k = d['phase'] == phase
        m['phase_sum'][phase] = dict(**summary(d['fn_N'][k], 2*d['target_N'][k]),
                                    start_route_s=float(t[k][0]), end_route_s=float(t[k][-1]))
    for j in range(6):
        k = (tr >= j*duration/6) & (tr < (j+1)*duration/6+1e-8)
        m['sixths'].append({'target_sum_median_N': float(np.median(2*target[k])),
                            'actual_sum_median_N': float(np.median(total[k]))})
    m['route_windows'] = {}
    for name, start, stop in [('without_last_2s',0,duration-2),
                               ('interior_1p5s_edges',1.5,duration-1.5),
                               ('first_1p5s',0,1.5), ('last_1p5s',duration-1.5,duration)]:
        k=(tr >= start)&(tr <= stop)
        m['route_windows'][name]=dict(start_s=start,end_s=stop,
                                     average_hand=summary(total[k]/2,target[k]))
    if 'ffi_seq' in d:
        changed = np.r_[False, np.diff(d['ffi_seq']) > 0] & np.isfinite(d['ffi_seq'])
        ff = {}
        for phase in ['to_mouth', 'route', 'slow_seat']:
            k = changed & (d['phase'] == phase)
            sample = d['phase'] == phase
            ff[phase] = {
                'distinct_updates': int(k.sum()),
                'update_dt_ms_p50_p95_max': np.percentile(d['ffi_dt_ms'][k], [50,95,100]).tolist(),
                'observed_update_wall_gap_ms_p50_p95_max':
                    (1000*np.percentile(np.diff(t[k]), [50,95,100])).tolist(),
                'sequence_skips_between_logged_updates': int(np.maximum(np.diff(d['ffi_seq'][k])-1,0).sum()),
                'trim_min_max_mm': [float(d['ffi_i_mm'][sample].min()), float(d['ffi_i_mm'][sample].max())],
                'ff_min_max_mm': [float(d['ffi_ff_mm'][sample].min()), float(d['ffi_ff_mm'][sample].max())],
                'trim_at_travel_bound_update_fraction': float(np.mean(abs(d['ffi_i_mm'][k]) >= 2.999)),
            }
            for flag in ['rate_limited', 'i_limited', 'ff_limited', 'position_limited', 'dt_clamped']:
                ff[phase][flag+'_update_fraction'] = float(np.mean(d['ffi_'+flag][k] > .5))
            ff[phase]['logged_control_error_rms_sum_N'] = rms(d['ffi_error_N'][k])
        m['ffi'] = ff
        # Is the rapid nominal request compatible with the configured speed?
        # This is a nominal-model feasibility calculation, not a plant ID.
        execution=meta['execution']
        plan_path=next((BASE/folder).glob('*offline_profile_*.csv'))
        with plan_path.open(newline='',encoding='utf-8') as fh:
            planned=list(csv.DictReader(fh))
        planned=[p for p in planned if p['phase']=='route']
        pt=np.array([float(p['t_in_phase']) for p in planned])
        pn=np.array([float(p['target_N']) for p in planned])
        late=np.flatnonzero(pt >= pt[-1]-1)
        step=int(late[np.argmax(np.diff(pn,prepend=pn[0])[late])])
        before=max(step-1,0)
        p_end=int(np.argmax(pn[step:])+step)
        delta=float(pn[p_end]-pn[before])
        keff=execution['ffi']['k_eff_N_per_m']
        vmax=execution['ffi']['total_speed_m_per_s']
        m['planned_final_rise']={
            'profile':str(plan_path.relative_to(ROOT)),
            'profile_sha256':hashlib.sha256(plan_path.read_bytes()).hexdigest(),
            'from_N_per_hand':float(pn[before]),'to_N_per_hand':float(pn[p_end]),
            'rise_duration_s':float(pt[p_end]-pt[before]),
            'profile_start_s':float(pt[before]),'profile_end_s':float(pt[p_end]),
            'nominal_delta_mm_per_hand':1000*delta/keff,
            'minimum_nominal_travel_time_s':delta/keff/vmax,
            'nominal_force_slew_limit_N_per_hand_s':keff*vmax,
        }
    u, b, keep = bandpass(tr, total, [.8, 2.5])
    return {'d': d, 't': t, 'route': route, 'normal': normal, 'metrics': m,
            'band': (u, b, keep), 'spectrum': (f, amp)}


def style():
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.titleweight': 'bold'})


def save(fig, name):
    fig.savefig(OUT/(name+'.png'), dpi=170, facecolor='white')
    plt.close(fig)


def plot_three(runs):
    fig, axes = plt.subplots(3, 3, figsize=(16, 10.8))
    fig.subplots_adjust(left=.06, right=.985, bottom=.14, top=.88, wspace=.22, hspace=.48)
    fig.suptitle('Tre inserimenti riusciti: forze e oscillazioni', x=.06, ha='left', y=.975,
                 fontsize=23, weight='bold', color='#172b40')
    fig.text(.06, .934, '23/10/2026 · FFI offline · k_eff 680 N/m · K_I 1,6 mm/(N s) · stessa route Z', fontsize=12)
    fig.text(.06, .906, 'Stesse scale nelle tre righe. Blu: sinistra; arancio: destra; tratteggio: target per mano.', fontsize=11)
    for row, run in enumerate(runs):
        d, t, route, m = (run[k] for k in ['d', 't', 'route', 'metrics'])
        a = axes[row,0]
        for j, c in enumerate(['#1675bb', '#df711c']):
            a.plot(t[route], run['normal'][j][route], color=c, lw=.8)
        a.plot(t[route], d['target_N'][route], 'k--', lw=1)
        a.axvspan(5,10,color='#dce6ef',alpha=.5)
        a.set(title=m['label']+' | intera route', xlim=(0,19.1), ylim=(0,7), ylabel='Forza per mano [N]')
        a = axes[row,1]
        zoom = route & (t >= 5) & (t <= 10)
        for j, c in enumerate(['#1675bb', '#df711c']):
            a.plot(t[zoom], run['normal'][j][zoom], color=c, lw=1.)
        a.plot(t[zoom], d['target_N'][zoom], 'k--', lw=1)
        a.set(title='Zoom 5–10 s · segnali registrati', xlim=(5,10), ylim=(3.2,5.3), ylabel='Forza per mano [N]')
        a = axes[row,2]
        u, b, keep = run['band']
        for j, c in enumerate(['#1675bb', '#df711c']):
            uu, bb, kk = bandpass(t[route], run['normal'][j][route], [.8,2.5])
            a.plot(uu[kk], bb[kk], color=c, lw=.8)
        a.plot(u[keep], b[keep], color='#8556a3', lw=1.1, label='Somma')
        r = m['bands']['[0.8, 2.5]']['rms_sum_N']
        hand = m['per_hand']['left']['band_rms_N']
        a.set(title=f'0,8–2,5 Hz | RMS/mano ≈ {hand:.2f} N'.replace('.',','),
              xlim=(0,19.1), ylim=(-1.1,1.1), ylabel='Variazione filtrata [N]')
        a.text(.03,.93,f'RMS somma {r:.2f} N'.replace('.',','),transform=a.transAxes,va='top',fontsize=9)
        a.axhline(0,color='grey',lw=.7)
        for a in axes[row]:
            a.grid(alpha=.2); a.set_xlabel('Tempo da inizio route [s]')
    fig.text(.06,.028,'Nel terzo pannello: viola = somma delle mani; blu/arancio = singole mani. RMS ≠ picco-picco.\n'
             'Detrend cubico, Butterworth ordine 2 avanti/indietro; esclusi 1,5 s ai bordi. Nessun filtro aggiunto ai primi due pannelli.',
             color='#526173',fontsize=10,linespacing=1.5)
    save(fig,'01_tre_run_forze')


def plot_phases(runs):
    fig, axes = plt.subplots(3, 3, figsize=(16, 10.8))
    fig.subplots_adjust(left=.06,right=.985,top=.88,bottom=.16,hspace=.45,wspace=.22)
    fig.suptitle('Dove resta il problema: transitori e limiti del comando', x=.06,ha='left',y=.975,
                 fontsize=21,weight='bold',color='#172b40')
    fig.text(.06,.93,'Tutte le fasi di trasporto: avvicinamento alla bocca → route → slow_seat. Tempo zero = inizio route.',fontsize=11)
    for col, run in enumerate(runs):
        d,t,m = run['d'],run['t'],run['metrics']
        mask = np.isin(d['phase'],['to_mouth','route','slow_seat'])
        end = m['phase_sum']['route']['end_route_s']
        for a in axes[:,col]:
            a.axvspan(-2,0,color='#e9eef3',alpha=.7)
            a.axvspan(end,21.2,color='#fbe5b9',alpha=.65)
            a.axvline(0,color='grey',lw=.7); a.axvline(end,color='grey',lw=.7)
            a.set_xlim(-2.15,21.2); a.grid(alpha=.2); a.set_xlabel('Tempo da inizio route [s]')
        a=axes[0,col]
        for j,c in enumerate(['#1675bb','#df711c']):
            a.plot(t[mask],run['normal'][j][mask],color=c,lw=.9,label=['Sinistra','Destra'][j])
        a.plot(t[mask],d['target_N'][mask],'k--',lw=1,label='Target/mano')
        a.set(title=m['label'],ylim=(2.5,7),ylabel='Forza per mano [N]')
        if col==0:a.legend(fontsize=8,ncol=3,loc='upper right')
        a=axes[1,col]
        a.plot(t[mask],(d['fn_N'][mask]-2*d['target_N'][mask])/2,color='#b34e59',lw=.9)
        a.axhline(0,color='grey',lw=.7)
        a.set(ylim=(-3,3),ylabel='Errore della media delle mani [N]')
        a=axes[2,col]
        for key,c,lab in [('ffi_ff_mm','#1675bb','Nominale'),('ffi_i_mm','#df711c','Trim I'),
                           ('ffi_output_mm','#263746','Totale')]:
            a.plot(t[mask],d[key][mask],color=c,lw=1,label=lab)
        limited=mask & (d['ffi_rate_limited']>.5)
        a.plot(t[limited],np.full(limited.sum(),1.5),'|',color='#a33d8d',ms=5,label='Limite velocità')
        a.axhline(-3,color='#df711c',ls=':',lw=.8)
        a.set(ylim=(-5,2),ylabel='Correzione per mano [mm]')
        if col==0:a.legend(fontsize=8,ncol=2,loc='lower left')
    fig.text(.06,.028,'Errore = forza misurata − target: valori negativi indicano forza insufficiente. La fascia gialla è slow_seat.\n'
             'I marcatori viola mostrano gli stati registrati con limite di velocità attivo; le percentuali nel report usano aggiornamenti FFI distinti.\n'
             'La linea arancio a −3 mm è il limite inferiore del solo trim, non del comando totale.',
             fontsize=10,color='#526173',linespacing=1.5)
    save(fig,'02_transitori_e_comando')


def plot_comparison(runs):
    fig, axes=plt.subplots(2,2,figsize=(14,10))
    fig.subplots_adjust(left=.075,right=.98,top=.87,bottom=.17,wspace=.28,hspace=.6)
    fig.suptitle('Confronto con le run precedenti',x=.075,ha='left',y=.975,fontsize=23,weight='bold',color='#172b40')
    fig.text(.075,.93,'Stessa analisi, intera route. Il confronto storico non isola il solo effetto del controllore.',fontsize=11)
    labels=['FF puro\n23/10 12:32','Online\n22/10 20:20','FFI\n17:43','FFI\n17:51','FFI\n17:56']
    band=[r['metrics']['bands']['[0.8, 2.5]'] for r in runs]
    values=[[b['rms_sum_N']/2 for b in band],[b['percent_mean_force'] for b in band],
            [r['metrics']['route_average_hand']['error_rms_N'] for r in runs]]
    titles=['Oscillazione: RMS della media delle mani','Oscillazione relativa alla forza media',
            'Errore completo rispetto al target']
    units=['N · banda 0,8–2,5 Hz','% · banda 0,8–2,5 Hz','N · RMS della media delle mani']
    for a,v,title,unit in zip(axes.flat,values,titles,units):
        a.bar(np.arange(5),v,color=COLORS,width=.65)
        a.set_xticks(np.arange(5),labels,fontsize=9)
        a.set(title=title,ylabel=unit,ylim=(0,max(v)*1.22))
        a.grid(axis='y',alpha=.2)
        for i,y in enumerate(v):a.text(i,y+max(v)*.035,f'{y:.2f}'.replace('.',','),ha='center',fontsize=10)
    a=axes[1,1]
    for run,c in zip(runs[1:],COLORS[1:]):
        f,amp=run['spectrum']
        a.plot(f,amp/2,color=c,lw=1.1,label=run['metrics']['label'])
    a.axvspan(.8,2.5,color='#dce6ef',alpha=.4)
    a.set(title='Spettro: controllare anche oltre 2,5 Hz',xlim=(0,10),
          xlabel='Frequenza [Hz]',ylabel='Ampiezza della media delle mani [N]')
    a.grid(alpha=.2); a.legend(fontsize=8)
    fig.text(.075,.042,'RMS della media delle mani = RMS della somma / 2; i valori delle singole mani sono nel report.\n'
             'Il feedforward puro lavora a circa metà target: il suo basso RMS assoluto non prova un controllo migliore.\n'
             'Banda: detrend cubico + Butterworth ordine 2 avanti/indietro, esclusi 1,5 s ai bordi. Spettro: Hann, dati detrendizzati.',
             fontsize=10,color='#526173',linespacing=1.5)
    save(fig,'03_confronto_storico')


def plot_final(runs):
    fig, axes=plt.subplots(2,1,figsize=(12,9),sharex=True)
    fig.subplots_adjust(left=.09,right=.98,top=.84,bottom=.19,hspace=.28)
    fig.suptitle('Il transitorio finale chiede più velocità del nominale',x=.09,ha='left',y=.975,
                 fontsize=20,weight='bold',color='#172b40')
    fig.text(.09,.925,'Target: 3,64 → 5,66 N/mano in 0,22 s. Modello a 680 N/m: circa 2,97 mm di chiusura.',fontsize=12)
    fig.text(.09,.891,'Con il limite di 2 mm/s il solo nominale richiede almeno 1,49 s: il riferimento non è seguibile in quel tempo.',fontsize=11)
    first=runs[0];d,t=first['d'],first['t']
    mask=np.isin(d['phase'],['route','slow_seat'])&(t>=17.8)&(t<=20.7)
    axes[0].plot(t[mask],d['target_N'][mask],'k--',lw=1.6,label='Target per mano')
    k=np.flatnonzero(mask&(t<=18.5))[-1]
    axes[1].plot(t[mask],1000*(d['target_N'][mask]-d['target_N'][k])/680,
                 'k--',lw=1.6,label='Variazione nominale richiesta dal target')
    for run,c in zip(runs,COLORS[2:]):
        d,t=run['d'],run['t']
        mask=np.isin(d['phase'],['route','slow_seat'])&(t>=17.8)&(t<=20.7)
        axes[0].plot(t[mask],d['fn_N'][mask]/2,color=c,lw=1.2,label=run['metrics']['label']+' · media mani')
        k=np.flatnonzero(mask&(t<=18.5))[-1]
        axes[1].plot(t[mask],d['ffi_ff_mm'][mask]-d['ffi_ff_mm'][k],color=c,lw=1.2,
                     label=run['metrics']['label']+' · nominale applicato')
    for a in axes:
        a.axvspan(18.76,18.98,color='#fbe5b9',alpha=.8)
        a.axvline(19.09,color='grey',ls=':',lw=1)
        a.grid(alpha=.2);a.legend(loc='upper left',fontsize=9,ncol=2)
        a.set_xlim(17.8,20.7)
    axes[0].set(ylabel='Forza per mano [N]',ylim=(2.8,6.5))
    axes[1].set(ylabel='Variazione nominale per mano [mm]',xlabel='Tempo da inizio route [s]',ylim=(-.8,4.3))
    fig.text(.09,.024,'Forze mostrate senza filtri aggiuntivi. Correzioni riferite al valore a 18,5 s; il trim I è mostrato nel grafico 02.\n'
             'La linea verticale segna circa l’ingresso in slow_seat. Il calcolo è sul modello nominale e sui limiti del software,\n'
             'non identifica la rigidezza reale e non spiega da solo le oscillazioni durante la parte centrale della route.',
             fontsize=10,color='#526173',linespacing=1.5)
    save(fig,'04_zoom_finale')


def main():
    OUT.mkdir(exist_ok=True)
    runs=[analyse(*spec) for spec in SOURCES]
    common=np.arange(1.5,min(r['metrics']['duration_s'] for r in runs)-1.5,.01)
    errors=[]
    for run in runs[2:]:
        d,t,k=run['d'],run['t'],run['route']
        errors.append(np.interp(common,t[k],d['fn_N'][k]-2*d['target_N'][k]))
    report={'method': {'force': 'ATI projected onto each measured flange +x, clipped >=0',
                       'error': 'measured minus target; no target time shift',
                       'route_statistics': 'all logged route samples; no transient omitted',
                       'band_method': 'median-dt interpolation, cubic detrend, order2 Butterworth SOS forward/backward; 1.5s edges omitted',
                       'band_fraction': 'band RMS sum divided by route mean sum',
                       'saturation': 'distinct ffi_seq changes only; i_limited includes rate clipping, not only travel saturation'},
            'runs':[r['metrics'] for r in runs],
            'new_run_error_correlation_common_1p5_to_17p5_s':np.corrcoef(errors).tolist()}
    (OUT/'metrics.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    keys=['label','rms_hand_N','rms_sum_N','band_percent','tracking_rms_mean_hand_N',
          'bias_mean_hand_N','min_hand_N','rate_limited_updates_percent','trim_abs_max_mm',
          'slow_seat_max_abs_error_mean_hand_N','rms_sum_2p5_10Hz_N']
    with (OUT/'summary.csv').open('w',newline='',encoding='utf-8') as fh:
        writer=csv.DictWriter(fh,fieldnames=keys);writer.writeheader()
        for run in runs:
            m=run['metrics'];b=m['bands']['[0.8, 2.5]']; ff=m.get('ffi',{}).get('route',{})
            row=dict(label=m['label'],rms_hand_N=m['per_hand']['left']['band_rms_N'],rms_sum_N=b['rms_sum_N'],
                     band_percent=b['percent_mean_force'],tracking_rms_mean_hand_N=m['route_average_hand']['error_rms_N'],
                     bias_mean_hand_N=m['route_average_hand']['error_mean_N'],
                     min_hand_N=min(x['min_N'] for x in m['per_hand'].values()),
                     rate_limited_updates_percent=100*ff.get('rate_limited_update_fraction',float('nan')),
                     trim_abs_max_mm=max(abs(x) for x in ff.get('trim_min_max_mm',[float('nan')])),
                     slow_seat_max_abs_error_mean_hand_N=m['phase_sum']['slow_seat']['error_abs_max_N']/2,
                     rms_sum_2p5_10Hz_N=m['bands']['[2.5, 10.0]']['rms_sum_N'])
            writer.writerow(row);print(json.dumps(row),flush=True)
    print('Error correlation:',report['new_run_error_correlation_common_1p5_to_17p5_s'])
    style();plot_three(runs[2:]);plot_phases(runs[2:]);plot_comparison(runs);plot_final(runs[2:])


if __name__=='__main__':
    main()
