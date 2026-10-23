"""Plot actual 22 Oct ATI trials from the user's NVIDIA_LOG_RUNS archive.

python docs/analysis_20261023/plot_gain01_external.py --source E:/TESI/NVIDIA_LOG_RUNS
Raw archive is read-only. No extra smoothing on force-tracking plots.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from kinova_dual_arm_dmp_insertion.audit_run import load


def normals_and_tangents(d):
    normal, tangent = {}, {}
    for side in 'LR':
        q = np.column_stack([d['mq'+c+side] for c in ('x','y','z','w')])
        axis = Rotation.from_quat(q).apply(np.tile([1.,0.,0.],(len(q),1)))
        force = np.column_stack([d['F'+c+side] for c in 'xyz'])
        signed = np.einsum('ij,ij->i',force,axis)
        normal[side] = np.maximum(signed,0.)
        tangent[side] = np.linalg.norm(force-signed[:,None]*axis,axis=1)
    return normal,tangent


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',default='E:/TESI/NVIDIA_LOG_RUNS')
    ap.add_argument('--out',default=str(Path(__file__).parent/'gain01_archive'))
    args = ap.parse_args()
    source,out = Path(args.source),Path(args.out)
    out.mkdir(parents=True,exist_ok=True)
    runs = []
    for stamp,gain in (('181557',.01),('182705',.1),('184327',1.)):
        paths = list(source.glob(f'*20261022_{stamp}.csv'))
        assert len(paths)==1,paths
        path = paths[0]
        meta = json.loads(path.with_name(path.stem+'_meta.json').read_text(encoding='utf-8'))
        argv = meta['argv']
        assert float(argv[argv.index('--squeeze-gain')+1])==gain
        d = load(path)
        route = d['phase']=='route'
        t0 = d['t_wall'][route][0]
        t = d['t_wall']-t0
        assert np.all(np.diff(t)>0)
        normal,tangent = normals_and_tangents(d)
        results = dict(file=str(path.resolve()),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                       argv=argv,gain=gain,result=meta.get('result'),
                       actual_controller_parameters_recorded=bool(meta.get('controller_parameters')))
        g = d['fn_N'][route]
        target = 2*d['target_N'][route]
        tt = t[route]
        results.update(route_duration_s=float(tt[-1]),route_force_median_N=float(np.median(g)),
                       route_target_median_N=float(np.median(target)),
                       measured_over_target_median=float(np.median(g/target)),
                       route_minimum_normal_sum_N=float(g.min()),
                       route_error_rms_N=float(np.sqrt(np.mean((g-target)**2))),
                       normal_projection_log_max_diff_N=float(np.max(abs(g-normal['L'][route]-normal['R'][route]))),
                       csv_dt_ms_p50_p99_max=(1000*np.percentile(np.diff(tt),[50,99,100])).tolist())
        bins = []
        for j in range(6):
            m = route & (t>=tt[-1]*j/6) & (t<=tt[-1]*(j+1)/6)
            bins.append(dict(start_s=tt[-1]*j/6,end_s=tt[-1]*(j+1)/6,
                             measured_sum_N=float(np.median(d['fn_N'][m])),
                             target_sum_N=float(np.median(2*d['target_N'][m]))))
        results['route_sixths']=bins
        final = route & (t>=tt[-1]-1.)
        results['last_second']=dict(measured_sum_N=float(np.median(d['fn_N'][final])),
                                    target_sum_N=float(np.median(2*d['target_N'][final])))
        runs.append(dict(d=d,t=t,route=route,normal=normal,tangent=tangent,metrics=results))

    # Compare exactly the same elapsed part of the route; omit initial transient.
    start,end = 1.,min(r['metrics']['route_duration_s'] for r in runs)-.5
    uniform = np.arange(start,end,.01)
    for run in runs:
        d,t,m = run['d'],run['t'],run['route']
        y = np.interp(uniform,t[m],d['fn_N'][m])
        detrended = y-np.polyval(np.polyfit(uniform-uniform[0],y,3),uniform-uniform[0])
        win = np.hanning(len(y))
        f = np.fft.rfftfreq(len(y),.01)
        amp = 2*np.abs(np.fft.rfft(detrended*win))/win.sum()
        band = (f>=.8)&(f<=2.5)
        peak = np.flatnonzero(band)[np.argmax(amp[band])]
        filtered = sosfiltfilt(butter(2,[.8,2.5],btype='band',fs=100,output='sos'),detrended)
        interior = (uniform>=start+1.5)&(uniform<=uniform[-1]-1.5)
        run['spectrum']=(f,amp)
        run['band']=(uniform,filtered)
        run['metrics']['common_window']=dict(start_route_s=start,end_route_s=end,
            peak_Hz=float(f[peak]),peak_amplitude_normal_sum_N=float(amp[peak]),
            band_0p8_2p5_rms_normal_sum_N=float(np.std(filtered[interior])),
            force_median_N=float(np.median(y)))

    primary = runs[1]
    d,t,route = primary['d'],primary['t'],primary['route']
    carry = np.isin(d['phase'],['to_mouth','route','slow_seat','seat_by_feel'])
    fig,ax = plt.subplots(3,2,figsize=(14,10),layout='constrained')
    fig.suptitle('ATI reali — Z, 22/10 ore 18:27, gain 0.1, con twist\n'
                 'Route interrotta da Ctrl-C a 16.77 s — nessuna ulteriore media nei plot di tracking',fontsize=13)
    for side,label in (('L','sinistra'),('R','destra')):
        ax[0,0].plot(t[carry],primary['normal'][side][carry],lw=.9,label=label)
    ax[0,0].plot(t[carry],d['target_N'][carry],'k--',lw=1,label='SOCP per mano')
    ax[0,0].set_ylabel('Normale per mano [N]')
    ax[0,1].plot(t[carry],d['fn_N'][carry],label='ATI somma',lw=.9)
    ax[0,1].plot(t[carry],2*d['target_N'][carry],'k--',label='SOCP somma')
    ax[0,1].set_ylabel('Normale totale [N]')
    ax[1,0].plot(t[carry],d['fn_N'][carry]-2*d['target_N'][carry],lw=.9,label='misurata - richiesta')
    ax[1,0].axhline(0,color='k',ls='--',lw=.8)
    ax[1,0].set_ylabel('Errore totale [N]')
    for side,label in (('L','sinistra'),('R','destra')):
        ax[1,1].plot(t[carry],primary['tangent'][side][carry],lw=.9,label=label)
    ax[1,1].set_ylabel('Forza tangenziale per mano [N]')
    ax[2,0].plot(t[carry],d['ds_mm'][carry],lw=.9,label='ds: correzione per mano')
    ax[2,0].set_ylabel('Chiusura aggiuntiva comandata [mm]')
    last = route & (t>=t[route][-1]-5.)
    ax[2,1].plot(t[last],d['fn_N'][last],label='ATI somma',lw=1)
    ax[2,1].plot(t[last],2*d['target_N'][last],'k--',label='SOCP somma')
    ax[2,1].set_title('Ultimi 5 s registrati: non tutta la parte finale')
    ax[2,1].set_ylabel('Normale totale [N]')
    for a in ax.flat:
        a.set_xlabel('Tempo da inizio route [s]');a.grid(alpha=.25);a.legend(fontsize=8)
        if a is not ax[2,1]:
            a.axvline(0,color='gray',lw=.8,ls=':')
    fig.savefig(out/'gain01_forze.png',dpi=150)
    fig.savefig(out/'gain01_forze.pdf')
    plt.close(fig)

    fig,ax = plt.subplots(3,2,figsize=(13,10),layout='constrained')
    fig.suptitle('Confronto gain: 0.01, 0.1, 1 — stessa Z e twist\n'
                 'Parametri K/D reali non registrati; confronto tra prove, non repliche controllate',fontsize=12)
    colors=['C2','C0','C3']
    for row,(run,color) in enumerate(zip(runs,colors)):
        d,t,m = run['d'],run['t'],run['route']
        met = run['metrics'];cw=met['common_window']
        ax[row,0].plot(t[m],d['fn_N'][m],color=color,lw=.8,label='ATI somma')
        ax[row,0].plot(t[m],2*d['target_N'][m],'k--',lw=.9,label='SOCP somma')
        ax[row,0].axvspan(start,end,color='gray',alpha=.08,label='finestra spettrale comune')
        status='Ctrl-C' if met['result'].get('stopped') else 'sequenza completata'
        ax[row,0].set_title(f"gain {met['gain']:g} — {status}; rapporto mediano {met['measured_over_target_median']:.2f}")
        ax[row,0].set_ylabel('Normale totale [N]');ax[row,0].set_xlabel('Tempo route [s]')
        f,amp=run['spectrum']
        ax[row,1].plot(f,amp,color=color,lw=1)
        ax[row,1].set_xlim(0,5);ax[row,1].set_ylim(0,.6)
        ax[row,1].set_title(f"Picco 0.8–2.5 Hz: {cw['peak_amplitude_normal_sum_N']:.3f} N a {cw['peak_Hz']:.2f} Hz")
        ax[row,1].set_ylabel('Ampiezza spettrale, somma [N]');ax[row,1].set_xlabel('Frequenza [Hz]')
        for a in ax[row]:a.grid(alpha=.25)
        ax[row,0].legend(fontsize=7)
    fig.savefig(out/'confronto_gain.png',dpi=150)
    fig.savefig(out/'confronto_gain.pdf')
    plt.close(fig)
    fig,ax = plt.subplots(1,2,figsize=(12,4.3),layout='constrained')
    fig.suptitle('Gain 0.1 — ATI reali, 22/10 ore 18:27; prova interrotta a 16.77 s di route')
    d,t,m = primary['d'],primary['t'],primary['route']
    for side,label in (('L','sinistra'),('R','destra')):
        ax[0].plot(t[m],primary['normal'][side][m],lw=.9,label=label)
    ax[0].plot(t[m],d['target_N'][m],'k--',lw=1,label='SOCP per mano')
    ax[0].set_xlabel('Tempo route [s]');ax[0].set_ylabel('Forza normale per mano [N]')
    for index,color in ((1,'C0'),(2,'C3')):
        f,amp=runs[index]['spectrum']
        ax[1].plot(f,amp,color=color,label=f"gain {runs[index]['metrics']['gain']:g}")
    ax[1].set_xlim(.5,3.);ax[1].set_ylim(0,.55)
    ax[1].set_xlabel('Frequenza [Hz]');ax[1].set_ylabel('Ampiezza spettrale, somma delle mani [N]')
    ax[1].set_title('Stessa finestra temporale e stessa elaborazione')
    for a in ax:a.grid(alpha=.25);a.legend(fontsize=8)
    fig.savefig(out/'gain01_sintesi.png',dpi=160)
    plt.close(fig)
    metrics=[r['metrics'] for r in runs]
    (out/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n',encoding='utf-8')
    for m in metrics:
        print(json.dumps(m,ensure_ascii=False))


if __name__=='__main__':
    main()
