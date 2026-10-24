"""Causal filter replay and explicitly conditional closed-loop sensitivity study.

Read NVIDIA_RUNS only; never connect to ROS/robot. Reuse the actual FFI classes
via AST, without importing the ROS application. Outputs live beside this file.
"""
import ast
import csv
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
REVIEW = OUT.parent / 'archive_review'
sys.path.insert(0, str(REVIEW))
import analyze as a
import bags
import numpy as np
from scipy.signal import lfilter
import matplotlib.pyplot as plt

DT = .005
WINDOWS = [0, .025, .05, .1, .25, .5]
COLORS = ['#263746', '#6ba6c7', '#40a69b', '#dd941f', '#9564b2', '#cf4e59']
tree = ast.parse((ROOT/'kinova_dual_arm_dmp_insertion/insert_ffi.py').read_text())
namespace = dict(np=np, dataclass=dataclass, asdict=asdict, Abort=RuntimeError)
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.ClassDef)
                             and n.name in ['FFISettings', 'NominalIntegral']],
                        type_ignores=[]), '<actual FFI classes>', 'exec'), namespace)
Settings, Controller = namespace['FFISettings'], namespace['NominalIntegral']


def dump(name, data):
    (OUT/name).write_text(json.dumps(data, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')


def ma(y, seconds):
    n = max(1, round(seconds/DT))
    # Constant extension before the first real sample; no zero-padding transient.
    return lfilter(np.ones(n)/n, [1], np.r_[np.repeat(y[0], n-1), y])[n-1:]


def band(t, y):
    _, b, keep = a.previous.bandpass(t-t[0], y, [.8, 2.5])
    return float(np.std(b[keep]))


def prepare(rec):
    # Load the CSV directly: avoid any writes/catalog refresh in the source.
    d = a.previous.load(ROOT/rec['source'])
    seq = d['ffi_seq']
    ix = np.flatnonzero(np.isfinite(seq) & (seq > 0) & np.r_[True, np.diff(seq) > 0])
    assert len(ix)>100 and np.all(np.diff(seq[ix]) == 1), rec['id']
    cfg = rec['meta']['execution']['ffi']
    settings = Settings(**{k:cfg[k] for k in Settings.__dataclass_fields__})
    target = cfg['pickup_measured_N_per_hand'] + cfg['k_eff_N_per_m']*d['ffi_ff_want_mm'][ix]/1000
    measurement = 2*target-d['ffi_error_N'][ix]
    te = d['t_wall'][ix]-d['t_wall'][ix[0]]
    ev = np.rint(te/DT).astype(int)
    assert np.all(np.diff(ev)>0)
    grid = np.arange(ev[-1]+1)*DT
    y0 = np.interp(grid, te, measurement)
    y0[ev] = measurement  # Exact unfiltered measurement at every controller tick.
    route0 = d['t_wall'][d['phase']=='route'][0]-d['t_wall'][ix[0]]
    route1 = d['t_wall'][d['phase']=='route'][-1]-d['t_wall'][ix[0]]
    route = (grid>=route0)&(grid<=route1)
    phases = d['phase'][ix]
    # The first sequence can be 2 because CSV logging starts after FFI starts.
    # Warm start every candidate from the SAME first recorded physical state.
    def make_controller():
        c = Controller(settings, cfg['k_eff_N_per_m'], cfg['pickup_measured_N_per_hand'])
        c.ff, c.trim, c.output = [float(d[k][ix[0]])/1000
                                for k in ['ffi_ff_mm','ffi_i_mm','ffi_output_mm']]
        c.time = 0.; c.sequence = int(seq[ix[0]])
        return c
    clock = np.cumsum(np.r_[0, d['ffi_dt_ms'][ix[1:]]/1000])
    c = make_controller(); baseline = [c.output]
    for k in range(1,len(ix)):
        baseline.append(c.update(target[k], measurement[k], clock[k], c.output))
    baseline = np.array(baseline)
    discrepancy_mm = float(np.max(abs(baseline-d['ffi_output_mm'][ix]/1000))*1000)
    assert discrepancy_mm < .002, (rec['id'], discrepancy_mm)
    p = dict(rec=rec, d=d, ix=ix, te=te, ev=ev, t=grid, y0=y0, target=target,
             baseline=baseline, clock=clock, make=make_controller, route=route,
             route0=route0, route1=route1, phase=phases,
             replay_error_mm=discrepancy_mm)
    return p


def simulate(p, window, stiffness, delay, step=DT):
    assert step == DT
    grid, ev, y0 = p['t'], p['ev'], p['y0']
    # Local force response to a DIFFERENCE from the original command only.
    # Same exogenous residual, target, contact, motion and update schedule.
    # Pure delay is a sensitivity model, not an identified model of the robot.
    src = np.searchsorted(ev, np.arange(len(grid))-round(delay/DT), side='right')-1
    delta = np.zeros(len(ev)); force = np.empty(len(grid)); seen = np.empty(len(grid))
    outputs = np.empty(len(ev)); outputs[0] = p['baseline'][0]
    c = p['make'](); n=max(1,round(window/DT)); total=0.; k=0
    initial=y0[0]
    for j in range(len(grid)):
        force[j] = y0[j] + (2*stiffness*delta[src[j]] if src[j]>=0 else 0.)
        total += force[j]-(force[j-n] if j>=n else initial)
        seen[j] = initial+total/n
        if k<len(ev) and j==ev[k]:
            if k:
                outputs[k]=c.update(p['target'][k], seen[j], p['clock'][k], c.output)
                delta[k]=outputs[k]-p['baseline'][k]
            k+=1
    assert k==len(ev)
    return dict(force=force, seen=seen, output=outputs, delta_mm=1000*delta)


def metrics(p, sim):
    route=p['route']; t=p['t']; y=sim['force']/2; seen=sim['seen']/2
    target=np.interp(t,p['te'],p['target'])
    seat=(t>=p['route1'])&(t<=p['route1']+1.6)
    return dict(route_band_physical_N=band(t[route],y[route]),
                route_band_seen_N=band(t[route],seen[route]),
                route_tracking_rmse_N=float(np.sqrt(np.mean((y[route]-target[route])**2))),
                first_seat_peak_physical_N=float(np.max(y[seat])),
                first_seat_peak_seen_N=float(np.max(seen[seat])),
                max_delta_command_mm=float(abs(sim['delta_mm']).max()),
                min_force_mean_hand_N=float(y.min()))


def filter_response_plot():
    f=np.linspace(.01,12,2000); rows=[]
    fig,axes=plt.subplots(1,2,figsize=(13,5.6))
    fig.subplots_adjust(left=.075,right=.98,top=.79,bottom=.23,wspace=.3)
    fig.suptitle('Moving average: pulisce il segnale, ma aggiunge ritardo',x=.075,ha='left',y=.96,fontsize=19,weight='bold')
    fig.text(.075,.87,'Filtro causale aggiuntivo a 200 Hz · calcolo esatto della risposta FIR',fontsize=11)
    for w,color in zip(WINDOWS[1:],COLORS[1:]):
        n=round(w/DT); h=abs(np.sin(np.pi*f*DT*n)/(n*np.sin(np.pi*f*DT)))
        attenuation=lambda hz: 1-abs(np.sin(np.pi*hz*DT*n)/(n*np.sin(np.pi*hz*DT)))
        delay=(n-1)*DT/2
        rows.append(dict(window_ms=w*1000,samples=n,delay_ms=delay*1000,
                         attenuation_amplitude_1Hz_percent=100*attenuation(1),
                         attenuation_amplitude_2Hz_percent=100*attenuation(2)))
        axes[0].plot(f,100*h,color=color,label=f'{w*1000:g} ms; ritardo {delay*1000:g} ms')
        axes[1].plot([w*1000],[100*attenuation(1)],'o',color=color,ms=8)
        if w>=.1:
            axes[1].annotate(f'{100*attenuation(1):.1f}%\n+{delay*1000:g} ms ritardo',
                             (w*1000,100*attenuation(1)),xytext=(5,8),textcoords='offset points',fontsize=9)
    axes[0].axvspan(.8,2.5,color='grey',alpha=.12,label='Banda osservata 0,8–2,5 Hz')
    axes[0].set(xlim=(0,10),ylim=(0,103),xlabel='Frequenza [Hz]',ylabel='Ampiezza conservata [%]')
    axes[0].legend(fontsize=8,loc='lower left')
    axes[1].set(xlim=(0,620),ylim=(-2,47),xlabel='Durata finestra [ms]',ylabel='Riduzione ampiezza a 1 Hz [%]')
    for ax in axes:ax.grid(alpha=.2)
    fig.text(.075,.055,'Una finestra da 100 ms toglie solo circa l’1,6% di un’onda a 1 Hz. Per toglierne circa il 36%\n'
             'servono 500 ms di storia, che aggiungono circa 248 ms di ritardo. Questo grafico riguarda il segnale, non la forza fisica.',fontsize=10,linespacing=1.5)
    fig.savefig(OUT/'01_risposta_filtro.png',dpi=160);plt.close(fig)
    dump('filter_theory.json',rows)


def bag_filter_plot(recs):
    catalogue=json.loads((REVIEW/'successful_and_reference_metrics.json').read_text())
    fig,axes=plt.subplots(2,2,figsize=(14,8))
    fig.subplots_adjust(left=.075,right=.98,top=.84,bottom=.16,hspace=.45,wspace=.24)
    fig.suptitle('Stessa forza registrata: cosa vedrebbe un ATI filtrato',x=.075,ha='left',y=.97,fontsize=20,weight='bold')
    fig.text(.075,.91,'Replay delle bag · curve filtrate soltanto dopo la registrazione: nessun cambiamento del robot',fontsize=11)
    results=[]
    for row,piece in enumerate('ZL'):
        rec=next(r for r in recs if r['piece']==piece)
        run=a.analyse_run(rec); m=next(m for m in catalogue if m['id']==rec['id'])
        normals,_=bags.normals_from_bag(run,bags.extract(m['bag_match']['path']))
        start=max(normals['L'][0][0],normals['R'][0][0]);end=min(normals['L'][0][-1],normals['R'][0][-1])
        u=np.arange(start,end,DT); y=sum(np.interp(u,*normals[s]) for s in 'LR')/2
        # Existing backend averages last two /ft_data_avg samples; reproduce this
        # approximately at uniform 200 Hz before adding the proposed window.
        y=ma(y,.01)
        t=u-m['route_epoch'][0]; route=(t>=0)&(t<=m['duration_s'])
        for w,col in zip(WINDOWS,COLORS):
            yy=ma(y,w)
            results.append(dict(id=rec['id'],piece=piece,window_ms=w*1000,
                                route_band_seen_N=band(t[route],yy[route])))
            for ax,(lo,hi) in zip(axes[row],[(5,10),(m['duration_s']-.8,m['duration_s']+1.8)]):
                k=(t>=lo)&(t<=hi)
                if w in [0,.1,.25,.5]:ax.plot(t[k],yy[k],color=col,lw=1,label='Attuale' if not w else f'MA {w*1000:g} ms')
        for ax in axes[row]:
            dd=run['d'];ax.plot(run['t'],dd['target_N'],'k--',lw=.9,label='Target/mano')
            ax.set(ylabel='Forza media mani [N]',xlabel='Tempo da inizio route [s]');ax.grid(alpha=.2)
        axes[row,0].set(xlim=(5,10),title=piece+' · '+rec['label']+' · oscillazione durante route')
        axes[row,1].set(xlim=(m['duration_s']-.8,m['duration_s']+1.8),title='Transizione route → slow_seat')
        # Scale only with visible samples, including the visible target segment.
        for ax in axes[row]:
            lo,hi=ax.get_xlim(); values=[]
            for line in ax.lines:
                xx,yy=line.get_data(); keep=(xx>=lo)&(xx<=hi)
                values.extend(yy[keep])
            ax.set_ylim(min(values)-.15,max(values)+.15)
        axes[row,0].legend(fontsize=8,ncol=2)
    fig.text(.075,.035,'Normale compensata e media delle due mani; il filtro proposto agirebbe solo sul feedback normale.\n'
             'La catena attuale include già medie ATI/backend. Le finestre qui indicate sono aggiuntive e causali.',fontsize=10,linespacing=1.5)
    fig.savefig(OUT/'02_bag_filtrate.png',dpi=160);plt.close(fig)
    dump('bag_filter_metrics.json',results)


def sensitivity_plot(prepared, traces, rows):
    fig,axes=plt.subplots(2,2,figsize=(14,9))
    fig.subplots_adjust(left=.075,right=.98,top=.84,bottom=.17,hspace=.43,wspace=.24)
    fig.suptitle('Se il filtro entra nel controller: simulazione condizionata',x=.075,ha='left',y=.97,fontsize=20,weight='bold')
    fig.text(.075,.91,'Modello centrale: 680 N/m per mano, ritardo 170 ms · stesso target, KI e limiti delle run',fontsize=11)
    for col,piece in enumerate('ZL'):
        p=next(p for p in prepared if p['rec']['piece']==piece);rid=p['rec']['id']
        x=p['t']-p['route0'];end=p['route1']-p['route0']
        for row,(lo,hi) in enumerate([(5,10),(end-.8,end+1.8)]):
            ax=axes[row,col];k=(x>=lo)&(x<=hi)
            ax.plot(x[k],p['y0'][k]/2,color=COLORS[0],lw=1.5,label='Forza registrata / base')
            for w in [.1,.25,.5]:
                sim=traces[(rid,w)];color=COLORS[WINDOWS.index(w)]
                ax.plot(x[k],sim['force'][k]/2,color=color,lw=1.15,label=f'Forza simulata MA {w*1000:g} ms')
                if w==.5:ax.plot(x[k],sim['seen'][k]/2,color=color,lw=.9,ls=':',label='MA 500 ms: forza vista da I')
            target=np.interp(p['t'],p['te'],p['target'])
            ax.plot(x[k],target[k],'k--',lw=.9,label='Target/mano')
            ax.set(title=piece+' · '+p['rec']['label']+(' · route' if row==0 else ' · ingresso slow_seat'),
                   ylabel='Forza media mani [N]',xlabel='Tempo da inizio route [s]',xlim=(lo,hi));ax.grid(alpha=.2)
        axes[0,col].legend(fontsize=7)
    fig.text(.075,.035,'Curve colorate = forza fisica prevista dal modello, linea puntinata = misura filtrata usata dall’integratore.\n'
             'Il modello conserva il disturbo e il contatto originali. Non prevede slittamento, perdita di presa o cambi di contatto:\n'
             'è un confronto di sensibilità, non una validazione sperimentale né una previsione certa del robot.',fontsize=10,linespacing=1.5)
    fig.savefig(OUT/'03_simulazione_forze.png',dpi=160);plt.close(fig)
    # Ranges across runs/models are scenario spans, never confidence intervals.
    fig,axes=plt.subplots(1,2,figsize=(13,6.3))
    fig.subplots_adjust(left=.08,right=.97,top=.80,bottom=.24,wspace=.3)
    fig.suptitle('Quanto dipende dal modello? Tutte le 12 run riuscite FFI',x=.08,ha='left',y=.96,fontsize=18,weight='bold')
    fig.text(.08,.87,'9 scenari per run: rigidezza 480 / 680 / 964 N/m, ritardo 120 / 170 / 230 ms',fontsize=11)
    baselines={r['id']:r for r in rows if r['window_ms']==0 and r['stiffness_N_m']==680 and r['delay_ms']==170}
    for i,piece in enumerate('ZLST'):
        for ai,metric in enumerate(['route_band_physical_N','route_tracking_rmse_N']):
            med=[];low=[];high=[]
            for w in WINDOWS:
                rr=[r for r in rows if r['piece']==piece and abs(r['window_ms']-w*1000)<.1]
                values=np.array([r[metric]/baselines[r['id']][metric] for r in rr])*100
                low.append(values.min());high.append(values.max());med.append(np.median(values))
            color=['#1675bb','#cf4e59','#40a69b','#9564b2'][i]
            xx=np.array(WINDOWS)*1000
            axes[ai].plot(xx,med,'o-',color=color,label=piece+' · mediana scenari')
            axes[ai].fill_between(xx,low,high,color=color,alpha=.10)
    for ax in axes:ax.axhline(100,color='grey',ls='--',lw=1);ax.grid(alpha=.2);ax.set_xlabel('Finestra MA aggiuntiva [ms]')
    axes[0].set(ylabel='Oscillazione fisica 0,8–2,5 Hz [% della base]',title='Forza fisica: più basso = meno oscillazione')
    axes[1].set(ylabel='RMSE forza–target [% della base]',title='Tracking durante route: più basso = meglio')
    axes[0].legend(fontsize=8,ncol=2)
    fig.text(.08,.045,'100% = controller attuale. Le aree rappresentano min–max tra run e modelli ipotizzati, non intervalli di confidenza.\n'
             'Una curva filtrata più liscia non garantisce una forza fisica più regolare. Le metriche escludono pickup e rilascio.',fontsize=10,linespacing=1.5)
    fig.savefig(OUT/'04_sensibilita_12_run.png',dpi=160);plt.close(fig)


def main():
    a.previous.style()
    recs=json.loads((REVIEW/'run_catalog.json').read_text())
    recs=[r for r in recs if r['status']=='curated_seat_found' and '_ffi_' in r['id']]
    assert len(recs)==12
    filter_response_plot();bag_filter_plot(recs)
    prepared=[];traces={};rows=[];validation=[]
    for rec in recs:
        p=prepare(rec);prepared.append(p)
        print('Prepared',rec['piece'],rec['label'],'replay max mm',p['replay_error_mm'],flush=True)
        val=dict(id=rec['id'],first_sequence=int(p['d']['ffi_seq'][p['ix'][0]]),
                 updates=len(p['ix']),replay_max_output_error_mm=p['replay_error_mm'])
        for w in WINDOWS:
            for stiffness in [480,680,964]:
                for delay in [.12,.17,.23]:
                    sim=simulate(p,w,stiffness,delay)
                    if w==0:
                        assert np.max(abs(sim['force']-p['y0']))<1e-8
                        assert np.max(abs(sim['delta_mm']))<1e-8
                    row=dict(id=rec['id'],piece=rec['piece'],label=rec['label'],window_ms=w*1000,
                             stiffness_N_m=stiffness,delay_ms=delay*1000,**metrics(p,sim))
                    rows.append(row)
                    if stiffness==680 and delay==.17:traces[(rec['id'],w)]=sim
        validation.append(val)
        print('Finished scenarios',rec['id'],flush=True)
    dump('replay_validation.json',validation);dump('simulation_metrics.json',rows)
    with (OUT/'simulation_metrics.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    sensitivity_plot(prepared,traces,rows)
    # Central scenario traces, for reproducible review beyond rendered graphs.
    for p in prepared:
        out={'t_s':p['t'],'baseline_mean_hand_N':p['y0']/2,
             'target_N':np.interp(p['t'],p['te'],p['target']),
             'control_time_s':p['te'],'baseline_command_m':p['baseline']}
        for w in WINDOWS:
            sim=traces[(p['rec']['id'],w)];tag=f'MA_{round(w*1000)}ms'
            out[tag+'_physical_mean_hand_N']=sim['force']/2
            out[tag+'_seen_mean_hand_N']=sim['seen']/2
            out[tag+'_delta_command_mm']=sim['delta_mm']
        np.savez_compressed(OUT/(p['rec']['id']+'_central.npz'),**out)
    print('DONE',len(rows),'scenario metrics',flush=True)


if __name__=='__main__':main()
