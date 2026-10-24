"""Read-only experimental archive analysis, deduplicating copies by run filename.

Writes only beside this script. Uses bundled Python plus local plotting and MCAP
dependencies. Run inventory.py before/verify separately to verify source hashes.
"""
import csv
import hashlib
import importlib.util
import json
import os
import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
SOURCE=ROOT/'NVIDIA_RUNS'
OUT=Path(__file__).resolve().parent
for p in ['tmp/force_plot_deps','tmp/bag_analysis_deps']:
    sys.path.insert(0,str(ROOT/p))
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'tmp/mplconfig'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import butter,sosfiltfilt
from scipy.spatial.transform import Rotation
from mcap.reader import make_reader

spec=importlib.util.spec_from_file_location('previous_review',OUT.parent/'ffi_three_successes/review.py')
previous=importlib.util.module_from_spec(spec);spec.loader.exec_module(previous)


def dump(name,value):
    (OUT/name).write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')


def time_label(name):
    match=re.search(r'(2026\d{4})_(\d{6})',name)
    return f'{match[1][6:8]}/{match[1][4:6]} {match[2][:2]}:{match[2][2:4]}' if match else name[:30]


def piece(name):
    if name.startswith('F75_'):return name[4]
    return name[0]


def catalog():
    grouped=defaultdict(list)
    for p in SOURCE.rglob('*_meta.json'):grouped[p.name].append(p)
    records=[]
    for name,paths in sorted(grouped.items()):
        paths.sort(key=lambda p:(0 if 'runs_riuscitee' in p.parts else 1,str(p)))
        p=paths[0];meta=json.loads(p.read_text(encoding='utf-8'))
        cp=p.with_name(name.replace('_meta.json','.csv'))
        aliases=[q.with_name(q.name.replace('_meta.json','.csv')) for q in paths]
        # Newline differences do not create independent runs.
        hashes={hashlib.sha256(q.read_bytes().replace(b'\r\n',b'\n')).hexdigest() for q in aliases if q.exists()}
        curated=any('runs_riuscitee' in q.parts for q in paths)
        label=time_label(cp.name)
        for folder_part in reversed(p.parts[:-1]):
            named=re.match(r'^(2026\d{4})_(\d{4})_',folder_part)
            if named:
                label=f'{named[1][6:8]}/{named[1][4:6]} {named[2][:2]}:{named[2][2:4]}'
                break
        result=meta.get('result') or {}
        flag=bool(result.get('seat_found'))
        status=('curated_seat_found' if curated and flag else
                'curated_interrupted' if curated else
                'metadata_seat_found_only' if flag else 'other_or_interrupted')
        records.append({'id':cp.stem,'source':str(cp.relative_to(ROOT)),
                        'metadata':str(p.relative_to(ROOT)),'copies':len(paths),
                        'csv_copy_hashes_normalized':sorted(hashes),'copy_conflict':len(hashes)>1,
                        'aliases':[str(q.relative_to(ROOT)) for q in aliases],
                        'piece':piece(cp.name),'label':label,'curated':curated,
                        'status':status,'meta':meta})
    # The oldest insertions have no metadata; do not count follow2/baseline CSVs.
    for cp in sorted((SOURCE/'runs_riuscitee').rglob('*.csv')):
        if cp.with_name(cp.stem+'_meta.json').exists():continue
        if cp.name.startswith(('follow','baseline','wrench','verifica')):continue
        with cp.open(encoding='utf-8') as f:header=f.readline()
        if not header.startswith('t,phase,'):continue
        records.append({'id':cp.stem,'source':str(cp.relative_to(ROOT)), 'metadata':None,'copies':1,
                        'csv_copy_hashes_normalized':[], 'copy_conflict':False,'aliases':[str(cp.relative_to(ROOT))],
                        'piece':piece(cp.name),'label':time_label(cp.name),'curated':True,
                        'status':'legacy_partial' if 'mezzo_successo' in str(cp) else 'legacy_curated_no_metadata',
                        'meta':{}})
    dump('run_catalog.json',records)
    with (OUT/'run_catalog.csv').open('w',newline='',encoding='utf-8-sig') as f:
        keys=['id','piece','label','status','copies','copy_conflict','source']
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader()
        for r in records:w.writerow({k:r[k] for k in keys})
    return records


def bands(t,y):
    values={}
    fs=1/np.median(np.diff(t))
    for lo,hi in [(.8,2.5),(2.5,10)]:
        if t[-1]-t[0]<4:continue
        if hi>=fs/2:continue
        u,b,k=previous.bandpass(t-t[0],y,[lo,hi])
        values[f'{lo}_{hi}']={'rms_N':float(np.std(b[k])), 'p95_absolute_N':float(np.percentile(abs(b[k]),95))}
    return values


def analyse_run(rec):
    p=ROOT/rec['source']; d=previous.load(p);meta=rec['meta']
    route=d['phase']=='route'
    if route.sum()<10:raise ValueError('no usable route')
    clock='t_wall' if 't_wall' in d else 't'
    t=d[clock]-d[clock][route][0];tr=t[route]
    assert np.all(np.diff(t)>0)
    total=d['fn_N'][route];target=d['target_N'][route]
    normals=None
    if all('mq'+q+'L' in d for q in 'xyzw'):
        normals=[]
        for side in 'LR':
            quat=np.column_stack([d['mq'+c+side] for c in 'xyzw'])
            a=Rotation.from_quat(quat).apply(np.tile([1.,0.,0.],(len(t),1)))
            f=np.column_stack([d['F'+c+side] for c in 'xyz'])
            normals.append(np.maximum(np.einsum('ij,ij->i',f,a),0.))
    e=total/2-target
    allband=bands(tr,total/2)
    m={k:rec[k] for k in ['id','piece','label','status','source','metadata','copies']}
    m.update({'clock':clock,'duration_s':float(tr[-1]),'rows':int(route.sum()),
              'mean_force_per_hand_N':float(total.mean()/2),'mean_target_per_hand_N':float(target.mean()),
              'ratio_median':float(np.median(total/(2*target))),
              'tracking_rmse_mean_hand_N':previous.rms(e),'bias_mean_hand_N':float(e.mean()),
              'tracking_p95_abs_mean_hand_N':float(np.percentile(abs(e),95)),
              'tracking_max_abs_mean_hand_N':float(abs(e).max()),
              'bands_mean_hand':allband,'route_dt_ms_p50_p99_max':(1000*np.percentile(np.diff(tr),[50,99,100])).tolist(),
              'result':meta.get('result'),'execution':meta.get('execution'),'wrench_source':meta.get('wrench_source'),
              'controller_parameters':meta.get('controller_parameters'),'phases':{},'windows':{}})
    if normals is not None:
        m['normal_projection_sum_max_difference_N']=float(np.max(abs(sum(normals)[route]-total)))
        m['per_hand']={side:{'min_N':float(n[route].min()),'max_N':float(n[route].max()),
                                'bands':bands(tr,n[route])} for side,n in zip('LR',normals)}
    for name,lo,hi in [('interior',1.5,tr[-1]-1.5),('without_last_2s',0,tr[-1]-2)]:
        k=(tr>=lo)&(tr<=hi)
        m['windows'][name]={'from_s':lo,'to_s':float(hi),'rmse_mean_hand_N':previous.rms(e[k]),
                             'bias_mean_hand_N':float(e[k].mean())}
    for ph in ['to_mouth','route','slow_seat','seat_by_feel']:
        k=d['phase']==ph
        if not k.any():continue
        ee=d['fn_N'][k]/2-d['target_N'][k]
        m['phases'][ph]={'from_s':float(t[k][0]),'to_s':float(t[k][-1]),
                         'rmse_mean_hand_N':previous.rms(ee),'bias_mean_hand_N':float(ee.mean()),
                         'max_abs_error_mean_hand_N':float(abs(ee).max()),
                         'min_sum_N':float(d['fn_N'][k].min()),'max_sum_N':float(d['fn_N'][k].max()),
                         'min_target_sum_N':float(2*d['target_N'][k].min())}
    if clock=='t_wall':
        m['route_epoch']=[float(d[clock][route][0]),float(d[clock][route][-1])]
    ffi=(meta.get('execution') or {}).get('ffi')
    if ffi and np.isfinite(d.get('ffi_seq',np.array([np.nan]))).any():
        changed=np.r_[False,np.diff(d['ffi_seq'])>0]&np.isfinite(d['ffi_seq'])
        m['ffi']={}
        for ph in ['route','slow_seat','to_mouth']:
            k=changed&(d['phase']==ph)
            if not k.any():continue
            sample=d['phase']==ph
            mm={'updates':int(k.sum()),'dt_ms_p50_p95_max':np.percentile(d['ffi_dt_ms'][k],[50,95,100]).tolist(),
                'trim_abs_max_mm':float(abs(d['ffi_i_mm'][sample]).max()),
                'trim_at_travel_bound_fraction':float(np.mean(abs(d['ffi_i_mm'][k])>=1000*ffi['trim_limit_m']-.001))}
            for flag in ['rate_limited','position_limited','ff_limited','i_limited','dt_clamped']:
                if 'ffi_'+flag in d:mm[flag+'_fraction']=float(np.mean(d['ffi_'+flag][k]>.5))
            m['ffi'][ph]=mm
        pp=Path((meta.get('execution') or {}).get('offline_profile','').replace('\\','/')).name
        candidates=list(SOURCE.rglob(pp)) if pp else []
        if candidates:
            profile=candidates[0]
            with profile.open(newline='') as f:rows=list(csv.DictReader(f))
            max_rate=0.; nfast=0; nseg=0; max_delta=0.;max_dt=0.
            for ph in ['to_mouth','route','slow_seat']:
                rr=[r for r in rows if r['phase']==ph]
                if len(rr)<2:continue
                pt=np.array([float(r['t_in_phase']) for r in rr]);pn=np.array([float(r['target_N']) for r in rr])
                rates=abs(np.diff(pn)/np.diff(pt));j=int(np.argmax(rates))
                nfast+=int(np.sum(rates>ffi['k_eff_N_per_m']*ffi['total_speed_m_per_s']));nseg+=len(rates)
                if rates[j]>max_rate:
                    max_rate=float(rates[j]);max_delta=float(abs(pn[j+1]-pn[j]));max_dt=float(pt[j+1]-pt[j])
            m['nominal_plan']={'profile':str(profile.relative_to(ROOT)),
                               'max_force_slew_N_per_hand_s':max_rate,
                               'nominal_force_slew_limit_N_per_hand_s':ffi['k_eff_N_per_m']*ffi['total_speed_m_per_s'],
                               'max_nominal_speed_required_mm_s':1000*max_rate/ffi['k_eff_N_per_m'],
                               'over_slew_segments':nfast,'segments':nseg,
                               'largest_slope_segment_delta_N':max_delta,'largest_slope_segment_dt_s':max_dt}
    return {'d':d,'t':t,'route':route,'normal':normals,'metrics':m,'rec':rec}


def bag_inventory():
    inventory=[]
    for p in sorted((SOURCE/'bags').rglob('*.mcap')):
        try:
            with p.open('rb') as f:
                r=make_reader(f);s=r.get_summary()
                topics={c.topic:{'type':s.schemas[c.schema_id].name,
                                  'messages':s.statistics.channel_message_counts.get(c.id,0)}
                        for c in s.channels.values()}
                inventory.append({'path':str(p.relative_to(ROOT)),'start_epoch':s.statistics.message_start_time/1e9,
                                  'end_epoch':s.statistics.message_end_time/1e9,'messages':s.statistics.message_count,
                                  'topics':topics})
        except Exception as e:inventory.append({'path':str(p.relative_to(ROOT)),'error':str(e)})
    dump('bag_inventory.json',inventory)
    return inventory


def match_bag(run,bags):
    m=run['metrics']
    if 'route_epoch' not in m:return None
    lo,hi=m['route_epoch']
    scored=[]
    for b in bags:
        if 'error' in b:continue
        overlap=max(0,min(hi,b['end_epoch'])-max(lo,b['start_epoch']))/(hi-lo)
        if overlap>.95:scored.append((overlap,-abs(lo-b['start_epoch']),b))
    if not scored:return None
    scored.sort(key=lambda x:(x[0],x[1]),reverse=True)
    b=scored[0][2]
    m['bag_match']={'path':b['path'],'route_coverage_fraction':scored[0][0],
                    'candidate_count':len(scored),'selection':'route timestamp overlap then latest preceding bag start'}
    return b


def plot_individual(run):
    m,d,t,k=run['metrics'],run['d'],run['t'],run['route']
    fig,axes=plt.subplots(3,1,figsize=(12,10.5))
    fig.subplots_adjust(left=.09,right=.98,top=.86,bottom=.14,hspace=.56)
    fig.suptitle(m['piece']+' · '+m['label']+' · '+('FFI' if 'ffi' in m else 'riferimento storico'),
                 x=.09,ha='left',y=.97,fontsize=20,weight='bold',color='#172b40')
    b=m['bands_mean_hand'].get('0.8_2.5',{})
    fig.text(.09,.92,f'RMS banda della media mani: {b.get("rms_N",float("nan")):.3f} N | '
             f'Errore RMS intera route: {m["tracking_rmse_mean_hand_N"]:.3f} N | '
             f'Errore RMS tratto centrale: {m["windows"]["interior"]["rmse_mean_hand_N"]:.3f} N',fontsize=10)
    a=axes[0]
    if run['normal'] is not None:
        for n,c,lab in zip(run['normal'],['#1675bb','#df711c'],['Sinistra','Destra']):a.plot(t[k],n[k],color=c,lw=.9,label=lab)
    else:a.plot(t[k],d['fn_N'][k]/2,color='#1675bb',lw=.9,label='Media mani registrata')
    a.plot(t[k],d['target_N'][k],'k--',lw=1.3,label='Target/mano');a.axvspan(5,10,color='#dce6ef',alpha=.5)
    a.set(title='1 | Intera route: segnali registrati',ylabel='Forza per mano [N]',xlim=(0,t[k][-1]))
    a.set_ylim(bottom=0);a.legend(fontsize=9,ncol=3)
    a=axes[1];zoom=k&(t>=5)&(t<=10)
    if zoom.sum()>5:
        if run['normal'] is not None:
            for n,c in zip(run['normal'],['#1675bb','#df711c']):a.plot(t[zoom],n[zoom],color=c,lw=1)
        else:a.plot(t[zoom],d['fn_N'][zoom]/2,lw=1)
        a.plot(t[zoom],d['target_N'][zoom],'k--',lw=1.3)
        a.set_xlim(5,10)
    a.set(title='2 | Zoom 5–10 s: nessun filtro temporale aggiunto',ylabel='Forza per mano [N]')
    a=axes[2]
    for y,c,lab in [(d['fn_N'][k]/2,'#8556a3','Media delle mani'),
                    *((n[k],c,lab) for n,c,lab in zip(run['normal'] or [],['#1675bb','#df711c'],['Sinistra','Destra']))]:
        u,bb,kk=previous.bandpass(t[k],y,[.8,2.5]);a.plot(u[kk],bb[kk],color=c,lw=1,label=lab)
    a.axhline(0,color='grey',lw=.6);a.legend(ncol=3,fontsize=9)
    a.set(title='3 | Oscillazioni filtrate 0,8–2,5 Hz',ylabel='Variazione [N]',xlim=(0,t[k][-1]))
    for a in axes:a.set_xlabel('Tempo da inizio route [s]');a.grid(alpha=.2)
    fig.text(.09,.035,'RMS banda ≠ errore RMS completo ≠ picco-picco. Filtraggio: detrend cubico + Butterworth ordine 2 avanti/indietro;\n'
             'esclusi 1,5 s a ciascun bordo. Media mani = (N_L+N_R)/2. Le scale verticali si adattano a ciascuna run.',
             fontsize=9,color='#526173',linespacing=1.5)
    p=OUT/'per_run';p.mkdir(exist_ok=True);fig.savefig(p/(m['id']+'.png'),dpi=125);plt.close(fig)


def plot_groups(runs):
    core=[r for r in runs if r['metrics']['status']=='curated_seat_found' and 'ffi' in r['metrics']]
    fig,axes=plt.subplots(4,2,figsize=(15,14))
    fig.subplots_adjust(left=.075,right=.98,top=.91,bottom=.1,wspace=.24,hspace=.6)
    fig.suptitle('12 run FFI archiviate: Z, L, S e T',x=.075,ha='left',y=.975,fontsize=24,weight='bold',color='#172b40')
    fig.text(.075,.945,'Tre prove per pezzo · stessa configurazione FFI · media delle due mani · segnali senza filtri aggiuntivi',fontsize=11)
    for row,piece_name in enumerate('ZLST'):
        rr=[r for r in core if r['metrics']['piece']==piece_name]
        for i,(r,c) in enumerate(zip(rr,['#1675bb','#df711c','#23845a'])):
            m,d,t,k=r['metrics'],r['d'],r['t'],r['route']
            carry=np.isin(d['phase'],['route','slow_seat'])
            axes[row,0].plot(t[carry],d['fn_N'][carry]/2,color=c,lw=.9,label=m['label'].split()[-1])
            axes[row,1].plot(t[carry],d['fn_N'][carry]/2-d['target_N'][carry],color=c,lw=.9)
            if i==0:
                axes[row,0].plot(t[carry],d['target_N'][carry],'k--',lw=1.2,label='Target/mano')
                for a in axes[row]:a.axvspan(t[k][-1],t[carry][-1],color='#fbe5b9',alpha=.55)
        for a in axes[row]:a.grid(alpha=.2);a.set_xlabel('Tempo da inizio route [s]')
        axes[row,0].set(title=piece_name+' | Forza e target',ylabel='Forza media mani [N]')
        axes[row,0].set_ylim(bottom=0);axes[row,0].legend(fontsize=8,ncol=4,loc='upper left')
        axes[row,1].set(title=piece_name+' | Errore completo, inclusi transitori',ylabel='Misurata − target [N]')
        axes[row,1].axhline(0,color='grey',lw=.6)
    fig.text(.075,.03,'Giallo = slow_seat. Le scale verticali variano tra pezzi per mostrare i transitori senza tagliarli.\n'
             'Queste sono le 12 run FFI selezionate come riuscite nell’archivio: non costituiscono tutti i tentativi eseguiti.',
             fontsize=10,color='#526173',linespacing=1.5)
    fig.savefig(OUT/'01_12_FFI_forze_errori.png',dpi=155);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(15,5.5))
    fig.subplots_adjust(left=.07,right=.98,top=.79,bottom=.22,wspace=.3)
    fig.suptitle('Meno oscillazione non equivale sempre a meno errore',x=.07,ha='left',y=.97,fontsize=22,weight='bold',color='#172b40')
    fig.text(.07,.87,'Ogni punto è una run riuscita; confronto storico con condizioni e controllori differenti.',fontsize=11)
    groups={'Z':['Z'],'L':['L'],'S':['S'],'T':['T']}
    for i,pn in enumerate(groups):
        cur=[r for r in core if r['metrics']['piece']==pn]
        old=[r for r in runs if r['metrics']['piece']==pn and r['metrics']['status']=='curated_seat_found' and
             'ffi' not in r['metrics'] and not any(x in r['metrics']['id'] for x in ['offline','online'])]
        for subset,offset,c,lab in [(old,-.15,'#999da3','Storiche ATI'),(cur,.15,'#1675bb','FFI')]:
            subset=[r for r in subset if r['metrics'].get('wrench_source',{}) and
                    all(s=='ft' for s in r['metrics']['wrench_source'].values())]
            vals=[[],[],[]]
            for r in subset:
                m=r['metrics'];b=m['bands_mean_hand']['0.8_2.5']['rms_N']
                vals[0].append(b);vals[1].append(100*b/m['mean_force_per_hand_N']);vals[2].append(m['tracking_rmse_mean_hand_N'])
            for a,v in zip(axes,vals):
                if v:a.scatter(np.linspace(i+offset-.025,i+offset+.025,len(v)),v,color=c,s=35,label=lab if i==0 else None)
    for a,title,unit in zip(axes,['RMS oscillazione 0,8–2,5 Hz','RMS banda / forza media','RMS tracking intera route'],['N per media mani','%','N per media mani']):
        a.set_xticks(range(4),list(groups));a.set(title=title,ylabel=unit);a.set_ylim(bottom=0);a.grid(alpha=.2)
    axes[0].legend(fontsize=9)
    fig.text(.07,.035,'I confronti non isolano l’effetto del solo FFI: sono variati impedenza, damping, canali e condizioni.\n'
             'Il target dei piani può avere transitori che aumentano l’errore completo pur con oscillazioni in banda più piccole.',
             fontsize=10,color='#526173',linespacing=1.5)
    fig.savefig(OUT/'02_confronto_per_pezzo.png',dpi=155);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(13,9))
    fig.subplots_adjust(left=.08,right=.98,top=.85,bottom=.15,wspace=.22,hspace=.4)
    fig.suptitle('Il finale cambia molto tra pezzi',x=.08,ha='left',y=.975,fontsize=23,weight='bold',color='#172b40')
    fig.text(.08,.92,'Ultimi 4 s di route e slow_seat · stesse scale · zero = ultimo campione route di ogni run',fontsize=11)
    for a,pn in zip(axes.flat,'ZLST'):
        rr=[r for r in core if r['metrics']['piece']==pn]
        for i,(r,c) in enumerate(zip(rr,['#1675bb','#df711c','#23845a'])):
            d,t,k=r['d'],r['t'],r['route'];x=t-t[k][-1]
            keep=np.isin(d['phase'],['route','slow_seat'])&(x>=-4)
            a.plot(x[keep],d['fn_N'][keep]/2,color=c,lw=1,label=r['metrics']['label'].split()[-1])
            if i==0:a.plot(x[keep],d['target_N'][keep],'k--',lw=1.4,label='Target/mano')
        a.axvspan(0,2.5,color='#fbe5b9',alpha=.5);a.axvline(0,color='grey',lw=.6)
        a.set(title=pn,xlim=(-4,2.5),ylim=(2,8),xlabel='Tempo da fine route [s]',ylabel='Forza media mani [N]')
        a.grid(alpha=.2);a.legend(fontsize=8,ncol=4,loc='upper left')
    fig.text(.08,.035,'Il target tratteggiato appartiene alla prima ripetizione; i piani sono ripetuti e i tempi reali di campionamento variano.\n'
             'Giallo = slow_seat. Media mani = (N_L+N_R)/2; queste curve comprendono transitori e contatto, senza filtri aggiuntivi.',
             fontsize=10,color='#526173',linespacing=1.5)
    fig.savefig(OUT/'05_finali_per_pezzo.png',dpi=155);plt.close(fig)


def main():
    recs=catalog();bags=bag_inventory();runs=[];issues=[]
    selected=[r for r in recs if r['curated'] or r['status']=='metadata_seat_found_only']
    for rec in selected:
        try:
            run=analyse_run(rec);match_bag(run,bags);runs.append(run)
            print(rec['status'],rec['id'],'RMS',round(run['metrics']['tracking_rmse_mean_hand_N'],3),flush=True)
        except Exception as e:
            issues.append({'id':rec['id'],'error':str(e)});print('ISSUE',rec['id'],repr(e),flush=True)
    dump('analysis_issues.json',issues)
    dump('successful_and_reference_metrics.json',[r['metrics'] for r in runs])
    with (OUT/'successful_summary.csv').open('w',newline='',encoding='utf-8-sig') as f:
        keys=['id','piece','label','status','duration_s','mean_force_per_hand_N','ratio_median',
              'tracking_rmse_mean_hand_N','bias_mean_hand_N','band_rms_mean_hand_N','band_percent_mean',
              'rate_limited_percent','trim_abs_max_mm','source']
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader()
        for r in runs:
            m=r['metrics'];row={k:m.get(k) for k in keys};b=m['bands_mean_hand'].get('0.8_2.5',{}).get('rms_N')
            row.update(band_rms_mean_hand_N=b,band_percent_mean=100*b/m['mean_force_per_hand_N'] if b is not None else None,
                       rate_limited_percent=100*m.get('ffi',{}).get('route',{}).get('rate_limited_fraction',float('nan')),
                       trim_abs_max_mm=m.get('ffi',{}).get('route',{}).get('trim_abs_max_mm'))
            w.writerow(row)
    previous.style()
    for run in runs:plot_individual(run)
    plot_groups(runs)
    print('ANALYSED',len(runs),'issues',len(issues),'bags',len(bags),'conflicting copies',sum(r['copy_conflict'] for r in recs),flush=True)


if __name__=='__main__':main()
