"""Autonomous toy plant benchmark, NEVER presented as identified robot physics.

Vectorized controllers x plant scenarios; no robot imports or connections.
Frozen candidate space and data split precede all computed scores.
"""
import csv
import hashlib
import itertools
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
sys.path.insert(0,str(ROOT/'tmp/force_plot_deps'))
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'tmp/mplconfig'))
import numpy as np
from scipy.signal import butter, sosfiltfilt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DT=.005
CTRL=.025
SPEED=.002
TRIM=.003
TOTAL=.012
KFF=680.


def dump(name,v):
    (OUT/name).write_text(json.dumps(v,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')


def freeze():
    dest=OUT/'frozen_design.json'
    if dest.exists():return json.loads(dest.read_text())
    recs=json.loads((OUT.parent/'archive_review/run_catalog.json').read_text())
    split={}
    for piece in 'ZLST':
        selected=sorted([r for r in recs if r['status']=='curated_seat_found' and '_ffi_' in r['id']
                         and r['piece']==piece],key=lambda r:r['id'][-15:])
        assert len(selected)==3
        def brief(r):return dict(id=r['id'],source=r['source'],metadata=r['metadata'],
                                 csv_sha256=hashlib.sha256((ROOT/r['source']).read_bytes()).hexdigest())
        split[piece]=dict(development=[brief(r) for r in selected[:2]],holdout=[brief(selected[-1])])
    candidates=[]
    for kp,ki,kd in itertools.product([0.,.00005,.0001,.0002],[0.,.0004,.0008,.0016],[0.,.000005,.000015]):
        if kp==0 and ki==0 and kd>0:continue
        family=('FF_only' if kp==ki==kd==0 else 'I' if kp==kd==0 else
                'P' if ki==kd==0 else 'PD' if ki==0 else 'PI' if kd==0 else 'PID')
        if kp==0 and kd>0:family='ID'
        name=f'{family}_P{kp*1000:g}_I{ki*1000:g}_D{kd*1000:g}'
        candidates.append(dict(id=name,family=family,kp_m_N=kp,ki_m_N_s=ki,kd_m_s_N=kd,
                               derivative_tau_s=.05))
    models=[]
    for k,delay,kind in itertools.product([480.,680.,964.],[.12,.17,.23],['delay','lag','resonant']):
        models.append(dict(id=f'k{k:g}_d{delay:g}_{kind}',stiffness_N_m_hand=k,delay_s=delay,
                           kind=kind,lag_tau_s=.06 if kind=='lag' else 0.,
                           resonance_Hz=2. if kind=='resonant' else 0.,
                           damping_ratio=.3 if kind=='resonant' else 1.))
    design=dict(created_utc=datetime.now(timezone.utc).isoformat(),
                note='Predeclared design, not fitted parameters or acceptance guarantees.',
                split=split,candidates=candidates,models=models,
                baseline_id='I_P0_I1.6_D0',dt_s=DT,control_dt_s=CTRL,
                tests=dict(duration_s=24,initial_target_N_hand=4,
                           up_step_at_s=3,up_step_N_hand=2,down_step_at_s=9,
                           output_disturbance_at_s=15,disturbance_N_hand=.5,
                           disturbance_removed_s=19,measurement_noise_std_N_hand=.013,seed=23102026),
                score='max over models of RMSE_total/0.2 + positive up overshoot/0.25 + down undershoot/0.25 + steady absolute error/0.1; reject force<=0',
                limits=dict(speed_m_s_hand=SPEED,trim_m_hand=TRIM,total_m_hand=TOTAL,
                            ff_priority=True),
                caveat='No contact loss/slip, robot servo/geometry or physical minimum force guarantee modeled. Same scalar normal response on both hands is assumed.')
    with dest.open('x',encoding='utf-8') as f:json.dump(design,f,indent=2,ensure_ascii=False)
    return design


def bounded_update(ff,trim,current,want,trial,dt):
    step=SPEED*dt
    total_lo=np.maximum(-TOTAL,current-step);total_hi=np.minimum(TOTAL,current+step)
    trim_lo=np.maximum(-TRIM,trim-step);trim_hi=np.minimum(TRIM,trim+step)
    ff_lo=np.maximum.reduce([np.full_like(ff,-TOTAL+TRIM),ff-step,total_lo-trim_hi])
    ff_hi=np.minimum.reduce([np.full_like(ff,TOTAL-TRIM),ff+step,total_hi-trim_lo])
    ff=np.clip(want,ff_lo,ff_hi)
    trim=np.clip(trial,np.maximum(trim_lo,total_lo-ff),np.minimum(trim_hi,total_hi-ff))
    return ff,trim,ff+trim,abs(trim-trial)>1e-12


def run(design):
    candidates=design['candidates'];models=design['models'];C=len(candidates);M=len(models)
    shape=(C,M)
    kp=np.array([c['kp_m_N'] for c in candidates])[:,None]
    ki=np.array([c['ki_m_N_s'] for c in candidates])[:,None]
    kd=np.array([c['kd_m_s_N'] for c in candidates])[:,None]
    tau=np.array([c['derivative_tau_s'] for c in candidates])[:,None]
    stiffness=np.array([m['stiffness_N_m_hand'] for m in models])[None,:]
    lag=np.array([m['lag_tau_s'] for m in models])[None,:]
    kind=np.array([m['kind'] for m in models])
    delay=np.rint(np.array([m['delay_s'] for m in models])/DT).astype(int)
    ringlen=delay.max()+2;history=np.zeros((ringlen,C,M))
    t=np.arange(0,24+DT/2,DT);target=np.where((t>=3)&(t<9),6.,4.)
    disturbance=np.where((t>=15)&(t<19),1.,0.) # Sum = 2 x 0.5 N/hand.
    noise=np.random.default_rng(23102026).normal(0,.026,size=(len(t),M))
    ff=np.zeros(shape);trim=np.zeros(shape);current=np.zeros(shape);integ=np.zeros(shape)
    displacement=np.zeros(shape);velocity=np.zeros(shape);filterstate=np.full(shape,8.)
    forces=np.empty((len(t),C,M),np.float32);commands=np.empty_like(forces)
    saturation=np.zeros(shape);max_slew=np.zeros(shape)
    tick=round(CTRL/DT);prior=current.copy()
    omega=2*np.pi*2.;zeta=.3
    for j,tt in enumerate(t):
        past=history[(j-delay)%ringlen,np.arange(C)[:,None],np.arange(M)[None,:]]
        # Model state is extra displacement relative to the 4 N pickup preload.
        displacement[:,kind=='delay']=past[:,kind=='delay']
        sel=kind=='lag'
        displacement[:,sel]+=(1-np.exp(-DT/lag[:,sel]))*(past[:,sel]-displacement[:,sel])
        sel=kind=='resonant'
        velocity[:,sel]+=DT*(omega**2*(past[:,sel]-displacement[:,sel])-2*zeta*omega*velocity[:,sel])
        displacement[:,sel]+=DT*velocity[:,sel]
        physical=8+2*stiffness*displacement+disturbance[j]
        measured=physical+noise[j][None,:]
        derivative=(measured-filterstate)/(tau+DT)
        filterstate+=DT*derivative
        if j%tick==0:
            e=2*target[j]-measured
            p=kp*e;d=-kd*derivative
            proposed_i=integ+ki*e*CTRL
            trial=p+d+proposed_i
            ff,trim,current,limited=bounded_update(ff,trim,current,(target[j]-4)/KFF,trial,CTRL)
            # Tracking anti-windup only for controllers that HAVE integral action;
            # P/PD never gain hidden integral state through saturation handling.
            integ=np.where(ki>0,trim-p-d,0.)
            saturation+=limited
            max_slew=np.maximum(max_slew,abs(current-prior)/CTRL)
            prior=current.copy()
        history[j%ringlen]=current
        forces[j]=physical/2;commands[j]=current*1000
    assert np.max(max_slew)<=SPEED+1e-10
    # Noncausal filter appears ONLY in offline scoring, never the controller.
    sos=butter(2,[.8,2.5],btype='bandpass',fs=1/DT,output='sos')
    err=forces-target[:,None,None]
    filtered=sosfiltfilt(sos,err,axis=0)
    ringing=(t>=3.6)&(t<=14.5)
    steady=((t>=7.5)&(t<9))|((t>=13.5)&(t<15))|((t>=22.5)&(t<=24))
    rows=[]
    for c,cfg in enumerate(candidates):
        for m,plant in enumerate(models):
            y=forces[:,c,m];error=err[:,c,m]
            rising=(t>=3)&(t<9);falling=(t>=9)&(t<15)
            rise_times=[t[rising][np.flatnonzero(y[rising]>=4+fraction*2)[0]]-3
                        if np.any(y[rising]>=4+fraction*2) else None for fraction in [.1,.9]]
            rmse=float(np.sqrt(np.mean(error[t>=3]**2)))
            overshoot=float(max(0.,np.max(y[rising]-6)))
            undershoot=float(max(0.,np.max(4-y[falling])))
            steadyerror=float(np.mean(abs(error[steady])))
            positive=bool(y.min()>0)
            score=rmse/.2+overshoot/.25+undershoot/.25+steadyerror/.1 if positive else 1e9
            rows.append(dict(candidate=cfg['id'],family=cfg['family'],model=plant['id'],
                             rmse_N_hand=rmse,up_overshoot_N_hand=overshoot,down_undershoot_N_hand=undershoot,
                             steady_mean_absolute_error_N_hand=steadyerror,
                             band_error_rms_N_hand=float(np.std(filtered[ringing,c,m])),
                             rise_10_s=rise_times[0],rise_90_s=rise_times[1],
                             rise_10_90_s=(rise_times[1]-rise_times[0] if None not in rise_times else None),
                             minimum_force_N_hand=float(y.min()),
                             limited_tick_fraction=float(saturation[c,m]/(len(t)//tick+1)),
                             max_command_abs_mm=float(abs(commands[:,c,m]).max()),
                             max_command_speed_mm_s=float(max_slew[c,m]*1000),score=score))
    return t,target,forces,commands,rows


def verify_update():
    # Match the real FFI exactly for P=D=0, including changes in desired nominal.
    import ast
    from dataclasses import dataclass,asdict
    source=(ROOT/'kinova_dual_arm_dmp_insertion/insert_ffi.py').read_text()
    tree=ast.parse(source);ns=dict(dataclass=dataclass,asdict=asdict,np=np,Abort=RuntimeError)
    exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.ClassDef)
                                 and n.name in ['FFISettings','NominalIntegral']],type_ignores=[]),'<FFI>','exec'),ns)
    c=ns['NominalIntegral'](ns['FFISettings'](),680,4)
    c.update(4,8,0,0)
    ff=trim=out=np.zeros((1,1));largest=0
    for j in range(1,1001):
        target=4+2*np.sin(j*.05);measured=8+3*np.cos(j*.07);e=2*target-measured
        actual=c.update(target,measured,j*CTRL,c.output)
        ff,trim,out,_=bounded_update(ff,trim,out,(target-4)/680,trim+.0016*e*CTRL,CTRL)
        largest=max(largest,abs(actual-out.item()))
    assert largest<1e-12,largest
    return largest


def main():
    design=freeze();match=verify_update()
    t,target,forces,commands,rows=run(design)
    dump('standalone_metrics.json',rows)
    with (OUT/'standalone_metrics.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    ranking=[]
    for cfg in design['candidates']:
        rr=[r for r in rows if r['candidate']==cfg['id']]
        ranking.append(dict(**cfg,worst_score=max(r['score'] for r in rr),
                            mean_rmse_N=float(np.mean([r['rmse_N_hand'] for r in rr])),
                            worst_up_overshoot_N=max(r['up_overshoot_N_hand'] for r in rr),
                            worst_down_undershoot_N=max(r['down_undershoot_N_hand'] for r in rr),
                            worst_steady_error_N=max(r['steady_mean_absolute_error_N_hand'] for r in rr)))
    ranking.sort(key=lambda r:r['worst_score']);dump('standalone_ranking.json',ranking)
    shortlist=[]
    for family in ['I','P','PI','PD','PID']:
        subset=[r for r in ranking if r['family']==family]
        if subset:shortlist.append(subset[0])
    dump('standalone_shortlist_not_robot_validated.json',shortlist)
    selected=[design['baseline_id']]+[r['id'] for r in shortlist if r['id']!=design['baseline_id']]
    ci={c['id']:i for i,c in enumerate(design['candidates'])}
    central=[next(i for i,m in enumerate(design['models']) if m['stiffness_N_m_hand']==680
                  and m['delay_s']==.17 and m['kind']==kind) for kind in ['delay','lag','resonant']]
    fig,axes=plt.subplots(3,2,figsize=(14,11));fig.subplots_adjust(left=.075,right=.98,top=.87,bottom=.13,hspace=.45,wspace=.25)
    fig.suptitle('Primo screening: controllori su impianti sintetici autonomi',x=.075,ha='left',y=.97,fontsize=19,weight='bold')
    fig.text(.075,.925,'Stessi limiti FFI · target 4→6→4 N/mano · disturbo +0,5 N/mano a 15–19 s · rumore sensoriale 0,013 N/mano',fontsize=10)
    colors=['#263746','#d87320','#1675bb','#40a69b','#9564b2','#cf4e59']
    for row,m in enumerate(central):
        for id,color in zip(selected,colors):
            label='FFI attuale' if id==design['baseline_id'] else id.replace('_P',' P=').replace('_I',' I=').replace('_D',' D=')
            axes[row,0].plot(t,forces[:,ci[id],m],color=color,lw=1,label=label)
            axes[row,1].plot(t,commands[:,ci[id],m],color=color,lw=1)
        axes[row,0].plot(t,target,'k--',lw=.9,label='Target/mano')
        axes[row,0].set(ylabel='Forza sintetica per mano [N]',title=design['models'][m]['kind']+' · 680 N/m, ritardo 170 ms')
        axes[row,1].set(ylabel='Correzione totale per mano [mm]',title='Nominale + feedback, massimo 2 mm/s')
        for ax in axes[row]:ax.grid(alpha=.2);ax.set(xlim=(2,24),xlabel='Tempo [s]')
    axes[0,0].legend(fontsize=7,ncol=2)
    fig.text(.075,.035,'Questi modelli NON sono stati identificati né validati sul robot. La graduatoria è esplorativa; nessun candidato è pronto al robot.\n'
             'La risposta risonante è una perturbazione scelta per stress, non una risonanza misurata. La forza sui due lati è assunta simmetrica.',fontsize=10,linespacing=1.5)
    fig.savefig(OUT/'01_screening_sintetico.png',dpi=155);plt.close(fig)
    np.savez_compressed(OUT/'standalone_traces_shortlist.npz',t_s=t,target_N_hand=target,
                        candidate_ids=np.array(selected),model_ids=np.array([design['models'][m]['id'] for m in central]),
                        force_N_hand=forces[:,[ci[id] for id in selected]][:,:,central],
                        command_mm=commands[:,[ci[id] for id in selected]][:,:,central])
    dump('standalone_validation.json',dict(actual_FFI_update_max_error_m=match,
         candidate_count=len(design['candidates']),model_count=len(design['models']),scenarios=len(rows),
         command_speed_limit_respected=all(r['max_command_speed_mm_s']<=2+1e-8 for r in rows)))
    print('Validated FFI scalar update error m',match,flush=True)
    print('Finished autonomous scenarios',len(rows),flush=True)
    for r in shortlist:print(json.dumps(r),flush=True)


if __name__=='__main__':main()
