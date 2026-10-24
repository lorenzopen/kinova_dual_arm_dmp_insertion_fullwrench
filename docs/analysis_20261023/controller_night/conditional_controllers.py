"""Controller comparison with fixed recorded residual: explicitly conditional.

Not a prediction of insertion success or a validated robot simulation. This
study uses only 8 development runs and retains rejected/poor controller cases.
"""
import sys
sys.dont_write_bytecode=True
import json
import itertools
from datetime import datetime,timezone
from pathlib import Path
import damping_physics as dp
import standalone as st
import numpy as np
from scipy.signal import butter,sosfiltfilt
from scipy.linalg import expm
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

OUT=dp.OUT;ROOT=dp.ROOT;DT=.005
sys.path.insert(0,str(OUT.parent/'moving_average_sim'))
import simulate as old_replay


def freeze():
    path=OUT/'conditional_controller_design.json'
    if path.exists():return json.loads(path.read_text())
    fixed=json.loads((OUT/'frozen_design.json').read_text());models=[]
    for stiffness,delay in itertools.product([480.,680.,964.,1500.],[.12,.18,.23]):
        models.append(dict(id=f'delay_k{stiffness:g}_d{delay:g}',kind='delay',
                           stiffness_N_m_hand=stiffness,delay_s=delay))
    for tau in [.03,.12]:models.append(dict(id=f'lag680_d18_t{tau:g}',kind='lag',
        stiffness_N_m_hand=680.,delay_s=.18,tau_s=tau))
    for zeta in [.3,.7]:models.append(dict(id=f'resonant680_d18_z{zeta:g}',kind='resonant',
        stiffness_N_m_hand=680.,delay_s=.18,frequency_Hz=1.5,damping_ratio=zeta))
    design=dict(created_utc=datetime.now(timezone.utc).isoformat(),dt_s=DT,
        candidates=fixed['candidates'],models=models,baseline_id=fixed['baseline_id'],
        data_split='Only the 8 development runs, no original holdout CSVs',
        model_caveat='Assumed delta-command-to-force responses; stiffness1500 is an upper-gain stress, not newly identified stiffness. Empirical free simulation passed no provisional model gate. These models do not map a change in cart_d.',
        replay_equation='F_alt_sum(t)=F_recorded_sum(t)+G_model[s_alt(t)-s_baseline(t)]; recorded residual held fixed despite changed controller.',
        controller='same nominal target and FF priority, total and trim speed2mm/s, trim+-3mm,total+-12mm; P and filtered D absolute position-form terms, no hidden I in P/PD; I tracking anti-windup',
        initialization='same first logged FF/trim/output; P/PD have a fixed entry bias; I/PI/PID integral initialized for bumpless entry; derivative zero with first measured force',
        derivative='D on measured sum force; causal backward-Euler derivative filter tau50ms on FFI update times, uses actual elapsed wall dt; I/rate bounds retain logged clamped ffi_dt_ms',
        score='mean over 8 runs of worst model (RMSEroute/0.2 + RMSband0.8-2.5/0.05 + maxpositiveerror_firstseat/0.25 + p95absrouteerror/0.5); reject if mean-of-hands force<=0 in any scenario. No individual-contact guarantee.',
        limitation='Original phase timing/stop events are fixed; contact, geometry, residual, failure mechanisms and left/right delta sharing are not re-simulated; derivative input has FFI measurement cadence, not full 200Hz ATI.')
    with path.open('x',encoding='utf-8') as f:json.dump(design,f,indent=2)
    return design


def model_arrays(models,dt):
    M=len(models);kind=np.array([m['kind'] for m in models]);aa=np.zeros((M,2,2));bb=np.zeros((M,2))
    for i,m in enumerate(models):
        if m['kind']=='lag':
            pole=np.exp(-dt/m['tau_s']);aa[i,0,0]=pole;bb[i,0]=1-pole
        elif m['kind']=='resonant':
            omega=2*np.pi*m['frequency_Hz'];zeta=m['damping_ratio']
            block=np.array([[0.,1.,0.],[-omega**2,-2*zeta*omega,omega**2],[0,0,0]])
            discrete=expm(block*dt);aa[i]=discrete[:2,:2];bb[i]=discrete[:2,2]
    return kind,aa,bb


def band_rms(t,values,bounds):
    xx=np.linspace(-1,1,len(t));vander=np.column_stack([xx**j for j in range(4)])
    flat=values.reshape(len(t),-1)
    coef=np.linalg.lstsq(vander,flat,rcond=None)[0];detrended=flat-vander@coef
    sos=butter(2,bounds,btype='bandpass',fs=1/DT,output='sos')
    filtered=sosfiltfilt(sos,detrended,axis=0);keep=(t>=t[0]+1.5)&(t<=t[-1]-1.5)
    return np.std(filtered[keep],axis=0).reshape(values.shape[1:])


def replay(p,design,dt=DT,update_fn=None,nominal_wants=None,feedback_targets=None):
    candidates=design['candidates'];models=design['models'];C=len(candidates);M=len(models)
    assert dt==DT
    ev=p['ev'];t=p['t'];y_event=p['y0'][ev]
    if nominal_wants is not None:
        assert nominal_wants.shape==(len(ev),C)
    if feedback_targets is not None:
        assert feedback_targets.shape==(len(ev),C)
    ix=np.searchsorted(ev,np.arange(len(t)),side='right')-1
    baseline_force=y_event[ix];baseline_output=p['baseline'][ix];target=p['target'][ix]
    kp=np.array([c['kp_m_N'] for c in candidates])[:,None]
    ki=np.array([c['ki_m_N_s'] for c in candidates])[:,None]
    kd=np.array([c['kd_m_s_N'] for c in candidates])[:,None]
    tau=np.array([c['derivative_tau_s'] for c in candidates])[:,None]
    pd_tau=np.array([c.get('pd_rolloff_tau_s',0.) for c in candidates])[:,None]
    initialized=p['make']();shape=(C,M)
    ff=np.full(shape,initialized.ff);trim=np.full(shape,initialized.trim);current=np.full(shape,initialized.output)
    error0=(2*p['target'][0]-y_event[0] if feedback_targets is None
            else 2*feedback_targets[0,:,None]-y_event[0])
    integral=np.where(ki>0,np.full(shape,initialized.trim)-kp*error0,0.)
    bias=np.where(ki==0,np.full(shape,initialized.trim)-kp*error0,0.)
    derivative=np.zeros(shape);prior_measurement=np.full(shape,y_event[0])
    filtered_pd=np.broadcast_to(kp*error0,shape).copy()
    kind,aa,bb=model_arrays(models,dt);state=np.zeros((C,M,2))
    stiffness=np.array([m['stiffness_N_m_hand'] for m in models])[None,:]
    delay=np.rint(np.array([m['delay_s'] for m in models])/dt).astype(int)
    ringlen=delay.max()+2;history=np.zeros((ringlen,C,M))
    forces=np.empty((len(t),C,M),dtype=np.float64)
    output_events=np.empty((len(ev),C,M));rate_limited=np.zeros(shape);trim_limited=np.zeros(shape)
    maxslew=np.zeros(shape);maxoutput=np.zeros(shape);k=0
    for j,tt in enumerate(t):
        delayed=history[(j-delay)%ringlen,np.arange(C)[:,None],np.arange(M)[None,:]]
        response=np.where(kind[None,:]=='delay',delayed,state[:,:,0])
        physical=baseline_force[j]+2*stiffness*response
        forces[j]=physical/2
        if k<len(ev) and j==ev[k]:
            if k:
                integration_dt=p['clock'][k]-p['clock'][k-1]
                actual_dt=p['te'][k]-p['te'][k-1]
                derivative=(tau*derivative+physical-prior_measurement)/(tau+actual_dt)
                ref=(p['target'][k] if feedback_targets is None else feedback_targets[k,:,None])
                e=2*ref-physical
                raw_pd=kp*e-kd*derivative
                filtered_pd+=(actual_dt/(pd_tau+actual_dt))*(raw_pd-filtered_pd)
                # Zero roll-off preserves the original kernel arithmetic exactly.
                pterm=np.where(pd_tau>0,filtered_pd,kp*e)
                dterm=np.where(pd_tau>0,0.,-kd*derivative)
                trial=bias+pterm+integral+ki*e*integration_dt+dterm
                before=current.copy()
                bounded=st.bounded_update if update_fn is None else update_fn
                want=(p['d']['ffi_ff_want_mm'][p['ix'][k]]/1000
                      if nominal_wants is None else nominal_wants[k,:,None])
                ff,trim,current,limited=bounded(ff,trim,current,
                    want,trial,integration_dt)
                integral=np.where(ki>0,trim-bias-pterm-dterm,0.)
                trim_limited+=limited
                rate_limited+=(abs(current-before)>=st.SPEED*integration_dt-1e-12)
                maxslew=np.maximum(maxslew,abs(current-before)/integration_dt)
            prior_measurement=physical.copy();output_events[k]=current
            maxoutput=np.maximum(maxoutput,abs(current));k+=1
        history[j%ringlen]=current-baseline_output[j]
        delayed=history[(j-delay)%ringlen,np.arange(C)[:,None],np.arange(M)[None,:]]
        state=np.einsum('mij,cmj->cmi',aa,state)+bb[None,:,:]*delayed[:,:,None]
    assert k==len(ev)
    bindex=next(i for i,c in enumerate(candidates) if c['id']==design['baseline_id'])
    baseline_error_N=float(abs(forces[:,bindex,:]-baseline_force[:,None]/2).max())
    baseline_error_mm=float(abs(output_events[:,bindex,:]-p['baseline'][:,None]).max()*1000)
    assert baseline_error_N<1e-8,(p['rec']['id'],baseline_error_N)
    assert baseline_error_mm<1e-8,(p['rec']['id'],baseline_error_mm)
    assert np.max(maxslew)<=.002+1e-9
    route=p['route'];seat=(t>=p['route1'])&(t<=p['route1']+1.6)
    error=forces-target[:,None,None]
    metrics=dict(route_rmse_N=np.sqrt(np.mean(error[route]**2,axis=0)),
                 route_p95_abs_error_N=np.percentile(abs(error[route]),95,axis=0),
                 route_max_positive_error_N=np.max(error[route],axis=0),
                 route_max_negative_error_N=np.max(-error[route],axis=0),
                 first_seat_peak_N=np.max(forces[seat],axis=0),
                 first_seat_max_positive_error_N=np.maximum(0,np.max(error[seat],axis=0)),
                 first_seat_max_negative_error_N=np.maximum(0,np.max(-error[seat],axis=0)),
                 minimum_mean_hand_force_N=np.min(forces,axis=0),
                 route_band_rms_N=band_rms(t[route],forces[route],[.8,2.5]),
                 route_high_band_rms_N=band_rms(t[route],forces[route],[2.5,10]),
                 rate_limited_tick_fraction=rate_limited/(len(ev)-1),
                 trim_limited_tick_fraction=trim_limited/(len(ev)-1),
                 max_command_speed_mm_s=maxslew*1000,max_command_abs_mm=maxoutput*1000)
    rows=[]
    for c,cfg in enumerate(candidates):
        for m,model in enumerate(models):
            mm={key:float(value[c,m]) for key,value in metrics.items()}
            score=(mm['route_rmse_N']/.2+mm['route_band_rms_N']/.05+
                   mm['first_seat_max_positive_error_N']/.25+mm['route_p95_abs_error_N']/.5)
            if mm['minimum_mean_hand_force_N']<=0:score=1e9
            rows.append(dict(id=p['rec']['id'],piece=p['rec']['piece'],candidate=cfg['id'],family=cfg['family'],
                             model=model['id'],**mm,score=score))
    # Observed individual normals are available, but the delta model is scalar:
    # never pretend it predicts left/right contact force independently.
    observed=[]
    d=p['d'];active=np.isin(d['phase'],['route','slow_seat'])
    for side in 'LR':
        normal=R.from_quat(np.column_stack([d['mq'+q+side] for q in 'xyzw'])).apply(np.tile([1.,0,0],(len(d['phase']),1)))
        force=np.column_stack([d['F'+q+side] for q in 'xyz'])
        values=np.maximum(np.sum(normal*force,axis=1),0)
        observed.append(float(values[active].min()))
    validation=dict(id=p['rec']['id'],baseline_force_replay_max_error_N=baseline_error_N,
                    baseline_command_replay_max_error_mm=baseline_error_mm,
                    max_command_speed_mm_s=float(maxslew.max()*1000),
                    observed_min_individual_normal_N_L_R=observed,
                    note='Individual observed normals are real; simulated minima are mean-of-hands only.')
    return rows,validation,dict(t=t,force=forces,target=target,baseline_force=baseline_force/2,
                               event_time=p['te'],commands=output_events*1000)


def rank(design,rows):
    ranking=[]
    for cfg in design['candidates']:
        rr=[r for r in rows if r['candidate']==cfg['id']]
        run_ids=sorted({r['id'] for r in rr})
        worst_by_run=[max(r['score'] for r in rr if r['id']==rid) for rid in run_ids]
        ranking.append(dict(**cfg,mean_run_worst_model_score=float(np.mean(worst_by_run)),
            worst_score=float(max(worst_by_run)),
            minimum_mean_hand_force_N=min(r['minimum_mean_hand_force_N'] for r in rr),
            maximum_seat_positive_error_N=max(r['first_seat_max_positive_error_N'] for r in rr)))
    ranking.sort(key=lambda r:r['mean_run_worst_model_score'])
    dp.dump('conditional_controller_ranking.json',ranking)
    shortlist=[]
    for family in ['I','P','PI','PD','PID']:
        subset=[r for r in ranking if r['family']==family]
        if subset:shortlist.append(subset[0])
    dp.dump('conditional_shortlist_development_only.json',dict(
        shortlist=shortlist,
        caveat='Preliminary development shortlist under fixed recorded residual. No original holdout evaluated; no robot recommendation. Model adequacy gate failed for all empirical families.'))
    return ranking,shortlist


def finish(design,rows,traces,validations):
    dp.dump('conditional_controller_metrics.json',rows)
    # Flattened metrics are convenient for independent review, no nested dicts.
    with (OUT/'conditional_controller_metrics.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=dp.csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    dp.dump('conditional_replay_validation.json',validations)
    ranking,shortlist=rank(design,rows)
    ci={c['id']:i for i,c in enumerate(design['candidates'])}
    selected=[design['baseline_id']]+[r['id'] for r in shortlist if r['id']!=design['baseline_id']]
    mcentral=next(i for i,m in enumerate(design['models']) if m['id']=='delay_k680_d0.18')
    fig,axes=plt.subplots(4,2,figsize=(14,12));fig.subplots_adjust(left=.08,right=.98,top=.89,bottom=.13,hspace=.57,wspace=.25)
    fig.suptitle('Cambiare controllore sulle run: replay condizionato',x=.08,ha='left',y=.975,fontsize=18,weight='bold')
    fig.text(.08,.935,'Residuo registrato mantenuto fisso · stesso nominale e limiti · modello centrale 680 N/m/mano, ritardo 180 ms',fontsize=10)
    colors=['#263746','#dd941f','#1675bb','#40a69b','#9564b2','#cf4e59']
    for row,piece in enumerate('ZLST'):
        p,trace=next(v for v in traces.values() if v[0]['rec']['piece']==piece)
        x=trace['t']-p['route0'];end=p['route1']-p['route0']
        for col,(lo,hi) in enumerate([(5,10),(end-.8,end+1.8)]):
            ax=axes[row,col];k=(x>=lo)&(x<=hi)
            for id,color in zip(selected,colors):
                label='Registrata / FFI' if id==design['baseline_id'] else id.replace('_P',' P=').replace('_I',' I=').replace('_D',' D=')
                ax.plot(x[k],trace['force'][k,ci[id],mcentral],color=color,lw=1,label=label)
            ax.plot(x[k],trace['target'][k],'k--',lw=.8,label='Target/mano')
            ax.set(title=piece+(' · route' if col==0 else ' · route→slow_seat'),xlabel='Tempo da inizio route [s]',ylabel='Forza media mani [N]',xlim=(lo,hi));ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=6.5,ncol=2)
    fig.text(.08,.025,'Curve colorate = ipotesi alternative con dinamica locale del delta-comando, non forze misurate con quei controllori.\n'
             'La risposta del contatto, il residuo e i tempi delle fasi restano imposti dai log; nessuna previsione di slip o riuscita.\n'
             'Shortlist scelta sulle 8 run di sviluppo e su 16 modelli; quattro run riservate ancora escluse.',fontsize=10,linespacing=1.5)
    fig.savefig(OUT/'07_controllori_replay_condizionato.png',dpi=155);plt.close(fig)
    # Save only shortlist traces across models to keep data volume bounded.
    for id,(p,tr) in traces.items():
        np.savez_compressed(OUT/(id+'_conditional_shortlist.npz'),t_s=tr['t'],target_N=tr['target'],
            candidate_ids=np.array(selected),model_ids=np.array([m['id'] for m in design['models']]),
            force_mean_hand_N=tr['force'][:,[ci[id] for id in selected],:],
            command_time_s=tr['event_time'],command_mm=tr['commands'][:,[ci[id] for id in selected],:])
    for r in shortlist:print('Shortlist',json.dumps(r),flush=True)
    print('TOTAL',len(rows),'conditional scenarios',flush=True)


def main():
    design=freeze();frozen=json.loads((OUT/'frozen_design.json').read_text())
    catalogue=json.loads((OUT.parent/'archive_review/run_catalog.json').read_text())
    ids=[entry['id'] for piece in 'ZLST' for entry in frozen['split'][piece]['development']]
    rows=[];traces={};validations=[]
    for rid in ids:
        rec=next(r for r in catalogue if r['id']==rid);p=old_replay.prepare(rec)
        c=p['make']();assert c.k_eff==680 and c.s.total_speed_m_per_s==.002 and c.s.trim_limit_m==.003
        rr,val,trace=replay(p,design);rows.extend(rr);validations.append(val)
        traces[rid]=(p,trace)
        print('Replayed',rec['piece'],rid,'baseline max N',val['baseline_force_replay_max_error_N'],flush=True)
    finish(design,rows,traces,validations)


if __name__=='__main__':main()
