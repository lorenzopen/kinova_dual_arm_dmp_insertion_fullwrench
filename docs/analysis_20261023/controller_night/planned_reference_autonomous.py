"""Autonomous uncertain plants driven by four real offline profile shapes.

Synthetic prescribed phase times, no contact events/board and no recorded-force
residual. Frozen offset-zero controllers, no retuning on these scenarios.
"""
import planned_reference as r
import json
import sys
from datetime import datetime,timezone
import numpy as np

OUT=r.OUT;c=r.c


def freeze():
    path=OUT/'planned_reference_autonomous_design.json'
    if path.exists():return json.loads(path.read_text())
    design=r.freeze()
    result=dict(created_utc=datetime.now(timezone.utc).isoformat(),
        candidates=[x for x in design['candidates'] if x['clock_offset_s']==0.],models=design['models'],
        variants=[dict(id='clean_fixed',noise_N_hand=0.,jitter=False,wave_N_hand=0.),
            dict(id='noise_jitter_wave',noise_N_hand=.013,jitter=True,wave_N_hand=.15)],
        timing='2s prelude at first route target, route duration=last profile route time, then slow_seat duration=last profile slow_seat time,4s terminal hold. Synthetic prescribed schedule is known in this benchmark; controllers only query current phase.',
        initialization='Symmetric steady force at first route target, zero nominal/trim/output, pickup reference equals that force. This differs from recorded measured pickups and is not replay.',
        disturbances='Second variant: mean-hand measurement noise0.013N at control events; physical additive0.15N/hand sinusoid1.2Hz from6s. Common noise across controllers/models. Fixed25ms or jitter15-35ms on2.5ms lattice, seed24402026.',
        limits='Original FF priority, FF/trim/sum2mm/s, FF+-9mm trim+-3mm total+-12mm.',
        score='Mean over4 profile shapes of worst model and disturbance variant; same weighted original-target routeRMSE,0.8-2.5Hz band,first-seat positive error and routep95 as conditional study.',
        models_not_validated=True,contact_bilateral_assumed_symmetric=True,
        no_claim='No Kinova cart_d mapping, contact loss/slip/event timing prediction or full-wrench margin validation. No future measured force. No holdout read.')
    with path.open('x',encoding='utf-8') as handle:json.dump(result,handle,indent=2)
    return result


def reference_arrays(t,profile,tables,configs):
    route_end=2.+float(profile['route'][0][-1])
    seat_end=route_end+float(profile['slow_seat'][0][-1])
    initial=float(profile['route'][1][0]);terminal=float(profile['slow_seat'][1][-1])
    original=np.full(len(t),initial);planned=np.empty((len(t),len(configs)))
    route=(t>=2)&(t<route_end);seat=(t>=route_end)&(t<seat_end)
    for mask,phase,start in [(route,'route',2.),(seat,'slow_seat',route_end)]:
        ts,force=profile[phase];original[mask]=np.interp(t[mask]-start,ts,force)
    original[t>=seat_end]=terminal
    for i,cfg in enumerate(configs):
        planned[:,i]=initial;planned[t>=seat_end,i]=terminal
        mode='original' if cfg['plan']=='logged' else cfg['plan']
        for mask,phase,start in [(route,'route',2.),(seat,'slow_seat',route_end)]:
            ts,force=tables[mode][phase];planned[mask,i]=np.interp(t[mask]-start,ts,force)
    feedback=np.column_stack([original if cfg['feedback_reference']=='original' else planned[:,i] for i,cfg in enumerate(configs)])
    return original,planned,feedback,route_end,initial


def simulate(profile,design,dt):
    c.DT=dt;tables=r.plan_profiles(profile);configs=design['candidates']
    scenarios=[dict(m,variant=v['id'],noise_N_hand=v['noise_N_hand'],jitter=v['jitter'],wave_N_hand=v['wave_N_hand'])
        for v in design['variants'] for m in design['models']]
    C=len(configs);M=len(scenarios);shape=(C,M)
    duration=2.+profile['route'][0][-1]+profile['slow_seat'][0][-1]+4.
    t=np.arange(round(duration/dt)+1)*dt
    original,planned,reference,route_end,initial=reference_arrays(t,profile,tables,configs)
    kp=np.array([x['kp_m_N'] for x in configs])[:,None];ki=np.array([x['ki_m_N_s'] for x in configs])[:,None]
    roll=np.array([x['pd_rolloff_tau_s'] for x in configs])[:,None]
    ff=np.zeros(shape);trim=ff.copy();out=ff.copy();integral=ff.copy();pstate=ff.copy()
    maxspeed=ff.copy();limited=ff.copy();tick_counts=np.zeros(M)
    kind,aa,bb=c.model_arrays(scenarios,dt);state=np.zeros((C,M,2))
    delay=np.rint(np.array([x['delay_s'] for x in scenarios])/dt).astype(int)
    stiffness=np.array([x['stiffness_N_m_hand'] for x in scenarios])[None,:]
    ringlen=delay.max()+2;history=np.zeros((ringlen,C,M))
    rng=np.random.default_rng(24402026);jit=[0]
    while jit[-1]*.0025<duration:jit.append(jit[-1]+int(rng.integers(6,15)))
    jitter=np.array(jit)*.0025;fixed=np.arange(0,duration+.025,.025)
    schedules=[np.rint((jitter if v['jitter'] else fixed)/dt).astype(int) for v in design['variants']]
    next_ticks=np.zeros(len(schedules),dtype=int);last=np.zeros(len(schedules))
    noise=np.random.default_rng(24402027).normal(size=4000)
    forces=np.empty((len(t),C,M));commands=np.empty_like(forces)
    force_wave=np.array([x['wave_N_hand'] for x in scenarios])[None,:]
    model_count=len(design['models'])
    for j,tt in enumerate(t):
        delayed=history[(j-delay)%ringlen,np.arange(C)[:,None],np.arange(M)[None,:]]
        response=np.where(kind[None,:]=='delay',delayed,state[:,:,0])
        physical=2*initial+2*stiffness*response+2*force_wave*np.sin(2*np.pi*1.2*(tt-6))*(tt>=6)
        forces[j]=physical/2
        for v,variant in enumerate(design['variants']):
            tick=next_ticks[v]
            if tick>=len(schedules[v]) or j!=schedules[v][tick]:continue
            next_ticks[v]+=1
            if j==0:continue
            actual=float(tt-last[v]);last[v]=tt
            sl=(slice(None),slice(v*model_count,(v+1)*model_count))
            measured=physical[sl]+2*variant['noise_N_hand']*noise[tick]
            error=2*reference[j,:,None]-measured
            pnew=pstate[sl]+actual/(roll+actual)*(kp*error-pstate[sl])
            pnew=np.where(roll>0,pnew,kp*error)
            trial=pnew+integral[sl]+ki*error*actual
            before=out[sl].copy()
            ff[sl],trim[sl],out[sl],sat=c.st.bounded_update(ff[sl],trim[sl],out[sl],
                (planned[j,:,None]-initial)/680,trial,actual)
            integral[sl]=trim[sl]-pnew;pstate[sl]=pnew
            maxspeed[sl]=np.maximum(maxspeed[sl],abs(out[sl]-before)/actual)
            limited[sl]+=sat;tick_counts[v*model_count:(v+1)*model_count]+=1
        commands[j]=out*1000;history[j%ringlen]=out
        delayed=history[(j-delay)%ringlen,np.arange(C)[:,None],np.arange(M)[None,:]]
        state=np.einsum('mij,cmj->cmi',aa,state)+bb[None,:,:]*delayed[:,:,None]
    route=(t>=2)&(t<route_end);seat=(t>=route_end)&(t<=route_end+1.6)
    error=forces-original[:,None,None]
    bands={name:c.band_rms(t[route],forces[route],bounds) for name,bounds in
        [('low_band_rms_N',[.2,.8]),('route_band_rms_N',[.8,2.5]),('route_high_band_rms_N',[2.5,10])]}
    rows=[]
    for i,cfg in enumerate(configs):
        for m,model in enumerate(scenarios):
            y=forces[:,i,m]
            rmse=float(np.sqrt(np.mean(error[route,i,m]**2)))
            p95=float(np.percentile(abs(error[route,i,m]),95))
            overshoot=float(max(0,error[seat,i,m].max()))
            score=rmse/.2+bands['route_band_rms_N'][i,m]/.05+overshoot/.25+p95/.5
            if y.min()<=0:score=1e9
            rows.append(dict(candidate=cfg['id'],base_controller=cfg['base_controller'],plan=cfg['plan'],
                feedback_reference=cfg['feedback_reference'],model=model['id'],variant=model['variant'],
                route_rmse_N=rmse,route_p95_abs_error_N=p95,first_seat_peak_N=float(y[seat].max()),
                first_seat_max_positive_error_N=overshoot,
                route_max_negative_error_N=float(np.maximum(0,-error[route,i,m]).max()),
                first_seat_max_negative_error_N=float(np.maximum(0,-error[seat,i,m]).max()),
                minimum_mean_hand_force_N=float(y.min()),max_command_speed_mm_s=float(maxspeed[i,m]*1000),
                max_command_abs_mm=float(abs(commands[:,i,m]).max()),trim_limited_tick_fraction=float(limited[i,m]/tick_counts[m]),
                score=float(score),**{name:float(value[i,m]) for name,value in bands.items()}))
    assert max(r['max_command_speed_mm_s'] for r in rows)<=2+1e-8
    assert max(r['max_command_abs_mm'] for r in rows)<=12+1e-8
    for base in ['I_P0_I1.6_D0','PI_P0.1_I0.8_D0_R0.1']:
        i=next(i for i,x in enumerate(configs) if x['id']==base)
        j=next(i for i,x in enumerate(configs) if x['id']==base+'__ABS_original_FBoriginal_O0')
        assert np.array_equal(forces[:,i],forces[:,j]) and np.array_equal(commands[:,i],commands[:,j])
    return rows,dict(t=t,target=original,force=forces,commands=commands,planned_targets=planned,
        feedback_targets=reference,route_end=route_end,initial=initial,
        model_ids=np.array([x['id'] for x in scenarios]),variant_ids=np.array([x['variant'] for x in scenarios]))


def main():
    design=freeze();dt=float(sys.argv[1]) if len(sys.argv)>1 else .0025
    frozen=json.loads((OUT/'frozen_design.json').read_text())
    catalogue=json.loads((OUT.parent/'archive_review/run_catalog.json').read_text());rows=[]
    for piece in 'ZLST':
        rid=frozen['split'][piece]['development'][0]['id'];rec=next(x for x in catalogue if x['id']==rid)
        profile,_=r.p.read_profile(rec);rr,trace=simulate(profile,design,dt)
        rows.extend([dict(piece=piece,**row) for row in rr])
        if dt==.0025:
            np.savez_compressed(OUT/(piece+'_planned_reference_autonomous.npz'),**trace,
                candidate_ids=np.array([x['id'] for x in design['candidates']]))
        print('Autonomous planned references',piece,len(rr),'cases at',dt,flush=True)
    suffix='' if dt==.0025 else '_1p25ms'
    c.dp.dump('planned_reference_autonomous_metrics'+suffix+'.json',rows)
    if dt!=.0025:
        original=json.loads((OUT/'planned_reference_autonomous_metrics.json').read_text())
        lookup={(x['piece'],x['candidate'],x['model'],x['variant']):x for x in original}
        fields=['route_rmse_N','route_band_rms_N','route_high_band_rms_N','low_band_rms_N','first_seat_peak_N']
        maximum={key:max(abs(x[key]-lookup[(x['piece'],x['candidate'],x['model'],x['variant'])][key]) for x in rows) for key in fields}
        passed=maximum['route_rmse_N']<.03 and maximum['route_band_rms_N']<.01 and maximum['first_seat_peak_N']<.05
        c.dp.dump('planned_reference_autonomous_refinement.json',dict(scenarios=len(rows),
            max_absolute_difference_N=maximum,identical_control_events_and_noise=True,passed=passed))
        assert passed,maximum
    print('Autonomous planning total',len(rows),flush=True)


if __name__=='__main__':main()
