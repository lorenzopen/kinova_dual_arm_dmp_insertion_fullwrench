"""Autonomous hypothetical plants: no recorded force residual or future force.

Uses a frozen development shortlist without retuning, and synthetic disturbance,
noise and jitter. This does not validate any robot dynamics or contact safety.
"""
import advanced_controllers as a
import json
import sys
from datetime import datetime,timezone
import numpy as np
import matplotlib.pyplot as plt

OUT=a.OUT


def freeze():
    path=OUT/'advanced_autonomous_design.json'
    if path.exists():return json.loads(path.read_text())
    advanced=a.freeze();short=json.loads((OUT/'advanced_shortlist_development_only.json').read_text())['shortlist']
    ids={x['id'] for x in short}|{'PD_P0.3_I0_D0.1_R0.1'}
    design=dict(created_utc=datetime.now(timezone.utc).isoformat(),
        candidates=[x for x in advanced['candidates'] if x['id'] in ids],models=advanced['models'],
        variants=[dict(id='clean',noise_N_hand=0.,jitter=False,wave_N_hand=0.,bias_N_hand=0.),
            dict(id='noise_jitter',noise_N_hand=.013,jitter=True,wave_N_hand=0.,bias_N_hand=0.),
            dict(id='physical_wave_noise_jitter',noise_N_hand=.013,jitter=True,wave_N_hand=.15,bias_N_hand=.5)],
        tests='32 seconds; target4 to6 at3s, back4 at9s. Wave1.2Hz starts12s; bias+0.5N/hand at16-22s in third variant. Jitter ticks 15-35ms quantized at2.5ms. Noise sampled at control events, common between candidates and plant scenarios. Seed23442026.',
        measurement='Sum force, derivative on measured sum with 50ms filter plus each selected PD roll-off. No measured-force preview. Same rate/travel/component bounds and allocation implementations as conditional replay.',
        initialization='4N/hand preload; zero FF/trim/output/I/PD/derivative. Nominal=(original target-4)/680.',
        limitation='Synthetic scalar symmetric contact; no real phase transitions, slip, board, friction or failures. cart_d unchanged and not represented as a tunable delay. No new model identification.',
        purpose='Independent autonomous stress and convergence check of frozen hypotheses, not parameter optimization or validated prediction.')
    with path.open('x',encoding='utf-8') as f:json.dump(design,f,indent=2)
    return design


def simulate(design,dt):
    c=a.c;c.DT=dt
    cfgs=design['candidates'];plants=design['models'];variants=design['variants']
    scenarios=[dict(m,variant=v['id'],noise_N_hand=v['noise_N_hand'],jitter=v['jitter'],
        wave_N_hand=v['wave_N_hand'],bias_N_hand=v['bias_N_hand']) for v in variants for m in plants]
    C=len(cfgs);M=len(scenarios);shape=(C,M)
    kp=np.array([x['kp_m_N'] for x in cfgs])[:,None];ki=np.array([x['ki_m_N_s'] for x in cfgs])[:,None]
    kd=np.array([x['kd_m_s_N'] for x in cfgs])[:,None];tau=np.array([x['derivative_tau_s'] for x in cfgs])[:,None]
    roll=np.array([x['pd_rolloff_tau_s'] for x in cfgs])[:,None]
    ff=np.zeros(shape);trim=ff.copy();out=ff.copy();integral=ff.copy();derivative=ff.copy();pd=ff.copy()
    previous=np.full(shape,8.);maxspeed=ff.copy();limited=ff.copy();tick_count=np.zeros(M)
    kind,aa,bb=c.model_arrays(scenarios,dt);state=np.zeros((C,M,2))
    delays=np.rint(np.array([x['delay_s'] for x in scenarios])/dt).astype(int)
    stiffness=np.array([x['stiffness_N_m_hand'] for x in scenarios])[None,:]
    rng=np.random.default_rng(23442026)
    fixed_times=np.arange(0,32.0001,.025)
    jitter_ticks=[0]
    while jitter_ticks[-1]*.0025<32:
        jitter_ticks.append(jitter_ticks[-1]+int(rng.integers(6,15)))
    jitter_times=np.array(jitter_ticks)*.0025
    schedules=[jitter_times if x['jitter'] else fixed_times for x in scenarios]
    indices=[np.rint(s/dt).astype(int) for s in schedules]
    next_tick=np.zeros(M,dtype=int);last_time=np.zeros(M)
    noise=np.random.default_rng(23442027).normal(size=2500)
    ringlen=delays.max()+2;history=np.zeros((ringlen,C,M))
    t=np.arange(0,32+dt/2,dt);target=np.where((t>=3)&(t<9),6.,4.)
    forces=np.empty((len(t),C,M));commands=np.empty_like(forces)
    update=a.allocator([x['allocation'] for x in cfgs])
    wave=np.array([x['wave_N_hand'] for x in scenarios])[None,:]
    bias=np.array([x['bias_N_hand'] for x in scenarios])[None,:]
    noise_std=np.array([x['noise_N_hand'] for x in scenarios])[None,:]
    for j,tt in enumerate(t):
        delayed=history[(j-delays)%ringlen,np.arange(C)[:,None],np.arange(M)[None,:]]
        response=np.where(kind[None,:]=='delay',delayed,state[:,:,0])
        disturbance=wave*np.sin(2*np.pi*1.2*(tt-12))*(tt>=12)+bias*((tt>=16)&(tt<22))
        physical=8+2*stiffness*response+2*disturbance
        forces[j]=physical/2
        hit=np.array([next_tick[m]<len(indices[m]) and j==indices[m][next_tick[m]] for m in range(M)])
        if hit.any():
            dt_tick=np.maximum(dt,tt-last_time)[None,:]
            measured=physical+2*noise_std*noise[np.minimum(next_tick,2499)][None,:]
            if j==0:
                previous=measured.copy();next_tick+=1
                commands[j]=0.;history[j%ringlen]=0.
                continue
            dnew=(tau*derivative+measured-previous)/(tau+dt_tick)
            error=2*target[j]-measured;raw=kp*error-kd*dnew
            pdnew=pd+dt_tick/(roll+dt_tick)*(raw-pd)
            pdnew=np.where(roll>0,pdnew,raw)
            trial=pdnew+integral+ki*error*dt_tick
            # Non-ticking scenarios are projected too, then discarded. Their state is unchanged.
            # Each scenario's step bound must use its own elapsed control dt.
            before=out.copy()
            # Allocator accepts broadcast dt, its own assertions use scalar maxima only;
            # dispatch each active scenario to retain exact independent step checks.
            for m in np.flatnonzero(hit):
                actual=float(dt_tick[0,m]);sl=(slice(None),slice(m,m+1))
                fnew,tnew,onew,sat=update(ff[sl],trim[sl],out[sl],(target[j]-4)/680,trial[sl],actual)
                ff[sl]=fnew;trim[sl]=tnew;out[sl]=onew
                integral[sl]=np.where(ki>0,tnew-pdnew[sl],0.)
                derivative[sl]=dnew[sl];pd[sl]=pdnew[sl];previous[sl]=measured[sl]
                maxspeed[sl]=np.maximum(maxspeed[sl],abs(onew-before[sl])/actual)
                limited[sl]+=sat;tick_count[m]+=1
                last_time[m]=tt;next_tick[m]+=1
        commands[j]=out*1000
        history[j%ringlen]=out
        delayed=history[(j-delays)%ringlen,np.arange(C)[:,None],np.arange(M)[None,:]]
        state=np.einsum('mij,cmj->cmi',aa,state)+bb[None,:,:]*delayed[:,:,None]
    errors=forces-target[:,None,None];up=(t>=3)&(t<9);down=(t>=9)&(t<12)
    final=(t>=24)&(t<=32);steady=((t>=7)&(t<9))|((t>=11)&(t<12))
    band=c.band_rms(t[final],forces[final],[.8,2.5]);high=c.band_rms(t[final],forces[final],[2.5,10])
    rows=[]
    for i,cfg in enumerate(cfgs):
        for m,model in enumerate(scenarios):
            y=forces[:,i,m];times=[]
            for fraction in [.1,.9]:
                crosses=np.flatnonzero(y[up]>=4+2*fraction)
                times.append(float(t[up][crosses[0]]-3) if len(crosses) else None)
            rows.append(dict(candidate=cfg['id'],family=cfg['family'],model=model['id'],variant=model['variant'],
                rmse_N_hand=float(np.sqrt(np.mean(errors[t>=3,i,m]**2))),
                up_overshoot_N_hand=float(max(0,np.max(y[up]-6))),
                down_undershoot_N_hand=float(max(0,np.max(4-y[down]))),
                steady_abs_error_N_hand=float(np.mean(abs(errors[steady,i,m]))),
                final_band_rms_N_hand=float(band[i,m]),final_high_band_rms_N_hand=float(high[i,m]),
                minimum_force_N_hand=float(y.min()),rise_10_s=times[0],rise_90_s=times[1],
                rise_10_90_s=None if None in times else times[1]-times[0],
                max_command_abs_mm=float(abs(commands[:,i,m]).max()),max_speed_mm_s=float(maxspeed[i,m]*1000),
                trim_limited_fraction=float(limited[i,m]/tick_count[m])))
    assert max(r['max_speed_mm_s'] for r in rows)<=2+1e-8
    return rows,dict(t=t,target=target,force=forces,commands=commands,scenarios=scenarios)


def main():
    design=freeze();dt=float(sys.argv[1]) if len(sys.argv)>1 else .0025
    rows,trace=simulate(design,dt)
    suffix='' if dt==.0025 else '_1p25ms'
    a.c.dp.dump('advanced_autonomous_metrics'+suffix+'.json',rows)
    if dt==.0025:
        models=trace.pop('scenarios');np.savez_compressed(OUT/'advanced_autonomous_traces.npz',**trace,
            candidate_ids=np.array([x['id'] for x in design['candidates']]),
            model_ids=np.array([x['id'] for x in models]),variant_ids=np.array([x['variant'] for x in models]))
    else:
        original=json.loads((OUT/'advanced_autonomous_metrics.json').read_text())
        lookup={(r['candidate'],r['model'],r['variant']):r for r in original}
        fields=['rmse_N_hand','up_overshoot_N_hand','down_undershoot_N_hand','final_band_rms_N_hand']
        differences={key:max(abs(r[key]-lookup[(r['candidate'],r['model'],r['variant'])][key]) for r in rows) for key in fields}
        a.c.dp.dump('advanced_autonomous_refinement.json',dict(scenarios=len(rows),
            identical_control_schedules_and_noise=True,max_difference_N=differences))
    print('Autonomous stress',len(rows),'scenarios at',dt,'seconds',flush=True)


if __name__=='__main__':main()
