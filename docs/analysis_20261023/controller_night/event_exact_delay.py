"""Event-exact pure-delay conditional replay: no control timestamp rounding.

Logged residual is still zero-order held, a hypothesis rather than a measured
continuous force. Control events and delayed command events are evaluated on
their union. Exact weighted RMSE and interval extrema; bands use a2.5ms grid.
No extension to the lag/resonant plant models is claimed here.
"""
import planned_reference as r
import json
import numpy as np

OUT=r.OUT;c=r.c


def replay(prepared,design):
    configs=design['candidates'];models=design['models'];C=len(configs);M=len(models)
    assert all(x['kind']=='delay' for x in models)
    assert all(x['kd_m_s_N']==0 for x in configs)
    profile,_=r.p.read_profile(prepared['rec'])
    nominal,feedback,*_=r.references(prepared,design,r.plan_profiles(profile))
    te=prepared['te'];K=len(te);measured=prepared['y0'][prepared['ev']]
    kplant=np.array([x['stiffness_N_m_hand'] for x in models])[None,:]
    delays=np.array([x['delay_s'] for x in models])
    prior_index=np.column_stack([np.searchsorted(te,te-delay,side='right')-1 for delay in delays])
    assert np.all(prior_index<np.arange(K)[:,None])
    kp=np.array([x['kp_m_N'] for x in configs])[:,None];ki=np.array([x['ki_m_N_s'] for x in configs])[:,None]
    roll=np.array([x['pd_rolloff_tau_s'] for x in configs])[:,None]
    initial=prepared['make']();shape=(C,M)
    ff=np.full(shape,initial.ff);trim=np.full(shape,initial.trim);out=np.full(shape,initial.output)
    error0=2*feedback[0,:,None]-measured[0]
    pd=np.broadcast_to(kp*error0,shape).copy();integral=trim-pd
    delta=np.zeros((K,C,M));commands=np.empty_like(delta);control_force=np.empty_like(delta)
    maxslew=np.zeros(shape)
    for k in range(K):
        indices=prior_index[k];valid=indices>=0
        response=np.where(valid[None,:],delta[np.maximum(indices,0),np.arange(C)[:,None],np.arange(M)[None,:]],0.)
        physical=measured[k]+2*kplant*response;control_force[k]=physical/2
        if k:
            dt_i=prepared['clock'][k]-prepared['clock'][k-1];dt_w=te[k]-te[k-1]
            error=2*feedback[k,:,None]-physical
            pd+=dt_w/(roll+dt_w)*(kp*error-pd)
            term=np.where(roll>0,pd,kp*error)
            trial=term+integral+ki*error*dt_i;before=out.copy()
            ff,trim,out,_=c.st.bounded_update(ff,trim,out,nominal[k,:,None],trial,dt_i)
            integral=trim-term;maxslew=np.maximum(maxslew,abs(out-before)/dt_i)
        commands[k]=out*1000;delta[k]=out-prepared['baseline'][k]
    baseline=next(i for i,x in enumerate(configs) if x['id']==design['baseline_id'])
    assert np.max(abs(control_force[:,baseline,:]-measured[:,None]/2))<1e-9
    assert np.max(abs(commands[:,baseline,:]-prepared['baseline'][:,None]*1000))<1e-9
    assert maxslew.max()<=.002+1e-9
    rows=[];traces={}
    for m,model in enumerate(models):
        # Merge sub-nanosecond float coincidences rather than invent zero-duration pulses.
        events=np.unique(np.round(np.r_[te,te+delays[m],0.,te[-1]],9))
        events=events[(events>=0)&(events<=te[-1]+1e-9)]
        if events[-1]<te[-1]:events=np.r_[events,te[-1]]
        base_index=np.clip(np.searchsorted(te,events+1e-9,side='right')-1,0,K-1)
        delayed_index=np.searchsorted(te,events-delays[m]+1e-9,side='right')-1
        response=np.where((delayed_index>=0)[:,None],delta[np.maximum(delayed_index,0),:,m],0.)
        force=measured[base_index,None]/2+float(kplant[0,m])*response
        target=prepared['target'][base_index];error=force-target[:,None]
        left=events[:-1];right=events[1:]
        route_weights=np.maximum(0,np.minimum(right,prepared['route1'])-np.maximum(left,prepared['route0']))
        seat_weights=np.maximum(0,np.minimum(right,prepared['route1']+1.6)-np.maximum(left,prepared['route1']))
        active_route=route_weights>1e-10;active_seat=seat_weights>1e-10
        duration=float(route_weights.sum())
        grid=np.arange(round(te[-1]/.0025)+1)*.0025
        gi=np.clip(np.searchsorted(events,grid,side='right')-1,0,len(events)-1)
        route=(grid>=prepared['route0'])&(grid<=prepared['route1']);c.DT=.0025
        bands={name:c.band_rms(grid[route],values[gi][route],bounds) for name,values,bounds in [
            ('route_band_rms_N',force,[.8,2.5]),('route_high_band_rms_N',force,[2.5,10]),
            ('route_error_band_rms_N',error,[.8,2.5]),('route_low_error_band_rms_N',error,[.2,.8])]}
        for i,cfg in enumerate(configs):
            rmse=float(np.sqrt(np.sum(error[:-1,i]**2*route_weights)/duration))
            order=np.argsort(abs(error[:-1,i]));cumulative=np.cumsum(route_weights[order])
            p95=float(abs(error[:-1,i])[order[min(np.searchsorted(cumulative,.95*duration),len(order)-1)]])
            peak=float(force[:-1,i][active_seat].max())
            pos=float(max(0,error[:-1,i][active_seat].max()))
            rows.append(dict(id=prepared['rec']['id'],piece=prepared['rec']['piece'],candidate=cfg['id'],model=model['id'],
                route_rmse_N=rmse,route_p95_abs_error_N=p95,first_seat_peak_N=peak,
                first_seat_max_positive_error_N=pos,minimum_mean_hand_force_N=float(force[:-1,i].min()),
                max_command_speed_mm_s=float(maxslew[i,m]*1000),
                **{name:float(value[i]) for name,value in bands.items()},
                score=rmse/.2+bands['route_band_rms_N'][i]/.05+pos/.25+p95/.5))
        traces[f'm{m}_time_s']=events;traces[f'm{m}_force_N']=force;traces[f'm{m}_target_N']=target
    traces.update(event_time_s=te,event_commands_mm=commands,control_force_N=control_force,
        candidate_ids=np.array([x['id'] for x in configs]),model_ids=np.array([x['id'] for x in models]))
    return rows,traces


def main():
    design=r.freeze();compact=dict(design,models=[x for x in design['models'] if x['id'] in
        ['delay_k480_d0.12','delay_k680_d0.18','delay_k1500_d0.23']])
    c.dp.dump('event_exact_delay_selection.json',dict(candidate_ids=[x['id'] for x in compact['candidates']],
        model_ids=[x['id'] for x in compact['models']],scope='First development per piece, all38 fixed reference variants. Pure-delay diagnostic after failed peak refinement, not retuning.'))
    original=json.loads((OUT/'planned_reference_metrics.json').read_text());old={(x['id'],x['candidate'],x['model']):x for x in original}
    half=json.loads((OUT/'planned_reference_refinement.json').read_text())
    half_delta={(x['id'],x['candidate'],x['model']):x['difference'] for x in half['cases']}
    frozen=json.loads((OUT/'frozen_design.json').read_text());catalogue=json.loads((OUT.parent/'archive_review/run_catalog.json').read_text())
    rows=[];comparisons=[]
    for piece in 'ZLST':
        rid=frozen['split'][piece]['development'][0]['id'];rec=next(x for x in catalogue if x['id']==rid)
        # prepare needs its original grid only to retrieve exact event samples.
        c.old_replay.DT=.005;prepared=c.old_replay.prepare(rec)
        rr,trace=replay(prepared,compact);rows.extend(rr)
        for row in rr:
            key=(row['id'],row['candidate'],row['model']);reference=old[key]
            comparisons.append(dict(id=rid,candidate=row['candidate'],model=row['model'],
                event_exact_seat_peak_N=row['first_seat_peak_N'],
                grid5ms_seat_peak_N=reference['first_seat_peak_N'],
                grid2p5ms_seat_peak_N=reference['first_seat_peak_N']+half_delta[key]['first_seat_peak_N'],
                event_exact_minus_grid5ms_peak_N=row['first_seat_peak_N']-reference['first_seat_peak_N']))
        np.savez_compressed(OUT/(rid+'_event_exact_delay.npz'),**trace)
        print('Event-exact delays',piece,len(rr),'cases',flush=True)
    c.dp.dump('event_exact_delay_metrics.json',rows)
    c.dp.dump('event_exact_delay_grid_comparison.json',comparisons)
    print('Event-exact total',len(rows),'cases',flush=True)


if __name__=='__main__':main()
