"""Exact rate feasibility of the nominal command, NOT a bound on real force.

At fixed k_ff=680 N/m, a2mm/s nominal command corresponds to1.36N/s
force-equivalent rate. External motion/contact can change real force faster.
The two-sided Lipschitz fit has unlimited within-phase plan preview, free
endpoints and no entry/delay constraints, so it is an optimistic command bound.
"""
import profile_preview as p
import json
import numpy as np

OUT=p.OUT;RATE=680*.002


def lipschitz_fit(t,y,rate,return_envelopes=False):
    """Minimax Lipschitz approximation on an irregular piecewise-linear grid.

    Upper=max_j(y_j-rate*|t_i-t_j|), lower=min_j(y_j+rate*|t_i-t_j|).
    Their midpoint is rate-feasible. Half max(upper-y) is the pairwise
    necessary L-infinity error bound, attained by that midpoint.
    """
    upper=y.copy();lower=y.copy()
    for i in range(1,len(t)):
        change=rate*(t[i]-t[i-1])
        upper[i]=max(upper[i],upper[i-1]-change)
        lower[i]=min(lower[i],lower[i-1]+change)
    for i in range(len(t)-2,-1,-1):
        change=rate*(t[i+1]-t[i])
        upper[i]=max(upper[i],upper[i+1]-change)
        lower[i]=min(lower[i],lower[i+1]+change)
    planned=(upper+lower)/2
    lower_bound=float(np.max(upper-y)/2)
    assert np.all(upper>=y-1e-12) and np.all(lower<=y+1e-12)
    assert abs(np.max(abs(planned-y))-lower_bound)<1e-9
    if len(t)>1:
        for trajectory in [planned,upper,lower]:
            assert np.max(abs(np.diff(trajectory)/np.diff(t)))<=rate+1e-8
    return (planned,lower_bound,upper,lower) if return_envelopes else (planned,lower_bound)


def checks():
    t=np.array([0.,.02]);y=np.array([4.,6.]);fit,bound=lipschitz_fit(t,y,RATE)
    assert abs(bound-(2-RATE*.02)/2)<1e-12
    rng=np.random.default_rng(24102026)
    for _ in range(50):
        times=np.cumsum(rng.uniform(.001,.5,20));target=rng.uniform(2,8,20)
        fit,bound=lipschitz_fit(times,target,RATE)
        brute=.5*np.max(np.maximum(0,abs(target[:,None]-target[None,:])-RATE*abs(times[:,None]-times[None,:])))
        assert abs(brute-bound)<1e-10
    return dict(analytic_two_point_case=True,random_pairwise_bound_cases=50,
        piecewise_linear_rate_and_bound_verified=True)


def main():
    validation=checks();frozen=json.loads((OUT/'frozen_design.json').read_text())
    catalogue=json.loads((OUT.parent/'archive_review/run_catalog.json').read_text())
    rows=[];boundaries=[]
    for piece in 'ZLST':
        for entry in frozen['split'][piece]['development']:
            rec=next(r for r in catalogue if r['id']==entry['id']);profile,meta=p.read_profile(rec)
            pickup=rec['meta']['execution']['ffi']['pickup_measured_N_per_hand']
            for phase,(times,target) in profile.items():
                planned,bound,upper,lower=lipschitz_fit(times,target,RATE,return_envelopes=True)
                duration=float(times[-1]-times[0]);delta_t=np.diff(times)
                slope=abs(np.diff(target)/delta_t) if len(times)>1 else np.array([])
                rows.append(dict(id=rec['id'],piece=piece,phase=phase,duration_s=duration,
                    fraction_phase_time_target_rate_exceeds_nominal_limit=float(np.sum(delta_t[slope>RATE])/duration) if duration>0 else 0.,
                    maximum_original_force_equivalent_rate_N_s=float(slope.max()) if len(slope) else 0.,
                    maximum_desired_nominal_rate_mm_s=float(slope.max()/680*1000) if len(slope) else 0.,
                    best_possible_max_nominal_error_force_equivalent_N=bound,
                    best_possible_max_nominal_position_error_mm=bound/680*1000,
                    majorant_max_additional_nominal_mm=float(np.max(upper-target)/680*1000),
                    note='Minimax fit may request less normal than original SOCP. Majorant never lowers scalar target, but full wrench margins are not revalidated. Free phase endpoints.',
                    actual_force_bound=False))
                if phase=='route':
                    np.savez_compressed(OUT/(rec['id']+'_nominal_feasibility.npz'),t=times,target=target,
                        ideal_best_nominal_force_equivalent=planned,desired_ff_mm=(target-pickup)/680*1000,
                        ideal_best_ff_mm=(planned-pickup)/680*1000,
                        majorant_ff_mm=(upper-pickup)/680*1000)
            for first,second in [('to_mouth','route'),('route','slow_seat')]:
                if first in profile and second in profile:
                    jump=float(profile[second][1][0]-profile[first][1][-1])
                    next_times,next_target=profile[second]
                    early=next_target[next_times<=.15]
                    early_min=float(early.min()) if len(early) else float(next_target[0])
                    early_delta=float(early_min-profile[first][1][-1])
                    boundaries.append(dict(id=rec['id'],piece=piece,from_phase=first,to_phase=second,
                        planned_endpoint_target_jump_N=jump,minimum_nominal_motion_time_s=abs(jump)/RATE,
                        next_phase_first150ms_minimum_target_N=early_min,
                        delta_from_previous_endpoint_to_first150ms_min_N=early_delta,
                        minimum_nominal_unloading_motion_time_s=max(0.,-early_delta)/RATE,
                        note='Known profile targets: raw endpoint and first150ms changes; actual state/FFI trim/contact and exact phase gap not included.'))
    p.c.dp.dump('profile_nominal_feasibility.json',dict(rate_force_equivalent_N_s=RATE,
        scope='Exact nominal FF command rate feasibility at k_ff680, not actual-force tracking bound or a modified SOCP plan.',
        validation=validation,phase_metrics=rows,boundaries=boundaries))
    print('Nominal feasibility',len(rows),'phase cases,',len(boundaries),'boundaries',flush=True)
    for row in rows:
        if row['phase']=='route':print(json.dumps(row),flush=True)


if __name__=='__main__':main()
