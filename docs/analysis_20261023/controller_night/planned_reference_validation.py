"""Production-class identity, causality and half-grid planned-reference checks."""
import planned_reference as r
import json
import numpy as np

OUT=r.OUT;c=r.c


def main():
    design=r.freeze();frozen=json.loads((OUT/'frozen_design.json').read_text())
    catalogue=json.loads((OUT.parent/'archive_review/run_catalog.json').read_text())
    all_rows=json.loads((OUT/'planned_reference_metrics.json').read_text())
    lookup={(row['id'],row['candidate'],row['model']):row for row in all_rows}
    bounded=[x for x in all_rows if x['plan'] in ['majorant','minimax']]
    maxslew=max(x['max_desired_nominal_slew_same_phase_mm_s'] for x in bounded)
    max_lowering=max(x['max_planned_below_original_same_clock_N'] for x in all_rows if x['plan']=='majorant')
    assert maxslew<=2+1e-8 and max_lowering<1e-9
    actual_error=0.;actual_updates=0;causal_checks=[]
    for piece in 'ZLST':
        rid=frozen['split'][piece]['development'][0]['id'];rec=next(x for x in catalogue if x['id']==rid)
        prepared=c.old_replay.prepare(rec)
        with np.load(OUT/(rid+'_planned_reference_central.npz')) as archive:trace={key:archive[key] for key in archive.files}
        for i,cfg in enumerate(design['candidates']):
            if cfg['base_controller']!='I_P0_I1.6_D0':continue
            if cfg['plan']!='logged' and cfg['feedback_reference']!='planned':continue
            ctrl=prepared['make']()
            for k in range(1,len(prepared['ev'])):
                force_sum=2*float(trace['force'][prepared['ev'][k],i])
                target=float(trace['feedback_targets_N'][k,i])
                command=ctrl.update(target,force_sum,float(prepared['clock'][k]),ctrl.output)*1000
                actual_error=max(actual_error,abs(command-trace['commands'][k,i]));actual_updates+=1
        profile,_=r.p.read_profile(rec);tables=r.plan_profiles(profile)
        nominal,feedback,*_=r.references(prepared,design,tables)
        cut=len(prepared['ix'])//2;rowcut=int(prepared['ix'][cut]);changed=dict(prepared)
        changed['d']=dict(prepared['d']);changed['phase']=prepared['phase'].copy();changed['target']=prepared['target'].copy()
        for key in ['phase','t_wall','ffi_ff_want_mm']:changed['d'][key]=prepared['d'][key].copy()
        changed['d']['phase'][rowcut:]='slow_seat';changed['phase'][cut:]='slow_seat'
        changed['d']['t_wall'][rowcut:]+=10.;changed['d']['ffi_ff_want_mm'][rowcut:]+=100.
        changed['target'][cut:]+=100.;changed['y0']=np.full_like(prepared['y0'],12345.)
        next_nominal,next_feedback,*_=r.references(changed,design,tables)
        assert np.array_equal(nominal[:cut],next_nominal[:cut])
        assert np.array_equal(feedback[:cut],next_feedback[:cut])
        causal_checks.append(dict(id=rid,prefix_unchanged=True,future_logs_and_all_force_values_perturbed=True))
    assert actual_error<1e-9
    compact=dict(design,models=[m for m in design['models'] if m['id'] in ['delay_k480_d0.12','delay_k680_d0.18',
        'delay_k1500_d0.23','resonant680_d18_z0.3']])
    c.DT=.0025;c.old_replay.DT=.0025;differences=[]
    for piece in 'ZLST':
        rid=frozen['split'][piece]['development'][0]['id'];rec=next(x for x in catalogue if x['id']==rid)
        prepared=c.old_replay.prepare(rec);profile,_=r.p.read_profile(rec)
        nominal,feedback,*_=r.references(prepared,compact,r.plan_profiles(profile))
        rows,_,_=c.replay(prepared,compact,dt=c.DT,nominal_wants=nominal,feedback_targets=feedback)
        for row in rows:
            ref=lookup[(row['id'],row['candidate'],row['model'])]
            differences.append(dict(id=rid,candidate=row['candidate'],model=row['model'],difference={key:row[key]-ref[key] for key in
                ['route_rmse_N','route_band_rms_N','route_high_band_rms_N','first_seat_peak_N','minimum_mean_hand_force_N']}))
        print('Refined plans',piece,len(rows),flush=True)
    maximum={key:max(abs(row['difference'][key]) for row in differences) for key in differences[0]['difference']}
    passed=maximum['route_rmse_N']<.03 and maximum['route_band_rms_N']<.01 and maximum['first_seat_peak_N']<.05
    c.dp.dump('planned_reference_software_validation.json',dict(actual_production_I_class_updates=actual_updates,
        maximum_production_command_difference_mm=actual_error,causal_checks=causal_checks,
        max_desired_slew_within_phase_mm_s=maxslew,majorant_max_lowering_same_clock_N=max_lowering))
    c.dp.dump('planned_reference_refinement.json',dict(scenarios=len(differences),maximum_absolute_difference_N=maximum,
        passed=passed,cases=differences))
    assert passed,maximum
    print('Planned-reference checks',actual_updates,actual_error,maximum,flush=True)


if __name__=='__main__':main()
