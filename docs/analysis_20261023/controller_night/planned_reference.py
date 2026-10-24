"""Rate-bounded within-phase plans, nominal and feedback reference separated.

Absolute plan lookup, not the previous delta-preview construction. Original
profile lookup is included as an explicit clock-reconstruction control. No
future measured force or observed future phase schedule is used.
"""
import sys
sys.dont_write_bytecode=True
import profile_preview as p
import profile_feasibility as f
import json
import itertools
from datetime import datetime,timezone
import numpy as np

OUT=p.OUT;c=p.c


def freeze():
    path=OUT/'planned_reference_design.json'
    if path.exists():return json.loads(path.read_text())
    prior=p.freeze();bases=[x for x in prior['candidates'] if x['preview']=='none']
    configs=[dict(x,id=x['base_controller'],plan='logged',feedback_reference='original',clock_offset_s=0.) for x in bases]
    for cfg,plan,reference,offset in itertools.product(bases,['original','majorant','minimax'],['original','planned'],[-.04,0.,.04]):
        configs.append(dict(cfg,id=f"{cfg['base_controller']}__ABS_{plan}_FB{reference}_O{offset:g}",
            plan=plan,feedback_reference=reference,clock_offset_s=offset))
    design=dict(created_utc=datetime.now(timezone.utc).isoformat(),candidates=configs,
        models=prior['models'],baseline_id=prior['baseline_id'],dt_s=c.DT,score=prior['score'],
        data_split='Fixed8 development only; four holdouts excluded.',
        absolute_plan='Nominal=(planned scalar force lookup - measured pickup preload)/680. No anchoring to original nominal. Feedback uses original logged target OR planned scalar force. All metrics remain against original logged target.',
        planners=dict(original='Unmodified offline profile: control for reconstruction of phase clock.',
            majorant='Pointwise least upper1.36N/s-Lipschitz envelope within the current phase. Does not lower original scalar profile at the same phase time.',
            minimax='Midpoint upper/lower1.36N/s envelopes, optimal within-phase L-infinity error with free endpoints. May lower original SOCP normal and is diagnostic only.'),
        clock='First-observed phase-row origin with -40/0/+40ms sensitivity, not identified clock-error bounds. Baseline logged is preserved separately. Offset worst-case, not optimized.',
        information='Entire offline profile of CURRENT observed phase available; next-phase transition time and force future not queried. Endpoints held.',
        constraints='Same original FF-priority component/sum2mm/s and travel limits. Plan curves have bounded within-phase slopes but free endpoints; phase transitions, entry and logged dt-clamping can still rate-limit execution.',
        margin_limit='No modified scalar plan has revalidated full wrench/equilibrium/friction margins. Majorant scalar domination is not an actual per-hand force guarantee.',
        model_limit='Fixed-recorded-residual conditional replay under hypothetical delta-command dynamics; no empirical plant passed predictive adequacy gate. No cart_d modification or robot recommendation.')
    with path.open('x',encoding='utf-8') as handle:json.dump(design,handle,indent=2)
    return design


def plan_profiles(profile):
    tables={mode:{} for mode in ['original','majorant','minimax']}
    for phase,(t,normal) in profile.items():
        fit,bound,upper,lower=f.lipschitz_fit(t,normal,1.36,return_envelopes=True)
        tables['original'][phase]=(t,normal);tables['majorant'][phase]=(t,upper);tables['minimax'][phase]=(t,fit)
    return tables


def references(prepared,design,tables):
    elapsed=p.causal_phase_clock(prepared['d'],prepared['ix'])
    pickup=prepared['rec']['meta']['execution']['ffi']['pickup_measured_N_per_hand']
    C=len(design['candidates']);K=len(elapsed)
    nominal=np.empty((K,C));feedback=np.empty((K,C));planned=np.empty((K,C));profile_now=np.empty((K,C))
    for j,cfg in enumerate(design['candidates']):
        if cfg['plan']=='logged':
            nominal[:,j]=prepared['d']['ffi_ff_want_mm'][prepared['ix']]/1000
            feedback[:,j]=prepared['target'];planned[:,j]=prepared['target'];profile_now[:,j]=prepared['target']
            continue
        for k,(phase,clock) in enumerate(zip(prepared['phase'],elapsed)):
            phase=str(phase);clock=max(0.,clock+cfg['clock_offset_s'])
            ts,y=tables[cfg['plan']][phase];planned[k,j]=np.interp(clock,ts,y)
            ts,y=tables['original'][phase];profile_now[k,j]=np.interp(clock,ts,y)
        nominal[:,j]=(planned[:,j]-pickup)/680
        feedback[:,j]=prepared['target'] if cfg['feedback_reference']=='original' else planned[:,j]
    return nominal,feedback,planned,profile_now,elapsed


def main():
    design=freeze();frozen=json.loads((OUT/'frozen_design.json').read_text())
    catalogue=json.loads((OUT.parent/'archive_review/run_catalog.json').read_text())
    old={(r['id'],r['candidate'],r['model']):r for r in json.loads((OUT/'advanced_controller_metrics.json').read_text())}
    rows=[];validations=[];reconstruction=[];regression=0.
    central=next(i for i,m in enumerate(design['models']) if m['id']=='delay_k680_d0.18')
    for piece in 'ZLST':
        for entry in frozen['split'][piece]['development']:
            rec=next(r for r in catalogue if r['id']==entry['id']);prepared=c.old_replay.prepare(rec)
            profile,audit=p.read_profile(rec);tables=plan_profiles(profile)
            nominal,feedback,planned,profile_now,elapsed=references(prepared,design,tables)
            metrics,val,tr=c.replay(prepared,design,nominal_wants=nominal,feedback_targets=feedback)
            route=prepared['phase']=='route';dt=np.diff(prepared['te']);same_phase=prepared['phase'][1:]==prepared['phase'][:-1]
            for row in metrics:
                j=next(i for i,x in enumerate(design['candidates']) if x['id']==row['candidate'])
                cfg=design['candidates'][j]
                row.update(plan=cfg['plan'],feedback_reference=cfg['feedback_reference'],base_controller=cfg['base_controller'],
                    max_planned_below_original_same_clock_N=float(np.maximum(0,profile_now[:,j]-planned[:,j]).max()),
                    max_planned_below_original_logged_target_N=float(np.maximum(0,prepared['target']-planned[:,j]).max()),
                    mean_route_planned_below_original_logged_target_N=float(np.maximum(0,prepared['target'][route]-planned[route,j]).mean()),
                    fraction_route_planned_below_original_logged_target=float(np.mean(planned[route,j]<prepared['target'][route]-.01)),
                    max_desired_nominal_slew_same_phase_mm_s=float(np.max(abs(np.diff(nominal[:,j]))[same_phase]/dt[same_phase])*1000))
                if cfg['plan']=='logged':
                    ref=old[(row['id'],row['candidate'],row['model'])]
                    for key in ['route_rmse_N','route_band_rms_N','route_high_band_rms_N','first_seat_peak_N']:
                        regression=max(regression,abs(row[key]-ref[key]))
            rows.extend(metrics);validations.append(val)
            for j,cfg in enumerate(design['candidates']):
                if cfg['plan']=='original':
                    reconstruction.append(dict(id=rec['id'],candidate=cfg['id'],
                        reference_profile_vs_logged_target_rmse_N=float(np.sqrt(np.mean((planned[:,j]-prepared['target'])**2)))))
            np.savez_compressed(OUT/(rec['id']+'_planned_reference_central.npz'),t=tr['t'],target=tr['target'],
                force=tr['force'][:,:,central],commands=tr['commands'][:,:,central],event_time=tr['event_time'],
                planned_targets_N=planned,feedback_targets_N=feedback,nominal_wants_mm=nominal*1000,
                route0=prepared['route0'],route1=prepared['route1'],candidate_ids=np.array([x['id'] for x in design['candidates']]))
            print('Planned reference',piece,rec['id'],len(metrics),'scenarios, baseline regression',regression,flush=True)
            del prepared,tr
    assert regression<1e-10
    grouped=[]
    for base,plan,reference in itertools.product(['I_P0_I1.6_D0','PI_P0.1_I0.8_D0_R0.1'],['original','majorant','minimax'],['original','planned']):
        configs=[x for x in design['candidates'] if x['base_controller']==base and x['plan']==plan and x['feedback_reference']==reference]
        ids={x['id'] for x in configs};rr=[r for r in rows if r['candidate'] in ids];runids=sorted({r['id'] for r in rr})
        score=float(np.mean([max(r['score'] for r in rr if r['id']==rid) for rid in runids]))
        grouped.append(dict(base_controller=base,plan=plan,feedback_reference=reference,
            mean_run_worst_model_and_clock_score=score,candidate_ids=sorted(ids),
            minimum_mean_hand_force_N=min(r['minimum_mean_hand_force_N'] for r in rr),
            maximum_same_phase_desired_nominal_slew_mm_s=max(r['max_desired_nominal_slew_same_phase_mm_s'] for r in rr)))
    grouped.sort(key=lambda r:r['mean_run_worst_model_and_clock_score'])
    c.dp.dump('planned_reference_metrics.json',rows);c.dp.dump('planned_reference_grouped_ranking.json',grouped)
    with (OUT/'planned_reference_metrics.csv').open('w',newline='',encoding='utf-8-sig') as handle:
        writer=p.csv.DictWriter(handle,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    c.dp.dump('planned_reference_validation.json',dict(scenarios=len(rows),candidate_count=len(design['candidates']),
        exact_logged_baseline_metric_regression_N=regression,replay_checks=validations,clock_control=reconstruction))
    print('Best grouped plans',json.dumps(grouped[:6]),flush=True)


if __name__=='__main__':main()
