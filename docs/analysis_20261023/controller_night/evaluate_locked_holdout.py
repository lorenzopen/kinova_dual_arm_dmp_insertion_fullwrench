"""Single locked retrospective holdout batch. NEVER tune or modify the freeze.

Completed runs are immutable and reused; interrupted incomplete attempts remain
as evidence. Only missing runs are computed on resume. No ROS or robot imports.
"""
import sys
sys.dont_write_bytecode=True
import conditional_controllers as c
import hashlib
import json
import os
from pathlib import Path
from datetime import datetime,timezone
import scipy
import numpy as np

OUT=c.OUT;ROOT=c.ROOT


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def now():return datetime.now(timezone.utc).isoformat()


def exclusive_json(path,value):
    with path.open('x',encoding='utf-8') as handle:json.dump(value,handle,indent=2)


def atomic_progress(value):
    temporary=OUT/'HOLDOUT_PROGRESS.tmp.json'
    temporary.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
    temporary.replace(OUT/'HOLDOUT_PROGRESS.json')


def verify_freeze():
    primary=OUT/'FINAL_COMPARISON_FREEZE.json';manifest=json.loads((OUT/'FINAL_COMPARISON_FREEZE_manifest.json').read_text())
    assert digest(primary)==manifest['sha256']
    design=json.loads(primary.read_text())
    supplement_path=OUT/'FINAL_COMPARISON_SOURCE_SUPPLEMENT.json'
    supplement=json.loads(supplement_path.read_text());sm=json.loads((OUT/'FINAL_COMPARISON_SOURCE_SUPPLEMENT_manifest.json').read_text())
    assert digest(supplement_path)==sm['sha256'] and supplement['primary_freeze_sha256']==manifest['sha256']
    fingerprints=design['source_fingerprints']+supplement['additional_source_fingerprints']
    for entry in fingerprints:assert digest(Path(entry['path']))==entry['sha256'],entry['path']
    assert sys.version==supplement['python'] and np.__version__==supplement['numpy'] and scipy.__version__==supplement['scipy']
    assert len(design['candidates'])==7 and len(design['models'])==16
    return design,manifest['sha256'],dict(primary_sha256=manifest['sha256'],supplement_sha256=sm['sha256'],
        source_files_verified=len(fingerprints),python=sys.version,numpy=np.__version__,scipy=scipy.__version__)


def trailing_mean(values,n):
    # Constant prehistory, causal n-sample mean. Analysis feature only.
    extended=np.concatenate([np.repeat(values[:1],n-1,axis=0),values],axis=0)
    sums=np.concatenate([np.zeros_like(extended[:1]),np.cumsum(extended,axis=0)],axis=0)
    return (sums[n:]-sums[:-n])/n


def feature_checks():
    const=np.full((100,2,3),4.);assert np.allclose(trailing_mean(const,80),const)
    ramp=np.arange(100.)[:,None];perturbed=ramp.copy();perturbed[60:]+=1000
    assert np.array_equal(trailing_mean(ramp,20)[:60],trailing_mean(perturbed,20)[:60])


def observed(prepared):
    d=prepared['d'];wall=d['t_wall'];route=d['phase']=='route';active=np.isin(d['phase'],['route','slow_seat'])
    sides=[]
    for side in 'LR':
        normal=c.R.from_quat(np.column_stack([d['mq'+q+side] for q in 'xyzw'])).apply(np.tile([1.,0,0],(len(wall),1)))
        force=np.column_stack([d['F'+q+side] for q in 'xyz'])
        sides.append(np.maximum(np.sum(normal*force,axis=1),0.))
    values=np.column_stack(sides);mean=values.mean(axis=1)
    cfg=prepared['rec']['meta']['execution']['ffi']
    target=cfg['pickup_measured_N_per_hand']+680*d['ffi_ff_want_mm']/1000
    assert np.isfinite(values[active]).all() and np.isfinite(target[active]).all()
    weights=np.r_[np.diff(wall),0.];route_duration=weights[route].sum()
    rel=wall-wall[np.flatnonzero(route)[0]]
    route_t=rel[route];grid=np.arange(0,route_t[-1],c.DT)
    interp_force=np.interp(grid,route_t,mean[route])
    tidx=np.maximum(0,np.searchsorted(route_t,grid,side='right')-1)
    interp_error=interp_force-target[route][tidx]
    bands={}
    for label,bounds in [('low',[.2,.8]),('mid',[.8,2.5]),('high',[2.5,10])]:
        bands['force_'+label+'_band_rms_N']=float(c.band_rms(grid,interp_force[:,None],bounds)[0])
        bands['error_'+label+'_band_rms_N']=float(c.band_rms(grid,interp_error[:,None],bounds)[0])
    metrics=dict(id=prepared['rec']['id'],piece=prepared['rec']['piece'],route_duration_s=float(route_duration),
        route_rmse_mean_hand_N=float(np.sqrt(np.sum((mean[route]-target[route])**2*weights[route])/route_duration)),
        active_min_hand_L_R_N=[float(values[active,j].min()) for j in [0,1]],
        route_min_hand_minus_common_request_L_R_N=[float((values[route,j]-target[route]).min()) for j in [0,1]],
        fraction_route_time_hand_below_common_request_by025N_L_R=[float(np.sum(weights[route]*(values[route,j]<target[route]-.25))/route_duration) for j in [0,1]],
        logged_sum_normal_reconstruction_max_discrepancy_N=float(abs(values[active].sum(axis=1)-d['fn_N'][active]).max()),
        band_method='Offline linear interpolation of observed CSV forces, ZOH logged target, cubic detrend/zero-phase bandpass. Not a controller filter.',
        limitation='Clipped projected normals as in controller. Deviation from common scalar request is NOT full-wrench/friction margin; logging target and sensor timestamps are not simultaneous.',**bands)
    keep=active
    trace=dict(observed_time_from_route_s=rel[keep],observed_normal_L_R_N=values[keep],observed_mean_hand_N=mean[keep],
        observed_common_request_N=target[keep],observed_phase=d['phase'][keep])
    return metrics,trace


def secondary_metrics(prepared,trace,rows,design):
    route=prepared['route'];error=trace['force']-trace['target'][:,None,None]
    extras={}
    for label,bounds in [('low',[.2,.8]),('mid',[.8,2.5]),('high',[2.5,10])]:
        extras['route_force_'+label+'_band_rms_N']=c.band_rms(trace['t'][route],trace['force'][route],bounds)
        extras['route_error_'+label+'_band_rms_N']=c.band_rms(trace['t'][route],error[route],bounds)
    seat=(trace['t']>=prepared['route1'])&(trace['t']<=prepared['route1']+1.6)
    n=round(.2/c.DT)
    extras['first_seat_max_200ms_trailing_mean_force_N']=np.max(trailing_mean(trace['force'],n)[seat],axis=0)
    extras['first_seat_max_200ms_trailing_mean_positive_error_N']=np.maximum(0,np.max(trailing_mean(error,n)[seat],axis=0))
    ci={x['id']:i for i,x in enumerate(design['candidates'])};mi={x['id']:i for i,x in enumerate(design['models'])}
    for row in rows:
        i=ci[row['candidate']];j=mi[row['model']]
        row.update({key:float(value[i,j]) for key,value in extras.items()})


def cached_payload(path,freeze_sha):
    result=json.loads(path.read_text());assert result['freeze_sha256']==freeze_sha
    assert digest(OUT/result['trace_file'])==result['trace_sha256']
    assert len(result['metrics'])==112
    return result


def main():
    design,freeze_sha,verification=verify_freeze();feature_checks()
    started=OUT/'HOLDOUT_EVALUATION_STARTED.json';runner_sha=digest(Path(__file__))
    entries=[design['data_split'][piece]['holdout'][0] for piece in 'ZLST']
    if not started.exists():
        exclusive_json(started,dict(created_utc=now(),freeze_sha256=freeze_sha,runner_sha256=runner_sha,pid=os.getpid(),
            argv=sys.argv,target_run_ids=[x['id'] for x in entries],expected_cases=448,verification=verification,
            statement='Created before the first reserved CSV is analyzed. Fixed hypotheses; no tuning allowed.'))
    original_start=json.loads(started.read_text())
    assert original_start['freeze_sha256']==freeze_sha and original_start['runner_sha256']==runner_sha
    catalogue=json.loads((OUT.parent/'archive_review/run_catalog.json').read_text())
    c.DT=design['time_grid_s'];c.old_replay.DT=c.DT
    compact=dict(candidates=design['candidates'],models=design['models'],baseline_id=design['baseline_id'])
    progress=dict(started_utc=original_start['created_utc'],updated_utc=now(),status='running',pid=os.getpid(),
        freeze_sha256=freeze_sha,runner_sha256=runner_sha,completed=[])
    all_rows=[];observations=[];validations=[];payloads=[]
    try:
        for entry in entries:
            result_path=OUT/(entry['id']+'_holdout_result.json')
            if result_path.exists():
                payload=cached_payload(result_path,freeze_sha)
                print('Reused immutable completed run',entry['id'],flush=True)
            else:
                # Exact original bytes are checked before numerical data loading.
                assert digest(ROOT/entry['source'])==entry['csv_sha256'],entry['id']
                rec=next(x for x in catalogue if x['id']==entry['id']);prepared=c.old_replay.prepare(rec)
                initial=prepared['make']();assert initial.k_eff==680 and initial.s.ki_m_per_N_s==.0016
                assert initial.s.total_speed_m_per_s==.002 and initial.s.trim_limit_m==.003
                metrics,validation,trace=c.replay(prepared,compact,dt=c.DT)
                secondary_metrics(prepared,trace,metrics,design)
                real_metrics,real_trace=observed(prepared)
                attempt=1
                while (OUT/(entry['id']+f'_holdout_attempt{attempt}_traces.npz')).exists():attempt+=1
                trace_path=OUT/(entry['id']+f'_holdout_attempt{attempt}_traces.npz')
                with trace_path.open('xb') as handle:
                    np.savez_compressed(handle,**trace,**real_trace,route0=prepared['route0'],route1=prepared['route1'],
                        candidate_ids=np.array([x['id'] for x in design['candidates']]),
                        model_ids=np.array([x['id'] for x in design['models']]))
                payload=dict(completed_utc=now(),id=entry['id'],piece=rec['piece'],freeze_sha256=freeze_sha,
                    source_csv_sha256=entry['csv_sha256'],trace_file=trace_path.name,trace_sha256=digest(trace_path),
                    metrics=metrics,observed=real_metrics,validation=validation)
                exclusive_json(result_path,payload)
                print('Locked evaluation completed',rec['piece'],entry['id'],len(metrics),'cases',flush=True)
                del prepared,trace
            all_rows.extend(payload['metrics']);observations.append(payload['observed']);validations.append(payload['validation'])
            payloads.append(dict(id=entry['id'],result_file=result_path.name,result_sha256=digest(result_path),
                trace_file=payload['trace_file'],trace_sha256=payload['trace_sha256']))
            progress['completed']=payloads.copy();progress['updated_utc']=now();atomic_progress(progress)
        assert len(all_rows)==448 and len(observations)==4
        completed=OUT/'HOLDOUT_EVALUATION_COMPLETED.json'
        if not completed.exists():
            exclusive_json(completed,dict(completed_utc=now(),freeze_sha256=freeze_sha,runner_sha256=runner_sha,
                total_cases=len(all_rows),results=payloads,verification=verification,
                baseline_force_max_error_N=max(x['baseline_force_replay_max_error_N'] for x in validations),
                baseline_command_max_error_mm=max(x['baseline_command_replay_max_error_mm'] for x in validations)))
        progress['status']='complete';progress['updated_utc']=now();progress['pid']=None;atomic_progress(progress)
        print('ONE locked holdout batch complete:',len(all_rows),'cases. Parameters unchanged.',flush=True)
    except Exception as exc:
        progress.update(status='interrupted_or_failed',updated_utc=now(),error=repr(exc),pid=None)
        atomic_progress(progress)
        raise


if __name__=='__main__':main()
