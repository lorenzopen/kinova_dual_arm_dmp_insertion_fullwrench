"""Actual offline-profile preview, development only, no future measured force.

Preview remains inside the CURRENT observed phase. No observed future phase
timestamps are queried. Recorded force residual and phase timing remain fixed
in conditional replay; no robot predictive adequacy is asserted.
"""
import sys
sys.dont_write_bytecode=True
import advanced_controllers as a
import hashlib
import csv
import json
import itertools
from datetime import datetime,timezone
from pathlib import Path
import numpy as np

c=a.c;OUT=a.OUT;ROOT=c.ROOT


def freeze():
    path=OUT/'profile_preview_design.json'
    if path.exists():return json.loads(path.read_text())
    previous=a.freeze();base_ids=['I_P0_I1.6_D0','PI_P0.1_I0.8_D0_R0.1']
    bases=[next(x for x in previous['candidates'] if x['id']==rid) for rid in base_ids]
    candidates=[dict(x,preview='none',preview_s=0.,clock_offset_s=0.,base_controller=x['id']) for x in bases]
    for cfg,mode,horizon,offset in itertools.product(bases,['shift','rise_envelope'],[.15,.3,.6,1.2],[-.04,0.,.04]):
        candidates.append(dict(cfg,id=f"{cfg['id']}__{mode}_H{horizon:g}_O{offset:g}",base_controller=cfg['id'],
            preview=mode,preview_s=horizon,clock_offset_s=offset))
    design=dict(created_utc=datetime.now(timezone.utc).isoformat(),candidates=candidates,
        models=previous['models'],baseline_id=previous['baseline_id'],dt_s=c.DT,
        score=previous['score'],data_split='Fixed8 development only, four holdouts excluded.',
        profile_source='Actual CSV named and SHA-256 referenced by each development metadata. Record whether newline normalization is needed.',
        clock='Causal first-observed-phase-row zero plus hypothetical offset -40/0/+40ms. Exact phase_t0 is not logged. These offsets are sensitivity scenarios, not identified error bounds.',
        shift='want_logged + [profile(current_phase,elapsed+h)-profile(current_phase,elapsed)]/680. Current-phase endpoint held, never uses next phase or future recorded phase times.',
        rise_envelope='want_logged + max_{0<=s<=h}[profile(current_phase,elapsed+s)-1.36*s-profile(current_phase,elapsed)]/680. This never decreases nominal relative to logged FF. The1.36N/s cone assumes680N/m and2mm/s; it is not a full globally feasible plan or a contact guarantee.',
        anchor='Delta-preview is anchored to exact original logged nominal; zero preview reproduces baseline regardless of CSV rounding/phase-clock error. I/P error always uses original logged target. Not an exact reconstruction of a deployed absolute-profile-preview controller.',
        limits='Original FF priority and same FF/trim/sum speed and travel bounds; recorded clamped integration dt retained.',
        limitation=previous['caveat']+' Full wrench margins/individual contact forces are not revalidated when normal feedforward changes. No cart_d modification.')
    with path.open('x',encoding='utf-8') as f:json.dump(design,f,indent=2)
    return design


def read_profile(rec):
    execution=rec['meta']['execution'];basename=execution['offline_profile'].split('/')[-1]
    path=(ROOT/rec['source']).parent/basename
    raw=path.read_bytes();expected=execution['offline_profile_sha256']
    raw_hash=hashlib.sha256(raw).hexdigest();lf_hash=hashlib.sha256(raw.replace(b'\r\n',b'\n')).hexdigest()
    assert expected in [raw_hash,lf_hash],(rec['id'],expected,raw_hash,lf_hash)
    with path.open(newline='',encoding='utf-8-sig') as f:rows=list(csv.DictReader(f))
    profile={}
    for phase in dict.fromkeys(r['phase'] for r in rows):
        rr=[r for r in rows if r['phase']==phase]
        times=np.array([float(r['t_in_phase']) for r in rr]);target=np.array([float(r['target_N']) for r in rr])
        assert np.all(np.diff(times)>0) and np.isfinite(target).all()
        profile[phase]=(times,target)
    return profile,dict(id=rec['id'],path=str(path.relative_to(ROOT)),metadata_sha256=expected,
        file_sha256=raw_hash,hash_matches_raw=expected==raw_hash,hash_matches_LF_normalized=expected==lf_hash,
        phases={ph:dict(samples=len(ts),first_s=float(ts[0]),last_s=float(ts[-1]),
            min_target_N=float(y.min()),max_target_N=float(y.max()),
            max_abs_target_slope_N_s=float(np.max(abs(np.diff(y)/np.diff(ts)))) if len(ts)>1 else 0.) for ph,(ts,y) in profile.items()})


def causal_phase_clock(d,ix):
    starts={};elapsed=np.empty(len(ix));phase_previous=None;phase_start=0.;j=0
    for i in range(ix[-1]+1):
        phase=str(d['phase'][i])
        if phase!=phase_previous:
            phase_start=float(d['t_wall'][i]);phase_previous=phase
        if i==ix[j]:
            elapsed[j]=float(d['t_wall'][i])-phase_start
            j+=1
            if j==len(ix):break
    assert j==len(ix)
    return elapsed


def preview_value(profile,phase,elapsed,horizon,mode):
    ts,y=profile[phase];base=float(np.interp(elapsed,ts,y))
    if mode=='none' or horizon==0:return base
    end=elapsed+horizon
    if mode=='shift':return float(np.interp(end,ts,y))
    if mode!='rise_envelope':raise ValueError(mode)
    nodes=np.r_[elapsed,ts[(ts>elapsed)&(ts<end)],end]
    values=np.interp(nodes,ts,y)-1.36*(nodes-elapsed)
    return float(max(base,values.max()))


def wants(p,design,profile):
    elapsed=causal_phase_clock(p['d'],p['ix'])
    original=p['d']['ffi_ff_want_mm'][p['ix']]/1000
    values=np.tile(original[:,None],(1,len(design['candidates'])))
    for i,cfg in enumerate(design['candidates']):
        if cfg['preview']=='none':continue
        for k,(phase,clock) in enumerate(zip(p['phase'],elapsed)):
            if str(phase) not in profile:raise ValueError((p['rec']['id'],phase))
            clock=max(0.,clock+cfg['clock_offset_s'])
            base=preview_value(profile,str(phase),clock,0.,'none')
            preview=preview_value(profile,str(phase),clock,cfg['preview_s'],cfg['preview'])
            values[k,i]+=(preview-base)/680.
    return values,elapsed


def checks():
    example={'route':(np.array([0.,1.,2.,3.]),np.array([4.,4.,6.,3.])),
             'slow_seat':(np.array([0.,1.]),np.array([1.,1.]))}
    # Plateau endpoints hold even though the next phase is known in the file.
    assert preview_value(example,'route',2.9,5.,'shift')==3.
    assert preview_value(example,'route',10.,1.,'rise_envelope')==3.
    assert preview_value(example,'route',.9,.5,'rise_envelope')>4.
    for clock,horizon in itertools.product(np.linspace(0,4,31),[0,.15,.6,1.2]):
        base=preview_value(example,'route',clock,0,'none')
        assert preview_value(example,'route',clock,horizon,'rise_envelope')>=base-1e-12
    # Offline future is allowed, but it is only the current phase's planned target.
    changed=dict(example,slow_seat=(np.array([0.,1.]),np.array([100.,100.])))
    for mode in ['shift','rise_envelope']:
        assert preview_value(example,'route',2.,2.,mode)==preview_value(changed,'route',2.,2.,mode)
    # Phase-clock prefixes cannot change when future rows are changed.
    d=dict(phase=np.array(['to_mouth']*4+['route']*6),t_wall=np.arange(10)*.01)
    clock=causal_phase_clock(d,np.arange(10))
    altered=dict(phase=d['phase'].copy(),t_wall=d['t_wall'].copy())
    altered['phase'][7:]='slow_seat';altered['t_wall'][7:]+=100
    assert np.array_equal(clock[:7],causal_phase_clock(altered,np.arange(10))[:7])
    return dict(endpoint_hold=True,no_cross_phase_preview=True,clock_prefix_invariance=True,
        rise_envelope_never_reduces_nominal=True,zero_horizon_identity=True)


def main():
    design=freeze();validation=checks();frozen=json.loads((OUT/'frozen_design.json').read_text())
    catalogue=json.loads((OUT.parent/'archive_review/run_catalog.json').read_text())
    old={(r['id'],r['candidate'],r['model']):r for r in json.loads((OUT/'advanced_controller_metrics.json').read_text())}
    rows=[];audit=[];regression=0.;replays=[]
    central=next(i for i,m in enumerate(design['models']) if m['id']=='delay_k680_d0.18')
    for piece in 'ZLST':
        for entry in frozen['split'][piece]['development']:
            rec=next(r for r in catalogue if r['id']==entry['id']);p=c.old_replay.prepare(rec)
            profile,meta=read_profile(rec);nominal,elapsed=wants(p,design,profile)
            rr,val,tr=c.replay(p,design,nominal_wants=nominal);rows.extend(rr);replays.append(val)
            for row in rr:
                if '__' in row['candidate']:continue
                ref=old[(row['id'],row['candidate'],row['model'])]
                for field in ['route_rmse_N','route_band_rms_N','route_high_band_rms_N','first_seat_peak_N']:
                    regression=max(regression,abs(row[field]-ref[field]))
            profile_at_clock=np.array([preview_value(profile,str(ph),tt,0,'none') for ph,tt in zip(p['phase'],elapsed)])
            audit.append(dict(meta,clock_reconstruction_target_rmse_N=float(np.sqrt(np.mean((profile_at_clock-p['target'])**2))),
                clock_reconstruction_target_max_error_N=float(abs(profile_at_clock-p['target']).max()),
                note='Target mismatch audits unlogged phase clock and CSV precision. It is NOT fitted away; delta-preview anchors exact baseline.'))
            np.savez_compressed(OUT/(rec['id']+'_preview_central.npz'),t=tr['t'],target=tr['target'],
                force=tr['force'][:,:,central],commands=tr['commands'][:,:,central],event_time=tr['event_time'],
                nominal_wants_mm=nominal*1000,phase_clock_s=elapsed,phases=p['phase'],route0=p['route0'],route1=p['route1'],
                candidate_ids=np.array([x['id'] for x in design['candidates']]))
            print('Profile preview',piece,rec['id'],len(rr),'scenarios, regression',regression,flush=True)
            del p,tr
    assert regression<1e-10,regression
    ranking=a.rank(rows,design)
    # Group clock hypotheses: never select a favorable clock offset.
    grouped=[]
    for base,mode,horizon in itertools.product(['I_P0_I1.6_D0','PI_P0.1_I0.8_D0_R0.1'],['shift','rise_envelope'],[.15,.3,.6,1.2]):
        configs=[x for x in design['candidates'] if x['base_controller']==base and x['preview']==mode and x['preview_s']==horizon]
        ids={x['id'] for x in configs};subset=[r for r in rows if r['candidate'] in ids]
        runids=sorted({r['id'] for r in subset})
        score=float(np.mean([max(r['score'] for r in subset if r['id']==rid) for rid in runids]))
        grouped.append(dict(base_controller=base,preview=mode,preview_s=horizon,
            mean_run_worst_model_and_clock_score=score,candidate_ids=sorted(ids),
            minimum_force_mean_hand_N=min(r['minimum_mean_hand_force_N'] for r in subset)))
    grouped.sort(key=lambda r:r['mean_run_worst_model_and_clock_score'])
    c.dp.dump('profile_preview_metrics.json',rows)
    with (OUT/'profile_preview_metrics.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    c.dp.dump('profile_preview_ranking.json',ranking);c.dp.dump('profile_preview_grouped_ranking.json',grouped)
    c.dp.dump('profile_preview_audit.json',audit)
    c.dp.dump('profile_preview_validation.json',dict(checks=validation,exact_no_preview_metric_regression_N=regression,
        scenarios=len(rows),candidates=len(design['candidates']),replay_checks=replays))
    print('Best grouped preview',json.dumps(grouped[:5]),flush=True)


if __name__=='__main__':main()
