"""Resolve a saved-output QA clock mismatch; timestamps/FFI dt only, no tuning."""
import sys
sys.dont_write_bytecode=True
import csv
import json
import hashlib
from pathlib import Path
from datetime import datetime,timezone
import conditional_controllers as c
import numpy as np
from evaluate_locked_holdout import verify_freeze

OUT=c.OUT;ROOT=c.ROOT
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    design,freeze,verification=verify_freeze()
    attempt=OUT/'audit_saved_evidence.py';snapshot=OUT/'audit_saved_evidence_attempt1.py'
    if not snapshot.exists():
        with snapshot.open('xb') as f:f.write(attempt.read_bytes())
    failure=OUT/'SAVED_EVIDENCE_ATTEMPT1_FAILURE.json'
    if not failure.exists():
        with failure.open('x',encoding='utf-8') as f:json.dump(dict(created_utc=datetime.now(timezone.utc).isoformat(),
            attempt_source=snapshot.name,attempt_source_sha256=sha(snapshot),
            log='evidence_audit_process.log',log_sha256=sha(OUT/'evidence_audit_process.log'),
            failed_assertion='max command increment / log-wall-time interval <=2mm/s',
            note='QA initially treated saved event_time (CSV write clock) as FFI integration dt. Preserve this failure; diagnose actual dt without changing frozen results.'),f,indent=2)
    results=[]
    for piece in 'ZLST':
        entry=design['data_split'][piece]['holdout'][0]
        path=ROOT/entry['source'];assert sha(path)==entry['csv_sha256']
        seq=[];wall=[];dt=[]
        with path.open(newline='',encoding='utf-8-sig') as f:
            for row in csv.DictReader(f):
                seq.append(float(row['ffi_seq']));wall.append(float(row['t_wall']));dt.append(float(row['ffi_dt_ms']))
        seq=np.array(seq);wall=np.array(wall);dt=np.array(dt)
        ix=np.flatnonzero(np.isfinite(seq)&(seq>0)&np.r_[True,np.diff(seq)>0])
        assert np.all(np.diff(seq[ix])==1)
        te=wall[ix]-wall[ix[0]];clock=np.cumsum(np.r_[0,dt[ix[1:]]/1000])
        saved=json.loads((OUT/(entry['id']+'_holdout_result.json')).read_text())
        with np.load(OUT/saved['trace_file'],allow_pickle=False) as z:
            assert np.array_equal(z['event_time'],te)
            commands=z['commands'];change=abs(np.diff(commands,axis=0))
            ratio_log=change/np.diff(te)[:,None,None]
            ratio_used=change/np.diff(clock)[:,None,None]
            key=np.unravel_index(np.argmax(ratio_log),ratio_log.shape);k,ci,mi=key
            max_used=float(ratio_used.max());assert max_used<=2+1e-9
            declared=max(r['max_command_speed_mm_s'] for r in saved['metrics'])
            assert abs(max_used-declared)<1e-12
            result=dict(id=entry['id'],piece=piece,source_csv_sha256=sha(path),events=len(te),
                max_increment_per_FFI_used_dt_mm_s=max_used,
                max_increment_per_log_time_dt_mm_s=float(ratio_log.max()),
                max_abs_log_dt_minus_FFI_dt_ms=float(max(abs(np.diff(te)-np.diff(clock)))*1000),
                log_dt_p50_min_max_ms=(np.array([np.median(np.diff(te)),min(np.diff(te)),max(np.diff(te))])*1000).tolist(),
                at_max_log_ratio=dict(event_index=int(k+1),candidate=str(z['candidate_ids'][ci]),model=str(z['model_ids'][mi]),
                    log_interval_ms=float(np.diff(te)[k]*1000),FFI_used_interval_ms=float(np.diff(clock)[k]*1000),
                    command_increment_mm=float(change[key])))
        clocks=OUT/(entry['id']+'_saved_clock_audit.npz')
        with clocks.open('xb') as f:np.savez_compressed(f,event_time=te,ffi_integration_clock=clock,ffi_seq=seq[ix])
        result.update(clock_file=clocks.name,clock_sha256=sha(clocks));results.append(result)
    output=dict(created_utc=datetime.now(timezone.utc).isoformat(),freeze_sha256=freeze,verification=verification,
        scope='Read only ffi_seq,t_wall,ffi_dt_ms from reserved CSVs solely to audit clocks; no force/target analysis, simulation, tuning or changed selection.',
        result='All saved alternatives respect2mm/s per FFI integration/clamped dt; dividing by logger-wall intervals is a different quantity and can exceed2.',
        limitation='This does not measure actual robot tool velocity or actual target consumption times. Frozen replay timing remains based on CSV event times; timing uncertainty is not removed.',runs=results)
    with (OUT/'SAVED_CLOCK_DIAGNOSTIC.json').open('x',encoding='utf-8') as f:json.dump(output,f,indent=2)
    for r in results:print(r['piece'],'software-dt max',r['max_increment_per_FFI_used_dt_mm_s'],'log-dt max',r['max_increment_per_log_time_dt_mm_s'],'dt difference ms',r['max_abs_log_dt_minus_FFI_dt_ms'])


if __name__=='__main__':main()
