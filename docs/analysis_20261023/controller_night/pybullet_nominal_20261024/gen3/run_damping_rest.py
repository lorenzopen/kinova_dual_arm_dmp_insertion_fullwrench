"""Sequential bounded D80 L and original-source checks after Z worker exits."""
import os,sys,json,subprocess
from pathlib import Path
from datetime import datetime,timezone
HERE=Path(__file__).resolve().parent;OUT=HERE.parent;STATE=HERE/'D80_campaign_state.json'
def now():return datetime.now(timezone.utc).isoformat()
def save(s):STATE.write_text(json.dumps(s,indent=2)+'\n')
def main():
    assert (OUT/'runs/Gen3_Z_central2700_implicit_D80_dt025/COMPLETED.json').exists(),'Wait for Z worker, do not overlap'
    jobs=[('L_D80','run_gen3_damping.py',['--piece','L','--tag','Gen3_L_central2700_implicit_D80_dt05','--dt','.0005','--servo','implicit','--damping','80']),
          ('Lab_D80','run_lab_damping.py',['--piece','Z','--tag','Gen3Lab_Z_central2700_warm_D80_dt025','--dt','.00025','--kalman-warm-seconds','2','--damping','80'])]
    state=dict(started_utc=now(),supervisor_pid=os.getpid(),active_child=None,completed=[]);save(state)
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
    for name,script,args in jobs:
        tag=args[args.index('--tag')+1];assert not (OUT/'runs'/tag).exists(),'Inspect existing checkpoint; never overwrite/relaunch'
        with (HERE/(name+'.log')).open('x') as log:
            process=subprocess.Popen([sys.executable,'-u',str(HERE/script),*args],env=env,stdout=log,stderr=subprocess.STDOUT)
            state['active_child']=dict(pid=process.pid,argv=[script,*args],started_utc=now());save(state)
            try:code=process.wait(timeout=900)
            except subprocess.TimeoutExpired:
                process.terminate();process.wait(timeout=15);code=-1
            state['completed'].append(dict(name=name,tag=tag,returncode=code,finished_utc=now()))
            state['active_child']=None;save(state)
    state.update(supervisor_pid=None,completed_utc=now());save(state);print(json.dumps(state,indent=2))
if __name__=='__main__':main()
