"""One bounded simulation child at a time, checkpointed after each run."""
import sys,os,json,subprocess,time,hashlib
from pathlib import Path
from datetime import datetime,timezone
sys.dont_write_bytecode=True
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[3]
JOBS=[('L_D60_dt05','L',.0005,60,True),('S_D60_dt05','S',.0005,60,False),
      ('T_D60_dt05','T',.0005,60,False),('Z_D60_dt025','Z',.00025,60,False),
      ('Z_D80_dt025','Z',.00025,80,False)]
env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1')
def save(x):(OUT/'campaign_state.json').write_text(json.dumps(x,indent=2)+'\n')
def main():
    state=dict(started_utc=datetime.now(timezone.utc).isoformat(),supervisor_pid=os.getpid(),active_child=None,completed=[],jobs=JOBS)
    save(state)
    for tag,piece,dt,damping,video in JOBS:
        dest=OUT/'runs'/tag
        if (dest/'COMPLETED.json').exists():state['completed'].append(tag);continue
        assert not dest.exists(),f'Existing unfinished run: inspect first {dest}'
        cmd=[sys.executable,'-u',str(OUT/'run_bullet.py'),'--piece',piece,'--dt',str(dt),'--damping',str(damping),'--tag',tag]
        if video:cmd.append('--video')
        with (OUT/(tag+'.log')).open('x') as log:
            child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,cwd=ROOT,env=env)
            state['active_child']=dict(pid=child.pid,command=cmd);save(state)
            print('Started',tag,child.pid,flush=True)
            try:code=child.wait(timeout=900)
            except subprocess.TimeoutExpired:
                child.terminate();child.wait(timeout=20);state['timeout']=tag;save(state);raise
        state['active_child']=None;state['completed'].append(tag);save(state)
        print('Finished',tag,'exit',code,flush=True)
        if code!=0:raise RuntimeError(f'{tag} failed: inspect log')
    state['completed_utc']=datetime.now(timezone.utc).isoformat();state['supervisor_pid']=None;save(state)
    print('Campaign complete; no persistent worker',flush=True)
if __name__=='__main__':main()
