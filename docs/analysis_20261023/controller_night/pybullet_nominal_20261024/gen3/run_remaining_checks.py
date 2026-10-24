"""Bounded sequential articulated check campaign. At most one physics child."""
import json,os,sys,subprocess
from datetime import datetime,timezone
from pathlib import Path
HERE=Path(__file__).resolve().parent;OUT=HERE.parent
STATE=HERE/'campaign_state.json'
def stamp():return datetime.now(timezone.utc).isoformat()
def write(state):STATE.write_text(json.dumps(state,indent=2)+'\n')
def main():
    # The preceding manual run must have released its worker before launch.
    previous=OUT/'runs/Gen3_Z_central2700_implicit_dt025/COMPLETED.json'
    assert previous.exists(),'Refinement still active or incomplete; do not duplicate it'
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
    jobs=[('L_articulated',[str(HERE/'run_gen3.py'),'--piece','L','--tag','Gen3_L_central2700_implicit_dt05','--dt','.0005','--servo','implicit']),
          ('articulated_sensor',[str(HERE/'check_articulated.py')]),
          ('floating_sensor',[str(OUT/'check_backend.py')])]
    state=dict(started_utc=stamp(),supervisor_pid=os.getpid(),active_child=None,completed=[])
    write(state)
    for name,args in jobs:
        if name=='L_articulated' and (OUT/'runs/Gen3_L_central2700_implicit_dt05').exists():
            raise RuntimeError('L destination exists; inspect checkpoint rather than overwrite/relaunch')
        with (HERE/(name+'.log')).open('x') as log:
            child=subprocess.Popen([sys.executable,'-u',*args],env=env,stdout=log,stderr=subprocess.STDOUT)
            state['active_child']=dict(name=name,pid=child.pid,argv=args,started_utc=stamp());write(state)
            code=child.wait()
            state['completed'].append(dict(name=name,returncode=code,finished_utc=stamp()))
            state['active_child']=None;write(state)
            # Preserve failed checks, continue independent remaining checks.
    state.update(supervisor_pid=None,completed_utc=stamp());write(state)
    print(json.dumps(state,indent=2))
if __name__=='__main__':main()
