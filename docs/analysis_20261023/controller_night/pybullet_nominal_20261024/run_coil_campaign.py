"""Nominal real axial spring, original pad lateral/rotational model held fixed.
This is a declared model sensitivity, not identification of the real free coil.
"""
import os,sys,json,subprocess
from pathlib import Path
from datetime import datetime,timezone
sys.dont_write_bytecode=True
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[3]
def save(x):(OUT/'coil_campaign_state.json').write_text(json.dumps(x,indent=2)+'\n')
def main():
    old=json.loads((OUT/'campaign_state.json').read_text())
    assert (old.get('completed_utc') or old.get('stopped_by_user_steering')) and old['active_child'] is None,'Original worker must finish first'
    state=dict(started_utc=datetime.now(timezone.utc).isoformat(),supervisor_pid=os.getpid(),active_child=None,completed=[])
    for piece,dt in [('Z',.0005),('L',.0005),('S',.0005),('T',.0005),('L',.00025)]:
        tag=f'{piece}_axial2700_D60_dt'+('05' if dt==.0005 else '025')
        dest=OUT/'runs'/tag
        if (dest/'COMPLETED.json').exists():state['completed'].append(tag);continue
        assert not dest.exists(),f'Inspect unfinished {dest}'
        cmd=[sys.executable,'-u',str(OUT/'run_bullet.py'),'--piece',piece,'--dt',str(dt),'--damping','60',
             '--pad-axial','2700','--tag',tag]
        with (OUT/(tag+'.log')).open('x') as log:
            child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,cwd=ROOT,
                env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1'))
            state['active_child']=dict(pid=child.pid,command=cmd);save(state);print('Started',tag,child.pid,flush=True)
            try:code=child.wait(timeout=900)
            except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=20);state['timeout']=tag;save(state);raise
        state['active_child']=None;state['completed'].append(tag);save(state);print('Finished',tag,code,flush=True)
        if code:raise RuntimeError(tag)
    state['completed_utc']=datetime.now(timezone.utc).isoformat();state['supervisor_pid']=None;save(state)
    print('Coil sensitivity complete',flush=True)
if __name__=='__main__':main()
