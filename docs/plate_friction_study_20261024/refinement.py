"""Single-worker time-step quality assurance; run only after the main campaign."""
from pathlib import Path
import os,json,hashlib,importlib.util,difflib,time
HERE=Path(__file__).resolve().parent

def main():
    state=json.loads((HERE/'STATE.json').read_text())
    assert state.get('complete') and state.get('category')=='runs' and not (HERE/'runs.lock').exists()
    manifest=json.loads((HERE/'pilot_manifest.json').read_text());jobs=manifest['jobs']
    source=(HERE/'study.py').read_text()
    assert hashlib.sha256((HERE/'study.py').read_bytes()).hexdigest()==manifest['source_sha256']
    assert source.count('DT=.0005')==1 and source.count('if step%20:continue')==1
    refined=(source.replace('DT=.0005','DT=.00025').replace('if step%20:continue','if step%round(.01/DT):continue')
        .replace('pads=FP.PadSprings(dt=DT)','pads=FP.PadSprings(dt=.0005)')
        .replace('centre=FP.CentralSpring(k_push=k,dt=DT)','centre=FP.CentralSpring(k_push=k,dt=.0005)'))
    path=HERE/'study_refinement.py';path.write_text(refined)
    (HERE/'refinement_source.diff').write_text(''.join(difflib.unified_diff(source.splitlines(True),refined.splitlines(True),fromfile='study.py',tofile='study_refinement.py')))
    spec=importlib.util.spec_from_file_location('study_refinement',path);worker=importlib.util.module_from_spec(spec);spec.loader.exec_module(worker)
    lock=HERE/'refinement.lock';fd=os.open(lock,os.O_CREAT|os.O_EXCL|os.O_WRONLY);os.write(fd,str(os.getpid()).encode());os.close(fd)
    try:
        worker.save(HERE/'refinement_manifest.json',dict(jobs=jobs,source_sha256=worker.sha(path),original_source_sha256=worker.sha(HERE/'study.py'),
            protocol_sha256=worker.sha(HERE/'REFINEMENT_PROTOCOL.md'),time=time.time()))
        for i,job in enumerate(jobs):
            dest=HERE/'refinement'/job['id']
            if (dest/'COMPLETED.json').exists():continue
            records=json.loads((HERE/'profiles'/(worker.profile_key(job)+'.json')).read_text())['records']
            worker.save(HERE/'REFINEMENT_STATE.json',dict(pid=os.getpid(),index=i,current=job['id'],total=len(jobs)))
            result=worker.run(job,records,dest)
            print(json.dumps(dict(index=i+1,id=job['id'],error=result['error'])),flush=True)
        worker.save(HERE/'REFINEMENT_STATE.json',dict(complete=True,pid=None,total=len(jobs)))
    finally:lock.unlink(missing_ok=True)

if __name__=='__main__':main()
