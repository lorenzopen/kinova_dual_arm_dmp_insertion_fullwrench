"""Read-only original/source preservation and fresh-SOCP versus archived-plan parity."""
import sys,json,hashlib,importlib.util,csv
from pathlib import Path
from datetime import datetime,timezone
sys.dont_write_bytecode=True
OUT=Path(__file__).resolve().parent;NIGHT=OUT.parent;ROOT=OUT.parents[3]
sys.path.insert(0,str(ROOT/'tmp/dryrun_deps'))
import numpy as np
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    spec=importlib.util.spec_from_file_location('archive_manifest_only',NIGHT.parent/'archive_review/inventory.py')
    inv=importlib.util.module_from_spec(spec);spec.loader.exec_module(inv)
    original=json.loads((NIGHT.parent/'archive_review/archive_manifest_before.json').read_text());now=inv.manifest()
    assert original==now,'Original archive changed'
    baseline=json.loads((NIGHT/'baseline_20261023/manifest.json').read_text())
    assert sha(NIGHT/'baseline_20261023/source_and_simulations.zip')==baseline['zip_sha256']
    runtime=[e for e in baseline['files'] if e['path'].split('/')[0] in ['kinova_dual_arm_dmp_insertion','config','launch','urdf']]
    assert all(sha(ROOT/e['path'])==e['sha256'] for e in runtime),'Runtime source changed'
    vendor=[]
    origin=Path('C:/Users/Lorenzo/Desktop/TESI_antiG/lorenzo_data_analysis/10_tripad_passivo')
    for target,src in [('lib/tool.py','lib/tool.py'),('lib/wrist.py','lib/wrist.py'),('spider_ee/floating_plate.py','spider_ee/floating_plate.py'),('shelf.py','tetris/shelf.py'),('pieces.py','tetris/pieces.py')]:
        assert sha(OUT/'vendor'/target)==sha(origin/src)
        vendor.append(dict(path='vendor/'+target,source=str(origin/src),sha256=sha(origin/src)))
    profiles=[]
    for run in sorted((OUT/'runs').glob('*')):
        if not (run/'result.json').exists():continue
        start=json.loads((run/'STARTED.json').read_text());args=start['argv'];piece=args[args.index('--piece')+1]
        # Metadata copies under docs/runs, matched to development run identity.
        times={'Z':'20261023_174143','L':'20261023_182743','S':'20261023_190407','T':'20261023_192128'}
        metas=list((ROOT/'docs/runs').rglob('*'+times[piece]+'_meta.json'));assert len(metas)==1
        meta=json.loads(metas[0].read_text());name=Path(meta['execution']['offline_profile']).name
        archived=list((ROOT/'NVIDIA_RUNS').rglob(name));assert archived,name
        assert all(sha(p)==meta['execution']['offline_profile_sha256'] for p in archived)
        with archived[0].open() as f:old=list(csv.reader(f))
        with (run/'profile.csv').open() as f:new=list(csv.reader(f))
        identical_phase=[x[0] for x in old]==[x[0] for x in new]
        delta=float(np.max(np.abs(np.array([x[1:] for x in old[1:]],float)-np.array([x[1:] for x in new[1:]],float)))) if len(old)==len(new) else None
        profiles.append(dict(run=run.name,archived_profile=str(archived[0].relative_to(ROOT)),archived_sha256=sha(archived[0]),
                             same_rows=len(old)==len(new),same_phases=identical_phase,max_abs_numeric_difference=delta))
    paths=[p for p in (ROOT/'kinova_dual_arm_dmp_insertion').glob('*.py')]
    provenance=[dict(path=str(p.relative_to(ROOT)),sha256=sha(p)) for p in paths]
    result=dict(verified_utc=datetime.now(timezone.utc).isoformat(),NVIDIA_originals_identical=True,
                NVIDIA_files=len(now),NVIDIA_bytes=sum(e['bytes'] for e in now),baseline_sha256_verified=True,
                unchanged_runtime_files=len(runtime),vendor=vendor,runtime_sources=provenance,profiles=profiles)
    (OUT/'input_integrity_and_profile_parity.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ['NVIDIA_originals_identical','NVIDIA_files','unchanged_runtime_files','profiles']},indent=2))
if __name__=='__main__':main()
