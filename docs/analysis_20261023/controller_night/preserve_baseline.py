"""Exclusive, verified source snapshot; reads original run data only."""
import hashlib
import json
import subprocess
import zipfile
from datetime import datetime, timezone
from pathlib import Path

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[2]
DEST=OUT/'baseline_20261023'


def git(*args):
    return subprocess.check_output(['git',*args],cwd=ROOT)


def main():
    if DEST.exists():
        meta=json.loads((DEST/'manifest.json').read_text())
        assert hashlib.sha256((DEST/'source_and_simulations.zip').read_bytes()).hexdigest()==meta['zip_sha256']
        print('Existing snapshot verified; left untouched.',flush=True)
        return
    DEST.mkdir()
    records=json.loads((OUT.parent/'archive_review/run_catalog.json').read_text())
    runs=[r for r in records if r['status']=='curated_seat_found' and '_ffi_' in r['id']]
    tracked=git('ls-files','-z').decode().split('\0')
    roots={'kinova_dual_arm_dmp_insertion','config','launch','urdf','resource','tests'}
    paths={p for p in tracked if p and (p.split('/')[0] in roots or '/' not in p)}
    paths.update(str(p.relative_to(ROOT)).replace('\\','/')
                 for p in (OUT.parent).rglob('*.py') if '__pycache__' not in p.parts)
    paths.update(['docs/FFI_CONTROLLER.md','AGENTS.md'])
    payload={p:(ROOT/p).read_bytes() for p in sorted(paths) if (ROOT/p).is_file()}
    run_info=[]
    for r in runs:
        meta=r['meta'];revision=meta['git']
        entry=dict(id=r['id'],source=r['source'],metadata=r['metadata'],git=revision,
                   git_dirty=meta.get('git_dirty'),argv=meta.get('argv'),
                   execution=meta.get('execution'),controller_parameters=meta.get('controller_parameters'))
        run_info.append(entry)
        payload['run_metadata/'+Path(r['metadata']).name]=(ROOT/r['metadata']).read_bytes()
        profile=Path(meta.get('execution',{}).get('offline_profile','').replace('\\','/')).name
        matches=list((ROOT/'NVIDIA_RUNS').rglob(profile)) if profile else []
        if matches:payload['offline_profiles/'+profile]=matches[0].read_bytes()
    # Preserve the source blob from the exact revision recorded by all 12 runs,
    # separately from the current working-tree snapshot, including dirty status.
    revisions=sorted({r['git'] for r in run_info})
    comparisons=[]
    for rev in revisions:
        for path in ['kinova_dual_arm_dmp_insertion/insert_ffi.py',
                     'kinova_dual_arm_dmp_insertion/controller.py',
                     'kinova_dual_arm_dmp_insertion/ros_backend.py']:
            raw=git('show',f'{rev}:{path}')
            payload[f'run_revisions/{rev}/{path}']=raw
            comparisons.append(dict(revision=rev,path=path,
                                    current_matches_run_normalized=raw.replace(b'\r\n',b'\n')==payload[path].replace(b'\r\n',b'\n')))
    payload['git_status.txt']=git('status','--short')
    payload['run_provenance.json']=(json.dumps(run_info,indent=2,ensure_ascii=False)+'\n').encode()
    manifest=dict(created_utc=datetime.now(timezone.utc).isoformat(),
                  head=git('rev-parse','HEAD').decode().strip(),
                  branch=git('branch','--show-current').decode().strip(),
                  run_revisions=revisions,source_comparison=comparisons,
                  files=[dict(path=p,bytes=len(v),sha256=hashlib.sha256(v).hexdigest()) for p,v in sorted(payload.items())])
    target=DEST/'source_and_simulations.zip'
    with zipfile.ZipFile(target,'x',zipfile.ZIP_DEFLATED) as z:
        for path,data in payload.items():z.writestr(path,data)
    with zipfile.ZipFile(target) as z:
        assert z.testzip() is None
        for entry in manifest['files']:
            assert hashlib.sha256(z.read(entry['path'])).hexdigest()==entry['sha256']
    manifest['zip_sha256']=hashlib.sha256(target.read_bytes()).hexdigest()
    (DEST/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    (DEST/'README.md').write_text('# Baseline conservata\n\nArchivio sorgenti e simulazioni precedente alla ricerca notturna. '
        'Include codice corrente, versioni di esecuzione verificate, configurazioni, metadati delle 12 FFI e profili offline. '
        'Manifest SHA-256 per file e ZIP; non sovrascrivere. I CSV/bag originali restano in NVIDIA_RUNS e non sono duplicati qui. '
        'Nessuna garanzia di ambiente ROS completo: le dipendenze Python locali non sono incluse.\n',encoding='utf-8')
    print('Saved',target,'files',len(payload),'bytes',target.stat().st_size,flush=True)
    print('Run revisions',revisions,'comparison',comparisons,flush=True)


if __name__=='__main__':main()
