"""Install only reviewed thesis files after optimistic concurrency checks.

Default: prepare a hash-identified install plan and unified source diffs.
--apply: apply the exact plan, checking all originals before any mutation.
"""
from pathlib import Path
import difflib
import hashlib
import json
import os
import shutil
import sys

HERE=Path(__file__).resolve().parent
STAGED=HERE/'project'
TARGET=Path('C:/Users/Lorenzo/Desktop/TESI_antiG/thesis_english_2026').resolve()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
old=json.loads((HERE/'original_manifest.json').read_text())
old_by_path={x['path']:x['sha256'] for x in old['files']}
edited=[x['path'] for x in old['files']]
added=['figures/scripts/fig_kinova_comparison.py']
added += [str(p.relative_to(STAGED)).replace('\\','/') for p in sorted((STAGED/'figures/data/kinova_comparison_20261024').iterdir()) if p.is_file()]
for stem in ['real_sim_normal','real_sim_zoom','real_sim_moment','original_source_stop']:
    added += ['figures/ch11_kinova/'+stem+ext for ext in ['.pdf','.png']]
paths=edited+added
assert len(paths)==len(set(paths))
plan=[]
for rel in paths:
    src=STAGED/rel;dst=(TARGET/rel).resolve()
    assert dst.is_relative_to(TARGET)
    assert 'NVIDIA_RUNS' not in dst.parts
    assert src.is_file(), src
    plan.append(dict(path=rel,sha256=sha(src),bytes=src.stat().st_size,
                     previous_sha256=old_by_path.get(rel)))

def verify_before():
    for entry in plan:
        dst=TARGET/entry['path'];expected=entry['previous_sha256']
        if expected is not None:
            assert dst.is_file() and sha(dst)==expected, 'Original changed concurrently: '+str(dst)
        else:
            assert not dst.exists(), 'New target already exists: '+str(dst)
    # The compiled PDF must use the same remaining sources/assets as the target.
    for src in sorted(STAGED.rglob('*')):
        if not src.is_file():continue
        rel=src.relative_to(STAGED)
        if rel.parts[0]=='build' or str(rel).replace('\\','/') in paths:continue
        dst=TARGET/rel
        assert dst.is_file() and sha(dst)==sha(src), 'Dependency changed concurrently: '+str(dst)

if '--apply' not in sys.argv:
    verify_before()
    (HERE/'install_plan.json').write_text(json.dumps(dict(target=str(TARGET),files=plan),indent=2)+'\n')
    diffs=[]
    for rel in edited:
        if not rel.endswith(('.tex','.md')):continue
        before=(HERE/'originals'/rel).read_text(encoding='utf-8').splitlines(True)
        after=(STAGED/rel).read_text(encoding='utf-8').splitlines(True)
        diffs.extend(difflib.unified_diff(before,after,fromfile='before/'+rel,tofile='after/'+rel))
    (HERE/'thesis_changes.diff').write_text(''.join(diffs),encoding='utf-8')
    print(json.dumps(dict(mode='reviewed_plan',target=str(TARGET),file_count=len(plan),bytes=sum(x['bytes'] for x in plan)),indent=2))
else:
    saved=json.loads((HERE/'install_plan.json').read_text())
    assert saved['files']==plan and saved['target']==str(TARGET), 'Staged files changed after review'
    verify_before()
    for entry in plan:
        rel=entry['path'];dst=TARGET/rel
        dst.parent.mkdir(parents=True,exist_ok=True)
        tmp=dst.with_name(dst.name+'.codex-thesis-update.tmp')
        assert not tmp.exists()
        try:
            shutil.copyfile(STAGED/rel,tmp)
            assert sha(tmp)==entry['sha256']
            os.replace(tmp,dst)
        finally:
            if tmp.exists():tmp.unlink()
    for entry in plan:assert sha(TARGET/entry['path'])==entry['sha256']
    (HERE/'installed.json').write_text(json.dumps(dict(target=str(TARGET),file_count=len(plan),all_hashes_verified=True,files=plan),indent=2)+'\n')
    print(json.dumps(dict(installed_files=len(plan),all_hashes_verified=True,pdf=str(TARGET/'build/main.pdf')),indent=2))
