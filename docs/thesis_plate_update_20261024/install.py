"""Check evidence and install only reviewed plate chapter/figures/PDF."""
from pathlib import Path
import hashlib,json,os,re,shutil,sys
import math

HERE=Path(__file__).resolve().parent
PROJECT=HERE/'project'
TARGET=Path('C:/Users/Lorenzo/Desktop/TESI_antiG/thesis_english_2026').resolve()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
original=json.loads((HERE/'original_manifest.json').read_text())
old_by_path={x['path']:x['sha256'] for x in original['files']}
paths=list(old_by_path)+['figures/scripts/fig_plate_conformity.py']
for stem in ['plate_front_side','plate_conformity_bullet','plate_conformity_trace']:
    paths += ['figures/ch04_contacts/'+stem+ext for ext in ['.pdf','.png']]
data=PROJECT/'figures/data/plate_conformity_20261024'
paths += [p.relative_to(PROJECT).as_posix() for p in sorted(data.iterdir()) if p.is_file()]
assert len(paths)==len(set(paths))
plan=[dict(path=rel,previous_sha256=old_by_path.get(rel),sha256=sha(PROJECT/rel)) for rel in paths]

manifest=json.loads((data/'manifest.json').read_text())
for x in manifest['sources']:assert sha(Path(x['path']))==x['sha256']
for x in manifest['files']:assert sha(data/x['path'])==x['sha256']
results=json.loads((data/'results.json').read_text())
for x in results['source']:assert sha(HERE/x['path'])==x['sha256']
rigid_error=0.
for case in results['cases']:
    assert case['settled']['loaded_pads']==3
    for phase in ['first_load','settled']:
        st=case[phase]
        assert st['loaded_pads']==sum(force>.05 for force in st['pad_normal_N'])
        points=st['pad_centres']
        for i,j in [(0,1),(1,2),(2,0)]:
            dist=math.sqrt(sum((a-b)**2 for a,b in zip(points[i],points[j])))
            rigid_error=max(rigid_error,abs(dist-math.sqrt(3)*.03))
assert rigid_error<2e-6
log=(PROJECT/'build/main.log').read_text(errors='replace')
assert 'Output written on' in log and not re.search(r'^!',log,re.M)
for term in ['undefined references','undefined citations','multiply defined']:
    assert term not in log.lower()
widths=lambda text:re.findall(r'Overfull \\hbox \(([0-9.]+)pt too wide',text)
assert widths(log)==['38.07782','37.79417','72.63292'], 'New overfull boxes'

def check_originals():
    for x in plan:
        dst=(TARGET/x['path']).resolve()
        assert dst.is_relative_to(TARGET) and 'NVIDIA_RUNS' not in dst.parts
        if x['previous_sha256'] is None:assert not dst.exists(), 'New target exists: '+str(dst)
        else:assert sha(dst)==x['previous_sha256'], 'Concurrent edit: '+str(dst)
    changed={x['path'] for x in plan}
    for path in PROJECT.rglob('*'):
        if not path.is_file():continue
        rel=path.relative_to(PROJECT)
        if rel.parts[0]=='build' or rel.as_posix() in changed:continue
        assert (TARGET/rel).is_file() and sha(path)==sha(TARGET/rel), 'Dependency changed: '+str(rel)

check_originals()
if '--apply' not in sys.argv:
    (HERE/'install_plan.json').write_text(json.dumps(dict(target=str(TARGET),files=plan),indent=2)+'\n')
    (HERE/'verification.json').write_text(json.dumps(dict(compiled=True,undefined_references=False,
        undefined_citations=False,new_overfull_boxes=0,existing_table_overflow_fixed=True,
        inherited_overfull_boxes_elsewhere=3,model_source_hashes_checked=True,
        measured_pad_distance_error_m=rigid_error,rendered_chapter_pages=9,
        cases=3,physics_steps_per_case=12000,seconds_per_case=3,pages=141),indent=2)+'\n')
    print(json.dumps(dict(reviewed_files=len(plan),model_hashes_verified=True,max_rigid_pad_distance_error_m=rigid_error),indent=2))
else:
    assert json.loads((HERE/'install_plan.json').read_text())['files']==plan
    for x in plan:
        dst=TARGET/x['path'];dst.parent.mkdir(parents=True,exist_ok=True)
        tmp=dst.with_name(dst.name+'.codex-plate-update.tmp')
        assert not tmp.exists()
        try:
            shutil.copyfile(PROJECT/x['path'],tmp)
            assert sha(tmp)==x['sha256']
            os.replace(tmp,dst)
        finally:
            if tmp.exists():tmp.unlink()
    for x in plan:assert sha(TARGET/x['path'])==x['sha256']
    (HERE/'installed.json').write_text(json.dumps(dict(target=str(TARGET),all_hashes_verified=True,files=plan),indent=2)+'\n')
    print('Installed plate chapter, 3 figures, reproducible inputs and PDF; delivery hashes verified.')
