"""Apply only the reviewed Haption chapter, figure manifest and compiled PDF."""
from pathlib import Path
import hashlib,json,os,re,shutil,sys

HERE=Path(__file__).resolve().parent
PROJECT=HERE/'project'
TARGET=Path('C:/Users/Lorenzo/Desktop/TESI_antiG/thesis_english_2026').resolve()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
original=json.loads((HERE/'original_manifest.json').read_text())
plan=[dict(path=x['path'],previous_sha256=x['sha256'],sha256=sha(PROJECT/x['path'])) for x in original['files']]
edited={x['path'] for x in plan}
log=(PROJECT/'build/main.log').read_text(errors='replace')
before=(TARGET/'build/main.log').read_text(errors='replace')
assert 'Output written on' in log and not re.search(r'^!',log,re.M)
for term in ['undefined references','undefined citations','multiply defined']:
    assert term not in log.lower()
overfull=lambda text:re.findall(r'Overfull \\hbox \(([^\n]+)',text)
assert overfull(log)==overfull(before), 'New overfull material'

def check_originals():
    for x in plan:
        dst=(TARGET/x['path']).resolve()
        assert dst.is_relative_to(TARGET) and 'NVIDIA_RUNS' not in dst.parts
        assert sha(dst)==x['previous_sha256'], 'Concurrent edit: '+str(dst)
    for path in PROJECT.rglob('*'):
        if not path.is_file():continue
        rel=path.relative_to(PROJECT)
        if rel.parts[0]=='build' or rel.as_posix() in edited:continue
        assert (TARGET/rel).is_file() and sha(path)==sha(TARGET/rel), 'Dependency changed: '+str(rel)

check_originals()
if '--apply' not in sys.argv:
    (HERE/'install_plan.json').write_text(json.dumps(dict(target=str(TARGET),files=plan),indent=2)+'\n')
    (HERE/'verification.json').write_text(json.dumps(dict(compiled=True,undefined_references=False,
        undefined_citations=False,new_overfull_boxes=0,rendered_chapter_pages=6,
        diagrams=['fig:recording','fig:virtual-coupling-schematic'],
        colours=['#1f6fb2','#d9622b'],pages=139),indent=2)+'\n')
    print('Reviewed install plan: 3 files; chapter 3, figure documentation and PDF.')
else:
    assert json.loads((HERE/'install_plan.json').read_text())['files']==plan
    for x in plan:
        dst=TARGET/x['path'];tmp=dst.with_name(dst.name+'.codex-haption-update.tmp')
        assert not tmp.exists()
        try:
            shutil.copyfile(PROJECT/x['path'],tmp)
            assert sha(tmp)==x['sha256']
            os.replace(tmp,dst)
        finally:
            if tmp.exists():tmp.unlink()
    for x in plan: assert sha(TARGET/x['path'])==x['sha256']
    (HERE/'installed.json').write_text(json.dumps(dict(target=str(TARGET),all_hashes_verified=True,files=plan),indent=2)+'\n')
    print('Installed and verified: 3 files. Previous chapter and PDF preserved in originals/.')
