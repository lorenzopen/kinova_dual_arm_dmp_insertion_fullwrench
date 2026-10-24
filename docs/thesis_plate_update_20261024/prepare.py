from pathlib import Path
import hashlib,json,shutil,difflib

HERE=Path(__file__).resolve().parent
PROJECT=HERE/'project'
ORIGINAL=Path('C:/Users/Lorenzo/Desktop/TESI_antiG/thesis_english_2026')
MODEL=ORIGINAL.parent/'lorenzo_data_analysis/10_tripad_passivo'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
paths=['chapters/04_contact_and_end_effectors.tex','figures/README.md','build/main.pdf']
manifest=HERE/'original_manifest.json'
if not manifest.exists():
    entries=[]
    for rel in paths:
        src=ORIGINAL/rel;dst=HERE/'originals'/rel
        dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
        entries.append(dict(path=rel,sha256=sha(src)))
    manifest.write_text(json.dumps(dict(root=str(ORIGINAL),files=entries),indent=2)+'\n')
data=PROJECT/'figures/data/plate_conformity_20261024'
data.mkdir(parents=True,exist_ok=True)
sources=[]
for rel in ['spider_ee/floating_plate.py','lib/tool.py']:
    assert sha(HERE/'vendor'/rel)==sha(MODEL/rel)
    sources.append(dict(path=str(MODEL/rel),sha256=sha(MODEL/rel)))
for path in [HERE/'bench_plate.py',HERE/'render_views.py',HERE/'bench/results.json']:
    sources.append(dict(path=str(path),sha256=sha(path)))
results=json.loads((HERE/'bench/results.json').read_text())
for entry in results['source']:
    assert sha(HERE/entry['path'])==entry['sha256'],entry['path']
for record in results['cases']:
    case=record['case']
    for phase in ['first_load','settled']:
        src=HERE/'bench/rendered'/(case+'_'+phase+'.png')
        shutil.copy2(src,data/src.name)
    shutil.copy2(HERE/'bench'/case/'trace.csv',data/(case+'_trace.csv'))
shutil.copy2(HERE/'bench/results.json',data/'results.json')
entries=[dict(path=p.name,sha256=sha(p)) for p in sorted(data.iterdir()) if p.is_file() and p.name!='manifest.json']
(data/'manifest.json').write_text(json.dumps(dict(kind='Illustrative fixed-flange contact bench',sources=sources,
    renderer='PyBullet TinyRenderer, recorded flange/plate poses; visual-only front-facet tints',
    files=entries),indent=2)+'\n')
source=(HERE/'originals'/paths[0]).read_text(encoding='utf-8')
start=source.index('\\begin{figure}[htbp]')
end=source.index('\\end{figure}',start)+len('\\end{figure}')
source=source[:start]+r'''\thesisfigure[1.0]{ch04_contacts/plate_front_side.pdf}{Three discrete contact spheres in front view (a), and the floating-plate mechanism in side view (b), introduced in Section~\ref{sec:floating-plate}. The side view exaggerates deformation and is not to scale; the lower two spheres overlap in this projection. Rigid supports place the zero-free-length spring anchors at the unloaded sphere centres. The central and peripheral springs connect the same flange and plate in parallel. Tangential contact forces create torsion through their lever arms; a point contact has no local moment.}{fig:tripad-geometry}
'''+source[end:]
anchor='\\section{Floating triangular plate}'
assert source.count(anchor)==1
source=source.replace(anchor,anchor+'\n\\label{sec:floating-plate}')
anchor='Let $\\vect\\xi=(\\vect d,\\vect\\phi)$ be a small displacement of the plate.'
assert source.count(anchor)==1
source=source.replace(anchor,(HERE/'plate_text.tex').read_text()+'\n'+anchor)
# World support anchors and rest-frame plate lever arms must have distinct symbols.
source=source.replace('At attachment $\\vect a_i$, measured from the plate centre,',
    'With rest-frame lever arm $\\vect r_i=\\vect c_i-[d,0,0]^\\top$ about the plate centre,')
source=source.replace('-\\crossmat{\\vect a_i}', '-\\crossmat{\\vect r_i}')
source=source.replace('\\sum_i\\vect a_i=0', '\\sum_i\\vect r_i=0')
anchor='\\section{Compliance centre and transmission}'
assert source.count(anchor)==1
source=source.replace(anchor,(HERE/'bench_text.tex').read_text()+'\n'+anchor)
# Keep the pre-existing wide audit table inside the margins of this edited chapter.
assert source.count('\\begin{tabular}{@{}lrr@{}}\\toprule')==1
assert source.count('\\bottomrule\\end{tabular}')==1
source=source.replace('\\begin{tabular}{@{}lrr@{}}\\toprule','\\resizebox{\\textwidth}{!}{\\begin{tabular}{@{}lrr@{}}\\toprule')
source=source.replace('\\bottomrule\\end{tabular}','\\bottomrule\\end{tabular}}')
(PROJECT/paths[0]).write_text(source,encoding='utf-8',newline='\n')
readme=(HERE/'originals'/paths[1]).read_text(encoding='utf-8')+'''
## Floating-plate mechanism and conformity (24 October 2026)

`scripts/fig_plate_conformity.py` creates `ch04_contacts/plate_front_side`,
`plate_conformity_bullet` and `plate_conformity_trace` as vector PDF/PNG.
Compact figure inputs and model/runtime identities are in
`data/plate_conformity_20261024/manifest.json`. The two copied model modules
match the originals by SHA-256. The bench is a prescribed fixed-flange contact
illustration (zero gravity, visual-only plate), not a new population or SOCP
campaign. The images render recorded poses, with visual-only facet tints.
The side view exaggerates deformation to show the point springs; unloaded
support tips and sphere centres coincide, giving l0=0 despite a 20 mm flange
offset. The plate and all three spheres remain one rigid body.
'''
(PROJECT/paths[1]).write_text(readme,encoding='utf-8',newline='\n')
diff=[]
for rel in paths[:2]:
    before=(HERE/'originals'/rel).read_text(encoding='utf-8').splitlines(True)
    after=(PROJECT/rel).read_text(encoding='utf-8').splitlines(True)
    diff.extend(difflib.unified_diff(before,after,fromfile='before/'+rel,tofile='after/'+rel))
(HERE/'changes.diff').write_text(''.join(diff),encoding='utf-8')
print('Staged chapter 4, source-checked bench evidence and figure data.')
