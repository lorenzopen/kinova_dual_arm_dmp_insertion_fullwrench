from pathlib import Path
import hashlib,json,shutil,difflib

HERE=Path(__file__).resolve().parent
PROJECT=HERE/'project'
ORIGINAL=Path('C:/Users/Lorenzo/Desktop/TESI_antiG/thesis_english_2026')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
paths=['chapters/03_haption_recording.tex','figures/README.md','build/main.pdf']
manifest=HERE/'original_manifest.json'
if not manifest.exists():
    entries=[]
    for rel in paths:
        src=ORIGINAL/rel;dst=HERE/'originals'/rel
        dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dst)
        entries.append(dict(path=rel,sha256=sha(src)))
    manifest.write_text(json.dumps(dict(root=str(ORIGINAL),files=entries),indent=2)+'\n')
source=(HERE/'originals'/paths[0]).read_text(encoding='utf-8')
start=source.index('\\begin{figure}[htbp]')
end=source.index('\\end{figure}',start)+len('\\end{figure}')
source=source[:start]+(HERE/'recording_architecture.tex').read_text(encoding='utf-8').rstrip()+source[end:]
anchor='For a proxy at $\\vect p_h$ with velocity $\\vect v_h$, the translational command is'
assert source.count(anchor)==1
source=source.replace(anchor,'Figure~\\ref{fig:virtual-coupling-schematic} shows one translational channel. The mapped handle target and the simulated proxy can have different poses; their displacement produces the elastic part of the interaction.\n\n'+anchor)
anchor="The rotational error is represented by the relative quaternion"
assert source.count(anchor)==1
source=source.replace(anchor,(HERE/'virtual_coupling.tex').read_text(encoding='utf-8')+'\n'+anchor)
(PROJECT/paths[0]).write_text(source,encoding='utf-8',newline='\n')
readme=(HERE/'originals'/paths[1]).read_text(encoding='utf-8')
readme+='''
## Haption and virtual coupling diagrams (24 October 2026)

Chapter 3 contains two editable TikZ illustrations: the recording architecture
with schematic Haption silhouettes, and one translational spring-damper
coupling. They use the existing blue `#1f6fb2` and orange `#d9622b` hand
colours with light tints and neutral grey objects. The silhouettes are
explanatory drawings, not device CAD. The diagram distinguishes the elastic
wrench log from the filtered/scaled/clipped feedback and from the complete
proxy command. No experimental values or controller source are changed.
'''
(PROJECT/paths[1]).write_text(readme,encoding='utf-8',newline='\n')
diff=[]
for rel in paths[:2]:
    before=(HERE/'originals'/rel).read_text(encoding='utf-8').splitlines(True)
    after=(PROJECT/rel).read_text(encoding='utf-8').splitlines(True)
    diff.extend(difflib.unified_diff(before,after,fromfile='before/'+rel,tofile='after/'+rel))
(HERE/'changes.diff').write_text(''.join(diff),encoding='utf-8')
print('Staged chapter 3: two coloured vector diagrams, existing notation and hand colours.')
