from pathlib import Path
from pypdf import PdfReader
import hashlib,json
root=Path(__file__).resolve().parent
papers=Path('C:/Users/Lorenzo/Desktop/TESI_antiG/PAPERS')
sources=[]
for key,name in [('friction_cone','friction_cone.pdf'),('wong','2026 Industrial Dual Arm Box Handling.pdf')]:
    path=papers/name
    reader=PdfReader(path)
    text='\n'.join(f'\n=== PAGE {i+1} ===\n'+page.extract_text() for i,page in enumerate(reader.pages))
    (root/'papers'/(key+'.txt')).write_text(text,encoding='utf-8')
    sources.append(dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),pages=len(reader.pages),metadata=str(reader.metadata)))
(root/'papers/sources.json').write_text(json.dumps(sources,indent=2)+'\n')
print(json.dumps(sources,indent=2))
