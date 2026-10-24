"""Check staged inputs/metrics and assemble rendered-page review sheets."""
from pathlib import Path
import hashlib
import json
import re
from PIL import Image, ImageDraw

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
PROJECT=HERE/'project'
DATA=PROJECT/'figures/data/kinova_comparison_20261024'
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
manifest=json.loads((DATA/'manifest.json').read_text())
for item in manifest['sources']:
    assert sha(ROOT/item['path'])==item['sha256'], item['path']
for item in manifest['local_arrays']:
    assert sha(DATA/item['path'])==item['sha256']
metrics=json.loads((DATA/'metrics.json').read_text())['cases']
expected={'Z':{'real':[.301,.125,.066,1.446], 'simulation':[.209,.042,.014,.850]},
          'L':{'real':[.282,.115,.066,2.638], 'simulation':[.278,.043,.012,1.470]}}
keys=['route_rmse_N','route_force_band_08_25_N','route_force_band_25_10_N','seat_positive_error_N']
for piece,origins in expected.items():
    for origin,values in origins.items():
        for key,value in zip(keys,values):
            assert abs(metrics[piece][origin][key]-value)<.000501, (piece,origin,key)
log=(PROJECT/'build/main.log').read_text(errors='replace')
assert 'Output written on' in log
assert 'undefined references' not in log.lower()
assert 'undefined citations' not in log.lower()
assert 'multiply defined' not in log.lower()
assert not re.search(r'^!',log,re.M)
old=Path('C:/Users/Lorenzo/Desktop/TESI_antiG/thesis_english_2026/build/main.log').read_text(errors='replace')
over=re.findall(r'Overfull \\hbox \(([^\n]+)',log)
assert over==re.findall(r'Overfull \\hbox \(([^\n]+)',old), 'New overfull boxes'
rendered=sorted((HERE/'review').glob('page-*.png'))
for k in range(0,len(rendered),4):
    files=rendered[k:k+4]
    sheet=Image.new('RGB',(1100,2*820),'#bbbbbb')
    draw=ImageDraw.Draw(sheet)
    for j,path in enumerate(files):
        im=Image.open(path).convert('RGB');im.thumbnail((540,790))
        x=(j%2)*550+(550-im.width)//2;y=(j//2)*820+22
        sheet.paste(im,(x,y));draw.text((j%2*550+12,j//2*820+5),path.stem,fill='black')
    sheet.save(HERE/'review'/f'sheet_{k//4+1:02d}.png')
report=dict(source_hashes_checked=len(manifest['sources']),local_arrays_checked=len(manifest['local_arrays']),
            metrics_table_verified=True,undefined_references=False,undefined_citations=False,
            new_overfull_boxes=0,inherited_overfull_boxes=len(over),rendered_pages=len(rendered))
(HERE/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
