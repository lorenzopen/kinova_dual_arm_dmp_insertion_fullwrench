"""Inventory protected archive, writing only outside NVIDIA_RUNS."""
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]
SOURCE=ROOT/'NVIDIA_RUNS'
OUT=Path(__file__).resolve().parent


def manifest():
    files=[]
    for p in sorted(SOURCE.rglob('*')):
        if not p.is_file():continue
        st=p.stat()
        sha=hashlib.sha256()
        with p.open('rb') as f:
            for block in iter(lambda:f.read(4*1024*1024),b''):sha.update(block)
        files.append({'path':str(p.relative_to(SOURCE)),'bytes':st.st_size,
                      'mtime_ns':st.st_mtime_ns,'sha256':sha.hexdigest()})
    return files


def main():
    OUT.mkdir(exist_ok=True)
    files=manifest()
    mode=sys.argv[1] if len(sys.argv)>1 else 'before'
    if mode=='before':
        target=OUT/'archive_manifest_before.json'
        if target.exists():raise RuntimeError('Preserve original before manifest')
        target.write_text(json.dumps(files,indent=2)+'\n',encoding='utf-8')
    elif mode=='verify':
        before=json.loads((OUT/'archive_manifest_before.json').read_text(encoding='utf-8'))
        result={'identical':before==files,'file_count_before':len(before),'file_count_after':len(files),
                'total_bytes':sum(f['bytes'] for f in files)}
        (OUT/'archive_integrity.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        assert before==files,'Archive changed: inspect differences before proceeding'
        print('Integrity:',result)
        return
    print('Protected files:',len(files),'GB:',round(sum(f['bytes'] for f in files)/1e9,3),flush=True)
    print('Extensions:',dict(Counter(Path(f['path']).suffix for f in files)),flush=True)
    runs=[]
    for p in sorted(SOURCE.rglob('*_meta.json')):
        try:m=json.loads(p.read_text(encoding='utf-8'))
        except Exception as e:
            print('metadata parse error',str(p),str(e));continue
        runs.append({'path':str(p.relative_to(SOURCE)), 'csv_exists':p.with_name(p.name.replace('_meta.json','.csv')).exists(),
                     'argv':m.get('argv'),'result':m.get('result'),'execution':m.get('execution'),
                     'controller_parameters':m.get('controller_parameters')})
    (OUT/'metadata_inventory.json').write_text(json.dumps(runs,indent=2)+'\n',encoding='utf-8')
    print('Metadata records:',len(runs),flush=True)
    for r in runs:
        if 'runs_riuscitee' in r['path'] or (r['result'] or {}).get('seat_found'):
            print(r['path'],'RESULT',r['result'],flush=True)


if __name__=='__main__':main()
