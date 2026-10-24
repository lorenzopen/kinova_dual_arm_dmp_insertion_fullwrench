"""Exclusive source/report/metric checkpoint, without duplicating large NPZ traces."""
import hashlib
import json
import re
import sys
import zipfile
from datetime import datetime,timezone
from pathlib import Path

OUT=Path(__file__).resolve().parent


def main():
    tag=sys.argv[1]
    if not re.fullmatch(r'[a-zA-Z0-9_-]+',tag):raise ValueError('Simple checkpoint tag required')
    folder=OUT/'checkpoints';folder.mkdir(exist_ok=True)
    target=folder/(tag+'.zip');manifest=folder/(tag+'_manifest.json')
    if target.exists():
        saved=json.loads(manifest.read_text())
        assert hashlib.sha256(target.read_bytes()).hexdigest()==saved['zip_sha256']
        print('Checkpoint already exists and is unchanged:',tag);return
    payload={p.name:p.read_bytes() for p in sorted(OUT.iterdir()) if p.is_file()
             and p.suffix in ['.py','.md','.json','.png','.log']}
    entries=[dict(path=name,bytes=len(data),sha256=hashlib.sha256(data).hexdigest()) for name,data in payload.items()]
    with zipfile.ZipFile(target,'x',zipfile.ZIP_DEFLATED) as z:
        for name,data in payload.items():z.writestr(name,data)
    with zipfile.ZipFile(target) as z:
        assert z.testzip() is None
        for item in entries:assert hashlib.sha256(z.read(item['path'])).hexdigest()==item['sha256']
    metadata=dict(created_utc=datetime.now(timezone.utc).isoformat(),tag=tag,files=entries,
                  zip_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
                  note='Read-only checkpoint of root-level campaign sources/reports/JSON/plots. Large NPZ and CSV results remain separately in campaign folder; protected baseline/NVIDIA_RUNS not included or modified.')
    with manifest.open('x',encoding='utf-8') as f:json.dump(metadata,f,indent=2)
    print('Saved verified checkpoint',tag,'files',len(entries),'bytes',target.stat().st_size,flush=True)


if __name__=='__main__':main()
