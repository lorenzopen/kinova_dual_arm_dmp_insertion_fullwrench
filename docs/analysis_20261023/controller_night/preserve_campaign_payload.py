"""Exclusive local payload of current results, including large NPZ traces.

No source deletion/extraction/modification; no NVIDIA file contents copied.
"""
import sys
sys.dont_write_bytecode=True
import hashlib
import json
from pathlib import Path
from datetime import datetime,timezone
import zipfile
from verify_campaign_payload import verify,hash_stream

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2]
TAG='controller_night_20261024_0344'
MUTABLE={'STATE.json','NIGHT_LOG.md','RESULTS.md','preservation_verification.json'}


def read(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):
    with path.open('rb') as f:return hash_stream(f)[0]


def main():
    assert not any(p.is_symlink() for p in OUT.iterdir())
    dest=OUT/'deliveries';dest.mkdir(exist_ok=True)
    target=dest/(TAG+'.zip');manifest_path=dest/(TAG+'_manifest.json')
    assert not target.exists() and not manifest_path.exists(),'Exclusive payload already present; verify it, do not rebuild'
    baseline=OUT/'baseline_20261023';bm=read(baseline/'manifest.json')
    assert sha(baseline/'source_and_simulations.zip')==bm['zip_sha256']
    frozen_path=OUT/'FINAL_COMPARISON_FREEZE.json';frozen=read(frozen_path)
    assert sha(frozen_path)==read(OUT/'FINAL_COMPARISON_FREEZE_manifest.json')['sha256']
    supplement=read(OUT/'FINAL_COMPARISON_SOURCE_SUPPLEMENT.json')
    assert sha(OUT/'FINAL_COMPARISON_SOURCE_SUPPLEMENT.json')==read(OUT/'FINAL_COMPARISON_SOURCE_SUPPLEMENT_manifest.json')['sha256']
    source_entries=frozen['source_fingerprints']+supplement['additional_source_fingerprints']
    for entry in source_entries:assert sha(Path(entry['path']))==entry['sha256']
    deliveries=['HOLDOUT_DELIVERY_MANIFEST.json','LINEAR_ROBUSTNESS_DELIVERY_MANIFEST.json',
        'SIGNAL_PIPELINE_DELIVERY_MANIFEST.json','THESIS_DELIVERY_MANIFEST.json','EVIDENCE_AUDIT_DELIVERY_MANIFEST.json']
    for name in deliveries:
        for entry in read(OUT/name)['files']:assert sha(OUT/entry['path'])==entry['sha256']
    # Include every root-level scientific output, including all61 NPZ. Avoid
    # live tee output and self-referencing payload pointers/manifests.
    excluded={'payload_build_process.log','payload_verify_process.log','CURRENT_PAYLOAD.json'}
    files={p.resolve() for p in OUT.iterdir() if p.is_file() and p.name not in excluded}
    files.update(p.resolve() for p in baseline.iterdir() if p.is_file())
    files.update(Path(e['path']).resolve() for e in source_entries)
    # Reports refer to these earlier read-only analyses. Preserve their sources,
    # tables and figures beside the campaign, with original relative paths.
    archive=OUT.parent/'archive_review'
    files.update(p.resolve() for p in archive.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    assert all(p.is_relative_to(ROOT.resolve()) for p in files)
    assert not any(p.is_relative_to((ROOT/'NVIDIA_RUNS').resolve()) for p in files)
    entries=[]
    with zipfile.ZipFile(target,'x',zipfile.ZIP_DEFLATED,compresslevel=1,allowZip64=True) as z:
        for path in sorted(files,key=lambda p:p.relative_to(ROOT).as_posix()):
            name=path.relative_to(ROOT).as_posix();h=hashlib.sha256();size=0
            with path.open('rb') as src,z.open(name,'w',force_zip64=True) as dst:
                while block:=src.read(1024*1024):h.update(block);size+=len(block);dst.write(block)
            entries.append(dict(path=name,bytes=size,sha256=h.hexdigest(),
                mutable_operational_snapshot=path.parent==OUT and path.name in MUTABLE))
    counts=dict(files=len(entries),bytes=sum(x['bytes'] for x in entries),
        campaign_npz=sum(p.parent==OUT and p.suffix=='.npz' for p in files),
        campaign_csv=sum(p.parent==OUT and p.suffix=='.csv' for p in files),
        campaign_python=sum(p.parent==OUT and p.suffix=='.py' for p in files),
        protected_baseline_entries=len(bm['files']))
    manifest=dict(created_utc=datetime.now(timezone.utc).isoformat(),zip_file=target.name,zip_sha256=sha(target),
        files=entries,counts=counts,frozen_comparison_sha256=sha(frozen_path),
        note='Local preserved scientific payload, not a robot controller release. No NVIDIA_RUNS content copied; original data remain read-only at their existing paths. Runtime dependencies and original CSV/bags are still needed to rerun analyses. Operational state/log/report/preservation files are snapshots at packaging, all other listed files compare to live workspace.',
        baseline_zip_sha256=bm['zip_sha256'],excluded='Older source checkpoints and caches are retained outside this ZIP; live payload logs/pointer are excluded to avoid self-reference.')
    with manifest_path.open('x',encoding='utf-8') as f:json.dump(manifest,f,indent=2)
    # Independent read-only verifier checks every decompressed entry and CRC,
    # whole ZIP SHA, then immutable live files; no extraction or simulation.
    verified=verify(manifest_path,compare_workspace=True)
    with (dest/(TAG+'_verification.json')).open('x',encoding='utf-8') as f:
        json.dump(dict(verified_utc=datetime.now(timezone.utc).isoformat(),**verified),f,indent=2)
    with (OUT/'CURRENT_PAYLOAD.json').open('x',encoding='utf-8') as f:
        json.dump(dict(manifest=manifest_path.relative_to(OUT).as_posix(),zip=target.relative_to(OUT).as_posix(),
            zip_sha256=manifest['zip_sha256'],created_utc=manifest['created_utc']),f,indent=2)
    print('Exclusive preserved payload verified:',counts,flush=True)
    print('ZIP bytes:',target.stat().st_size,'SHA256:',manifest['zip_sha256'],flush=True)


if __name__=='__main__':main()
