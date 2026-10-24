"""Verify protected archive and saved runtime sources; write only in this campaign."""
import hashlib
import importlib.util
import json
import zipfile
from datetime import datetime, timezone
from pathlib import Path

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2]


def main():
    src=OUT.parent/'archive_review/inventory.py'
    spec=importlib.util.spec_from_file_location('readonly_manifest',src)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    # Only the manifest function is called: module.main would overwrite an old
    # analysis output, which this campaign deliberately does not do.
    now=module.manifest()
    before=json.loads((src.parent/'archive_manifest_before.json').read_text())
    assert now==before,'Protected archive differs from original manifest'
    dest=OUT/'baseline_20261023';meta=json.loads((dest/'manifest.json').read_text())
    target=dest/'source_and_simulations.zip'
    assert hashlib.sha256(target.read_bytes()).hexdigest()==meta['zip_sha256']
    checked=0
    with zipfile.ZipFile(target) as z:
        assert z.testzip() is None
        for entry in meta['files']:
            assert hashlib.sha256(z.read(entry['path'])).hexdigest()==entry['sha256']
            if entry['path'].split('/')[0] in ['kinova_dual_arm_dmp_insertion','config','launch','urdf']:
                assert hashlib.sha256((ROOT/entry['path']).read_bytes()).hexdigest()==entry['sha256']
                checked+=1
    result=dict(verified_utc=datetime.now(timezone.utc).isoformat(),archive_identical=True,
                files=len(now),bytes=sum(r['bytes'] for r in now),baseline_zip_sha256_verified=True,
                preserved_zip_files_verified=len(meta['files']),unchanged_runtime_source_files=checked)
    (OUT/'preservation_verification.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(result,flush=True)


if __name__=='__main__':main()
