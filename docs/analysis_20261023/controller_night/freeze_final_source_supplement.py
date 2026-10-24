"""Exclusive transitive source/runtime fingerprints, still before holdout."""
import sys
sys.dont_write_bytecode=True
import advanced_controllers as a
import hashlib
import json
import scipy
from datetime import datetime,timezone


def main():
    OUT=a.OUT;dest=OUT/'FINAL_COMPARISON_SOURCE_SUPPLEMENT.json'
    if dest.exists():print('Existing source supplement retained');return
    primary=OUT/'FINAL_COMPARISON_FREEZE.json';manifest=json.loads((OUT/'FINAL_COMPARISON_FREEZE_manifest.json').read_text())
    digest=hashlib.sha256(primary.read_bytes()).hexdigest();assert digest==manifest['sha256']
    paths=[OUT/'standalone.py',OUT/'damping_physics.py',OUT/'rigid_model.py',
        OUT/'freeze_final_comparison.py',OUT/'freeze_final_source_supplement.py']
    result=dict(created_utc=datetime.now(timezone.utc).isoformat(),primary_freeze_sha256=digest,
        additional_source_fingerprints=[dict(path=str(path.resolve()),sha256=hashlib.sha256(path.read_bytes()).hexdigest()) for path in paths],
        python=sys.version,numpy=a.np.__version__,scipy=scipy.__version__,
        baseline_snapshot_sha256=json.loads((OUT/'baseline_20261023/manifest.json').read_text())['zip_sha256'],
        note='Adds transitive helper/runtime evidence before holdout; original frozen parameters and file are unchanged. No reserved CSV read.')
    with dest.open('x',encoding='utf-8') as handle:json.dump(result,handle,indent=2)
    with (OUT/'FINAL_COMPARISON_SOURCE_SUPPLEMENT_manifest.json').open('x',encoding='utf-8') as handle:
        json.dump(dict(file=dest.name,sha256=hashlib.sha256(dest.read_bytes()).hexdigest()),handle,indent=2)
    print('Frozen extra sources/runtime; primary freeze unchanged',digest,flush=True)


if __name__=='__main__':main()
