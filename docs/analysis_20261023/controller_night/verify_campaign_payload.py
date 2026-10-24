"""Standard-library read-only verification of the local campaign payload."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2]


def hash_stream(stream):
    digest=hashlib.sha256();size=0
    while block:=stream.read(1024*1024):digest.update(block);size+=len(block)
    return digest.hexdigest(),size


def verify(manifest_path,compare_workspace=False):
    manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
    package=manifest_path.parent/manifest['zip_file']
    with package.open('rb') as f:actual,_=hash_stream(f)
    assert actual==manifest['zip_sha256'],'Payload ZIP SHA mismatch'
    immutable=0;snapshots=0
    with zipfile.ZipFile(package) as z:
        assert len(z.namelist())==len(set(z.namelist()))==len(manifest['files'])
        assert set(z.namelist())=={x['path'] for x in manifest['files']}
        for entry in manifest['files']:
            with z.open(entry['path']) as f:digest,size=hash_stream(f)
            assert digest==entry['sha256'] and size==entry['bytes'],entry['path']
            if compare_workspace:
                if entry['mutable_operational_snapshot']:
                    snapshots+=1;continue
                path=(ROOT/entry['path']).resolve()
                assert path.is_relative_to(ROOT.resolve()),entry['path']
                with path.open('rb') as f:current,current_size=hash_stream(f)
                assert current==entry['sha256'] and current_size==entry['bytes'],entry['path']
                immutable+=1
    return dict(zip_verified=True,files=len(manifest['files']),bytes=sum(x['bytes'] for x in manifest['files']),
        campaign_npz=manifest['counts']['campaign_npz'],immutable_workspace_files_verified=immutable,
        operational_snapshots_not_compared_to_live=snapshots,
        note='Checks bytes/CRC/SHA only. No simulations, ROS, robot, writes, extraction or predictive-model validation.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path)
    parser.add_argument('--compare-workspace',action='store_true')
    args=parser.parse_args()
    path=args.manifest
    if path is None:
        pointer=json.loads((OUT/'CURRENT_PAYLOAD.json').read_text())
        path=OUT/pointer['manifest']
    print(json.dumps(verify(path,args.compare_workspace),indent=2))


if __name__=='__main__':main()
