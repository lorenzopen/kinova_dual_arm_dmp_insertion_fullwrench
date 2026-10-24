"""Fetch official conda-forge binaries into this experiment only, no system install."""
import sys, json, hashlib, urllib.request, zipfile, tarfile, io
from pathlib import Path
sys.dont_write_bytecode = True
OUT = Path(__file__).resolve().parent
sys.path.insert(0,str(OUT/'deps/bootstrap'))
import zstandard

def get_json(url):
    with urllib.request.urlopen(url,timeout=90) as f:return json.load(f)

records=[]
for name,build in [('pybullet','py312h975a970_4'),('bullet-cpp','h72972c8_4'),('libzlib',None)]:
    meta=get_json('https://api.anaconda.org/package/conda-forge/'+name)
    files=[f for f in meta['files'] if f['attrs'].get('subdir')=='win-64']
    if build:files=[f for f in files if f['attrs'].get('build')==build]
    else:files=[f for f in files if f['version']=='1.3.1']
    entry=files[-1]
    cache=OUT/'deps/downloads'/Path(entry['basename']).name
    cache.parent.mkdir(parents=True,exist_ok=True)
    if not cache.exists():
        print('Downloading',entry['basename'],flush=True)
        urllib.request.urlretrieve('https:'+entry['download_url'],cache)
    raw=cache.read_bytes();digest=hashlib.sha256(raw).hexdigest()
    assert digest==entry['sha256'],(cache,digest,entry['sha256'])
    dest=OUT/'deps/conda';dest.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(cache) as archive:
        for member in archive.namelist():
            if member.startswith('pkg-') and member.endswith('.tar.zst'):
                with zstandard.ZstdDecompressor().stream_reader(io.BytesIO(archive.read(member))) as stream:
                    with tarfile.open(fileobj=stream,mode='r|') as tar:
                        for item in tar:
                            # Only runtime binary/DLL/license files, not unrelated
                            # robot-learning examples with Windows-long paths.
                            keep=(item.name.endswith(('.pyd','.dll')) or 'license' in item.name.lower())
                            if not item.isfile() or not keep:continue
                            target=(dest/item.name).resolve()
                            assert target.is_relative_to(dest.resolve())
                            target.parent.mkdir(parents=True,exist_ok=True)
                            target.write_bytes(tar.extractfile(item).read())
    records.append(dict(name=name,basename=entry['basename'],sha256=digest,depends=entry['attrs']['depends']))
(OUT/'runtime_provenance.json').write_text(json.dumps(records,indent=2)+'\n')
print('Binaries preserved locally',flush=True)
