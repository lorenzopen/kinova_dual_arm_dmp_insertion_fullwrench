"""Private conda-forge Pinocchio runtime; no system/environment mutations."""
import sys,os,json,hashlib,urllib.request,zipfile,tarfile,io,concurrent.futures,re
from pathlib import Path
HERE=Path(__file__).resolve().parent;OUT=HERE.parent
sys.path.insert(0,str(OUT/'deps/bootstrap'))
import zstandard
from pip._vendor.packaging.version import Version
from pip._vendor.packaging.specifiers import SpecifierSet
DEST=OUT/'deps/pin_conda';CACHE=OUT/'deps/pin_downloads';CACHE.mkdir(parents=True,exist_ok=True)
# These supply compile-time headers or the already-running CPython/Windows CRT.
SKIP={'python','python_abi','vc','ucrt','vc14_runtime','libboost-devel','libboost-python-devel','eigen','eigen-abi','urdfdom_headers','pkg-config','cmake','ninja'}
def getmeta(name):
    path=CACHE/(name+'.metadata.json')
    if path.exists():return json.loads(path.read_text())
    data=json.load(urllib.request.urlopen('https://api.anaconda.org/package/conda-forge/'+name,timeout=60))
    path.write_text(json.dumps(data));return data
def matches(entry,requirements):
    try:version=Version(entry['version'])
    except Exception:return False  # historical OpenSSL1.0.2h, outside modern constraints
    for requirement in requirements:
        bits=requirement.split()
        if len(bits)>1:
            spec=bits[1]
            if not spec.startswith(('<','>','=','!','~')):spec='=='+spec
            elif spec.startswith('=') and not spec.startswith('=='):spec='='+spec
            try:
                if version not in SpecifierSet(spec):return False
            except Exception:raise ValueError((entry['version'],requirement))
        if len(bits)>2:
            import fnmatch
            if not fnmatch.fnmatch(entry['attrs'].get('build',''),bits[2]):return False
    build=entry['attrs'].get('build','')
    if re.search(r'py\d{2,3}',build) and not ('py312' in build or 'py3h' in build):return False
    return True
def main():
    requirements={'pinocchio-python':{'pinocchio-python ==4.1.0'}};metas={};selected={}
    for iteration in range(30):
        missing=set(requirements)-set(metas)
        with concurrent.futures.ThreadPoolExecutor(6) as pool:
            for name,data in zip(sorted(missing),pool.map(getmeta,sorted(missing))):metas[name]=data
        changed=False
        for name in list(requirements):
            entries=[x for x in metas[name]['files'] if x['attrs'].get('subdir') in ('win-64','noarch') and matches(x,requirements[name])]
            assert entries,(name,requirements[name])
            entry=max(entries,key=lambda x:(Version(x['version']),x['attrs'].get('timestamp',0)))
            if selected.get(name,{}).get('basename')==entry['basename']:continue
            selected[name]=entry;changed=True
            for dep in entry['attrs'].get('depends',[]):
                dn=dep.split()[0]
                if dn in SKIP or dn.startswith('__') or dn=='numpy':continue
                requirements.setdefault(dn,set()).add(dep)
        if not changed and not (set(requirements)-set(metas)):break
    else:raise RuntimeError('Dependency resolution did not stabilize')
    records=[]
    for name,entry in selected.items():
        path=CACHE/Path(entry['basename']).name
        print('Runtime package',entry['basename'],flush=True)
        if not path.exists():urllib.request.urlretrieve('https:'+entry['download_url'],path)
        raw=path.read_bytes();digest=hashlib.sha256(raw).hexdigest()
        if entry.get('sha256'):assert digest==entry['sha256'],path
        else:assert hashlib.md5(raw).hexdigest()==entry['md5'],path
        def extract(tar):
            for item in tar:
                # Runtime Python, shared libraries and licenses only.
                keep=item.name.endswith(('.py','.pyd','.dll')) or 'license' in item.name.lower()
                if not item.isfile() or not keep:continue
                target=(DEST/item.name).resolve();assert target.is_relative_to(DEST.resolve())
                target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(tar.extractfile(item).read())
        if path.name.endswith('.conda'):
            with zipfile.ZipFile(path) as z:
                for member in z.namelist():
                    if member.startswith('pkg-') and member.endswith('.tar.zst'):
                        with zstandard.ZstdDecompressor().stream_reader(io.BytesIO(z.read(member))) as stream:
                            with tarfile.open(fileobj=stream,mode='r|') as tar:extract(tar)
        else:
            with tarfile.open(path,'r:bz2') as tar:extract(tar)
        records.append(dict(name=name,basename=entry['basename'],sha256=digest,
            metadata_digest='sha256' if entry.get('sha256') else 'md5',depends=entry['attrs'].get('depends',[])))
    (HERE/'pin_runtime_provenance.json').write_text(json.dumps(records,indent=2)+'\n')
    print('Private Pinocchio runtime extracted',flush=True)
if __name__=='__main__':main()
