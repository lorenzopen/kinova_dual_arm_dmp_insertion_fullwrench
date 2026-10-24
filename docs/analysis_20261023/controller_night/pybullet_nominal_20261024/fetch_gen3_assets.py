"""Official Kinova mesh assets; keep lab's kinematics/inertia in a derived URDF."""
import json,sys,urllib.request,hashlib
from pathlib import Path
sys.dont_write_bytecode=True
OUT=Path(__file__).resolve().parent
def get(url):
    with urllib.request.urlopen(url,timeout=90) as f:return f.read()
def main():
    repo='Kinovarobotics/ros2_kortex'
    tree=json.loads(get('https://api.github.com/repos/'+repo+'/git/trees/main?recursive=1'))
    commit=tree['sha'];prefix='kortex_description/arms/gen3/7dof/meshes/'
    files=[e for e in tree['tree'] if e['path'].startswith(prefix) and e['path'].lower().endswith('.stl')]
    assert files,commit
    out=OUT/'gen3/assets/meshes';out.mkdir(parents=True,exist_ok=True)
    manifest=[]
    for e in files:
        path=out/Path(e['path']).name
        url=f'https://raw.githubusercontent.com/{repo}/{commit}/{e["path"]}'
        raw=get(url);path.write_bytes(raw)
        assert hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()==e['sha']
        manifest.append(dict(path=str(path.relative_to(OUT)),url=url,git_blob=e['sha'],sha256=hashlib.sha256(raw).hexdigest()))
        print(path.name,len(raw),flush=True)
    (OUT/'gen3/mesh_provenance.json').write_text(json.dumps(dict(repository=repo,commit=commit,files=manifest),indent=2)+'\n')
if __name__=='__main__':main()
