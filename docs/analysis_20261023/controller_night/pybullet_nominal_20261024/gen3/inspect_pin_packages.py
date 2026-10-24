import json,urllib.request,concurrent.futures
from pathlib import Path
HERE=Path(__file__).resolve().parent
def get(name):
    m=json.load(urllib.request.urlopen('https://api.anaconda.org/package/conda-forge/'+name,timeout=45))
    f=[x for x in m['files'] if x['attrs'].get('subdir')=='win-64' and x['version']=='4.1.0']
    if name=='pinocchio-python':f=[x for x in f if 'py312' in x['attrs'].get('build','')]
    return name,[{k:x[k] for k in ['basename','version','download_url','sha256','attrs']} for x in f[-2:]]
data=dict(concurrent.futures.ThreadPoolExecutor(2).map(get,['libpinocchio','pinocchio-python']))
(HERE/'pin_split_metadata.json').write_text(json.dumps(data,indent=2)+'\n')
print(json.dumps(data,indent=2))
