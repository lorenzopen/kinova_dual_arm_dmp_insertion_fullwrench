"""New parameterized copies: retain all earlier simulator/controller code unchanged."""
from pathlib import Path
import json,hashlib,ast
HERE=Path(__file__).resolve().parent
def replace(text,old,new,count=1):
    assert text.count(old)==count,(old,text.count(old),count)
    return text.replace(old,new)
def write(path,text):
    ast.parse(text)
    if path.exists():assert path.read_text(encoding='utf-8')==text,'Do not overwrite different experiment source'
    else:path.write_text(text,encoding='utf-8')
def main():
    source=(HERE/'run_gen3.py').read_text(encoding='utf-8')
    g=replace(source,"def __init__(self,r,cfg,dest,dt,video=False,servo='implicit'):","def __init__(self,r,cfg,dest,dt,video=False,servo='implicit',damping=60.):")
    g=replace(g,'super().__init__(r,cfg,dest,dt,60.,video,','super().__init__(r,cfg,dest,dt,damping,video,')
    g=replace(g,'np.array([60.]*3+[7.]*3)','np.array([self.damping]*3+[7.]*3)')
    g=replace(g,'D=np.diag([60.]*3+[7.]*3)','D=np.diag([self.damping]*3+[7.]*3)')
    g=replace(g,"D60/7,1kHz", "D requested/7,1kHz")
    g=replace(g,"ap.add_argument('--servo',choices=['implicit','explicit'],default='implicit');a=ap.parse_args()", "ap.add_argument('--servo',choices=['implicit','explicit'],default='implicit');ap.add_argument('--damping',type=float,default=80.);a=ap.parse_args()")
    g=replace(g,'b=ArticulatedBackend(r,cfg,dest,a.dt,a.video,a.servo)','b=ArticulatedBackend(r,cfg,dest,a.dt,a.video,a.servo,a.damping)')
    g=replace(g,'cart_d=[60]*3+[7]*3','cart_d=[a.damping]*3+[7]*3')
    write(HERE/'run_gen3_damping.py',g)
    source_lab=(HERE/'run_lab_impedance.py').read_text(encoding='utf-8')
    lab=replace(source_lab,'import run_gen3 as G','import run_gen3_damping as G')
    lab=replace(lab,'def __init__(self,r,cfg,dest,dt,video=False,kalman_warm_seconds=2.):',
        'def __init__(self,r,cfg,dest,dt,video=False,kalman_warm_seconds=2.,damping=60.):')
    lab=replace(lab,"super().__init__(r,cfg,dest,dt,video,servo='explicit')","super().__init__(r,cfg,dest,dt,video,servo='explicit',damping=damping)")
    lab=replace(lab,'D=[60]*3+[7]*3','D=[damping]*3+[7]*3')
    lab=replace(lab,'cart_d=[60]*3+[7]*3','cart_d=[self.damping]*3+[7]*3',count=1) if lab.count('cart_d=[60]*3+[7]*3')==1 else lab
    # Constructor metadata and main-result metadata have different scopes.
    lab=replace(lab,'control_hz=1000,cart_k=[1500]*3+[20]*3,cart_d=[60]*3+[7]*3',
        'control_hz=1000,cart_k=[1500]*3+[20]*3,cart_d=[self.damping]*3+[7]*3')
    lab=replace(lab,'cart_k=[1500]*3+[20]*3,cart_d=[60]*3+[7]*3,ffi=',
        'cart_k=[1500]*3+[20]*3,cart_d=[a.damping]*3+[7]*3,ffi=')
    lab=replace(lab,"ap.add_argument('--kalman-warm-seconds',type=float,default=2.)", "ap.add_argument('--kalman-warm-seconds',type=float,default=2.);ap.add_argument('--damping',type=float,default=80.)")
    lab=replace(lab,'b=LaboratoryBackend(r,cfg,dest,a.dt,a.video,a.kalman_warm_seconds)',
        'b=LaboratoryBackend(r,cfg,dest,a.dt,a.video,a.kalman_warm_seconds,a.damping)')
    write(HERE/'run_lab_damping.py',lab)
    manifest=dict(purpose='D-only comparison; original experiment runners retained',
        base_gen3_sha256=hashlib.sha256(source.encode()).hexdigest(),base_lab_sha256=hashlib.sha256(source_lab.encode()).hexdigest(),
        modified_gen3_sha256=hashlib.sha256(g.encode()).hexdigest(),modified_lab_sha256=hashlib.sha256(lab.encode()).hexdigest(),
        change='Translation D parameter only, rotation D7, all K/masses/springs/FFI/limits/periods unchanged')
    (HERE/'damping_runner_provenance.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))
if __name__=='__main__':main()
