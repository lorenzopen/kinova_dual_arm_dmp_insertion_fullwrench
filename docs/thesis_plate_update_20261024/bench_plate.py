"""Bounded illustrative contact bench using an unchanged copy of the plate model.

Three declared surfaces; prescribed flange translation, fixed orientation,
zero gravity, no SOCP or wrist controller. Not an insertion success campaign.
All generated files remain inside this task directory.
"""
import csv,hashlib,json,os,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
RUNTIME=ROOT/'docs/analysis_20261023/controller_night/pybullet_nominal_20261024/deps/conda'
sys.dont_write_bytecode=True
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
sys.path[:0]=[str(ROOT/'tmp/dryrun_deps'),str(RUNTIME/'Lib/site-packages'),str(HERE/'vendor')]
DLL=os.add_dll_directory(str(RUNTIME/'Library/bin'))
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as Rot
from PIL import Image
from lib import tool as LT
LT._TMP=str(HERE/'bench/scratch')
from spider_ee import floating_plate as FP
FP.PLATE_COLLISION=False
DT=.00025
STEPS=12000
OUT=HERE/'bench'
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()

def wall(case):
    if case in ('flat','tilted'):
        angle=0. if case=='flat' else 12.
        R=Rot.from_euler('y',angle,degrees=True)
        half=[.025,.075,.065]
        c=p.createCollisionShape(p.GEOM_BOX,halfExtents=half)
        v=p.createVisualShape(p.GEOM_BOX,halfExtents=half,rgbaColor=[.65,.69,.73,1])
        body=p.createMultiBody(0,c,v,R.apply([half[0],0,0]),R.as_quat())
        slopes=np.array([0.,np.tan(np.radians(angle))])
        surface=lambda y,z:slopes[1]*z
        slope=lambda y,z:slopes
    else:
        # Convex faceted object: x >= az*z + ay*y + ridge*|y-y0| - ridge*y0.
        az,ay,ridge,y0=.10,.10,.35,.008
        surface=lambda y,z:az*z+ay*y+ridge*abs(y-y0)-ridge*y0
        slope=lambda y,z:np.array([ay+(ridge if y>y0 else -ridge),az])
        ys=[-.075,y0,.075];zs=[-.065,.065]
        front=[[surface(y,z),y,z] for y in ys for z in zs]
        back=[[.050,y,z] for y in ys for z in zs]
        vertices=front+back
        faces=[]
        def quad(a,b,c,d):faces.extend([a,b,c,a,c,d])
        for i in [0,2]:quad(i,i+2,i+3,i+1);quad(i+6,i+7,i+9,i+8)
        quad(0,1,7,6);quad(4,10,11,5)
        for i in [0,2]:quad(i,i+6,i+8,i+2);quad(i+1,i+3,i+9,i+7)
        c=p.createCollisionShape(p.GEOM_MESH,vertices=vertices,indices=faces)
        v=p.createVisualShape(p.GEOM_MESH,vertices=vertices,indices=faces,rgbaColor=[.65,.69,.73,1])
        body=p.createMultiBody(0,c,v)
    p.changeDynamics(body,-1,lateralFriction=.7,contactStiffness=8000,contactDamping=207)
    # Fixed advance relative to nominal tangency averaged at the three pads.
    tangent=[surface(y,z)-LT.PAD_R*np.sqrt(1+sum(slope(y,z)**2))-LT.PAD_X for _,y,z in LT.PADS_LOCAL]
    target=float(np.mean(tangent)+.0012)
    return body,target

def capture(path):
    view=p.computeViewMatrix([-.125,-.19,.105],[-.008,0,0],[0,0,1])
    projection=p.computeProjectionMatrixFOV(40,1.30,.005,1.)
    rgba=p.getCameraImage(780,600,view,projection,renderer=p.ER_TINY_RENDERER,
        lightDirection=[-2,-3,5],shadow=1,lightAmbientCoeff=.75,lightDiffuseCoeff=.55)[2]
    Image.fromarray(np.asarray(rgba,dtype=np.uint8).reshape(600,780,4)[:,:,:3]).save(path)

def state(tool,body,t):
    fs=[];positions=[];normals=[]
    for i in range(3):
        hits=p.getContactPoints(bodyA=tool.plate_id,bodyB=body,linkIndexA=i)
        fs.append(float(sum(max(0,c[9]) for c in hits)))
        positions.append([list(c[5]) for c in hits])
        normals.append([list(c[7]) for c in hits])
    x=tool.deflection6()
    plate_tilt=float(np.degrees(np.linalg.norm(x[4:])))
    return dict(t=t,pad_normal_N=fs,loaded_pads=int(np.sum(np.array(fs)>.05)),
                plate_tilt_deg=plate_tilt,deflection6=x.tolist(),travel_mm=(1000*tool.travel()).tolist(),
                flange_position=tool.pose()[0].tolist(),plate_position=tool.plate_pose()[0].tolist(),
                plate_quat=tool.plate_pose()[1].tolist(),contact_positions=positions,contact_normals=normals,
                flange_quat=tool.pose()[1].tolist(),anchors=tool.anchors_world().tolist(),
                pad_centres=tool.pads_world().tolist())

def run(case):
    dest=OUT/case
    assert not dest.exists(), 'Preserve previous bench: '+str(dest)
    dest.mkdir(parents=True)
    p.connect(p.DIRECT)
    p.setTimeStep(DT);p.setGravity(0,0,0)
    p.setPhysicsEngineParameter(numSolverIterations=150,contactBreakingThreshold=.0005,enableFileCaching=0)
    body,target=wall(case)
    tool=FP.FloatingPlateTool([-.060,0,0],[0,0,0,1],pads=FP.PadSprings(dt=DT),
                              centre=FP.CentralSpring(dt=DT),mu=.7,dt=DT,quiet=True)
    tool.pin_base()
    p.changeVisualShape(tool.id,-1,rgbaColor=[.12,.44,.70,1])
    p.changeVisualShape(tool.plate_id,-1,rgbaColor=[.91,.73,.30,1])
    for i in range(3):p.changeVisualShape(tool.plate_id,i,rgbaColor=[.85,.34,.16,1])
    rows=[];early=None
    for step in range(STEPS):
        t=step*DT
        s=min(1.,t/1.4);s=s*s*(3-2*s)
        command=(1-s)*(-.060)+s*target
        p.changeConstraint(tool.constraint,[command,0,0],jointChildFrameOrientation=[0,0,0,1],maxForce=1e6)
        tool.update_springs();p.stepSimulation()
        if step%40==0:
            record=state(tool,body,t+DT)
            rows.append([record['t'],*record['pad_normal_N'],record['loaded_pads'],record['plate_tilt_deg'],*record['travel_mm']])
            assert np.max(np.abs(record['deflection6'][:3]))<.03, 'Large displacement'
            if early is None and sum(record['pad_normal_N'])>1:
                early=record;capture(dest/'first_load.png')
    final=state(tool,body,STEPS*DT)
    assert early is not None, 'No contact'
    capture(dest/'settled.png')
    with (dest/'trace.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['t','N1','N2','N3','loaded_pads','plate_tilt_deg','travel1_mm','travel2_mm','travel3_mm']);w.writerows(rows)
    result=dict(case=case,flange_target_x=target,first_load=early,settled=final,
                loaded_threshold_N=.05,plate_collision=False,gravity=[0,0,0],dt=DT,seconds=STEPS*DT)
    (dest/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    p.disconnect()
    print(json.dumps(dict(case=case,early=early['loaded_pads'],final=final['loaded_pads'],
                         tilt_deg=final['plate_tilt_deg'],normal_N=final['pad_normal_N'])),flush=True)
    return result

if __name__=='__main__':
    cases=['flat','tilted','faceted']
    results=[run(case) for case in cases]
    provenance=dict(kind='Illustrative fixed-flange contact bench; no success-rate estimate',
        source=[dict(path=str(path.relative_to(HERE)),sha256=sha(path)) for path in [HERE/'vendor/spider_ee/floating_plate.py',HERE/'vendor/lib/tool.py',Path(__file__)]],
        parameters=dict(PAD_X=LT.PAD_X,PAD_R=LT.PAD_R,R_TRI=LT.R_TRI,central_linear=[20000]*3,
                        central_rotational=[40,0,0],peripheral_linear=[500]*3),cases=results)
    (OUT/'results.json').write_text(json.dumps(provenance,indent=2)+'\n')
