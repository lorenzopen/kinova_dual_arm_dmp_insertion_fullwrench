"""Known-load mechanics study; unchanged six-point SOCP and passive plate source."""
import argparse,csv,hashlib,json,os,sys,time,traceback,platform
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
DEP=ROOT/'docs/analysis_20261023/controller_night/pybullet_nominal_20261024/deps/conda'
sys.dont_write_bytecode=True
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1')
sys.path[:0]=[str(ROOT/'tmp/dryrun_deps'),str(DEP/'Lib/site-packages'),str(HERE/'vendor'),str(ROOT)]
DLL=os.add_dll_directory(str(DEP/'Library/bin'))
import numpy as np
import cvxpy as cp
import pybullet as p
from scipy.spatial.transform import Rotation as Rot
from lib import tool as LT
LT._TMP=str(HERE/'scratch')
from spider_ee import floating_plate as FP
from kinova_dual_arm_dmp_insertion.prior import HumanPrior,MODEL
from socp_reference import GraspSOCP,wrench_map
FP.PLATE_COLLISION=False
DT=.00025
GRAV=np.array([0.,0.,-9.81])
CENTER=np.array([0.,0.,.35])
KW=1500.;DW=15.;KR=40.;DR=1.15
ANGLES=np.arange(0,15.0001,2.5)
MIRROR=Rot.from_euler('z',180,degrees=True)
METHODS=['none','fixed','adaptive','wong']
COMS=dict(centered=[0,0,0],planar=[.015,0,0],spatial=[.015,.015,0])
sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()

def serial(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    if isinstance(x,Path):return str(x)
    raise TypeError(type(x).__name__)

def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(value,indent=2,default=serial,allow_nan=True)+'\n',encoding='utf-8')
    os.replace(temp,path)

def geometry(angle):
    R=Rot.from_euler('y',angle,degrees=True)
    bases=[];points=[];normals=[];rotations=[]
    for side in range(2):
        Rh=R if side==0 else R*MIRROR
        base=CENTER+R.apply([-.09 if side==0 else .09,0,0])
        pts=base+Rh.apply(LT.PADS_LOCAL+np.array([LT.PAD_R,0,0]))
        n=np.tile(-Rh.apply([1.,0,0]),(3,1))
        bases.append(base);points.extend(pts);normals.extend(n);rotations.append(Rh)
    return R,np.array(bases),np.array(points),np.array(normals),rotations

def exact_profile(prior,mass,com,mu,method,fixed=.5):
    solver=GraspSOCP(prior=prior,mu=mu,w_force=0,no_bending=False,
        margin_mode={'none':'none','fixed':'fixed','adaptive':'task_adaptive'}[method],
        margin_task_scale=.25,margin_floor_N=1.,margin_fixed_N=fixed)
    records=[]
    for angle in ANGLES:
        R,bases,points,normals,_=geometry(angle)
        result=solver.solve(posL=points[:3],posR=points[3:],baseL=bases[0],baseR=bases[1],
            nrmL=normals[:3],nrmR=normals[3:],m_hat=mass,p_com=CENTER+R.apply(com),orientation=R.as_matrix())
        assert result['ok'], (angle,result['reason'])
        records.append(dict(angle=angle,wrench=np.array([result['wrench_L'],result['wrench_R']]),
                            witness=np.array(result['f_L']+result['f_R']),rho=result['required_margin_N'],
                            points=points,normals=normals,bases=bases,
                            combined_force=float(sum(np.linalg.norm(w[:3]) for w in [result['wrench_L'],result['wrench_R']])),
                            force_residual=result['residual_force'],moment_residual=result['residual_torque']))
    return records

def wong_profile(mass,com,mu):
    """Named aggregate-patch baseline; physical spheres still carry no local moment."""
    records=[]
    reff=.0328;rs=.10
    for angle in ANGLES:
        R,bases,points,normals,_=geometry(angle)
        poles=np.array([points[:3].mean(0),points[3:].mean(0)])
        n=np.array([normals[0],normals[3]])
        w=cp.Variable((2,6));constraints=[]
        force=sum(w[i,:3] for i in range(2))
        torque=sum(np.array([[0,-(poles[i]-CENTER)[2],(poles[i]-CENTER)[1]],
            [(poles[i]-CENTER)[2],0,-(poles[i]-CENTER)[0]],[-(poles[i]-CENTER)[1],(poles[i]-CENTER)[0],0]])@w[i,:3]+w[i,3:] for i in range(2))
        constraints += [force==-mass*GRAV,torque==-np.cross(R.apply(com),mass*GRAV)]
        for i in range(2):
            projection=np.eye(3)-np.outer(n[i],n[i]);N=-n[i]@w[i,:3]
            constraints += [N>=0,projection@w[i,3:]==0,
                cp.norm(cp.hstack([projection@w[i,:3],(n[i]@w[i,3:])/reff])) <= (1-rs)*mu*N]
        cost=cp.sum_squares(w[:,:3])+cp.sum_squares(w[:,3:]/reff)
        problem=cp.Problem(cp.Minimize(cost),constraints);problem.solve(solver='CLARABEL')
        assert w.value is not None and problem.status in ('optimal','optimal_inaccurate')
        at_pole=np.array(w.value);wrists=at_pole.copy()
        wrists[:,3:]+=np.cross(poles-bases,wrists[:,:3])
        f=cp.Variable(18)
        H=np.zeros((12,18));H[:6,:9]=wrench_map(points[:3],bases[0]);H[6:,9:]=wrench_map(points[3:],bases[1])
        witness_constraints=[H@f==wrists.ravel()]
        for j in range(6):
            fi=f[3*j:3*j+3];N=-normals[j]@fi
            witness_constraints += [N>=0,cp.norm((np.eye(3)-np.outer(normals[j],normals[j]))@fi)<=mu*N]
        feasibility=cp.Problem(cp.Minimize(cp.sum_squares(f)),witness_constraints)
        feasibility.solve(solver='CLARABEL')
        realizable=f.value is not None and feasibility.status in ('optimal','optimal_inaccurate')
        records.append(dict(angle=angle,wrench=wrists,witness=(f.value.reshape(6,3) if realizable else np.full((6,3),np.nan)),
            rho=np.zeros(6),points=points,normals=normals,bases=bases,discrete_witness_feasible=realizable,
            combined_force=float(np.linalg.norm(wrists[:,:3],axis=1).sum()),patch_wrench=at_pole,
            force_residual=float(np.linalg.norm(at_pole[:,:3].sum(0)+mass*GRAV)),
            moment_residual=float(np.linalg.norm(sum(np.cross(poles[i]-CENTER,at_pole[i,:3])+at_pole[i,3:] for i in range(2))+np.cross(R.apply(com),mass*GRAV)))))
    return records

def config_table():
    jobs=[]
    for mechanism in ['welded','floating']:
        for method in METHODS:
            for case,com in COMS.items():
                for seed in range(3):jobs.append(dict(group='core',mechanism=mechanism,method=method,case=case,seed=seed,mass=1.9,model_mass=1.9,com=com,model_com=com,mu=.75,model_mu=.75))
    stress=[('light_known',1.7,1.7,COMS['spatial'],COMS['spatial'],.75),
            ('heavy_known',2.2,2.2,COMS['spatial'],COMS['spatial'],.75),
            ('mass_under10',1.9,1.71,COMS['spatial'],COMS['spatial'],.75),
            ('com_unmodelled',1.9,1.9,COMS['spatial'],COMS['centered'],.75),
            ('mu_over10',1.9,1.9,COMS['spatial'],COMS['spatial'],.825)]
    for mechanism in ['welded','floating']:
        for case,mass,model_mass,com,model_com,model_mu in stress:
            for seed in range(3):jobs.append(dict(group='stress',mechanism=mechanism,method='adaptive',case=case,seed=seed,mass=mass,model_mass=model_mass,com=com,model_com=model_com,mu=.75,model_mu=model_mu))
    for case,com in COMS.items():
        for seed in range(3):jobs.append(dict(group='central2700',mechanism='floating2700',method='adaptive',case=case,seed=seed,mass=1.9,model_mass=1.9,com=com,model_com=com,mu=.75,model_mu=.75))
    assert len(jobs)==111
    for job in jobs:job['id']='_'.join(str(job[k]) for k in ['group','mechanism','method','case','seed'])
    return jobs

def profile_key(job):
    return hashlib.sha256(json.dumps([job['method'],job['model_mass'],job['model_com'],job['model_mu']]).encode()).hexdigest()[:12]

def build_profiles(jobs):
    prior=HumanPrior(str(ROOT/'kinova_dual_arm_dmp_insertion/prior_torsional_C.npz'))
    profiles={};calibration={}
    for job in jobs:
        key=profile_key(job);path=HERE/'profiles'/(key+'.json')
        if path.exists():profiles[key]=json.loads(path.read_text())['records'];continue
        if job['method']=='wong':records=wong_profile(job['model_mass'],job['model_com'],job['model_mu'])
        elif job['method']=='fixed':
            adaptive=exact_profile(prior,job['model_mass'],job['model_com'],job['model_mu'],'adaptive')
            target=np.mean([r['combined_force'] for r in adaptive]);lo,hi=0.,8.
            for _ in range(13):
                mid=(lo+hi)/2
                trial=exact_profile(prior,job['model_mass'],job['model_com'],job['model_mu'],'fixed',mid)
                if np.mean([r['combined_force'] for r in trial])<target:lo=mid
                else:hi=mid
            rho=(lo+hi)/2;records=exact_profile(prior,job['model_mass'],job['model_com'],job['model_mu'],'fixed',rho)
            calibration[key]=dict(rho_per_pad_N=rho,target_mean_force_N=target,matched_mean_force_N=float(np.mean([r['combined_force'] for r in records])))
        else:records=exact_profile(prior,job['model_mass'],job['model_com'],job['model_mu'],job['method'])
        save(path,dict(job_inputs=job,records=records,calibration=calibration.get(key)))
        profiles[key]=records
        print('PROFILE '+key+' '+job['method']+' '+job['case'],flush=True)
    return profiles

def interpolate(records,angle):
    knots=np.array([r['angle'] for r in records])
    def value(key,shape):
        values=np.array([r[key] for r in records]);flat=values.reshape(len(knots),-1)
        return np.array([np.interp(angle,knots,flat[:,i]) for i in range(flat.shape[1])]).reshape(shape)
    return value('wrench',(2,6)),value('witness',(6,3)),value('rho',(6,))

def native_contacts(body,tools):
    """Use native A/B order; never rely on swapped friction directions."""
    F=np.zeros((6,3));P=np.zeros((6,3));N=np.zeros((6,3));weight=np.zeros(6)
    velocities=np.zeros(6);entry_psi=[];max_spread=np.zeros(6);all_normals=[[] for _ in range(6)]
    op,oq=p.getBasePositionAndOrientation(body);ov,ow=p.getBaseVelocity(body)
    tool_map={tool.plate_id:(i,tool) for i,tool in enumerate(tools)}
    for c in p.getContactPoints():
        if c[1]==body and c[2] in tool_map:sgn=1.;other=c[2];link=c[4];point=np.array(c[5]);n=-np.array(c[7])
        elif c[2]==body and c[1] in tool_map:sgn=-1.;other=c[1];link=c[3];point=np.array(c[6]);n=np.array(c[7])
        else:continue
        if link not in (0,1,2):continue
        hand,tool=tool_map[other];j=3*hand+link
        f=sgn*(c[9]*np.array(c[7])+c[10]*np.array(c[11])+c[12]*np.array(c[13]))
        F[j]+=f;w=max(c[9],1e-9);P[j]+=w*point;N[j]+=w*n;weight[j]+=w;all_normals[j].append(n)
        tp,_=tool.plate_pose();tv,tw=p.getBaseVelocity(tool.plate_id)
        vrel=np.array(ov)+np.cross(ow,point-op)-np.array(tv)-np.cross(tw,point-tp)
        velocities[j]=max(velocities[j],np.linalg.norm(vrel-n*(n@vrel)))
        tang=np.linalg.norm(f+n*(n@(-f)))
        entry_psi.append([j,c[9],np.linalg.norm(f-(f@n)*n),*f,*n,*point])
    for j in range(6):
        if weight[j]>0:
            P[j]/=weight[j];N[j]/=max(np.linalg.norm(N[j]),1e-12)
            max_spread[j]=max(np.degrees(np.arccos(np.clip(n@N[j],-1,1))) for n in all_normals[j])
    return F,P,N,weight,velocities,max_spread,entry_psi

def screenshot(path):
    from PIL import Image
    view=p.computeViewMatrix([.50,-.68,.64],CENTER,[0,0,1])
    proj=p.computeProjectionMatrixFOV(38,1.3,.01,3)
    rgba=p.getCameraImage(780,600,view,proj,renderer=p.ER_TINY_RENDERER)[2]
    Image.fromarray(np.asarray(rgba,dtype=np.uint8).reshape(600,780,4)[:,:,:3]).save(path)

def run(job,records,dest):
    dest.mkdir(parents=True,exist_ok=False)
    save(dest/'STARTED.json',dict(job=job,pid=os.getpid(),source_sha256=sha(__file__),time=time.time()))
    p.connect(p.DIRECT);p.setTimeStep(DT);p.setGravity(*GRAV)
    p.setPhysicsEngineParameter(numSolverIterations=120,contactBreakingThreshold=.0005,enableFileCaching=0,enableConeFriction=1)
    tools=[];welds=[]
    R0,bases,_,_,rotations=geometry(0)
    for h in range(2):
        k=2700. if job['mechanism']=='floating2700' else 20000.
        pos=bases[h]+np.array([-.01 if h==0 else .01,0,0])
        tool=FP.FloatingPlateTool(pos,rotations[h].as_quat(),pads=FP.PadSprings(dt=.0005),centre=FP.CentralSpring(k_push=k,dt=.0005),mu=job['mu'],dt=DT,quiet=True)
        tools.append(tool)
        if job['mechanism']=='welded':
            weld=p.createConstraint(tool.id,-1,tool.plate_id,-1,p.JOINT_FIXED,[0,0,0],[LT.PAD_X,0,0],[0,0,0])
            p.changeConstraint(weld,maxForce=1e6);welds.append(weld)
        for link in tool.pads:assert abs(p.getDynamicsInfo(tool.plate_id,link)[1]-job['mu'])<1e-10
    com=np.array(job['com']);initialR=Rot.from_euler('y',[0,1,-1][job['seed']],degrees=True)
    col=p.createCollisionShape(p.GEOM_BOX,halfExtents=[.06]*3,collisionFramePosition=(-com).tolist())
    vis=p.createVisualShape(p.GEOM_BOX,halfExtents=[.06]*3,visualFramePosition=(-com).tolist(),rgbaColor=[.62,.66,.70,1])
    body=p.createMultiBody(job['mass'],col,vis,CENTER+initialR.apply(com),initialR.as_quat())
    p.changeDynamics(body,-1,lateralFriction=1.,spinningFriction=0,rollingFriction=0,
        linearDamping=0,angularDamping=0,localInertiaDiagonal=[job['mass']*.12**2/6]*3,
        contactStiffness=8000,contactDamping=207)
    assert p.getDynamicsInfo(body,-1)[1]==1.
    fixture=p.createConstraint(body,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],CENTER+initialR.apply(com),childFrameOrientation=initialR.as_quat())
    p.changeConstraint(fixture,maxForce=1e6)
    names=['t','angle','phase','force','points','normals','normal_N','slip_speed','normal_spread','desired_wrist','measured_wrist','witness','rho','nominal_normals','object_pos','object_quat','tool_pos','tool_quat','plate_deflection','object_vel','entries']
    data={key:[] for key in names};error=None;last_capture=False;entry_rows=[]
    started=time.perf_counter()
    for step in range(round(4.5/DT)):
        t=step*DT
        if fixture is not None and t>=1.0:p.removeConstraint(fixture);fixture=None
        progress=np.clip((t-1.5)/2.5,0,1);s=progress**3*(10-15*progress+6*progress*progress);angle=15*s
        R,bases,pts,nom_n,orientations=geometry(angle)
        wr,witness,rho=interpolate(records,angle)
        entry=np.clip(t/.8,0,1);entry=entry*entry*(3-2*entry)
        for h,tool in enumerate(tools):
            open_shift=R.apply([(-.01 if h==0 else .01)*(1-entry),0,0])
            target=bases[h]+open_shift+entry*wr[h,:3]/KW
            qdes=Rot.from_rotvec(entry*wr[h,3:]/KR)*orientations[h]
            pos,quat=tool.pose();vel,omega=p.getBaseVelocity(tool.id);Rh=Rot.from_quat(quat)
            force=KW*(target-pos)-DW*np.array(vel)-tool.mass*GRAV
            torque=KR*(qdes*Rh.inv()).as_rotvec()-DR*np.array(omega)
            mobile=FP.PLATE_MASS+3*FP.PAD_MASS
            torque-=np.cross(Rh.apply([LT.PAD_X,0,0]),mobile*GRAV)
            p.applyExternalForce(tool.id,-1,force,pos,p.WORLD_FRAME);p.applyExternalTorque(tool.id,-1,torque,p.WORLD_FRAME)
            tool.update_springs()
        p.stepSimulation()
        if step%round(.01/DT):continue
        F,points,normals,N,slip,spread,entries=native_contacts(body,tools)
        op,oq=p.getBasePositionAndOrientation(body);ov,ow=p.getBaseVelocity(body)
        measured=np.zeros((2,6));toolposes=[];toolquats=[];deflections=[]
        for h,tool in enumerate(tools):
            tp,tq=tool.pose();toolposes.append(tp);toolquats.append(tq);deflections.append(tool.deflection6())
            measured[h,:3]=F[3*h:3*h+3].sum(0)
            measured[h,3:]=np.cross(points[3*h:3*h+3]-tp,F[3*h:3*h+3]).sum(0)
        values=[t+DT,angle,'fixture' if fixture is not None else ('hold' if t<1.5 else 'wield'),F,points,normals,N,slip,spread,wr,measured,witness,rho,nom_n,op,oq,toolposes,toolquats,deflections,np.r_[ov,ow],len(entries)]
        for name,value in zip(names,values):data[name].append(value)
        for e in entries:entry_rows.append([t+DT,*e])
        if t>1.2 and not last_capture:screenshot(dest/'after_release.png');last_capture=True
        if t>1.0 and (op[2]<.20 or np.linalg.norm(np.array(op)-CENTER)>.12):error='object_lost';break
        if any(np.linalg.norm(tool.deflection6()[:3])>.04 for tool in tools):error='tool_deflection_limit';break
    screenshot(dest/'final.png')
    np.savez_compressed(dest/'trace.npz',**{k:np.asarray(v) for k,v in data.items()})
    np.savez_compressed(dest/'manifold.npz',entries=np.asarray(entry_rows),columns=np.array(['t','pad','normal_N','tangential_N','fx','fy','fz','nx','ny','nz','px','py','pz']))
    p.disconnect()
    summary=dict(job=job,error=error,seconds=data['t'][-1],wall_seconds=time.perf_counter()-started,
                 stage='airborne' if data['t'][-1]>1 else 'fixture',samples=len(data['t']),source_sha256=sha(__file__))
    save(dest/'COMPLETED.json',summary)
    return summary

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--pilot',action='store_true');ap.add_argument('--limit',type=int);args=ap.parse_args()
    jobs=config_table()
    if args.pilot:jobs=[jobs[18],jobs[54],jobs[26],jobs[71]]
    if args.limit:jobs=jobs[:args.limit]
    category='pilot' if args.pilot else 'runs'
    lock=HERE/(category+'.lock')
    fd=os.open(lock,os.O_CREAT|os.O_EXCL|os.O_WRONLY);os.write(fd,str(os.getpid()).encode());os.close(fd)
    try:
        profiles=build_profiles(jobs)
        source=dict(python=platform.python_version(),numpy=np.__version__,cvxpy=cp.__version__,solvers=cp.installed_solvers(),
            source_sha256=sha(__file__),protocol_sha256=sha(HERE/'PROTOCOL.md'),prior_sha256=sha(ROOT/'kinova_dual_arm_dmp_insertion/prior_torsional_C.npz'),
            solver_sha256=sha(HERE/'socp_reference.py'),plate_sha256=sha(HERE/'vendor/spider_ee/floating_plate.py'),jobs=jobs)
        save(HERE/(category+'_manifest.json'),source)
        for i,job in enumerate(jobs):
            dest=HERE/category/job['id']
            if (dest/'COMPLETED.json').exists():continue
            if dest.exists():raise RuntimeError('Incomplete directory preserved: '+str(dest))
            save(HERE/'STATE.json',dict(category=category,pid=os.getpid(),current=job['id'],index=i,total=len(jobs)))
            try:result=run(job,profiles[profile_key(job)],dest)
            except Exception as exc:
                try:p.disconnect()
                except Exception:pass
                result=dict(job=job,error='exception: '+repr(exc),traceback=traceback.format_exc())
                save(dest/'FAILED.json',result)
            print(json.dumps(dict(index=i+1,total=len(jobs),id=job['id'],error=result['error'],seconds=result.get('seconds'))),flush=True)
        save(HERE/'STATE.json',dict(category=category,pid=None,complete=True,total=len(jobs)))
    finally:lock.unlink(missing_ok=True)

if __name__=='__main__':main()
