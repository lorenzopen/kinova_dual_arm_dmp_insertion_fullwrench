"""Autonomous Bullet insertion with unchanged production FFI and SOCP.

All outputs, generated meshes and tool URDFs stay in this experiment.
No ROS imports, observed-force replay or robot connections.
"""
import sys, os, json, csv, hashlib, argparse, copy, contextlib, io, time, traceback
from pathlib import Path
from collections import deque
sys.dont_write_bytecode=True
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[3]
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1')
os.environ['MPLCONFIGDIR']=str(OUT/'mplconfig')
sys.path[:0]=[str(ROOT/'tmp/dryrun_deps'),str(OUT/'deps/conda/Lib/site-packages'),str(ROOT),str(OUT/'vendor')]
DLL=os.add_dll_directory(str(OUT/'deps/conda/Library/bin'))
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as Rot
from kinova_dual_arm_dmp_insertion import main as M, geometry as GE, insert_open as IO
from kinova_dual_arm_dmp_insertion.insert_ffi import FFIProfileInsertion, FFISettings
from kinova_dual_arm_dmp_insertion.route import Route
from kinova_dual_arm_dmp_insertion.dryrun import DryRun
from kinova_dual_arm_dmp_insertion.controller import Abort
from lib import tool as LT
# Geometry is configured before importing the floating-plate implementation.
LT.PAD_X=.085;LT.PAD_R=.015;LT.R_TRI=.0404
LT.PADS_LOCAL=np.array([[LT.PAD_X,0,LT.R_TRI],[LT.PAD_X,LT.R_TRI*np.sqrt(3)/2,-LT.R_TRI/2],
                       [LT.PAD_X,-LT.R_TRI*np.sqrt(3)/2,-LT.R_TRI/2]])
LT._TMP=str(OUT/'scratch')
from spider_ee import floating_plate as FP
from lib import wrist as WR
import shelf as SH
from pieces import PIECES
SH.MESHES=str(OUT/'scratch/meshes')

META={
 'Z':'docs/runs/20261023_1743_F75_Zt1_s9012_ffi_riuscito/F75_Zt1_s9012_c100_five_cut800_ffi_20261023_174143_meta.json',
 'L':'docs/runs/20261023_1827_F75_Lt1_s9014_ffi_riuscito/F75_Lt1_s9014_c100_five_cut800_ffi_20261023_182743_meta.json',
 'S':'docs/runs/20261023_1904_S_s7073m3_ffi_riuscito/S_s7073m3_c100_narrow75_ffi_20261023_190407_meta.json',
 'T':'docs/runs/20261023_1921_F75_Tt1_s9007_ffi_riuscito/F75_Tt1_s9007_c100_five_cut800_ffi_20261023_192128_meta.json'}

def serial(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    if isinstance(x,Rot):return x.as_quat().tolist()
    if isinstance(x,Path):return str(x)
    raise TypeError(type(x).__name__)

def save(path,obj):path.write_text(json.dumps(obj,indent=2,default=serial)+'\n',encoding='utf-8')
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

class AuditPlanner(IO.NominalPlanner):
    def __init__(self,*args,**kw):super().__init__(*args,**kw);self.allocations=[]
    def _socp_apply(self,inp,res,ms):
        super()._socp_apply(inp,res,ms)
        self.allocations.append(dict(phase=self.phase,input=inp,result=res,solve_ms=ms))

def plan(r,cfg,slow,dest):
    file=dest/'profile.csv';alloc=dest/'socp_allocations.json'
    if file.exists():
        with file.open() as f:rows=list(csv.DictReader(f))
        trace=[(d['phase'],*[float(d[k]) for k in ['t_in_phase','target_N',*IO.WRENCH_COLS]]) for d in rows]
        return IO.Profile(trace,str(file)),json.loads(alloc.read_text())['summary']
    b=DryRun(r,period=1/cfg['rate_hz'],k_face=680.)
    planner=M.slowed(AuditPlanner(r,b,k_eff=680.,confirm=lambda _:None,**M.socp_kwargs(cfg)),slow)
    with contextlib.redirect_stdout(io.StringIO()):planner.run()
    assert planner.stats['socp_ok']==planner.stats['socp_ticks']
    profile=IO.Profile(planner.trace);profile.require_phases(('to_mouth','route','slow_seat','seat_by_feel'))
    with file.open('x',newline='') as f:
        writer=csv.writer(f);writer.writerow(['phase','t_in_phase','target_N',*IO.WRENCH_COLS]);writer.writerows(planner.trace)
    summary=dict(feasible=planner.stats['socp_ok'],solves=planner.stats['socp_ticks'],samples=len(planner.trace),
                 profile_sha256=sha(file),method='unchanged NominalPlanner / GraspSOCP / TP-GMM, recomputed before Bullet')
    save(alloc,dict(summary=summary,allocations=planner.allocations))
    return profile,summary

def obj(name,cell,mass):
    xy=np.array(PIECES[name][0],float);xy=(xy-xy.mean(axis=0))*cell
    return dict(name=name,mass=mass,parts=[dict(origin=[x,y,0],half=[cell/2]*3) for x,y in xy])

def spring_wrench(tool):
    """Wrench flange exerts on attached plate: exact reaction of model springs.

    Opposite of applied spring reaction on flange, moment about flange origin.
    Gravity removed separately; inertial wrench remains, as with a wrist sensor.
    """
    pf,qf=tool.pose();pp,qp=tool.plate_pose();R=Rot.from_quat(qf)
    vf,wf=[np.array(v) for v in p.getBaseVelocity(tool.id)]
    vp,wp=[np.array(v) for v in p.getBaseVelocity(tool.plate_id)]
    x=tool.deflection6()
    F=R.apply(-tool.centre.k_lin*x[:3]-tool.centre.c_lin*R.inv().apply(vp-vf))
    T=R.apply(-tool.centre.k_rot*x[3:]-tool.centre.c_rot*R.inv().apply(wp-wf))+np.cross(pp-pf,F)
    d=tool.deflection_local();anchors=tool.anchors_world();pads=tool.pads_world()
    for i in range(3):
        vr=R.inv().apply(vp+np.cross(wp,pads[i]-pp)-vf-np.cross(wf,anchors[i]-pf))
        fi=R.apply(-tool.pads_spring.K_local*d[i]-tool.pads_spring.C_local*vr)
        F+=fi;T+=np.cross(anchors[i]-pf,fi)
    raw=np.r_[F,T]
    # Exact known attached mass and instantaneous model CoM for gravity-only
    # compensation. Controller never gets object pose or contact positions.
    mass=FP.PLATE_MASS+3*FP.PAD_MASS
    com=(FP.PLATE_MASS*pp+FP.PAD_MASS*pads.sum(axis=0))/mass
    weight=np.array([0.,0.,-9.81*mass])
    return raw,raw+np.r_[weight,np.cross(com-pf,weight)]

def contact_wrench(body,tool,pole):
    """Unfiltered contact query preserves Bullet's pair order, including plate.
    Return hand-on-active-piece wrench and three pad normal magnitudes.
    """
    F=np.zeros(3);T=np.zeros(3);pads=np.zeros(3);plate=0.
    for c in p.getContactPoints():
        if c[1]==body and c[2]==tool.plate_id:
            f=c[9]*np.array(c[7])+c[10]*np.array(c[11])+c[12]*np.array(c[13]);at=np.array(c[5]);link=c[4]
        elif c[2]==body and c[1]==tool.plate_id:
            f=-(c[9]*np.array(c[7])+c[10]*np.array(c[11])+c[12]*np.array(c[13]));at=np.array(c[6]);link=c[3]
        else:continue
        F+=f;T+=np.cross(at-pole,f)
        if link in (0,1,2):pads[link]+=c[9]
        else:plate+=c[9]
    return np.r_[F,T],pads,plate

def apply_springs_and_sense(tool):
    """Read the exact spring reactions applied by the untouched tool method.

    Same spring computations as spring_wrench; no duplicate spring evaluation.
    Wrapper lifetime is one call in this single-threaded DIRECT backend.
    """
    pf,_=tool.pose();pp,_=tool.plate_pose();raw=np.zeros(6)
    force_fn,torque_fn=p.applyExternalForce,p.applyExternalTorque
    def force(body,link,f,at,frame):
        if body==tool.id:
            f=np.array(f);raw[:3]-=f;raw[3:]-=np.cross(np.array(at)-pf,f)
        return force_fn(body,link,f,at,frame)
    def torque(body,link,t,frame):
        if body==tool.id:raw[3:]-=np.array(t)
        return torque_fn(body,link,t,frame)
    p.applyExternalForce,p.applyExternalTorque=force,torque
    try:tool.update_springs()
    finally:p.applyExternalForce,p.applyExternalTorque=force_fn,torque_fn
    weight=np.array([0.,0.,-9.81*(FP.PLATE_MASS+3*FP.PAD_MASS)])
    return raw,raw+np.r_[weight,np.cross(pp-pf,weight)]

class BulletBackend:
    def __init__(self,r,cfg,dest,dt,damping,video=False,pad_axial=21500.,spring_reference_dt=.00025):
        self.r,self.cfg,self.dest,self.dt=r,cfg,dest,dt
        self.period=1/cfg['rate_hz'];self.substeps=round(self.period/dt)
        assert abs(self.substeps*dt-self.period)<1e-10
        self.t=0.;self.count=0;self.damping=damping;self.fixture=True
        self.tools={};self.buffers={s:deque(maxlen=2) for s in 'LR'}
        self.acc={s:np.zeros(6) for s in 'LR'};self.sensor_steps=round(.005/dt)
        self.rows=[];self.physics=[];self.physics_raw=[];self.sensor_blocks=[];self.frames=[];self.video=video
        p.connect(p.DIRECT);p.setGravity(0,0,-9.81);p.setTimeStep(dt)
        p.setPhysicsEngineParameter(numSolverIterations=200,contactBreakingThreshold=.001,
                                   deterministicOverlappingPairs=1,enableFileCaching=0)
        cell=float(r.raw['piece_model']['cell'])*r.eff_scale
        SH.SEAM=r.shelf['seam']*r.eff_scale;SH.WALL_GAP=r.shelf['wall_gap']*r.eff_scale
        SH.WALL=r.shelf['wall'];SH.CHAMFER=.008*(cell/.12);SH.ANCHOR=False
        SH.CONTACT=(8000.,207.)
        self.shelf=SH.build(r.shelf['rows'],r.shelf['cols'],r.shelf['pitch']*r.eff_scale,cell+SH.SEAM)
        self.neighbours=[]
        for n in r.neighbours:
            self.neighbours.append(SH.spawn(obj(n['piece'],cell,r.mass),n['position'],n['quat']))
        active=obj(r.piece,cell,r.mass)
        # Verify exact active piece geometry against exported route.
        assert np.allclose(np.array([q['origin'] for q in active['parts']]),r.cells,atol=1e-7)
        self.body=SH.spawn(active,r.grasp_c,r.grasp_R.as_quat(),mu=SH.MU)
        self.constraint=p.createConstraint(self.body,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],
                                           r.grasp_c,childFrameOrientation=r.grasp_R.as_quat())
        p.changeConstraint(self.constraint,maxForce=1e5)
        a=r.axis/np.linalg.norm(r.axis);reach=.5*r.opening+GE.PAD_X+GE.PAD_R+r.c['open_clear']
        for side,sgn in [('L',1),('R',-1)]:
            pos=r.centre-sgn*reach*a;q=GE.quat_from_x(sgn*a)
            # Per-body friction: piece= sqrt(.5), pad=mu_eff/piece.
            self.tools[side]=FP.FloatingPlateTool(pos,q.as_quat(),mu=r.c['mu_eff']/SH.MU,dt=dt,
                pads=FP.PadSprings(dt=spring_reference_dt),
                centre=FP.CentralSpring(k_push=pad_axial-1500.,dt=spring_reference_dt))
        self.cmd=tuple(v for s in 'LR' for v in (self.tools[s].pose()[0],Rot.from_quat(self.tools[s].pose()[1])))
        self.k_tilt=8.;self.raw={};self.ins=None
        self.last_raw={s:np.zeros(6) for s in 'LR'};self.last_gc=copy.deepcopy(self.last_raw)
        self.frame_step=max(1,round(.1/dt))
    def now(self):return self.t
    def command(self,tL,qL,tR,qR,k_tilt):self.cmd=(np.array(tL),qL,np.array(tR),qR);self.k_tilt=k_tilt
    def tare(self):
        # Known gravity removed continuously, no bias/noise/pose-error injected.
        pass
    def confirm(self,msg):
        if 'let go' in msg:
            p.removeConstraint(self.constraint);self.fixture=False
            print('  Fixture released at',self.t,flush=True)
    def wait(self):
        for _ in range(self.substeps):
            for i,s in enumerate('LR'):
                tool=self.tools[s];target,q=self.cmd[2*i:2*i+2]
                # Declared ideal Cartesian impedance; actual inner Kinova law
                # remains unavailable. Isotropic K1500/20 and D60/7 from run.
                WR.vc(tool,target,q,k_tilt=20.,kp=1500.,kd=self.damping,kpr=20.,kdr=7.)
                raw,gc=apply_springs_and_sense(tool);self.last_raw[s]=raw;self.last_gc[s]=gc
                self.acc[s]+=gc
            p.stepSimulation();self.count+=1;self.t=self.count*self.dt
            if self.count%round(.001/self.dt)==0:
                self.physics_raw.append(np.r_[self.t-self.dt,*[self.last_raw[s] for s in 'LR'],*[self.last_gc[s] for s in 'LR']])
            if self.count%self.sensor_steps==0:
                for s in 'LR':
                    self.buffers[s].append(self.acc[s]/self.sensor_steps);self.acc[s][:]=0
                self.physics.append(np.r_[self.t,*[self.last_raw[s] for s in 'LR'],*[self.last_gc[s] for s in 'LR']])
                self.sensor_blocks.append(np.r_[self.t,*[self.buffers[s][-1] for s in 'LR']])
            if self.video and self.count%self.frame_step==0:self.capture()
    def read(self):
        m={}
        for s in 'LR':
            pos,q=self.tools[s].pose();w=np.mean(self.buffers[s],axis=0) if self.buffers[s] else np.zeros(6)
            m.update({'p'+s:pos,'q'+s:q,'F'+s:w[:3],'T'+s:w[3:]})
            R=Rot.from_quat(q);self.raw['ft'+s]=np.r_[R.inv().apply(self.last_raw[s][:3]),R.inv().apply(self.last_raw[s][3:])]
        return m
    def capture(self):
        from PIL import Image
        view=p.computeViewMatrix(cameraEyePosition=[1.10,-1.65,1.0],cameraTargetPosition=[.30,0,.35],cameraUpVector=[0,0,1])
        projection=p.computeProjectionMatrixFOV(48,960/720,.02,4)
        rgba=p.getCameraImage(960,720,view,projection,renderer=p.ER_TINY_RENDERER)[2]
        arr=np.array(rgba,dtype=np.uint8).reshape(720,960,4)[:,:,:3]
        self.frames.append(Image.fromarray(arr).resize((640,480)))
    def record(self,ins):
        row=dict(t=self.t,phase=ins.phase,target=ins.target if ins.target is not None else 6.,ds=ins.ds,
                 fixture=int(self.fixture),object_pose=np.r_[*p.getBasePositionAndOrientation(self.body)],
                 object_velocity=np.r_[*p.getBaseVelocity(self.body)],
                 ffi=dict(getattr(getattr(ins,'normal_ffi',None),'diag',{})),
                 desired={s:np.r_[*ins.w_des[s]] if s in ins.w_des else np.full(6,np.nan) for s in 'LR'})
        for i,s in enumerate('LR'):
            tool=self.tools[s];pf,q=tool.pose();cw,pads,plate=contact_wrench(self.body,tool,pf)
            row[s]=dict(position=pf,quat=q,command=self.cmd[2*i],command_quat=self.cmd[2*i+1].as_quat(),
                        measured=np.r_[ins.meas['F'+s],ins.meas['T'+s]],raw=self.last_raw[s].copy(),
                        contact=cw,pad_normals=pads,plate_normal=plate,deflection=tool.deflection6())
        self.rows.append(row)
        if len(self.rows)%500==0:print('  progress',ins.phase,round(self.t,2),'s',flush=True)
    def finish(self):
        self.capture()
        if self.video and self.frames:
            self.frames[0].save(self.dest/'insertion.gif',save_all=True,append_images=self.frames[1:],duration=100,loop=0)
        self.frames[-1].save(self.dest/'final_scene.png')
        rows=self.rows
        arrays=dict(t=np.array([x['t'] for x in rows]),phase=np.array([x['phase'] for x in rows]),
                    target=np.array([x['target'] for x in rows]),ds=np.array([x['ds'] for x in rows]),
                    object_pose=np.array([x['object_pose'] for x in rows]),
                    object_velocity=np.array([x['object_velocity'] for x in rows]),
                    fixture=np.array([x['fixture'] for x in rows]),sensor_200Hz=np.array(self.physics),
                    raw_wrist_1kHz=np.array(self.physics_raw),sensor_block_average_200Hz=np.array(self.sensor_blocks))
        for s in 'LR':
            for key in rows[0][s]:arrays[s+'_'+key]=np.array([x[s][key] for x in rows])
            arrays[s+'_desired']=np.array([x['desired'][s] for x in rows])
        keys=set().union(*(x['ffi'].keys() for x in rows))
        for key in keys:arrays[key]=np.array([x['ffi'].get(key,np.nan) for x in rows])
        np.savez_compressed(self.dest/'trace.npz',**arrays)
        with (self.dest/'trace.csv').open('x',newline='') as f:
            writer=csv.writer(f);writer.writerow(['t','phase','target_N','ds_m',*[s+'_'+k for s in 'LR' for k in ['Fx','Fy','Fz','Tx','Ty','Tz']]])
            writer.writerows([x['t'],x['phase'],x['target'],x['ds'],*x['L']['measured'],*x['R']['measured']] for x in rows)
        actual=np.array(p.getBasePositionAndOrientation(self.body)[0]);R=Rot.from_quat(p.getBasePositionAndOrientation(self.body)[1])
        cell_error=1000*np.max(np.linalg.norm(actual+R.apply(self.r.cells)-(self.r.target+self.r.R_target.apply(self.r.cells)),axis=1))
        result=dict(final_worst_cell_mm=float(cell_error),final_centroid_mm=float(1000*np.linalg.norm(actual-self.r.target)),
                    final_orientation_deg=float(np.degrees((self.r.R_target.inv()*R).magnitude())),
                    physics_steps=self.count,seconds=self.t)
        p.disconnect();return result

class Recorder:
    def __init__(self,backend):self.b=backend
    def row(self,ins,*_):self.b.record(ins)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--piece',choices=list(META),default='Z')
    ap.add_argument('--dt',type=float,default=.001);ap.add_argument('--damping',type=float,default=60.)
    ap.add_argument('--tag',required=True);ap.add_argument('--video',action='store_true')
    ap.add_argument('--pad-axial',type=float,default=21500.,help='total axial K of parallel pad+central springs')
    ap.add_argument('--spring-reference-dt',type=float,default=.00025,
                    help='fixed reference for original explicit-damping cap; held fixed under physics dt refinement')
    a=ap.parse_args();assert a.pad_axial>=1500.
    dest=OUT/'runs'/a.tag;dest.mkdir(parents=True,exist_ok=False)
    save(dest/'STARTED.json',dict(pid=os.getpid(),argv=sys.argv))
    (dest/'runner_source.py').write_bytes(Path(__file__).read_bytes())
    meta=json.loads((ROOT/META[a.piece]).read_text());cfg=meta['config']
    argv=meta['argv'];world=argv[2];arm=argv[argv.index('--arm')+1];slow=float(argv[argv.index('--slow')+1])
    GE.set_tool(**{k:cfg['tool'][k] for k in ('pad_x','r_tri','pad_r')})
    r=Route(world,arm,scale=cfg['scale'],mass=cfg['piece_mass'])
    profile,summary=plan(r,cfg,slow,dest)
    settings=FFISettings()
    b=BulletBackend(r,cfg,dest,a.dt,a.damping,a.video,a.pad_axial,a.spring_reference_dt)
    ins=M.slowed(FFIProfileInsertion(r,b,profile=profile,k_eff=680.,ffi_settings=settings,log=Recorder(b),
        confirm=b.confirm,f_abort=cfg['f_abort'],track_abort=cfg['track_abort'],
        jam_scale=cfg['jam_scale'],jam_min=cfg['jam_min'],squeeze_gain=.1,**M.socp_kwargs(cfg)),slow)
    result={};error=None;t0=time.perf_counter()
    try:result=ins.run()
    except Exception as exc:error=repr(exc);traceback.print_exc()
    if not b.rows:b.record(ins)
    truth=b.finish()
    save(dest/'result.json',dict(controller_result=result,error=error,truth=truth,planning=summary,
        wall_seconds=time.perf_counter()-t0,dt=a.dt,cart_k=[1500]*3+[20]*3,cart_d=[a.damping]*3+[7]*3,
        ffi=vars(settings),feedforward_k_eff=680.,piece_mass=r.mass,tool_mass=.19,
        mechanical_pad_K=np.diag(FP.stiffness_6x6(b.tools['L'].pads_spring,b.tools['L'].centre)),
        central_C_linear=b.tools['L'].centre.c_lin,central_C_rotational=b.tools['L'].centre.c_rot,
        pad_C_linear=b.tools['L'].pads_spring.C_local,spring_damping_reference_dt=a.spring_reference_dt,
        pad_axial_N_per_m=a.pad_axial,
        assumptions=['Floating Cartesian wrist proxy, not full Kinova torque servo or seven-link dynamics',
        'Original floating-plate model: attached plate/pads .14kg, flange .05kg, inflated .05kgm2 flange inertia',
        f'K/D from archived nominal Cartesian gains; total tool axial spring {a.pad_axial:g}N/m; other tool axes original uncalibrated nominal',
        'Neighbour and held piece contact 8000N/m,207Ns/m; neighbours .05kg; uncalibrated source nominal assumptions',
        'No injected bias, communication delay, jitter, device noise or hardware saturation',
        'Gravity-compensated spring-reaction wrist sensor, 5ms block then last-two block average',
        'Truth object pose and contact forces for evaluation only, never fed to controller']))
    save(dest/'COMPLETED.json',dict(pid=None,error=error,trace_sha256=sha(dest/'trace.npz')))
    print(json.dumps(dict(error=error,truth=truth,controller=result),default=serial),flush=True)

if __name__=='__main__':main()
