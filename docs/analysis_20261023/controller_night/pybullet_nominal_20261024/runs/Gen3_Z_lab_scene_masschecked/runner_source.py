"""Two articulated Gen3 in lab frames, torque Cartesian impedance, actual fixed-joint FT.
No production or original data edits. Not a replica of the unavailable Kinova servo.
"""
import sys,os,json,csv,copy,time,argparse,traceback
from pathlib import Path
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parent))
import run_bullet as B
from run_bullet import p,np,Rot,FP,GE,ROOT,OUT,save,serial,sha,M,Route,FFIProfileInsertion,FFISettings,Recorder
from kinova_dual_arm_dmp_insertion.reach_ik import Chain
from build_lab_urdf import build

class RobotPad(FP.FloatingPlateTool):
    def __init__(self,robot,ee,sensor,tool_in_ee,dt):
        self.id,self.ee,self.sensor=robot,ee,sensor
        self.tool_in_ee=tool_in_ee;self.dt=dt
        self.pads_spring=FP.PadSprings(dt=.00025)
        self.centre=FP.CentralSpring(k_push=2700.,dt=.00025)
        pf,q=self.pose();R=Rot.from_quat(q)
        path=OUT/'scratch'/f'gen3_plate_{os.getpid()}.urdf'
        FP.write_plate_urdf(str(path),FP.PAD_MASS,FP.PLATE_MASS)
        self.plate_id=p.loadURDF(str(path),(pf+R.apply([B.LT.PAD_X,0,0])).tolist(),q.tolist(),flags=p.URDF_USE_INERTIA_FROM_FILE)
        p.changeDynamics(self.plate_id,-1,linearDamping=0,angularDamping=0)
        self.pads=[0,1,2];self.pad_ids=[self.plate_id]*3;self.contact_body=self.plate_id
        self.mass=FP.PLATE_MASS+3*FP.PAD_MASS
        for i in self.pads:p.changeDynamics(self.plate_id,i,lateralFriction=1/B.SH.MU,spinningFriction=0,rollingFriction=0)
        for own in [ee,sensor,*[j for j in range(p.getNumJoints(robot)) if p.getJointInfo(robot,j)[12].decode()=='ati_tool_payload']]:
            for link in [-1,0,1,2]:p.setCollisionFilterPair(robot,self.plate_id,own,link,0)
    def pose(self):
        st=p.getLinkState(self.id,self.ee,computeForwardKinematics=1)
        pe,R=np.array(st[4]),Rot.from_quat(st[5]);t,q=self.tool_in_ee
        return pe+R.apply(t),(R*q).as_quat()
    def velocity(self):
        st=p.getLinkState(self.id,self.ee,computeLinkVelocity=1,computeForwardKinematics=1)
        omega=np.array(st[7]);pt,_=self.pose()
        return np.array(st[6])+np.cross(omega,pt-np.array(st[0])),omega
    def update_springs(self):
        velocity_fn,force_fn,torque_fn=p.getBaseVelocity,p.applyExternalForce,p.applyExternalTorque
        def velocity(body):return self.velocity() if body==self.id else velocity_fn(body)
        def force(body,link,f,at,frame):return force_fn(body,self.sensor if body==self.id and link==-1 else link,f,at,frame)
        def torque(body,link,t,frame):return torque_fn(body,self.sensor if body==self.id and link==-1 else link,t,frame)
        p.getBaseVelocity,p.applyExternalForce,p.applyExternalTorque=velocity,force,torque
        try:super().update_springs()
        finally:p.getBaseVelocity,p.applyExternalForce,p.applyExternalTorque=velocity_fn,force_fn,torque_fn

class ArticulatedBackend(B.BulletBackend):
    def __init__(self,r,cfg,dest,dt,video=False):
        # Model mass beyond ATI is the actual nominal40g in archived config.
        FP.PLATE_MASS=.080*(.04/.14);FP.PAD_MASS=.020*(.04/.14)
        super().__init__(r,cfg,dest,dt,60.,video,pad_axial=4200.,spring_reference_dt=.00025)
        for tool in self.tools.values():p.removeBody(tool.plate_id);p.removeBody(tool.id)
        self.tools={};self.arms={};self.joint_rows=[];self.diagnostics=[];self.tau={};self.last_gravity={}
        built=build();urdf=dest/'robot_used.urdf';urdf.write_bytes(built.read_bytes())
        chain=Chain(str(ROOT/'urdf/gen3_ati_tool.urdf'),tip='end_effector_link')
        cell_a=r.axis/np.linalg.norm(r.axis);reach=.5*r.opening+GE.PAD_X+GE.PAD_R+r.c['open_clear']
        meta_path=ROOT/B.META[r.piece]
        csv_path=meta_path.with_name(meta_path.name.replace('_meta.json','.csv'))
        with csv_path.open() as f:logged=next(csv.DictReader(f))
        for side,name,sgn in [('L','left',1),('R','right',-1)]:
            c=cfg[name];shelf_R=Rot.from_quat(c['shelf_in_base']['quat']);shelf_t=np.array(c['shelf_in_base']['xyz'])
            base_R=shelf_R.inv();base_t=-base_R.apply(shelf_t)
            robot=p.loadURDF(str(urdf),base_t,base_R.as_quat(),useFixedBase=True,
                flags=p.URDF_USE_INERTIA_FROM_FILE|p.URDF_USE_SELF_COLLISION|p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)
            links={p.getJointInfo(robot,j)[12].decode():j for j in range(p.getNumJoints(robot))}
            joints=[j for j in range(p.getNumJoints(robot)) if p.getJointInfo(robot,j)[2] in (p.JOINT_REVOLUTE,p.JOINT_PRISMATIC)]
            assert len(joints)==7
            p.setJointMotorControlArray(robot,joints,p.VELOCITY_CONTROL,forces=[0]*7)
            for j in range(p.getNumJoints(robot)):p.changeDynamics(robot,j,linearDamping=0,angularDamping=0,jointDamping=0)
            q_seed=np.array([float(logged[f'q{side}_j{j}']) for j in range(1,8)])
            assert np.isfinite(q_seed).all()
            for j,q in zip(joints,q_seed):p.resetJointState(robot,j,float(q))
            T,_=chain.fk(q_seed);expected_p=base_t+base_R.apply(T[:3,3]);expected_R=base_R*Rot.from_matrix(T[:3,:3])
            ee=links['end_effector_link'];sensor=links['wrist_sensor_link']
            st=p.getLinkState(robot,ee,computeForwardKinematics=1)
            fk_pos_error=float(np.linalg.norm(np.array(st[4])-expected_p))
            fk_rot_error=float((expected_R.inv()*Rot.from_quat(st[5])).magnitude())
            assert fk_pos_error<1e-5 and fk_rot_error<1e-5,(side,fk_pos_error,fk_rot_error)
            tool_T=(np.array(c['tool_in_ee']['xyz']),Rot.from_quat(c['tool_in_ee']['quat']))
            logged_tool=expected_p+expected_R.apply(tool_T[0])
            logged_error=float(np.linalg.norm(logged_tool-np.array([float(logged['m'+a+side]) for a in 'xyz'])))
            # Independent recorded FK check; logging rounds positions to10um.
            assert logged_error<.002,(side,logged_error)
            want_p=r.centre-sgn*reach*cell_a;want_R=GE.quat_from_x(sgn*cell_a)
            ee_R=want_R*tool_T[1].inv();ee_p=want_p-ee_R.apply(tool_T[0])
            pb=shelf_t+shelf_R.apply(ee_p);Rb=shelf_R*ee_R
            q,ep,er,_=chain.ik(pb,Rb.as_matrix(),q_seed,iters=1000)
            assert ep<.002 and er<.02,(side,'open-pose unreachable',ep,er)
            for j,value in zip(joints,q):p.resetJointState(robot,j,float(value),0)
            p.enableJointForceTorqueSensor(robot,sensor,1)
            self.tools[side]=RobotPad(robot,ee,sensor,tool_T,dt)
            self.arms[side]=dict(robot=robot,joints=joints,ee=ee,sensor=sensor,
                base_R=base_R,base_t=base_t,q0=q.copy(),torque_limits=np.array([p.getJointInfo(robot,j)[10] for j in joints]))
            self.tau[side]=np.zeros(7)
            jac_error,jac_rotated=self.check_jacobian(side)
            self.arms[side]['jac_rotate_base']=jac_rotated
            self.diagnostics.append(dict(side=side,base_position_shelf=base_t,base_quat_shelf=base_R.as_quat(),
                fk_vs_independent_chain_position_error_m=fk_pos_error,fk_rotation_error_rad=fk_rot_error,
                recorded_tool_position_error_m=logged_error,open_ik_error_m=ep,open_ik_error_rad=er,
                jacobian_finite_difference_max_error=jac_error,jacobian_requires_base_rotation=jac_rotated,
                initial_q=q,torque_limits=self.arms[side]['torque_limits']))
        self.cmd=tuple(v for s in 'LR' for v in (self.tools[s].pose()[0],Rot.from_quat(self.tools[s].pose()[1])))
        save(dest/'lab_scene_checks.json',dict(checks=self.diagnostics,world='shelf',kinova_source=str(urdf),
            base_separation_m=float(np.linalg.norm(self.arms['L']['base_t']-self.arms['R']['base_t'])),
            law='tau=J.T@(K*pose_error-D*twist+plate_gravity_wrench)+inverseDynamics(q,dq,0)+N*(-.2*dq)',
            limitation='Declared Cartesian impedance, not verified unavailable inner lab controller source'))
        self.capture();self.frames[-1].save(dest/'lab_setup.png')
    def qstate(self,side):
        a=self.arms[side];st=p.getJointStates(a['robot'],a['joints'])
        return np.array([x[0] for x in st]),np.array([x[1] for x in st])
    def jac(self,side,rotate=None):
        a=self.arms[side];q,dq=self.qstate(side)
        jt,jr=p.calculateJacobian(a['robot'],a['ee'],[0,0,0],q.tolist(),[0.]*7,[0.]*7)
        J=np.r_[jt,jr]
        if rotate if rotate is not None else a.get('jac_rotate_base',False):
            J=np.r_[a['base_R'].as_matrix()@J[:3],a['base_R'].as_matrix()@J[3:]]
        return J
    def check_jacobian(self,side):
        a=self.arms[side];q,_=self.qstate(side);st=p.getLinkState(a['robot'],a['ee'],computeForwardKinematics=1)
        p0,R0=np.array(st[4]),Rot.from_quat(st[5]);numeric=np.zeros((6,7));eps=1e-4
        # Bullet link-frame pose is float32: finite difference step chosen above
        # its position/orientation quantization, not the controller timestep.
        for k,j in enumerate(a['joints']):
            p.resetJointState(a['robot'],j,float(q[k]+eps))
            st=p.getLinkState(a['robot'],a['ee'],computeForwardKinematics=1)
            numeric[:,k]=np.r_[(np.array(st[4])-p0)/eps,(Rot.from_quat(st[5])*R0.inv()).as_rotvec()/eps]
            p.resetJointState(a['robot'],j,float(q[k]))
        errors=[float(np.max(abs(self.jac(side,rotate=x)-numeric))) for x in [False,True]]
        best=int(np.argmin(errors));assert errors[best]<.004,(side,errors)
        return errors[best],bool(best)
    def impedance(self,side,target,q_des):
        a=self.arms[side];tool=self.tools[side]
        st=p.getLinkState(a['robot'],a['ee'],computeLinkVelocity=1,computeForwardKinematics=1)
        pos,R=np.array(st[4]),Rot.from_quat(st[5]);v,w=np.array(st[6]),np.array(st[7])
        tmount,Rmount=tool.tool_in_ee
        Rdes=q_des*Rmount.inv();pdes=target-Rdes.apply(tmount)
        error=np.r_[pdes-pos,(Rdes*R.inv()).as_rotvec()]
        wrench=np.array([1500.]*3+[20.]*3)*error-np.array([60.]*3+[7.]*3)*np.r_[v,w]
        pp,_=tool.plate_pose();weight=np.array([0.,0.,tool.mass*9.81])
        wrench+=np.r_[weight,np.cross(pp-pos,weight)]
        q,dq=self.qstate(side);J=self.jac(side)
        gravity=np.array(p.calculateInverseDynamics(a['robot'],q.tolist(),dq.tolist(),[0.]*7))
        null=np.eye(7)-J.T@np.linalg.solve(J@J.T+1e-7*np.eye(6),J)
        tau=J.T@wrench+gravity+null@(-.2*dq)
        self.tau[side]=np.clip(tau,-a['torque_limits'],a['torque_limits'])
        p.setJointMotorControlArray(a['robot'],a['joints'],p.TORQUE_CONTROL,forces=self.tau[side].tolist())
        tool.update_springs()
    def sample_sensor(self,side):
        a=self.arms[side];tool=self.tools[side]
        st=p.getLinkState(a['robot'],a['sensor'],computeForwardKinematics=1)
        pole,R=np.array(st[4]),Rot.from_quat(st[5])
        reaction=np.array(p.getJointState(a['robot'],a['sensor'])[2])
        raw=np.r_[R.apply(reaction[:3]),R.apply(reaction[3:])]
        pp,_=tool.plate_pose();weight=np.array([0.,0.,-9.81*tool.mass])
        return raw,raw+np.r_[weight,np.cross(pp-pole,weight)]
    def wait(self):
        for _ in range(self.substeps):
            for i,s in enumerate('LR'):self.impedance(s,*self.cmd[2*i:2*i+2])
            p.stepSimulation();self.count+=1;self.t=self.count*self.dt
            for s in 'LR':
                raw,gc=self.sample_sensor(s);self.last_raw[s]=raw;self.last_gc[s]=gc;self.acc[s]+=gc
            if self.count%round(.001/self.dt)==0:
                self.physics_raw.append(np.r_[self.t,*[self.last_raw[s] for s in 'LR'],*[self.last_gc[s] for s in 'LR']])
            if self.count%self.sensor_steps==0:
                for s in 'LR':self.buffers[s].append(self.acc[s]/self.sensor_steps);self.acc[s][:]=0
                self.physics.append(np.r_[self.t,*[self.last_raw[s] for s in 'LR'],*[self.last_gc[s] for s in 'LR']])
                self.sensor_blocks.append(np.r_[self.t,*[self.buffers[s][-1] for s in 'LR']])
            if self.video and self.count%self.frame_step==0:self.capture()
    def record(self,ins):
        super().record(ins)
        row=[self.t]
        for s in 'LR':q,dq=self.qstate(s);row.extend([*q,*dq,*self.tau[s]])
        self.joint_rows.append(row)
    def capture(self):
        from PIL import Image
        view=p.computeViewMatrix([1.70,-1.7,1.15],[.4,0,.4],[0,0,1]);projection=p.computeProjectionMatrixFOV(48,960/720,.02,5)
        rgba=p.getCameraImage(960,720,view,projection,renderer=p.ER_TINY_RENDERER)[2]
        self.frames.append(Image.fromarray(np.array(rgba,dtype=np.uint8).reshape(720,960,4)[:,:,:3]).resize((800,600)))
    def finish(self):
        np.savez_compressed(self.dest/'joint_telemetry.npz',data=np.array(self.joint_rows),
            columns=np.array(['t',*[s+'_'+kind+str(j) for s in 'LR' for kind in ['q','dq','tau_command'] for j in range(1,8)]]))
        return super().finish()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--piece',choices=list(B.META),default='Z')
    ap.add_argument('--tag',required=True);ap.add_argument('--dt',type=float,default=.0005)
    ap.add_argument('--scene-only',action='store_true');ap.add_argument('--video',action='store_true');a=ap.parse_args()
    dest=OUT/'runs'/a.tag;dest.mkdir(parents=True,exist_ok=False)
    save(dest/'STARTED.json',dict(pid=os.getpid(),argv=sys.argv))
    (dest/'runner_source.py').write_bytes(Path(__file__).read_bytes())
    meta=json.loads((ROOT/B.META[a.piece]).read_text());cfg=meta['config'];argv=meta['argv']
    GE.set_tool(**{k:cfg['tool'][k] for k in ('pad_x','r_tri','pad_r')})
    r=Route(argv[2],argv[argv.index('--arm')+1],scale=cfg['scale'],mass=cfg['piece_mass'])
    profile,summary=B.plan(r,cfg,float(argv[argv.index('--slow')+1]),dest)
    b=ArticulatedBackend(r,cfg,dest,a.dt,a.video)
    ins=M.slowed(FFIProfileInsertion(r,b,profile=profile,k_eff=680.,ffi_settings=FFISettings(),log=Recorder(b),confirm=b.confirm,
        f_abort=cfg['f_abort'],track_abort=cfg['track_abort'],jam_scale=cfg['jam_scale'],jam_min=cfg['jam_min'],
        squeeze_gain=.1,**M.socp_kwargs(cfg)),float(argv[argv.index('--slow')+1]))
    error=None;result={};t0=time.perf_counter()
    try:
        if not a.scene_only:result=ins.run()
        else:
            for _ in range(100):b.wait()
            ins.meas=b.read();b.record(ins)
    except Exception as exc:error=repr(exc);traceback.print_exc()
    if not b.rows:ins.meas=b.read();b.record(ins)
    checks=b.diagnostics;truth=b.finish()
    save(dest/'result.json',dict(error=error,controller_result=result,truth=truth,planning=summary,dt=a.dt,
        cart_k=[1500]*3+[20]*3,cart_d=[60]*3+[7]*3,ffi=vars(FFISettings()),feedforward_k_eff=680.,
        pad_axial_N_per_m=4200.,central_axial_N_per_m=2700.,pad_each_axial_N_per_m=500.,
        mechanical_pad_K=np.diag(FP.stiffness_6x6(FP.PadSprings(dt=.00025),FP.CentralSpring(k_push=2700.,dt=.00025))),
        spring_damping_reference_dt=.00025,tool_mass=.15,plate_mass=.04,lab_scene_checks=checks,
        central_C_linear=b.tools['L'].centre.c_lin,central_C_rotational=b.tools['L'].centre.c_rot,pad_C_linear=b.tools['L'].pads_spring.C_local,
        wall_seconds=time.perf_counter()-t0,law='Articulated Cartesian impedance in torque, gravity+coriolis, .2 nullspace damping',
        limitation='Same outer FFI/SOCP, physical Kinova URDF; inner law declared proxy, not verified lab servo source'))
    save(dest/'COMPLETED.json',dict(pid=None,error=error,trace_sha256=sha(dest/'trace.npz')))
    print(json.dumps(dict(error=error,truth=truth,controller_result=result),default=serial),flush=True)
if __name__=='__main__':main()
