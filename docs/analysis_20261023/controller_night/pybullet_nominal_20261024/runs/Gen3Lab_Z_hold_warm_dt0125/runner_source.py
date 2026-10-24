"""Articulated Bullet plant + verbatim laboratory Cartesian controller and Kalman.

The pure control classes are unchanged. Hardware/ROS transport is replaced by
deterministic virtual-time joint feedback and torque application, never real ROS.
"""
import sys,os,json,time,argparse,traceback
from pathlib import Path
HERE=Path(__file__).resolve().parent;OUT=HERE.parent
sys.dont_write_bytecode=True
sys.path.insert(0,str(OUT/'deps/pin_conda/Lib/site-packages'))
PIN_DLL=os.add_dll_directory(str(OUT/'deps/pin_conda/Library/bin'))
import run_gen3 as G
from run_gen3 import B,p,np,Rot,ROOT,GE,FP,Route,M,save,serial,sha,FFIProfileInsertion,FFISettings,Recorder
sys.path.insert(0,str(HERE/'lab_source'))
import pinocchio as pin
from controller_math import CartesianImpedanceController,OnlineKalmanState,URDFStribeckFriction

class LaboratoryBackend(G.ArticulatedBackend):
    def __init__(self,r,cfg,dest,dt,video=False,kalman_warm_seconds=2.):
        super().__init__(r,cfg,dest,dt,video,servo='explicit')
        self.ctrl={};self.kalman={};self.safe={s:False for s in 'LR'}
        self.watch_count={s:0 for s in 'LR'};self.previous={};self.lab_rows=[];self.events=[];self.feedback_rows=[];self.publish_rows=[]
        self.published_pose={s:self.tools[s].pose() for s in 'LR'}
        self.model=pin.buildModelFromUrdf(str(ROOT/'urdf/gen3_ati_tool.urdf'))
        assert self.model.nv==7 and self.model.nq==7
        self.source=json.loads((HERE/'lab_source/provenance.json').read_text())
        self.friction_scale=0. # exact source node default, not overridden by robot.launch.py
        for s in 'LR':
            q,dq=self.qstate(s);q=np.mod(q,2*np.pi)
            fric=URDFStribeckFriction(str(ROOT/'urdf/gen3_ati_tool.urdf'),scale=0.)
            # Missing deployed gen3_bare.urdf cannot affect this scale0 case.
            c=CartesianImpedanceController(self.model,'end_effector_link',K=[1500]*3+[20]*3,D=[60]*3+[7]*3,
                joint_damping=.001,max_torque=[35]*4+[12]*3,filter_hz=100.,friction_observer=fric,
                null_space_stiffness=0.)
            c.init_target_from_state(q)
            self.ctrl[s]=c;self.kalman[s]=OnlineKalmanState(7,r_q=1e-6,r_dq=2.5e-5,q_ddq=1.)
            self.kalman[s].reset(q,dq)
            # Actual insert starts after homing/approach with a running filter.
            # Constant-pose warm-up settles covariance without changing gains.
            for _ in range(round(kalman_warm_seconds/.001)):self.kalman[s].update(q,dq,.001)
            self.previous[s]=pin.computeGeneralizedGravity(self.model,c.data,q).copy()
            self.tau[s]=self.previous[s].copy()
            # Independent check: Pinocchio original model against Bullet actual
            # FK/Jacobian, before any force or trajectory is applied.
            pose=c.get_current_pose(q);a=self.arms[s]
            st=p.getLinkState(a['robot'],a['ee'],computeForwardKinematics=1)
            bp=a['base_R'].inv().apply(np.array(st[4])-a['base_t'])
            br=a['base_R'].inv()*Rot.from_quat(st[5])
            pe=float(np.linalg.norm(pose.translation-bp));re=float((Rot.from_matrix(pose.rotation).inv()*br).magnitude())
            Jpin=pin.computeFrameJacobian(self.model,c.data,q,c.ee_frame_id,pin.LOCAL_WORLD_ALIGNED)
            Jbullet=self.jac(s,rotate=False);je=float(np.max(abs(Jpin-Jbullet)))
            assert pe<1e-5 and re<1e-5 and je<1e-5,(s,pe,re,je)
            self.diagnostics.append(dict(side=s,pin_original_vs_bullet_position_error_m=pe,
                pin_original_vs_bullet_rotation_error_rad=re,pin_original_vs_bullet_jacobian_max_error=je))
        save(dest/'laboratory_controller.json',dict(source=self.source,pinocchio_version=pin.__version__,
            control_hz=1000,cart_k=[1500]*3+[20]*3,cart_d=[60]*3+[7]*3,joint_damping=.001,
            max_torque=[35]*4+[12]*3,filter_hz=100,kalman=dict(r_q=1e-6,r_dq=2.5e-5,q_ddq=1.),
            tau_slew_per_tick=.5,velocity_watchdog_limit=2.,velocity_watchdog_ticks=20,
            friction_enable=True,friction_scale=0.,friction_ramp_seconds=3.,null_space_stiffness=0.,
            q_feedback='Exact simulated joint states converted through degree/radian and0..360 Kortex convention; no invented noise',
            transport='No ROS/Kortex network; deterministic1kHz sample/hold, outer100Hz/FFI50Hz',
            dynamics='Bullet actual arms+compliant plate; original unsplit150g URDF used by Pinocchio controller, as laboratory launch',
            startup=dict(pose='Grasp open pose, homing trajectory not simulated',kalman_covariance_constant_pose_warm_seconds=kalman_warm_seconds,
                previous_command='Nominal gravity',RC='Zero actual starting velocity'),
            copied_loop_features='watchdog, friction ramp, Kalman then Cartesian class, slew, finite check',
            limitations='Source hash on currently deployed NVIDIA not confirmed; gen3_bare friction file unavailable but scale0 makes it inactive; no actuator gear/rotor/friction plant identification'))
    def impedance(self,s,target,q_des,update=True):
        a=self.arms[s];tool=self.tools[s]
        if update:
            dt=.001;q,dq=self.qstate(s)
            # Kortex feedback uses degrees; actual _update_state converts them.
            q=np.radians(np.mod(np.degrees(q),360.));dq=np.radians(np.degrees(dq))
            tau_sensor=-self.previous[s].copy()
            if np.any(abs(dq)>2.):
                self.watch_count[s]+=1
                if self.watch_count[s]==20:
                    self.safe[s]=True;self.events.append(dict(t=self.t,side=s,event='velocity_watchdog_gravity_only'))
            else:self.watch_count[s]=0
            c=self.ctrl[s];c.friction_observer.set_scale(min(1.,self.t/3.)*self.friction_scale)
            q_use,dq_use,ddq_use=self.kalman[s].update(q,dq,dt)
            Rdes=q_des*tool.tool_in_ee[1].inv();pdes=target-Rdes.apply(tool.tool_in_ee[0])
            Rb=a['base_R'].inv()*Rdes;pb=a['base_R'].inv().apply(pdes-a['base_t'])
            c.set_target(pin.SE3(Rb.as_matrix(),pb))
            if self.safe[s]:raw=pin.computeGeneralizedGravity(self.model,c.data,q_use).copy()
            else:raw=c.compute_torque(q_use,dq_use,tau_sensor,dt)
            delta=np.clip(raw-self.previous[s],-.5,.5)
            cmd=self.previous[s]+delta
            if not np.isfinite(cmd).all():
                self.events.append(dict(t=self.t,side=s,event='nonfinite_command_held'));cmd=self.tau[s].copy()
            self.previous[s]=cmd.copy();self.tau[s]=cmd
            self.lab_rows.append(np.r_[self.t,0 if s=='L' else 1,q_use,dq_use,ddq_use,c._dq_filt,raw,cmd,int(self.safe[s]),int(np.any(abs(raw)>=(np.array([35]*4+[12]*3)-1e-8))),int(np.any(abs(raw-cmd)>1e-8))])
            self.feedback_rows.append(np.r_[self.t,0 if s=='L' else 1,q,dq,tau_sensor])
            if (self.count//round(.001/self.dt)+1)%4==0:
                estimated=c.estimate_wrench(q,tau_sensor,dq)
                pos,quat=tool.pose();self.published_pose[s]=(pos.copy(),quat.copy())
                self.publish_rows.append(np.r_[self.t,0 if s=='L' else 1,estimated,pos,quat])
        p.setJointMotorControlArray(a['robot'],a['joints'],p.TORQUE_CONTROL,forces=self.tau[s].tolist())
        tool.update_springs()
    def read(self):
        m=super().read()
        for s in 'LR':m['p'+s],m['q'+s]=self.published_pose[s]
        return m
    def finish(self):
        np.savez_compressed(self.dest/'kortex_feedback_1kHz.npz',data=np.array(self.feedback_rows),
            columns=np.array(['t','side',*[k+str(i) for k in ['q_raw','dq_raw','tau_sensor'] for i in range(1,8)]]))
        np.savez_compressed(self.dest/'node_publish_250Hz.npz',data=np.array(self.publish_rows),
            columns=np.array(['t','side',*['wrench_estimated_ee_'+x for x in ['Fx','Fy','Fz','Tx','Ty','Tz']],*['tool_p'+x for x in 'xyz'],*['tool_q'+x for x in 'xyzw']]))
        np.savez_compressed(self.dest/'laboratory_loop_1kHz.npz',data=np.array(self.lab_rows),
            columns=np.array(['t','side',*[k+str(i) for k in ['q_filtered','dq_kalman','ddq_kalman','dq_RC','tau_before_slew','tau_applied'] for i in range(1,8)],'safe_mode','torque_clipped','slew_active']))
        save(self.dest/'laboratory_events.json',self.events)
        return super().finish()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--piece',choices=list(B.META),default='Z');ap.add_argument('--tag',required=True)
    ap.add_argument('--dt',type=float,default=.00025);ap.add_argument('--scene-only',action='store_true');ap.add_argument('--video',action='store_true')
    ap.add_argument('--kalman-warm-seconds',type=float,default=2.)
    a=ap.parse_args();dest=OUT/'runs'/a.tag;dest.mkdir(exist_ok=False)
    save(dest/'STARTED.json',dict(pid=os.getpid(),argv=sys.argv));(dest/'runner_source.py').write_bytes(Path(__file__).read_bytes())
    (dest/'controller_math_used.py').write_bytes((HERE/'lab_source/controller_math.py').read_bytes())
    meta=json.loads((ROOT/B.META[a.piece]).read_text());cfg=meta['config'];argv=meta['argv']
    GE.set_tool(**{k:cfg['tool'][k] for k in ('pad_x','r_tri','pad_r')})
    r=Route(argv[2],argv[argv.index('--arm')+1],scale=cfg['scale'],mass=cfg['piece_mass'])
    profile,summary=B.plan(r,cfg,float(argv[argv.index('--slow')+1]),dest)
    b=LaboratoryBackend(r,cfg,dest,a.dt,a.video,a.kalman_warm_seconds)
    ins=M.slowed(FFIProfileInsertion(r,b,profile=profile,k_eff=680.,ffi_settings=FFISettings(),log=Recorder(b),confirm=b.confirm,
        f_abort=cfg['f_abort'],track_abort=cfg['track_abort'],jam_scale=cfg['jam_scale'],jam_min=cfg['jam_min'],
        squeeze_gain=.1,**M.socp_kwargs(cfg)),float(argv[argv.index('--slow')+1]))
    error=None;result={};started=time.perf_counter()
    try:
        if not a.scene_only:result=ins.run()
        else:
            for _ in range(100):
                b.wait();ins.meas=b.read();b.record(ins)
                ins._safety(b.cmd[0],b.cmd[2])
    except Exception as exc:error=repr(exc);traceback.print_exc()
    if not b.rows:ins.meas=b.read();b.record(ins)
    truth=b.finish()
    save(dest/'result.json',dict(error=error,controller_result=result,truth=truth,planning=summary,dt=a.dt,
        cart_k=[1500]*3+[20]*3,cart_d=[60]*3+[7]*3,ffi=vars(FFISettings()),feedforward_k_eff=680.,
        pad_axial_N_per_m=4200.,central_axial_N_per_m=2700.,pad_each_axial_N_per_m=500.,
        mechanical_pad_K=np.diag(FP.stiffness_6x6(b.tools['L'].pads_spring,b.tools['L'].centre)),
        central_C_linear=b.tools['L'].centre.c_lin,central_C_rotational=b.tools['L'].centre.c_rot,pad_C_linear=b.tools['L'].pads_spring.C_local,
        spring_damping_reference_dt=.00025,tool_mass=.15,plate_mass=.04,servo_discretization='laboratory_source_verbatim',
        torque_control_hz=1000,lab_scene_checks=b.diagnostics,lab_source_sha256=b.source['source_sha256'],pinocchio_version=pin.__version__,
        laboratory_events=b.events,wall_seconds=time.perf_counter()-started,
        law='Verbatim laboratory CartesianImpedanceController + OnlineKalmanState, copied node slew/watchdog/ramp, Bullet plant',
        limitation='Source found locally, deployed NVIDIA hash unverified; plant/contact/actuator dynamics not hardware-validated'))
    save(dest/'COMPLETED.json',dict(pid=None,error=error,trace_sha256=sha(dest/'trace.npz')))
    print(json.dumps(dict(error=error,truth=truth,controller_result=result,events=b.events),default=serial),flush=True)
if __name__=='__main__':main()
