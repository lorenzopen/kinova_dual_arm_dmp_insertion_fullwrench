"""Independent Pinocchio/Bullet FK, Jacobian, gravity and inertia audit."""
from run_lab_impedance import *
def main():
    meta=json.loads((ROOT/B.META['Z']).read_text());cfg=meta['config']
    GE.set_tool(**{k:cfg['tool'][k] for k in ('pad_x','r_tri','pad_r')})
    argv=meta['argv'];r=Route(argv[2],argv[argv.index('--arm')+1],scale=cfg['scale'],mass=cfg['piece_mass'])
    dest=HERE/'lab_model_check';dest.mkdir(exist_ok=False)
    b=LaboratoryBackend(r,cfg,dest,.00025)
    rng=np.random.default_rng(2403);checks=[]
    p.performCollisionDetection()
    contacts=[]
    robots={a['robot'] for a in b.arms.values()}
    for c in p.getContactPoints():
        if c[1] in robots or c[2] in robots:contacts.append(dict(bodyA=c[1],bodyB=c[2],linkA=c[3],linkB=c[4],distance=c[8]))
    for s in 'LR':
        a=b.arms[s];q0,_=b.qstate(s);ctrl=b.ctrl[s];rows=[]
        for k in range(20):
            q=q0+(rng.normal(0,.08,7) if k else 0.)
            for j,x in zip(a['joints'],q):p.resetJointState(a['robot'],j,float(x),0.)
            pose=ctrl.get_current_pose(q)
            st=p.getLinkState(a['robot'],a['ee'],computeForwardKinematics=1)
            bp=a['base_R'].inv().apply(np.array(st[4])-a['base_t'])
            br=a['base_R'].inv()*Rot.from_quat(st[5])
            Jpin=pin.computeFrameJacobian(b.model,ctrl.data,q,ctrl.ee_frame_id,pin.LOCAL_WORLD_ALIGNED)
            Jbullet=b.jac(s,rotate=False)
            gp=pin.computeGeneralizedGravity(b.model,ctrl.data,q).copy()
            gb=np.array(p.calculateInverseDynamics(a['robot'],q.tolist(),[0.]*7,[0.]*7))
            up=a['base_R'].inv().apply([0.,0.,.04*9.81])
            # Explicit40g in nominal rigid CoM location makes110g mount +40g
            # equal the original150g,35mm CoM for independent gravity parity.
            nominal_plate=pose.rotation@np.array([0.,0.,.085])
            gb+=Jbullet.T@np.r_[up,np.cross(nominal_plate,up)]
            rows.append(dict(position_error_m=float(np.linalg.norm(pose.translation-bp)),
                rotation_error_rad=float((Rot.from_matrix(pose.rotation).inv()*br).magnitude()),
                jacobian_error=float(np.max(abs(Jpin-Jbullet))),gravity_error_Nm=float(abs(gp-gb).max())))
        assert max(x['position_error_m'] for x in rows)<1e-6
        assert max(x['jacobian_error'] for x in rows)<1e-6
        assert max(x['gravity_error_Nm'] for x in rows)<1e-4
        for j,x in zip(a['joints'],q0):p.resetJointState(a['robot'],j,float(x),0.)
        M=pin.crba(b.model,ctrl.data,q0);J=pin.computeFrameJacobian(b.model,ctrl.data,q0,ctrl.ee_frame_id,pin.LOCAL_WORLD_ALIGNED)
        Kq=J.T@np.diag([1500]*3+[20]*3)@J;Dq=J.T@np.diag([60]*3+[7]*3)@J+.001*np.eye(7)
        rates=np.real(np.linalg.eigvals(np.linalg.solve(M,Dq)))
        checks.append(dict(side=s,tests=rows,q0=q0,mass_matrix=M,mass_eigenvalues=np.linalg.eigvalsh(M),
            damping_inverse_mass_eigenvalues_per_s=rates,
            max_damping_rate_times_control_dt=float(rates.max()*.001),
            note='Large sampled damping/inertia ratio is a diagnostic; not a full stability proof with Kalman, RC, slew and contacts'))
    result=dict(checks=checks,initial_robot_contacts=contacts,
        pure_classes_verbatim_source_sha256=b.source['source_sha256'],
        deployed_hash_verified=False,gravity_comparison='Original Pin150g rigid tool vs Bullet110g + analytically added nominal40g load')
    save(HERE/'lab_model_checks.json',result);p.disconnect();print(json.dumps(result,default=serial),flush=True)
if __name__=='__main__':main()
