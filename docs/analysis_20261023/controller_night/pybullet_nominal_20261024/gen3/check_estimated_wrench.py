"""Verify supplied Kortex sign convention and EE-local estimator interface."""
from run_lab_impedance import *
def main():
    model=pin.buildModelFromUrdf(str(ROOT/'urdf/gen3_ati_tool.urdf'))
    checks=[]
    for case in json.loads((HERE/'lab_model_checks.json').read_text())['checks']:
        q=np.array(case['q0']);ctrl=CartesianImpedanceController(model,'end_effector_link',[1500]*3+[20]*3,[60]*3+[7]*3)
        ctrl.init_target_from_state(q)
        g=pin.computeGeneralizedGravity(model,ctrl.data,q).copy()
        ctrl.compute_torque(q,np.zeros(7),-g,.001)
        zero=ctrl.estimate_wrench(q,-g,np.zeros(7))
        assert np.linalg.norm(zero)<1e-12
        J=pin.computeFrameJacobian(model,ctrl.data,q,ctrl.ee_frame_id,pin.LOCAL)
        desired=np.array([.4,-.3,.2,.01,-.02,.03])
        motor=g-J.T@desired;sensor=-motor
        estimated=ctrl.estimate_wrench(q,sensor,np.zeros(7))
        error=float(abs(estimated-desired).max());assert error<1e-12
        checks.append(dict(side=case['side'],zero_load_estimated_norm=float(np.linalg.norm(zero)),
            prescribed_external_wrench_local=desired,estimated=estimated,max_abs_error=error))
    save(HERE/'estimated_wrench_sign_check.json',dict(checks=checks,
        relation='sensor=-motor; quasistatic motor=g-J_local.T external_wrench',
        limitation='Quasistatic estimator interface check, not dynamic ATI/contact equivalence'))
    print(json.dumps(checks,default=serial))
if __name__=='__main__':main()
