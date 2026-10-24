"""Independent rigid-URDF Pinocchio ABA hold probe, no Bullet/contact/pads.

Diagnostic only: cross-check whether the nominal inertial plant can support
the sampled original controller. Not an insertion and not hardware evidence.
"""
from run_lab_impedance import *
def trial(q0,dt,warm_seconds=0.):
    model=pin.buildModelFromUrdf(str(ROOT/'urdf/gen3_ati_tool.urdf'));data=model.createData()
    ctrl=CartesianImpedanceController(model,'end_effector_link',[1500]*3+[20]*3,[60]*3+[7]*3,
        joint_damping=.001,max_torque=[35]*4+[12]*3,filter_hz=100.,null_space_stiffness=0.)
    ctrl.init_target_from_state(q0)
    kalman=OnlineKalmanState(7);kalman.reset(np.mod(q0,2*np.pi),np.zeros(7))
    for _ in range(round(warm_seconds/.001)):kalman.update(np.mod(q0,2*np.pi),np.zeros(7),.001)
    q=q0.copy();q[-1]+=1e-7;dq=np.zeros(7)
    previous=pin.computeGeneralizedGravity(model,data,q0).copy();tau=previous.copy()
    rows=[];watch=0;safe=False;event=None;sub=round(.001/dt)
    for step in range(round(.15/dt)):
        t=step*dt
        if step%sub==0:
            watch=watch+1 if np.any(abs(dq)>2.) else 0
            if watch==20:safe=True;event=t
            qu,du,au=kalman.update(np.mod(q,2*np.pi),dq,.001)
            raw=pin.computeGeneralizedGravity(model,data,qu).copy() if safe else ctrl.compute_torque(qu,du,np.zeros(7),.001)
            tau=previous+np.clip(raw-previous,-.5,.5);previous=tau.copy()
            rows.append(np.r_[t,q,dq,qu,du,tau,int(safe)])
        ddq=pin.aba(model,data,q,dq,tau)
        dq=dq+dt*ddq;q=q+dt*dq
        assert np.isfinite(q).all() and np.isfinite(dq).all()
    return dict(dt=dt,warm_seconds=warm_seconds,watchdog_time_s=event,peak_abs_dq_rad_s=float(abs(np.array(rows)[:,8:15]).max()),
        rows=np.array(rows))
def main():
    q0=np.array(json.loads((HERE/'lab_model_checks.json').read_text())['checks'][0]['q0'])
    results=[]
    warm='--warm' in sys.argv
    for dt in [.000025,.0000125]:
        x=trial(q0,dt,2. if warm else 0.);name='rigid_pin_hold_'+str(round(dt*1e7))+('_warm' if warm else '')
        np.savez_compressed(HERE/(name+'.npz'),data=x.pop('rows'))
        results.append(x)
    save(HERE/('rigid_pin_hold_warm.json' if warm else 'rigid_pin_hold.json'),dict(perturbation_j7_rad=1e-7,plant='Original rigid150g tool URDF, Pinocchio ABA, semi-implicit Euler',
        controller='Verbatim source Cartesian+Kalman, same1kHz, clip/slew/watchdog; friction scale0',
        limitation='Isolated rigid plant diagnostic, no tri-pad, contacts, robot or hardware identification',trials=results))
    print(json.dumps(results),flush=True)
if __name__=='__main__':main()
