from run_gen3 import *
def main():
    cfg=json.loads((ROOT/B.META['Z']).read_text())['config'];GE.set_tool(**{k:cfg['tool'][k] for k in ('pad_x','r_tri','pad_r')})
    r=Route('F75_Zt1_s9012','c100_five_cut800',scale=cfg['scale'],mass=cfg['piece_mass'])
    dest=HERE/'dynamics_probe';dest.mkdir(exist_ok=True)
    b=ArticulatedBackend(r,cfg,dest,.0005)
    p.performCollisionDetection()
    contacts=[dict(A=x[1],B=x[2],linkA=x[3],linkB=x[4],distance=x[8]) for x in p.getContactPoints()]
    result=dict(initial_contacts=contacts,steps=[])
    for step in range(10):
        row=dict(step=step,arms={})
        for i,s in enumerate('LR'):
            a=b.arms[s];q,dq=b.qstate(s);tool=b.tools[s]
            J=b.jac(s);g=p.calculateInverseDynamics(a['robot'],q.tolist(),dq.tolist(),[0.]*7)
            target,q_des=b.cmd[2*i:2*i+2]
            st=p.getLinkState(a['robot'],a['ee'],computeLinkVelocity=1,computeForwardKinematics=1)
            R=Rot.from_quat(st[5]);rd=q_des*tool.tool_in_ee[1].inv()
            e=np.r_[target-np.array(st[4]),(rd*R.inv()).as_rotvec()]
            b.impedance(s,target,q_des)
            row['arms'][s]=dict(q=q,dq=dq,error=e,gravity=g,tau=b.tau[s],J=J)
        result['steps'].append(row);p.stepSimulation()
    save(HERE/'dynamics_probe.json',result);print(json.dumps(result,default=serial),flush=True);p.disconnect()
if __name__=='__main__':main()
