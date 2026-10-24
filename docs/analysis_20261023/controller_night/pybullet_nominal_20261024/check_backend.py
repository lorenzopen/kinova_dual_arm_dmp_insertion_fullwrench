"""Independent checks of sensor/reaction signs, frames, mechanics and free-air drift."""
from run_bullet import *
def main():
    meta=json.loads((ROOT/META['Z']).read_text());cfg=meta['config']
    GE.set_tool(**{k:cfg['tool'][k] for k in ('pad_x','r_tri','pad_r')})
    r=Route(meta['argv'][2],meta['argv'][meta['argv'].index('--arm')+1],scale=cfg['scale'],mass=cfg['piece_mass'])
    dest=OUT/'backend_checks';dest.mkdir(exist_ok=True)
    b=BulletBackend(r,cfg,dest,.0005,60.,pad_axial=4200.)
    rng=np.random.default_rng(20261024);errors=[]
    for s in 'LR':
        tool=b.tools[s];pf,qf=tool.pose();pp,qp=tool.plate_pose()
        for i in range(30):
            p.resetBasePositionAndOrientation(tool.plate_id,pp+rng.normal(0,1e-4,3),
                (Rot.from_rotvec(rng.normal(0,.001,3))*Rot.from_quat(qp)).as_quat())
            p.resetBaseVelocity(tool.plate_id,rng.normal(0,.01,3),rng.normal(0,.01,3))
            expected=spring_wrench(tool);actual=apply_springs_and_sense(tool)
            errors.append(float(np.max(np.abs(np.array(expected)-np.array(actual)))))
        tool.place(pf,qf)
    assert max(errors)<1e-10,errors
    # Clear applied-force accumulators from static equivalence probe.
    p.disconnect();b=BulletBackend(r,cfg,dest,.0005,60.,pad_axial=4200.)
    assert b.tools['L'].centre.k_lin[0]==2700.
    assert FP.stiffness_6x6(b.tools['L'].pads_spring,b.tools['L'].centre)[0,0]==4200.
    free=[]
    for _ in range(80):
        b.wait();m=b.read()
        if b.t>.5:free.append(np.r_[m['FL'],m['TL'],m['FR'],m['TR']])
    free=np.array(free)
    free_force=max(np.linalg.norm(free[:,:3],axis=1).max(),np.linalg.norm(free[:,6:9],axis=1).max())
    assert free_force<.02,free_force
    # Known compression against the fixed piece: sensor must have positive
    # push normals, contact force on piece must have same sign.
    a=r.axis/np.linalg.norm(r.axis)
    reach=.5*r.opening+GE.PAD_X+GE.PAD_R
    qL,qR=GE.quat_from_x(a),GE.quat_from_x(-a)
    b.command(r.centre-(reach-.004)*a,qL,r.centre+(reach-.004)*a,qR,8.)
    for _ in range(90):b.wait()
    m=b.read();stat={}
    for s in 'LR':
        tool=b.tools[s];pf,q=tool.pose();cw,pads,plate=contact_wrench(b.body,tool,pf)
        axis=Rot.from_quat(q).apply([1.,0,0]);w=np.r_[m['F'+s],m['T'+s]]
        stat[s]=dict(wrist_normal=float(w[:3]@axis),contact_normal=float(cw[:3]@axis),
                     wrench_difference=w-cw,pad_normal_N=pads,plate_normal_N=plate,
                     effective_pad_piece_mu=float(p.getDynamicsInfo(tool.plate_id,0)[1]*p.getDynamicsInfo(b.body,-1)[1]))
        assert min(stat[s]['wrist_normal'],stat[s]['contact_normal'])>1
        assert np.linalg.norm(w-cw)<.1,stat[s]
        assert abs(stat[s]['effective_pad_piece_mu']-1)<1e-12
    result=dict(central_axial_N_per_m=2700.,pad_each_axial_N_per_m=500.,total_axial_N_per_m=4200.,
                reaction_equivalence_cases=len(errors),max_abs_reaction_error=max(errors),
                free_air_max_force_after_settle_N=float(free_force),static_compression=stat,
                source='unchanged floating_plate.update_springs, captured actual applied reaction vs independent algebra',
                contact_order='unfiltered body-pair order; force on A = +normal/friction, on B opposite')
    save(OUT/'backend_checks_central2700.json',result);p.disconnect();print(json.dumps(result,default=serial),flush=True)
if __name__=='__main__':main()
