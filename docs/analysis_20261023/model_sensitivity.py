"""Reallocate sampled real-run geometries, changing only the no-bending constraint.
This is a model sensitivity check, not a reconstruction of asynchronous historical solves.
"""
import json
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from kinova_dual_arm_dmp_insertion import geometry as GE
from kinova_dual_arm_dmp_insertion.audit_run import load
from kinova_dual_arm_dmp_insertion.controller import PRIOR
from kinova_dual_arm_dmp_insertion.prior import HumanPrior
from kinova_dual_arm_dmp_insertion.route import Route
from kinova_dual_arm_dmp_insertion.socp import GraspSOCP

path=ROOT/'docs/runs/20261022_F75_Zt1_s9012_squeeze_gain/F75_Zt1_s9012_c100_five_cut800_20261022_173458.csv'
d=load(path)
cfg=json.loads(path.with_name(path.stem+'_meta.json').read_text())['config']
GE.set_tool(**{k:cfg['tool'][k] for k in ('pad_x','r_tri','pad_r')})
r=Route('F75_Zt1_s9012','c100_five_cut800',cfg['scale'],cfg['piece_mass'])
prior=HumanPrior(PRIOR)
solvers=[]
for bend in (False,True):
    opts=dict(cfg['socp'],no_bending=bend)
    solvers.append(GraspSOCP(prior=prior,mu=r.c['mu_eff'],**opts))
rows=[]
indices=np.flatnonzero((d['phase']=='route') & np.isfinite(d['wdesL_fx']))[::60]
for k in indices:
    p={s:np.array([d['m'+q+s][k] for q in 'xyz']) for s in 'LR'}
    R={s:Rot.from_quat([d['mq'+q+s][k] for q in 'xyzw']) for s in 'LR'}
    first,second=('R','L') if r.swapped else ('L','R')
    qcmd=Rot.from_quat([d[q+first][k] for q in ('qx','qy','qz','qw')])
    off=Rot.from_rotvec([d['off'+first+'_'+q][k] for q in ('rx','ry','rz')])
    nominal=off.inv()*qcmd
    R_obj=nominal*GE.quat_from_x(r.axis if first=='L' else -r.axis).inv()*r.rots[0]
    c=.5*(p['L']+p['R'])
    pole=c-R_obj.apply(r.r_plan)
    force=sum(np.array([d['wdes'+s+'_f'+q][k] for q in 'xyz']) for s in 'LR')
    torque=sum(np.array([d['wdes'+s+'_t'+q][k] for q in 'xyz']) for s in 'LR')+np.cross(c-pole,force)
    pA,nA=GE.pad_contacts(p[first],R[first]); pB,nB=GE.pad_contacts(p[second],R[second])
    values=[]
    for solver in solvers:
        result=solver.solve(posL=pA,posR=pB,baseL=p[first],baseR=p[second],nrmL=nA,nrmR=nB,
                            m_hat=r.mass,p_com=pole,orientation=qcmd.as_matrix(),w_des=np.r_[force,torque])
        assert result['ok'], result['reason']
        values.append(max(sum(result['fn_L']),sum(result['fn_R'])))
    rows.append(values)
a=np.array(rows)
out=dict(samples=len(a),original_export_mass_kg=r.raw['piece_model']['mass'],current_mass_kg=r.mass,
         automatic_contact_force_scale=r.mass/r.raw['piece_model']['mass'],
         normal_target_median_bending_allowed_N=float(np.median(a[:,0])),
         normal_target_median_no_bending_N=float(np.median(a[:,1])),
         no_bending_to_allowed_ratio_p50_p95_max=np.percentile(a[:,1]/a[:,0],[50,95,100]).tolist(),
         all_allocations_feasible=True)
print(json.dumps(out,indent=2))
(Path(__file__).resolve().parent/'model_sensitivity.json').write_text(json.dumps(out,indent=2))
