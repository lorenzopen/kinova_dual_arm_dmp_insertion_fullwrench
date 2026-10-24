"""Independent per-pad and wrench audit of completed trials; no simulation tuning."""
from pathlib import Path
import csv,json,hashlib,os,sys
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path.insert(0,str(ROOT/'tmp/dryrun_deps'));sys.dont_write_bytecode=True
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
import numpy as np
from scipy.spatial.transform import Rotation as Rot

def measure(d,job):
    F=d['force'];n=d['normals'];N=d['normal_N'];nom=d['nominal_normals'];mu=job['mu'];mup=job['model_mu']
    def psi(normal,coefficient):
        comp=-np.sum(F*normal,axis=2)
        tangent=np.linalg.norm(F+comp[:,:,None]*normal,axis=2)
        return (coefficient*comp-tangent)/np.sqrt(1+coefficient**2),comp,tangent
    actual,comp,tangent=psi(n,mu);nominal,_,_=psi(nom,mup);actual_planmu,_,_=psi(n,mup)
    eta=np.divide(tangent,mu*comp,out=np.full_like(comp,np.nan),where=comp>.05)
    present=N>.05
    witness=d['witness'];fnw=-np.sum(witness*nom,axis=2)
    psiw=(mup*fnw-np.linalg.norm(witness+fnw[:,:,None]*nom,axis=2))/np.sqrt(1+mup*mup)
    mismatch=np.linalg.norm(F-witness,axis=2)
    bound=psiw-mismatch
    valid=np.isfinite(bound)
    lipschitz=float(np.min((nominal-bound)[valid])) if valid.any() else None
    if lipschitz is not None:assert lipschitz>=-1e-7
    R=Rot.from_euler('y',d['angle'][:,None],degrees=True)
    actualR=Rot.from_quat(d['object_quat'])
    centre=d['object_pos']-actualR.apply(np.tile(job['com'],(len(F),1)))
    ref=np.array([0.,0.,.35]);position_error=np.linalg.norm(centre-ref,axis=1)
    rotation_error=np.degrees((R.inv()*actualR).magnitude())
    bases=np.stack([ref+R.apply(np.tile([x,0,0],(len(F),1))) for x in [-.09,.09]],axis=1)
    matched=d['measured_wrist'].copy()
    matched[:,:,3:]+=np.cross(d['tool_pos']-bases,matched[:,:,:3])
    error=matched-d['desired_wrist']
    # Wong-style aggregate limit-surface diagnostic about each nominal contact centroid.
    offsets=np.stack([R.apply(np.tile([x,0,0],(len(F),1))) for x in [.03,-.03]],axis=1)
    contact_moment=matched[:,:,3:]-np.cross(offsets,matched[:,:,:3])
    contact_moment_plan=d['desired_wrist'][:,:,3:]-np.cross(offsets,d['desired_wrist'][:,:,:3])
    hand_normals=nom[:,[0,3],:]
    hand_N=-np.sum(matched[:,:,:3]*hand_normals,axis=2)
    hand_Ft=np.linalg.norm(matched[:,:,:3]+hand_N[:,:,None]*hand_normals,axis=2)
    hand_Mn=np.sum(contact_moment*hand_normals,axis=2)
    hand_bending=np.linalg.norm(contact_moment-hand_Mn[:,:,None]*hand_normals,axis=2)
    hand_eta=np.sqrt((hand_Ft/(mu*np.maximum(hand_N,1e-9)))**2+(hand_Mn/(mu*.0328*np.maximum(hand_N,1e-9)))**2)
    stage=d['t']>=1.3
    if not stage.any():stage=d['t']>=1.0001
    assert stage.any()
    p=present[stage];e=eta[stage];a=actual[stage]
    strict=(N[stage]<=.5)|(e>1)
    tolerant=(N[stage]<=.5)|(e>1.001)
    nonphysical=(~p)|(a < -.001)
    sliding=p&(d['slip_speed'][stage]>.001)&(e>.95)
    total=F.sum(1)+np.array([0.,0.,-9.81*job['mass']])
    inertia_ratio=np.linalg.norm(total,axis=1)/(9.81*job['mass'])
    # Numerical sign/balance verification uses a terminal settled interval, separately from motion.
    terminal=(d['t']>=4.1)
    terminal_residual=float(np.linalg.norm(total[terminal].mean(0))) if terminal.any() else None
    out=dict(samples=int(stage.sum()),exposure_s=float(d['t'][-1]-d['t'][np.flatnonzero(stage)[0]]),
        all_six_present_percent=float(100*np.mean(p.all(1))),
        each_hand_present_percent=float(100*np.mean(p[:,:3].any(1)&p[:,3:].any(1))),
        absent_pad_percent=float(100*np.mean(~p)),
        cone_violation_loaded_percent=float(100*np.mean(a[p]<-.001)) if p.any() else None,
        paper_violation_strict_percent=float(100*np.mean(strict)),paper_violation_tolerant_percent=float(100*np.mean(tolerant)),
        contact_or_cone_bad_percent=float(100*np.mean(nonphysical)),
        margin_deficit_percent=float(100*np.mean((~p)|(a<d['rho'][stage]-.001))))
    out.update(sliding_percent=float(100*np.mean(sliding)),
        psi_min_loaded_N=float(np.min(a[p])) if p.any() else None,
        psi_median_loaded_N=float(np.median(a[p])) if p.any() else None,
        eta_p95_loaded=float(np.nanpercentile(e,95)) if p.any() else None,
        minimum_pad_normal_N=float(N[stage].min()),
        force_wrench_rmse_N=float(np.sqrt(np.mean(np.sum(error[stage,:,:3]**2,axis=2)))),
        moment_wrench_rmse_Nm=float(np.sqrt(np.mean(np.sum(error[stage,:,3:]**2,axis=2)))),
        final_position_mm=float(1000*position_error[-1]),final_attitude_error_deg=float(rotation_error[-1]),
        max_position_mm=float(1000*position_error[stage].max()),
        max_plate_tilt_deg=float(np.degrees(np.linalg.norm(d['plate_deflection'][stage,:,4:],axis=2)).max()),
        max_normal_change_deg=float(np.degrees(np.arccos(np.clip(np.sum(n[stage]*nom[stage],axis=2),-1,1)))[p].max()) if p.any() else None,
        normal_spread_max_deg=float(d['normal_spread'][stage].max()),
        inertia_to_weight_ratio_p95=float(np.percentile(inertia_ratio[stage],95)),
        terminal_force_balance_N=terminal_residual,
        force_space_bound_min_slack_N=lipschitz,
        geometry_psi_gap_p95_N=float(np.percentile(np.abs(actual_planmu[stage]-nominal[stage])[p],95)) if p.any() else None,
        friction_psi_gap_p95_N=float(np.percentile(np.abs(actual[stage]-actual_planmu[stage])[p],95)) if p.any() else None)
    known=np.isfinite(bound[stage])
    out.update(witness_difference_p95_N=float(np.percentile(mismatch[stage][known],95)) if known.any() else None,
        sufficient_force_bound_percent=float(100*np.mean(bound[stage][known]>=0)) if known.any() else None)
    out.update(aggregate_limit_surface_p95=float(np.percentile(hand_eta[stage],95)),
        aggregate_bending_p95_Nm=float(np.percentile(hand_bending[stage],95)))
    derived=dict(actual_psi=actual,nominal_psi=nominal,planned_psi=psiw,eta=eta,present=present,
        witness_difference=mismatch,lower_bound=bound,matched_wrist=matched,position_error=position_error,
        attitude_error=rotation_error,inertia_ratio=inertia_ratio,hand_N=hand_N,hand_Ft=hand_Ft,
        hand_Mn=hand_Mn,hand_eta=hand_eta,hand_bending=hand_bending,contact_moment_plan=contact_moment_plan)
    return out,derived

def process(folder):
    summary=json.loads((folder/'COMPLETED.json').read_text());job=summary['job']
    d=dict(np.load(folder/'trace.npz',allow_pickle=False));metrics,_=measure(d,job)
    entries=dict(np.load(folder/'manifold.npz',allow_pickle=False))['entries']
    active=entries[entries[:,0]>=1.3]
    if not len(active):active=entries[entries[:,0]>1.0]
    if len(active):
        psi_entry=(job['mu']*active[:,2]-active[:,3])/np.sqrt(1+job['mu']**2)
        metrics['manifold_cone_violation_percent']=float(100*np.mean(psi_entry<-.001))
        metrics['manifold_min_psi_N']=float(psi_entry.min())
    else:
        metrics['manifold_cone_violation_percent']=None;metrics['manifold_min_psi_N']=None
    outcome='complete' if summary['error'] is None else summary['error']
    physical_endpoint=bool(summary['error'] is None and metrics['final_position_mm']<=20 and metrics['final_attitude_error_deg']<=5)
    return dict(**job,outcome=outcome,endpoint_ok=physical_endpoint,seconds=summary['seconds'],**metrics)

def main():
    records=[];errors=[]
    for path in sorted((HERE/'runs').glob('*/COMPLETED.json')):
        try:records.append(process(path.parent))
        except Exception as exc:errors.append(dict(path=str(path),error=repr(exc)))
    if errors:raise RuntimeError(errors)
    failures=[dict(path=str(p),**json.loads(p.read_text())) for p in sorted((HERE/'runs').glob('*/FAILED.json'))]
    (HERE/'metrics.json').write_text(json.dumps(dict(records=records,exceptions=failures,
        definitions='Post-release t>=1.3 s (or recorded airborne prefix); all failed trials retained. Paper N<=0.5 convention separately from N<=0.05 presence; cone tolerance psi<-0.001 N. Wrist moments transported to nominal reference pole.'),indent=2)+'\n')
    if records:
        with (HERE/'metrics.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
    groups=[]
    for mechanism in ['welded','floating','floating2700']:
        for method in ['none','fixed','adaptive','wong']:
            group=[r for r in records if r['mechanism']==mechanism and r['method']==method and r['group'] in ['core','central2700']]
            if not group:continue
            fields=['all_six_present_percent','contact_or_cone_bad_percent','paper_violation_tolerant_percent','margin_deficit_percent',
                    'sliding_percent','force_wrench_rmse_N','moment_wrench_rmse_Nm','final_position_mm','final_attitude_error_deg']
            groups.append(dict(mechanism=mechanism,method=method,n=len(group),airborne_completed=sum(r['outcome']=='complete' for r in group),
                endpoint_ok=sum(r['endpoint_ok'] for r in group),**{field:float(np.median([r[field] for r in group])) for field in fields}))
    (HERE/'groups.json').write_text(json.dumps(groups,indent=2)+'\n')
    print(json.dumps(dict(completed=len(records),exceptions=len(failures),groups=groups),indent=2))

if __name__=='__main__':main()
