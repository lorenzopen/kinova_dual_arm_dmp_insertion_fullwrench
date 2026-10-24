"""Independent algebra, interpolation, provenance and contact-sign checks."""
from analyze import *

def balance(F,points,com,mass):
    return (np.linalg.norm(F.sum(0)+np.array([0,0,-9.81*mass])),
            np.linalg.norm(np.cross(points-com,F).sum(0)))

def audit_profile(path):
    data=json.loads(path.read_text());job=data['job_inputs'];records=data['records']
    base=records[0];mass=job['model_mass'];mu=job['model_mu']
    p0=np.array(base['points']);b0=np.array(base['bases']);n0=np.array(base['normals']);centre=np.array([0,0,.35])
    def check(angle,w,f,rho):
        R=Rot.from_euler('y',angle,degrees=True)
        points=centre+R.apply(p0-centre);bases=centre+R.apply(b0-centre);normals=R.apply(n0)
        com=centre+R.apply(job['model_com'])
        force=np.linalg.norm(w[:,:3].sum(0)+np.array([0,0,-9.81*mass]))
        moment=np.linalg.norm((np.cross(bases-com,w[:,:3])+w[:,3:]).sum(0))
        if np.isfinite(f).all():
            fw,mw=balance(f,points,com,mass)
            compression=-np.sum(f*normals,axis=1)
            psi=(mu*compression-np.linalg.norm(f+compression[:,None]*normals,axis=1))/np.sqrt(1+mu*mu)
            reconstructed=np.array([np.r_[f[h*3:h*3+3].sum(0),np.cross(points[h*3:h*3+3]-bases[h],f[h*3:h*3+3]).sum(0)] for h in range(2)])
            allocation_mismatch=np.linalg.norm((reconstructed-w)[:,:3],axis=1).max()
            moment_mismatch=np.linalg.norm((reconstructed-w)[:,3:],axis=1).max()
            deficit=max(0.,float(np.max(rho-psi)))
        else:fw=mw=allocation_mismatch=moment_mismatch=deficit=None
        poles=np.array([points[:3].mean(0),points[3:].mean(0)])
        moments=w[:,3:]-np.cross(poles-bases,w[:,:3]);n=normals[[0,3]]
        N=-np.sum(n*w[:,:3],axis=1);Ft=w[:,:3]+N[:,None]*n;Mn=np.sum(n*moments,axis=1)
        ellipse=np.sqrt(np.sum(Ft*Ft,axis=1)+(Mn/.0328)**2)-.9*mu*N
        bend=np.linalg.norm(moments-Mn[:,None]*n,axis=1)
        return dict(wrench_balance_force_N=float(force),wrench_balance_moment_Nm=float(moment),
            witness_balance_force_N=fw,witness_balance_moment_Nm=mw,margin_deficit_N=deficit,
            witness_to_wrist_force_N=allocation_mismatch,witness_to_wrist_moment_Nm=moment_mismatch,
            wong_ellipse_excess_N=float(ellipse.max()),wong_bending_Nm=float(bend.max()))
    keys=['wrench','witness','rho'];knots=np.array([r['angle'] for r in records])
    grid=[]
    for angle in np.linspace(0,15,601):
        values=[]
        for key in keys:
            v=np.array([r[key] for r in records]);flat=v.reshape(len(v),-1)
            values.append(np.array([np.interp(angle,knots,flat[:,j]) for j in range(flat.shape[1])]).reshape(v.shape[1:]))
        grid.append(check(angle,*values))
    nodes=[check(r['angle'],*[np.array(r[k]) for k in keys]) for r in records]
    def maximum(rows):
        return {key:max(float(r[key]) for r in rows if r[key] is not None) if any(r[key] is not None for r in rows) else None for key in rows[0]}
    return dict(path=path.name,method=job['method'],case=job['case'],knots=maximum(nodes),
        dense_interpolation=maximum(grid),discrete_witness_knots=int(sum(np.isfinite(r['witness']).all() for r in records)),
        combined_force_range_N=[min(r['combined_force'] for r in records),max(r['combined_force'] for r in records)],
        rho_range_N=[float(np.min([r['rho'] for r in records])),float(np.max([r['rho'] for r in records]))],
        fixed_calibration=data.get('calibration'))

def physics_checks(folder):
    summary=json.loads((folder/'COMPLETED.json').read_text());job=summary['job']
    d=dict(np.load(folder/'trace.npz',allow_pickle=False));m,x=measure(d,job)
    valid=d['t']>=1.3
    # Newton balance over recorded motion, a diagnostic of 100 Hz sampling rather than a tick-level proof.
    acceleration=np.gradient(d['object_vel'][:,:3],d['t'],axis=0)
    residual=d['force'].sum(1)+np.array([0,0,-9.81*job['mass']])-job['mass']*acceleration
    moment=np.cross(d['points']-d['object_pos'][:,None,:],d['force']).sum(1)
    alpha=np.gradient(d['object_vel'][:,3:],d['t'],axis=0)
    angular_residual=moment-job['mass']*.12**2/6*alpha
    terminal=d['t']>=4.1
    return dict(id=job['id'],outcome=summary['error'],recorded_sample_interval_s=float(np.median(np.diff(d['t']))),
        force_newton_residual_rms_N=float(np.sqrt(np.mean(np.sum(residual[valid]**2,axis=1)))) if valid.any() else None,
        moment_euler_residual_rms_Nm=float(np.sqrt(np.mean(np.sum(angular_residual[valid]**2,axis=1)))) if valid.any() else None,
        terminal_contact_force_N=d['force'][terminal].sum(1).mean(0).tolist() if terminal.any() else None,
        terminal_force_balance_N=m['terminal_force_balance_N'],
        lipschitz_min_slack_N=m['force_space_bound_min_slack_N'],
        manifold_max_normal_spread_deg=m['normal_spread_max_deg'])

def main():
    manifest=json.loads((HERE/'runs_manifest.json').read_text())
    sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    pairs=[('frozen_simulator',HERE/'study.py',manifest['source_sha256']),('frozen_protocol',HERE/'PROTOCOL.md',manifest['protocol_sha256']),
        ('prior',ROOT/'kinova_dual_arm_dmp_insertion/prior_torsional_C.npz',manifest['prior_sha256']),
        ('solver_copy',HERE/'socp_reference.py',manifest['solver_sha256']),
        ('production_solver',ROOT/'kinova_dual_arm_dmp_insertion/socp.py',manifest['solver_sha256']),
        ('plate_copy',HERE/'vendor/spider_ee/floating_plate.py',manifest['plate_sha256']),
        ('plate_original',Path('C:/Users/Lorenzo/Desktop/TESI_antiG/lorenzo_data_analysis/10_tripad_passivo/spider_ee/floating_plate.py'),manifest['plate_sha256'])]
    pairs.extend(('paper_'+str(i),Path(p['path']),p['sha256']) for i,p in enumerate(json.loads((HERE/'papers/sources.json').read_text())))
    provenance=[dict(name=name,path=str(path),expected=expected,actual=sha(path),ok=sha(path)==expected) for name,path,expected in pairs]
    assert all(p['ok'] for p in provenance)
    profiles=[audit_profile(path) for path in sorted((HERE/'profiles').glob('*.json'))]
    for profile in profiles:
        k=profile['knots']
        assert k['wrench_balance_force_N']<1e-4 and k['wrench_balance_moment_Nm']<1e-5
        if profile['method']!='wong':assert k['margin_deficit_N']<1e-4
        else:assert k['wong_ellipse_excess_N']<1e-4 and k['wong_bending_Nm']<1e-5
    physics=[physics_checks(path.parent) for path in sorted((HERE/'pilot').glob('*/COMPLETED.json'))]
    result=dict(provenance=provenance,profiles=profiles,pilot_physics=physics,
        interpolation_note='Linear interpolation of world-frame wrenches is not exact rigid rotation. Knot checks pass; dense in-between residuals are reported without claiming hard continuous feasibility.',
        physics_note='Contact-reconstructed virtual wrist resultant, not a native ATI measurement. Newton/Euler residuals use finite differences at 100 Hz and do not replace time-step convergence.')
    (HERE/'AUDIT.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Independent audits saved; '+str(len(profiles))+' profiles, all original hashes match.')

if __name__=='__main__':main()
