"""Diagnostic relaxation only: quantify aggregate allocations outside point cones."""
from study import np,cp,json,HERE,save

def skew(v):return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])

def main():
    output=[]
    for path in sorted((HERE/'profiles').glob('*.json')):
        profile=json.loads(path.read_text());job=profile['job_inputs']
        if job['method']!='wong':continue
        for record in profile['records']:
            for hand in range(2):
                f=cp.Variable((3,3));s=cp.Variable(nonneg=True)
                p=np.array(record['points'])[hand*3:hand*3+3];n=np.array(record['normals'])[hand*3:hand*3+3]
                b=np.array(record['bases'])[hand];w=np.array(record['wrench'])[hand]
                con=[cp.sum(f,axis=0)==w[:3],sum(skew(p[j]-b)@f[j] for j in range(3))==w[3:]]
                for j in range(3):
                    N=-n[j]@f[j];con.extend([N>=0,cp.norm(f[j]+N*n[j])<=job['model_mu']*N+s])
                problem=cp.Problem(cp.Minimize(s),con);problem.solve(solver='CLARABEL',tol_gap_abs=1e-9,tol_feas=1e-9)
                assert f.value is not None
                result=f.value;force_res=np.linalg.norm(result.sum(0)-w[:3]);moment_res=np.linalg.norm(np.cross(p-b,result).sum(0)-w[3:])
                assert force_res<1e-5 and moment_res<1e-6
                N=-np.sum(result*n,axis=1);excess=np.linalg.norm(result+N[:,None]*n,axis=1)-job['model_mu']*N
                assert N.min()>-1e-6 and excess.max()<=s.value+1e-5
                output.append(dict(case=job['case'],angle=record['angle'],hand=hand,status=problem.status,
                    diagnostic_minimum_cone_excess_N=max(0.,float(s.value)),force_residual_N=force_res,moment_residual_Nm=moment_res))
    save(HERE/'WONG_DISCRETE_AUDIT.json',dict(note='Relaxation is a diagnostic witness of incompatibility; never used in active planner or physical execution.',records=output))
    print(json.dumps(output,indent=2))

if __name__=='__main__':main()
