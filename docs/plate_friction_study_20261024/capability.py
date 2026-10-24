"""Diagnostic wrench-set slices and wrist-observability example; no controller change."""
from study import np,cp,json,HERE,ROOT,sha,save,wrench_map

def solve_slice(points,normals,force_axis,normal_force=15.,mu=.75):
    # Transform measured geometry to a frame whose x axis opposes mean outward normal.
    x=force_axis/np.linalg.norm(force_axis)
    seed=np.array([0.,0.,1.]);z=seed-x*(x@seed);z/=np.linalg.norm(z);y=np.cross(z,x)
    basis=np.column_stack([x,y,z]);pole=points.mean(0)
    pts=(points-pole)@basis;nrm=normals@basis
    f=cp.Variable(9);T=cp.Parameter();M=cp.Variable();sign=cp.Parameter()
    H=wrench_map(pts,np.zeros(3))
    constraints=[H@f==cp.hstack([normal_force,0,T,M,0,0])]
    for i in range(3):
        fi=f[3*i:3*i+3];N=-nrm[i]@fi
        constraints += [N>=0,cp.norm((np.eye(3)-np.outer(nrm[i],nrm[i]))@fi)<=mu*N]
    problem=cp.Problem(cp.Maximize(sign*M),constraints)
    grid=np.linspace(-mu*normal_force,mu*normal_force,81)
    curves=[]
    for sense in [-1,1]:
        sign.value=sense;values=[]
        for tangential in grid:
            T.value=tangential;problem.solve(solver='CLARABEL')
            if f.value is None or problem.status not in ('optimal','optimal_inaccurate'):
                values.append(float('nan'));continue
            F=f.value.reshape(3,3);comp=-np.sum(F*nrm,axis=1)
            tangent=F+comp[:,None]*nrm
            residual=H@f.value-np.array([normal_force,0,tangential,M.value,0,0])
            assert np.linalg.norm(residual)<2e-5
            assert np.max(np.linalg.norm(tangent,axis=1)-mu*comp)<2e-5
            values.append(float(M.value))
        curves.append(values)
    return dict(T=grid,lower=curves[0],upper=curves[1],normal_force=normal_force,mu=mu,
                frame=basis,points=pts,normals=nrm,condition='Zero bending slice at contact centroid; diagnostic only')

def main():
    source=HERE/'inputs/contact_geometry_snapshot.json'
    assert sha(source)==sha(ROOT/'docs/thesis_plate_update_20261024/bench/results.json')
    previous=json.loads(source.read_text());result={}
    for case in previous['cases']:
        state=case['settled'];points=[];normals=[]
        for pos,n in zip(state['contact_positions'],state['contact_normals']):
            assert len(pos)==1 and len(n)==1,'Multiple manifolds need separate treatment'
            points.append(pos[0]);normals.append(n[0])
        points=np.array(points);normals=np.array(normals)
        result[case['case']]=solve_slice(points,normals,-normals.mean(0))
    # Same wrist wrench with altered internal tangential forces along an inter-pad edge.
    profile=json.loads((HERE/'profiles/a5d6647ab2f9.json').read_text())['records'][0]
    points=np.array(profile['points'][:3]);F=np.array(profile['witness'][:3]);n=np.array(profile['normals'][:3])
    H=wrench_map(points,points.mean(0));edge=points[1]-points[0];edge/=np.linalg.norm(edge)
    null=np.zeros((3,3));null[0]=edge;null[1]=-edge
    assert np.linalg.norm(H@null.ravel())<1e-12
    alpha=np.linspace(0,8,161);distances=[];wrench_errors=[];force_errors=[];moment_errors=[]
    for value in alpha:
        candidate=F+value*null;N=-np.sum(candidate*n,axis=1);tangent=np.linalg.norm(candidate+N[:,None]*n,axis=1)
        distances.append((.75*N-tangent)/1.25)
        delta=H@(candidate-F).ravel()
        wrench_errors.append(np.linalg.norm(delta));force_errors.append(np.linalg.norm(delta[:3]));moment_errors.append(np.linalg.norm(delta[3:]))
    save(HERE/'capability.json',dict(slices=result,source_sha256=sha(source),
        observability=dict(alpha=alpha,signed_distance=np.array(distances),wrench_error=wrench_errors,force_error_N=force_errors,moment_error_Nm=moment_errors,
            null_dimension=int(9-np.linalg.matrix_rank(H)),interpretation='Force hypotheses with identical wrist resultant; not independently actuated pad commands')))
    print('Three exact cone slices and a rank-6 / nullity-3 wrist example saved.')

if __name__=='__main__':main()
