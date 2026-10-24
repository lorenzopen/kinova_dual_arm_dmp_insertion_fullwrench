"""Known-system recovery and causal prefix checks, independent of robot data."""
import empirical_models as e
import numpy as np
from scipy.signal import lfilter,butter,sosfilt


def main():
    design=e.freeze();rng=np.random.default_rng(231023);records=[]
    known=np.array([1.2,-.3,.05,.03]);delay=8
    for i in range(8):
        u=rng.normal(size=1500)
        shifted=np.r_[np.zeros(delay),u[:-delay]]
        y=lfilter(known[2:],[1.,-known[0],-known[1]],shifted)
        keep=np.arange(len(u))>=50
        records.append(dict(u=u,y=y,force=2*y,keep=keep,
                            motion_variance=float(np.mean(y[keep]**2)),
                            force_variance=float(np.mean((2*y[keep])**2))))
    features={d:[e.arx_features(r,d) for r in records] for d in design['arx']['delays_samples']}
    fit,rejected=e.fit_arx(records,list(range(7)),design,features)
    error=float(abs(np.array(fit['coefficients'])-known).max())
    assert fit['delay_samples']==delay and error<1e-8,(fit,error)
    prediction=e.arx_free(fit,records[7]['u'])
    assessment=e.assess(records[7]['y'],prediction,records[7]['keep'])
    assert assessment['r2']>1-1e-10,assessment
    # Causal filtering and model responses cannot depend on altered future data.
    u=records[0]['u'];changed=u.copy();changed[500:]+=rng.normal(size=len(u)-500)*100
    sos=butter(2,[.3,8],btype='bandpass',fs=100,output='sos')
    prefix=float(abs(sosfilt(sos,u)[:500]-sosfilt(sos,changed)[:500]).max())
    assert prefix==0
    for model in design['models']:
        assert np.max(abs(e.shape(model,u)[:500]-e.shape(model,changed)[:500]))==0
    assert np.max(abs(e.arx_free(fit,u)[:500]-e.arx_free(fit,changed)[:500]))==0
    result=dict(known_system_delay_samples=delay,recovered_delay_samples=fit['delay_samples'],
                max_coefficient_error=error,independent_input_sequence_free_simulation=assessment,
                causal_prefix_error=prefix,LTI_model_shapes_checked=len(design['models']),
                note='Software/math verification on an independently generated stable known plant; not validation of the robot model.')
    e.dp.dump('empirical_software_validation.json',result)
    print(result,flush=True)


if __name__=='__main__':main()
