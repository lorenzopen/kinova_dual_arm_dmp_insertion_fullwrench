"""Compare isotropic and base-y-only damping in the declared URDF model."""
import damping_physics as dp
import numpy as np
from rigid_model import system,frequency_response


def main():
    ps=dp.json.loads((dp.OUT/'urdf_postures_development.json').read_text());hz=np.array([1.,2.])
    rows=[]
    for i,p in enumerate(ps):
        mass=np.array(p['cart_mass']);n=np.array(p['normal_base'])
        for kc in dp.CONTACTS:
            reference=None
            for label,d in [('baseline',[60,60,60]),('isotropic80',[80,80,80]),('base_y80',[60,80,60])]:
                aa,bb,cc,diag=system(mass,n,kc,d)
                h=frequency_response(aa,bb,cc,hz)
                # Same plant, vertical Cartesian command -> vertical motion.
                inp=np.zeros((12,1));inp[6:,0]=np.linalg.solve(mass,np.array([0,0,1500.,0,0,0]))
                out=np.zeros((1,12));out[0,2]=1
                vertical=frequency_response(aa,inp,out,hz)
                if label=='baseline':reference=(h.copy(),vertical.copy())
                row=dict(posture_index=i,piece=p['piece'],side=p['side'],contact_stiffness_N_m=kc,
                         setting=label,cart_d=d+[7,7,7])
                for j,f in enumerate([1,2]):
                    row[f'normal_gain_vs_base_{f}Hz']=float(abs(h[j]/reference[0][j]))
                    row[f'vertical_extra_phase_lag_ms_{f}Hz']=float(-np.angle(vertical[j]/reference[1][j])/(2*np.pi*f)*1000)
                rows.append(row)
    dp.dump('damping_axis_metrics.json',rows)
    summary=[]
    for setting in ['isotropic80','base_y80']:
        rr=[r for r in rows if r['setting']==setting]
        row=dict(setting=setting)
        for key in ['normal_gain_vs_base_1Hz','normal_gain_vs_base_2Hz','vertical_extra_phase_lag_ms_1Hz','vertical_extra_phase_lag_ms_2Hz']:
            row[key+'_min_median_max']=np.percentile([r[key] for r in rr],[0,50,100]).tolist()
        summary.append(row)
    dp.dump('damping_axis_summary.json',summary)
    for row in summary:print(dp.json.dumps(row),flush=True)


if __name__=='__main__':main()
