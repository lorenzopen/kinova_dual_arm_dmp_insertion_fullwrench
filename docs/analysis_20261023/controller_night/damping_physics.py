"""URDF/posture based damping sensitivity, explicitly idealized and unvalidated.

Uses only 8 development run files (4 holdout runs remain unused by this script).
Computes physical Cartesian inertia then varies cart_d in a 6D impedance model.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['OMP_NUM_THREADS']='1'
from pathlib import Path
import sys
import json
import csv
from datetime import datetime,timezone
import standalone as st
from rigid_model import RigidURDF,system,frequency_response
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.linalg import expm
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2]
sys.path.insert(0,str(OUT.parent/'archive_review'))
import analyze as a

DAMPINGS=[60,80,100,120]
CONTACTS=[1500*680/(1500-680),2700.]


def dump(name,value):
    (OUT/name).write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')


def load_postures():
    design=json.loads((OUT/'frozen_design.json').read_text());urdf=RigidURDF(ROOT/'urdf/gen3_ati_tool.urdf')
    postures=[];validation=[]
    for piece in 'ZLST':
        for rec in design['split'][piece]['development']:
            meta=json.loads((ROOT/rec['metadata']).read_text());d=a.previous.load(ROOT/rec['source'])
            ix=np.flatnonzero(d['phase']=='route');clock=d['t_wall'];rel=clock-clock[ix[0]]
            for fraction in [.1,.5,.9]:
                i=ix[round(fraction*(len(ix)-1))]
                for side,name in [('L','left'),('R','right')]:
                    q=np.array([d[f'q{side}_j{j}'][i] for j in range(1,8)])
                    calc=urdf.compute(q);cfg=meta['config'][name]
                    mount=R.from_quat(cfg['tool_in_ee']['quat']);base=R.from_quat(cfg['shelf_in_base']['quat'])
                    normal=calc['ee'][:3,:3]@mount.apply([1.,0,0])
                    pt=calc['ee'][:3,3]+calc['ee'][:3,:3]@np.array(cfg['tool_in_ee']['xyz'])
                    measured_shelf=np.array([d['m'+c+side][i] for c in 'xyz'])
                    measured_base=base.apply(measured_shelf)+np.array(cfg['shelf_in_base']['xyz'])
                    predicted_rot=R.from_matrix(calc['ee'][:3,:3])*mount
                    measured_rot=base*R.from_quat([d['mq'+c+side][i] for c in 'xyzw'])
                    position_error=float(np.linalg.norm(pt-measured_base))
                    orientation_error=float((predicted_rot*measured_rot.inv()).magnitude())
                    jacerr=urdf.verify_jacobian(q) if fraction==.5 else None
                    assert position_error<1e-5,(rec['id'],side,position_error)
                    # CSV measured quaternions/positions have FIVE decimals;
                    # q_j columns retain full precision. Bound rounding error.
                    assert orientation_error<3e-5,(rec['id'],side,orientation_error)
                    entry=dict(id=rec['id'],piece=piece,side=side,route_fraction=fraction,
                               route_time_s=float(rel[i]),q_rad=q.tolist(),normal_base=normal.tolist(),
                               cart_mass=calc['cart_mass'].tolist(),
                               cart_mass_condition=float(np.linalg.cond(calc['cart_mass'])),
                               apparent_mass_free_normal_kg=float(1/(normal@np.linalg.inv(calc['cart_mass'])[:3,:3]@normal)),
                               apparent_mass_constrained_normal_kg=float(np.r_[normal,[0,0,0]]@calc['cart_mass']@np.r_[normal,[0,0,0]]))
                    postures.append(entry)
                    validation.append(dict(id=rec['id'],side=side,route_fraction=fraction,
                                           fk_position_error_m=position_error,fk_orientation_error_rad=orientation_error,
                                           finite_difference_jacobian_max_error=jacerr,
                                           joint_mass_min_eigenvalue=float(np.linalg.eigvalsh(calc['joint_mass']).min())))
    dump('urdf_postures_development.json',postures)
    dump('urdf_model_validation.json',dict(urdf='urdf/gen3_ati_tool.urdf',sha256=urdf.sha256,
         model_note='Rigid link inertial parameters, no reflected drive inertia/friction/flexible joints; actual internal controller source not found on Desktop.',
         moving_link_mass_kg=calc['moving_link_mass_kg'],samples=validation))
    print('URDF FK verified',len(postures),'postures',flush=True)
    return postures


def frequency_sweep(postures):
    hz=np.unique(np.r_[np.geomspace(.2,30,250),1.,2.]);rows=[];responses={}
    for i,p in enumerate(postures):
        for kc in CONTACTS:
            for damping in DAMPINGS:
                aa,bb,cc,diag=system(np.array(p['cart_mass']),np.array(p['normal_base']),kc,damping)
                h=frequency_response(aa,bb,cc,hz);dc=diag['dc_gain_N_m'];phase=np.unwrap(np.angle(h))
                eigen=np.linalg.eigvals(aa);assert np.max(eigen.real)<1e-6
                row=dict(posture_index=i,id=p['id'],piece=p['piece'],side=p['side'],route_fraction=p['route_fraction'],
                         contact_stiffness_N_m=kc,damping_N_s_m=damping,dc_gain_N_m=dc,
                         max_real_pole=float(eigen.real.max()))
                for f in [1,2]:
                    j=np.flatnonzero(hz==f)[0]
                    row[f'gain_{f}Hz_N_m']=float(abs(h[j]))
                    row[f'phase_{f}Hz_deg']=float(np.degrees(phase[j]))
                    row[f'phase_equivalent_lag_{f}Hz_ms']=float(-phase[j]/(2*np.pi*f)*1000)
                band=(hz>=.8)&(hz<=2.5)
                row['max_gain_in_observed_band_over_dc']=float(abs(h[band]).max()/dc)
                responses[(i,kc,damping)]=(hz,h/dc)
                rows.append(row)
    dump('damping_frequency_metrics.json',rows)
    with (OUT/'damping_frequency_metrics.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    summary=[]
    for kc in CONTACTS:
        for damping in DAMPINGS:
            rr=[r for r in rows if r['contact_stiffness_N_m']==kc and r['damping_N_s_m']==damping]
            base=[r for r in rows if r['contact_stiffness_N_m']==kc and r['damping_N_s_m']==60]
            out=dict(contact_stiffness_N_m=kc,damping=damping)
            for f in [1,2]:
                out[f'gain_ratio_vs_D60_{f}Hz_min_median_max']=np.percentile([r[f'gain_{f}Hz_N_m']/b[f'gain_{f}Hz_N_m'] for r,b in zip(rr,base)],[0,50,100]).tolist()
                out[f'phase_lag_ms_{f}Hz_min_median_max']=np.percentile([r[f'phase_equivalent_lag_{f}Hz_ms'] for r in rr],[0,50,100]).tolist()
            summary.append(out)
    dump('damping_frequency_summary.json',summary)
    fig,axes=plt.subplots(2,2,figsize=(13.5,9))
    fig.subplots_adjust(left=.08,right=.98,top=.86,bottom=.16,hspace=.4,wspace=.25)
    fig.suptitle('Damping interno: modello meccanico dalle posture registrate',x=.08,ha='left',y=.97,fontsize=18,weight='bold')
    fig.text(.08,.915,'48 posture/braccio dalle 8 run di sviluppo · inerzie URDF · K=1500/20 · D rotazionale=7',fontsize=11)
    for col,kc in enumerate(CONTACTS):
        for damping,color in zip(DAMPINGS,['#263746','#1675bb','#dd941f','#9564b2']):
            hh=np.array([responses[(i,kc,damping)][1] for i in range(len(postures))])
            gain=abs(hh);lag=-np.unwrap(np.angle(hh),axis=1)/(2*np.pi*hz[None,:])*1000
            for ax,values in [(axes[0,col],gain),(axes[1,col],lag)]:
                ax.plot(hz,np.median(values,axis=0),color=color,lw=1.4,label=f'D {damping}')
                ax.fill_between(hz,np.min(values,axis=0),np.max(values,axis=0),color=color,alpha=.08)
                ax.axvspan(.8,2.5,color='grey',alpha=.08);ax.set(xlim=(.3,5),xlabel='Frequenza [Hz]');ax.grid(alpha=.2)
        axes[0,col].set(title=f'Contatto k={kc:.0f} N/m · k_eff={1500*kc/(1500+kc):.0f} N/m',ylabel='Guadagno forza / guadagno statico')
        axes[1,col].set(ylabel='Ritardo equivalente dalla fase [ms]')
        axes[0,col].legend(fontsize=8,ncol=2)
    fig.text(.08,.04,'Linee: mediana posture; aree: min–max, non intervalli di confidenza. Banda grigia: oscillazione osservata.\n'
             'Modello ideale a 6 DoF, non validato come dinamica del robot: mancano attrito, rotori, filtri e servo.\n'
             'Il ritardo equivalente dalla fase non è una misura di latenza né il tempo di risposta a uno step.',fontsize=10,linespacing=1.5)
    fig.savefig(OUT/'02_damping_modello_fisico.png',dpi=155);plt.close(fig)
    return rows,summary


def discretize(aa,bb,cc,dt):
    extended=np.zeros((13,13));extended[:12,:12]=aa;extended[:12,12:]=bb
    dd=expm(extended*dt)
    return dd[:12,:12],dd[:12,12],cc[0]


def closed_loop(postures,dt=.005,save=True):
    # Two arms per route at a frozen MID posture, to keep bilateral asymmetry.
    reps=[]
    for piece in 'ZLST':
        first=next(p['id'] for p in postures if p['piece']==piece)
        reps.append([next(p for p in postures if p['id']==first and p['side']==side and p['route_fraction']==.5) for side in 'LR'])
    models=[];ads=[];bds=[];cs=[]
    for arms in reps:
        for kc in CONTACTS:
            for scale in [1.,2.]:
                for delay in [0.,.12,.23]:
                    for damping in DAMPINGS:
                        model=dict(piece=arms[0]['piece'],id=arms[0]['id'],contact_stiffness_N_m=kc,
                                   inertia_scale=scale,extra_command_delay_s=delay,damping=damping)
                        three=[discretize(*system(np.array(p['cart_mass']),np.array(p['normal_base']),kc,damping,inertia_scale=scale)[:3],dt) for p in arms]
                        ads.append([v[0] for v in three]);bds.append([v[1] for v in three]);cs.append([v[2] for v in three]);models.append(model)
    ads=np.array(ads);bds=np.array(bds);cs=np.array(cs);M=len(models)
    delay=np.rint(np.array([m['extra_command_delay_s'] for m in models])/dt).astype(int)
    ringlen=delay.max()+2;history=np.zeros((ringlen,M));state=np.zeros((M,2,12))
    ff=np.zeros(M);trim=np.zeros(M);current=np.zeros(M)
    tick=round(.025/dt);ctrl_dt=tick*dt
    t=np.arange(0,24+dt/2,dt);target=np.where((t>=3)&(t<9),6.,4.)
    forces=np.empty((len(t),M,2),dtype=np.float32);commands=np.empty((len(t),M),dtype=np.float32)
    saturation=np.zeros(M);maxslew=np.zeros(M)
    for j,tt in enumerate(t):
        # Outputs measured before the newly computed command affects the plant.
        physical=4+np.einsum('mai,mai->ma',cs,state)+(0.5 if 15<=tt<19 else 0.)
        if j%tick==0 and j>0:
            prior=current.copy();e=2*target[j]-physical.sum(axis=1)
            ff,trim,current,limited=st.bounded_update(ff,trim,current,(target[j]-4)/680,
                                                    trim+.0016*e*ctrl_dt,ctrl_dt)
            saturation+=limited;maxslew=np.maximum(maxslew,abs(current-prior)/ctrl_dt)
        history[j%ringlen]=current
        delayed=history[(j-delay)%ringlen,np.arange(M)]
        state=np.einsum('maij,maj->mai',ads,state)+bds*delayed[:,None,None]
        forces[j]=physical;commands[j]=current*1000
    assert np.max(maxslew)<.002+1e-10
    rows=[];mean=forces.mean(axis=2)
    for m,model in enumerate(models):
        y=mean[:,m];error=y-target;rise=(t>=3)&(t<9);fall=(t>=9)&(t<15)
        steady=((t>=7.5)&(t<9))|((t>=13.5)&(t<15))|((t>=22.5)&(t<=24))
        t90=t[rise][np.flatnonzero(y[rise]>=5.8)[0]]-3 if np.any(y[rise]>=5.8) else None
        rows.append(dict(**model,rmse_N_hand=float(np.sqrt(np.mean(error[t>=3]**2))),
                         up_overshoot_N_hand=float(max(0,np.max(y[rise]-6))),
                         down_undershoot_N_hand=float(max(0,np.max(4-y[fall]))),
                         steady_mean_absolute_error_N_hand=float(np.mean(abs(error[steady]))),
                         rise_to_90_s=float(t90) if t90 is not None else None,
                         minimum_individual_force_N=float(forces[:,m,:].min()),
                         limited_tick_fraction=float(saturation[m]/(len(t)//tick)),
                         max_command_speed_mm_s=float(maxslew[m]*1000)))
    if save:
        dump('damping_closedloop_metrics.json',rows)
        with (OUT/'damping_closedloop_metrics.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
        central=[]
        for piece in 'ZLST':
            central.append([i for i,m in enumerate(models) if m['piece']==piece and m['contact_stiffness_N_m']==CONTACTS[0]
                            and m['inertia_scale']==1 and m['extra_command_delay_s']==.12])
        fig,axes=plt.subplots(2,2,figsize=(13.5,9));fig.subplots_adjust(left=.08,right=.98,top=.86,bottom=.16,hspace=.43,wspace=.25)
        fig.suptitle('FFI invariato, cambia solo il damping interno: simulazione',x=.08,ha='left',y=.97,fontsize=18,weight='bold')
        fig.text(.08,.915,'Posture reali, dinamica ideale URDF · k_eff=680 N/m · ritardo aggiunto fisso 120 ms · nessun rumore aggiunto',fontsize=10)
        for ax,ix in zip(axes.flat,central):
            for i,color in zip(ix,['#263746','#1675bb','#dd941f','#9564b2']):
                ax.plot(t,mean[:,i],color=color,lw=1.15,label=f"D {models[i]['damping']}")
            ax.plot(t,target,'k--',lw=.8,label='Target/mano')
            ax.set(title=models[ix[0]]['piece'],xlim=(2.5,24),xlabel='Tempo [s]',ylabel='Forza media mani [N]');ax.grid(alpha=.2)
        axes[0,0].legend(fontsize=8,ncol=3)
        fig.text(.08,.04,'Questo confronto non riproduce le run: è un esperimento sintetico con modelli meccanici congelati alle posture reali.\n'
                 'Non simula attrito, saturazione coppie, flessibilità dei giunti, perdita di contatto, board o variazioni di postura.\n'
                 'Lo sweep include anche k_eff=964, inerzia ×2 e ritardi 0/120/230 ms; i risultati completi sono nei CSV.',fontsize=10,linespacing=1.5)
        fig.savefig(OUT/'03_damping_FFI_sintetico.png',dpi=155);plt.close(fig)
        np.savez_compressed(OUT/'damping_closedloop_traces.npz',t_s=t,target_N_hand=target,
                            force_per_hand_N=forces,command_mm=commands,
                            model_metadata_json=np.array(json.dumps(models)))
    return rows


def main():
    postures=load_postures();freq,summary=frequency_sweep(postures)
    rows=closed_loop(postures)
    refined=closed_loop(postures,.0025,save=False)
    diffs={key:max(abs(r[key]-v[key]) for r,v in zip(rows,refined))
           for key in ['rmse_N_hand','up_overshoot_N_hand','down_undershoot_N_hand']}
    assert diffs['rmse_N_hand']<.025,diffs
    assert diffs['up_overshoot_N_hand']<.1,diffs
    dump('damping_numerical_validation.json',dict(grid_s=[.005,.0025],max_metric_difference_N=diffs,
         note='Controller ticks remain exactly 25 ms on both grids; exact ZOH discretization for plant.'))
    print('Frequency scenarios',len(freq),'closed-loop scenarios',len(rows),'plus refinement',len(refined),flush=True)
    print('Numerical refinement',diffs,flush=True)
    for v in summary:print(json.dumps(v),flush=True)


if __name__=='__main__':main()
