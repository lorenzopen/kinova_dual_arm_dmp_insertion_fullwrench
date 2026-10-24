"""Plots and metrics from saved autonomous Bullet traces, no simulation/tuning."""
import sys,os,json,csv
from pathlib import Path
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[3]
sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/'tmp/dryrun_deps'))
os.environ['MPLCONFIGDIR']=str(OUT/'mplconfig')
import numpy as np
from scipy.signal import butter,sosfiltfilt
from scipy.spatial.transform import Rotation as Rot
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'figure.dpi':135})

def band(x,lo,hi):
    if len(x)<50:return float('nan')
    filtered=sosfiltfilt(butter(3,[lo,hi],btype='bandpass',fs=100,output='sos'),x)
    return float(np.sqrt(np.mean(filtered**2)))

def load(folder):
    data=dict(np.load(folder/'trace.npz',allow_pickle=False));result=json.loads((folder/'result.json').read_text())
    for s in 'LR':
        axis=Rot.from_quat(data[s+'_quat']).apply(np.tile([1.,0,0],(len(data['t']),1)))
        data[s+'_normal']=np.sum(axis*data[s+'_measured'][:,:3],axis=1)
        data[s+'_contact_normal']=np.sum(axis*data[s+'_contact'][:,:3],axis=1)
        data[s+'_torque_local']=Rot.from_quat(data[s+'_quat']).inv().apply(data[s+'_measured'][:,3:])
    data['mean_normal']=.5*(data['L_normal']+data['R_normal'])
    return data,result

def metrics(d,r):
    if r['error'] or not np.any(d['phase']=='route'):return dict(error=r['error'],**r['truth'])
    mask=d['phase']=='route';active=np.isin(d['phase'],['to_mouth','route','slow_seat','seat_by_feel'])
    force=d['mean_normal'];error=force-d['target'];seat=np.isin(d['phase'],['slow_seat','seat_by_feel'])
    rate=np.diff(d['ds'])[np.diff(d.get('ffi_seq',np.zeros(len(d['t']))))>0] if 'ffi_seq' in d else []
    # Bound evaluated on actual controller's used dt, not all streaming ticks.
    used=d.get('ffi_dt_ms',np.full(len(d['t']),np.nan))*.001
    seq=d.get('ffi_seq',np.full(len(d['t']),np.nan));idx=np.flatnonzero(np.r_[False,np.diff(seq)>0])
    speed=[]
    for i in idx:
        j=i-1
        if np.isfinite(used[i]) and used[i]>0:speed.append(abs(d['ds'][i]-d['ds'][j])/used[i])
    return dict(error=None,**r['truth'],route_rmse_N=float(np.sqrt(np.mean(error[mask]**2))),
        route_force_band_08_25_N=band(force[mask],.8,2.5),route_error_band_08_25_N=band(error[mask],.8,2.5),
        route_force_band_25_10_N=band(force[mask],2.5,10),route_force_band_10_40_N=band(force[mask],10,40),
        active_min_L_N=float(d['L_normal'][active].min()),active_min_R_N=float(d['R_normal'][active].min()),
        active_max_mean_N=float(force[active].max()),seat_positive_error_N=float(error[seat].max()) if seat.any() else None,
        max_normal_correction_speed_mm_s=1000*max(speed,default=0.),
        max_abs_normal_correction_mm=float(1000*np.max(abs(d['ds'][active]))),
        rate_limited_fraction=float(np.nanmean(d['ffi_rate_limited'][active])),
        max_wrist_torque_norm_Nm=float(max(np.linalg.norm(d[s+'_measured'][active,3:],axis=1).max() for s in 'LR')),
        max_plate_contact_N=float(max(d[s+'_plate_normal'][active].max() for s in 'LR')),
        route_fraction_fewer_than_3_pads_L=float(np.mean(np.sum(d['L_pad_normals'][mask]>.01,axis=1)<3)),
        route_fraction_fewer_than_3_pads_R=float(np.mean(np.sum(d['R_pad_normals'][mask]>.01,axis=1)<3)),
        active_max_pad_plane_tilt_deg=float(max(np.degrees(np.linalg.norm(d[s+'_deflection'][active,4:6],axis=1)).max() for s in 'LR')),
        normal_wrist_contact_rmse_N=float(np.sqrt(np.mean(np.concatenate([d[s+'_normal'][active]-d[s+'_contact_normal'][active] for s in 'LR'])**2))))

def main():
    rows=[]
    folders=sorted((OUT/'runs').glob('*'))
    for folder in folders:
        if not (folder/'COMPLETED.json').exists():continue
        d,r=load(folder);m=metrics(d,r);rows.append(dict(run=folder.name,dt=r['dt'],damping=r['cart_d'][0],
            spring_axial_N_per_m=r.get('pad_axial_N_per_m',float(r['mechanical_pad_K'][0])),**m))
        if r['error'] or not np.any(d['phase']=='route'):continue
        fig,axes=plt.subplots(4,1,figsize=(12,11),sharex=True,layout='constrained')
        t=d['t'];ph=d['phase'];start=np.flatnonzero(ph=='to_mouth')[0];t=t-t[start]
        regulated=np.isin(ph,['to_mouth','route','slow_seat','seat_by_feel'])
        ref=np.where(regulated,d['target'],np.nan)
        axes[0].plot(t,ref,'k--',lw=1.5,label='Riferimento SOCP / mano')
        for s,col in [('L','#2563eb'),('R','#e87924')]:
            axes[0].plot(t,d[s+'_normal'],c=col,lw=.8,label='Polso '+s)
            axes[1].plot(t,d[s+'_normal']-ref,c=col,lw=.8,label=s)
            axes[2].plot(t,np.linalg.norm(d[s+'_measured'][:,3:],axis=1),c=col,lw=.9,label=s)
        axes[0].set_ylabel('Normale [N]');axes[1].set_ylabel('Errore [N]');axes[2].set_ylabel('Coppia al polso\nmodulo [Nm]')
        axes[3].plot(t,1000*d['ds'],c='#7c3aed',label='Correzione totale')
        if 'ffi_ff_mm' in d:
            axes[3].plot(t,d['ffi_ff_mm'],c='#059669',label='Feedforward')
            axes[3].plot(t,d['ffi_i_mm'],c='#dc2626',label='Integrale')
        axes[3].set_ylabel('Correzione [mm]');axes[3].set_xlabel('Tempo dalla partenza [s]')
        for ax in axes:
            ax.grid(alpha=.2);ax.legend(loc='upper right',ncol=3,fontsize=8)
            for i in np.flatnonzero(np.r_[True,ph[1:]!=ph[:-1]]):
                ax.axvline(t[i],c='.6',lw=.6,ls=':')
            ax.set_xlim(-1,t[-1])
        for i in np.flatnonzero(np.r_[True,ph[1:]!=ph[:-1]]):
            if t[i]>=0:axes[0].text(t[i],1.01,ph[i],transform=axes[0].get_xaxis_transform(),rotation=20,fontsize=8)
        backend='Gen3 articolati / impedenza dichiarata' if folder.name.startswith(('Gen3_','Gen3Lab_')) else 'Floating-plate / impedenza ideale'
        fig.suptitle(folder.name+' — PyBullet autonomo, FFI invariata\n'
                     f"RMSE route {m['route_rmse_N']:.3f} N; RMS banda {m['route_force_band_08_25_N']:.3f} N; "
                     f"errore finale cella peggiore {m['final_worst_cell_mm']:.1f} mm\n"
                     +backend+': non replica verificata del servo Kinova',fontsize=12)
        fig.savefig(folder/'force_tracking.png');plt.close(fig)
        fig,axes=plt.subplots(3,2,figsize=(13,9),sharex=True,layout='constrained')
        for j,label in enumerate(['x','y','z']):
            for s,col in [('L','#2563eb'),('R','#e87924')]:
                localF=Rot.from_quat(d[s+'_quat']).inv().apply(d[s+'_measured'][:,:3])
                axes[j,0].plot(t,localF[:,j],c=col,lw=.7,label=s)
                axes[j,1].plot(t,d[s+'_torque_local'][:,j],c=col,lw=.7,label=s)
            axes[j,0].set_ylabel('F'+label+' [N]');axes[j,1].set_ylabel('T'+label+' [Nm]')
        for ax in axes.flat:ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_xlim(-1,t[-1])
        for ax in axes[-1]:ax.set_xlabel('Tempo dalla partenza [s]')
        fig.suptitle(folder.name+' — sei componenti al polso, assi locali tool (x = spinta)\n'
                     'Gravità tool rimossa, inerzia conservata; media 5 ms × ultimi 2 blocchi')
        fig.savefig(folder/'wrist_wrench.png');plt.close(fig)
        fig,axes=plt.subplots(2,2,figsize=(13,8),sharex=True,layout='constrained')
        centre=.5*(d['L_position']+d['R_position'])
        for j,s in enumerate('LR'):
            for k,label,col in zip(range(3),'xyz',['#2563eb','#e87924','#059669']):
                measuredF=d[s+'_measured'][:,k]
                torque=d[s+'_measured'][:,3:]+np.cross(d[s+'_position']-centre,d[s+'_measured'][:,:3])
                axes[0,j].plot(t,measuredF,c=col,lw=.7,label='F'+label+' misurata')
                axes[0,j].plot(t,np.where(regulated,d[s+'_desired'][:,k],np.nan),c=col,lw=1,ls='--',alpha=.7)
                axes[1,j].plot(t,torque[:,k],c=col,lw=.7,label='T'+label+' misurata')
                axes[1,j].plot(t,np.where(regulated,d[s+'_desired'][:,k+3],np.nan),c=col,lw=1,ls='--',alpha=.7)
            axes[0,j].set_title('Polso '+s+' — assi scaffale')
            axes[0,j].set_ylabel('Forza [N]');axes[1,j].set_ylabel('Momento al centro presa [Nm]')
            axes[1,j].set_xlabel('Tempo dalla partenza [s]')
        for ax in axes.flat:ax.grid(alpha=.2);ax.legend(fontsize=8);ax.set_xlim(0,t[-1])
        fig.suptitle(folder.name+' — wrench SOCP completo (tratteggio) e misura (continuo)\n'
                     'Misura al centro presa corrente, target al centro nominale del piano. FFI regola solo la normale.')
        fig.savefig(folder/'socp_wrench.png');plt.close(fig)
        # Full precision convenient columns, traces additionally contain contacts/poses/SOCP.
        with (folder/'wrist_wrench_full.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['t','phase',*[s+'_'+kind+axis for s in 'LR' for kind in ['F','T'] for axis in 'xyz']])
            w.writerows([d['t'][i],str(ph[i]),*d['L_measured'][i],*d['R_measured'][i]] for i in range(len(t)))
    (OUT/'metrics.json').write_text(json.dumps(rows,indent=2)+'\n')
    if rows:
        keys=list(dict.fromkeys(k for r in rows for k in r))
        with (OUT/'metrics.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,keys);w.writeheader();w.writerows(rows)
    print(json.dumps(rows,indent=2))

if __name__=='__main__':main()
