"""Observational command/motion/force alignment, NOT causal identification.

Only development FFI plus two near-zero-I diagnostic runs; reserved runs unused.
Projects pair closing along a fixed opposing normal at route midpoint. This is
a geometric coordinate, not independently measured pad compression.
"""
from pathlib import Path
import sys
import json
import damping_physics as dp
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from scipy.signal import lsim
from rigid_model import RigidURDF,system

OUT=dp.OUT;ROOT=dp.ROOT;a=dp.a


def alignment(t,x,y):
    delays=np.arange(0,.401,.005);keep=(t>=t[0]+2)&(t<=t[-1]-2)
    results=[]
    for delay in delays:
        lagged=np.interp(t[keep]-delay,t,x);observed=y[keep]
        gain=float(lagged@observed/(lagged@lagged))
        residual=float(np.mean((observed-gain*lagged)**2))
        results.append(dict(delay_s=float(delay),gain=gain,
                            r2=1-residual/float(np.mean(observed**2)),residual_rms=float(np.sqrt(residual))))
    # Positive alignment is a diagnostic convention, not a structural constraint
    # that can establish a causal plant from a closed-loop record.
    return max([r for r in results if r['gain']>0],key=lambda r:r['r2']),results


def main():
    design=json.loads((OUT/'frozen_design.json').read_text());catalogue=json.loads((OUT.parent/'archive_review/run_catalog.json').read_text())
    records=[]
    for piece in 'ZLST':
        for entry in design['split'][piece]['development']:
            records.append((next(r for r in catalogue if r['id']==entry['id']),'development'))
    for stamp in ['165531','165902']:
        records.append((next(r for r in catalogue if '_ffi_20261023_'+stamp in r['id']),'near_zero_I_diagnostic'))
    rows=[];traces={};urdf=RigidURDF(ROOT/'urdf/gen3_ati_tool.urdf')
    for rec,role in records:
        d=a.previous.load(ROOT/rec['source']);ix=np.flatnonzero(d['phase']=='route')
        tt=d['t_wall'][ix]-d['t_wall'][ix[0]];t=np.arange(0,tt[-1],.01)
        mid=ix[len(ix)//2]
        normals=[R.from_quat([d['mq'+c+s][mid] for c in 'xyzw']).apply([1.,0,0]) for s in 'LR']
        n=normals[0]-normals[1];n/=np.linalg.norm(n)
        cmd=[np.column_stack([d[c+s][ix] for c in 'xyz']) for s in 'LR']
        meas=[np.column_stack([d['m'+c+s][ix] for c in 'xyz']) for s in 'LR']
        u=(cmd[0]-cmd[1])@n*.5*1000 # mm/hand projected pair closure coordinate.
        x=(meas[0]-meas[1])@n*.5*1000
        force=d['fn_N'][ix]/2
        band=[]
        for value in [u,x,force]:
            _,b,keep=a.previous.bandpass(t,np.interp(t,tt,value),[.8,2.5]);band.append(b)
        bm,gm=alignment(t,band[0],band[1]);bf,gf=alignment(t,band[1],band[2])
        model_comparisons=[];prediction_by_contact={};full_inputs={};rigids={}
        for side,name in [('L','left'),('R','right')]:
            q=np.array([d[f'q{side}_j{j}'][mid] for j in range(1,8)])
            rigid=urdf.compute(q);rigids[side]=rigid;cfg=rec['meta']['config'][name]
            base=R.from_quat(cfg['shelf_in_base']['quat']);mount=R.from_quat(cfg['tool_in_ee']['quat'])
            cmd_tool=R.from_quat(np.column_stack([d[c+side][ix] for c in ['qx','qy','qz','qw']]))
            cmd_ee=base*cmd_tool*mount.inv()
            cmd_point=base.apply(np.column_stack([d[c+side][ix] for c in 'xyz']))+cfg['shelf_in_base']['xyz']-cmd_ee.apply(cfg['tool_in_ee']['xyz'])
            cmd_angle=(cmd_ee*R.from_matrix(rigid['ee'][:3,:3]).inv()).as_rotvec()
            six=np.column_stack([cmd_point,cmd_angle]);six_band=[]
            for column in six.T:
                _,value,_=a.previous.bandpass(t,np.interp(t,tt,column),[.8,2.5]);six_band.append(value)
            full_inputs[side]=np.column_stack(six_band)
        for kc in dp.CONTACTS:
            predictions=[];full_predictions=[]
            for side,name in [('L','left'),('R','right')]:
                rigid=rigids[side];cfg=rec['meta']['config'][name]
                normal=rigid['ee'][:3,:3]@R.from_quat(cfg['tool_in_ee']['quat']).apply([1.,0,0])
                aa,bb,cc,_=system(rigid['cart_mass'],normal,kc,60)
                # Normal-only excitation. Same u to two idealized contacts;
                # ignores off-normal commanded motion and moving piece support.
                _,pred,_=lsim((aa,bb,cc/kc,np.zeros((1,1))),U=band[0],T=t)
                predictions.append(pred)
                k=np.diag([1500.]*3+[20.]*3)
                bfull=np.vstack([np.zeros((6,6)),np.linalg.solve(rigid['cart_mass'],k)])
                _,predfull,_=lsim((aa,bfull,cc/kc,np.zeros((1,6))),U=full_inputs[side],T=t)
                full_predictions.append(predfull*1000)
            pred=np.mean(predictions,axis=0);prediction_by_contact[kc]=pred
            fullpred=np.mean(full_predictions,axis=0)
            central=(t>=2)&(t<=t[-1]-2)
            r2=1-float(np.mean((pred[central]-band[1][central])**2))/float(np.mean(band[1][central]**2))
            fit_model,_=alignment(t,band[0],pred)
            model_comparisons.append(dict(contact_stiffness_N_m=kc,
                predicted_motion_vs_recorded_motion_r2=r2,
                predicted_motion_rms_mm=float(np.std(pred[central])),
                full_6D_command_model_motion_r2=1-float(np.mean((fullpred[central]-band[1][central])**2))/float(np.mean(band[1][central]**2)),
                full_6D_model_motion_rms_mm=float(np.std(fullpred[central])),
                model_command_to_motion_alignment=fit_model))
        row=dict(id=rec['id'],piece=rec['piece'],role=role,label=rec['label'],
                 ki_m_N_s=rec['meta']['execution']['ffi']['ki_m_per_N_s'],
                 damping_L=rec['meta']['controller_parameters']['L']['cart_d'],
                 opposing_normal_cosine=float(-normals[0]@normals[1]),
                 force_band_rms_N_hand=float(np.std(band[2][keep])),
                 commanded_projected_closure_band_rms_mm_hand=float(np.std(band[0][keep])),
                 measured_projected_closure_band_rms_mm_hand=float(np.std(band[1][keep])),
                 command_to_motion_observational_alignment=bm,
                 motion_to_force_observational_alignment=bf,
                 normal_only_mechanical_model_diagnostic=model_comparisons)
        rows.append(row);traces[rec['id']]=(t,band,prediction_by_contact)
        print(json.dumps(row),flush=True)
    dp.dump('model_observational_consistency.json',dict(
        caveat='Lag and gain fits on correlated closed-loop signals are observational alignment only. No independent excitation, residual prediction or causal identification. Fixed-normal closing coordinate is not direct pad compression. Diagnostic runs differ in preload/force and are interrupted.',
        runs=rows))
    fig,axes=plt.subplots(2,2,figsize=(13,8.5));fig.subplots_adjust(left=.08,right=.98,top=.85,bottom=.17,hspace=.43,wspace=.25)
    fig.suptitle('Prima della scelta del damping: il modello descrive il ritardo?',x=.08,ha='left',y=.97,fontsize=18,weight='bold')
    fig.text(.08,.91,'Dati registrati D=60 · coordinate di chiusura proiettate sulla normale opposta a metà route · banda 0,8–2,5 Hz',fontsize=10)
    for col,role in enumerate(['development','near_zero_I_diagnostic']):
        r=next(r for r in rows if r['piece']=='Z' and r['role']==role);t,b,preds=traces[r['id']]
        fit=r['command_to_motion_observational_alignment'];pred=np.interp(t-fit['delay_s'],t,b[0])*fit['gain']
        keep=(t>=5)&(t<=10)
        axes[0,col].plot(t[keep],b[1][keep],color='#1675bb',lw=1,label='Chiusura misurata')
        axes[0,col].plot(t[keep],pred[keep],color='#dd941f',lw=1,label='Comando allineato + scalato')
        axes[0,col].plot(t[keep],preds[dp.CONTACTS[0]][keep],color='#9564b2',lw=1,ls='--',label='Modello meccanico, k_eff 680')
        axes[0,col].set(title=r['label']+f" · I={r['ki_m_N_s']*1000:g} mm/(N s)",ylabel='Chiusura proiettata [mm/mano]',xlabel='Tempo da inizio route [s]')
        axes[0,col].legend(fontsize=8);axes[0,col].grid(alpha=.2)
    xx=np.arange(len(rows));lags=[r['command_to_motion_observational_alignment']['delay_s']*1000 for r in rows]
    axes[1,0].bar(xx,lags,color=['#1675bb' if r['role']=='development' else '#40a69b' for r in rows])
    axes[1,0].set_xticks(xx,[r['piece']+' '+r['label'].split()[-1] for r in rows],rotation=55,ha='right',fontsize=8)
    axes[1,0].plot(xx,[r['normal_only_mechanical_model_diagnostic'][0]['model_command_to_motion_alignment']['delay_s']*1000 for r in rows],
                   'o-',color='#9564b2',label='Stesso fit sul moto del modello, k_eff 680')
    axes[1,0].set(ylabel='Allineamento comando→moto [ms]',title='Migliore allineamento osservazionale');axes[1,0].legend(fontsize=8);axes[1,0].grid(axis='y',alpha=.2)
    axes[1,1].bar(xx-.18,[r['command_to_motion_observational_alignment']['r2'] for r in rows],width=.35,color='#1675bb',label='Comando scalato e ritardato (fit)')
    axes[1,1].bar(xx+.18,[r['normal_only_mechanical_model_diagnostic'][0]['predicted_motion_vs_recorded_motion_r2'] for r in rows],width=.35,color='#9564b2',label='Modello meccanico senza fit')
    axes[1,1].set_xticks(xx,[r['piece']+' '+r['label'].split()[-1] for r in rows],rotation=55,ha='right',fontsize=8)
    axes[1,1].set(title='Confronto diagnostico: non è validazione causale',ylabel='R² in banda');axes[1,1].grid(axis='y',alpha=.2)
    axes[1,1].legend(fontsize=7)
    fig.text(.08,.025,'Non confrontare questi ritardi con una latenza pura identificata. Il modello URDF è soltanto meccanico e non include il servo reale.\n'
             'Le due prove a I quasi spenta sono diagnostiche e hanno forze/presa diverse; nessuna delle 4 run riservate è usata qui.',fontsize=10,linespacing=1.5)
    fig.savefig(OUT/'04_verifica_modello_vs_log.png',dpi=155);plt.close(fig)


if __name__=='__main__':main()
