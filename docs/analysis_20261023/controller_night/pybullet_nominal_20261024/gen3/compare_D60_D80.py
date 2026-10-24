"""Matched D-only comparison of saved articulated trials, no tuning."""
import sys,json,csv,hashlib
from pathlib import Path
HERE=Path(__file__).resolve().parent;OUT=HERE.parent;sys.path.insert(0,str(OUT))
from analyze_bullet import load,metrics,np,plt,Rot,band
from compare_real import metric
DEST=OUT/'damping80_20261024';DEST.mkdir(exist_ok=True)
PAIRS={
 'Z':('Gen3_Z_central2700_implicit_dt025','Gen3_Z_central2700_implicit_D80_dt025'),
 'L':('Gen3_L_central2700_implicit_dt05','Gen3_L_central2700_implicit_D80_dt05'),
 'Originale':('Gen3Lab_Z_central2700_warm_dt025','Gen3Lab_Z_central2700_warm_D80_dt025')}
COL={60:'#1971c2',80:'#e67700'}
ACTIVE=['to_mouth','route','slow_seat','seat_by_feel']
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def timebase(d):return d['t']-d['t'][np.flatnonzero(d['phase']=='to_mouth')[0]]
def response_window(d):
    m=d['phase']=='route';t=d['t'][m];e=abs(d['mean_normal'][m]-d['target'][m])
    for i in range(len(t)):
        if t[i]-t[0]>3:break
        z=(t>=t[i])&(t<t[i]+.2-1e-8)
        if z.sum()>=19 and np.all(e[z]<=.1):return float(t[i]-t[0])
    return None
def main():
    comparisons=[];datasets={};checks=[]
    same=['dt','cart_k','ffi','feedforward_k_eff','pad_axial_N_per_m','central_axial_N_per_m',
        'pad_each_axial_N_per_m','mechanical_pad_K','central_C_linear','central_C_rotational','pad_C_linear',
        'spring_damping_reference_dt','tool_mass','plate_mass','servo_discretization','torque_control_hz']
    for label,(old,new) in PAIRS.items():
        a,ra=load(OUT/'runs'/old);b,rb=load(OUT/'runs'/new)
        for key in same:assert ra[key]==rb[key],(label,key,ra[key],rb[key])
        assert ra['cart_d']==[60]*3+[7]*3 and rb['cart_d']==[80]*3+[7]*3
        assert sha(OUT/'runs'/old/'robot_used.urdf')==sha(OUT/'runs'/new/'robot_used.urdf')
        assert sha(OUT/'runs'/old/'profile.csv')==sha(OUT/'runs'/new/'profile.csv')
        checks.append(dict(case=label,identical_other_parameters=same,identical_robot_urdf=True,identical_offline_profile=True,
            changed_only='cart_d translation60→80, rotation7 unchanged'))
        if label=='Originale':
            for folder in [old,new]:
                c=json.loads((OUT/'runs'/folder/'laboratory_controller.json').read_text())
                if folder==old:previous=c
                else:
                    for key in ['control_hz','joint_damping','max_torque','filter_hz','kalman','tau_slew_per_tick','velocity_watchdog_limit','velocity_watchdog_ticks','friction_scale','null_space_stiffness','startup']:
                        assert c[key]==previous[key],key
            assert sha(OUT/'runs'/old/'controller_math_used.py')==sha(OUT/'runs'/new/'controller_math_used.py')
            comparisons.append(dict(case=label,D60=dict(error=ra['error'],end_s=ra['truth']['seconds'],events=ra['laboratory_events']),
                D80=dict(error=rb['error'],end_s=rb['truth']['seconds'],events=rb['laboratory_events'])))
            datasets[label]=[(a,ra,old),(b,rb,new)];continue
        ma=metrics(a,ra);mb=metrics(b,rb)
        for d,m in [(a,ma),(b,mb)]:
            assert m['max_normal_correction_speed_mm_s']<=2.000001
            assert m['max_abs_normal_correction_mm']<=12.000001
            m['first_route_200ms_within_01N_s']=response_window(d)
            e=d['mean_normal']-d['target'];route=d['phase']=='route';seat=np.isin(d['phase'],['slow_seat','seat_by_feel'])
            m['route_positive_error_peak_N']=float(e[route].max());m['route_negative_error_peak_N']=float(e[route].min())
            m['route_error_band_25_10_N']=band(e[route],2.5,10)
            m['route_error_band_10_40_N']=band(e[route],10,40)
            m['seat_negative_error_peak_N']=float(e[seat].min()) if seat.any() else None
            for s in 'LR':m[s+'_band_08_25_N']=band(d[s+'_normal'][route],.8,2.5)
        differences={k:mb[k]-ma[k] for k in ['route_rmse_N','route_force_band_08_25_N','route_error_band_08_25_N','route_force_band_25_10_N','route_positive_error_peak_N','active_min_L_N','active_min_R_N','final_worst_cell_mm']}
        comparisons.append(dict(case=label,D60=ma,D80=mb,D80_minus_D60=differences))
        datasets[label]=[(a,ra,old),(b,rb,new)]
    fig,axes=plt.subplots(2,2,figsize=(14,8),layout='constrained')
    for i,piece in enumerate('ZL'):
        for j,s in enumerate('LR'):
            ax=axes[i,j]
            for damping,(d,r,name) in zip([60,80],datasets[piece]):
                t=timebase(d);mask=np.isin(d['phase'],ACTIVE)
                ax.plot(t[mask],d[s+'_normal'][mask],c=COL[damping],lw=.9,label='D '+str(damping))
                ax.plot(t[mask],d['target'][mask],c='.25' if damping==60 else '.65',ls='--' if damping==60 else ':',lw=.9,label='SOCP, tempi D'+str(damping))
            ax.set_title(piece+' — mano '+s);ax.set_ylabel('Forza normale [N]');ax.set_xlabel('Tempo dalla partenza [s]')
            ax.grid(alpha=.2);ax.legend(fontsize=8,ncol=2)
    fig.suptitle('Stesso inserimento nei Gen3: damping traslazionale 60 → 80 Ns/m\n'
        'D rotazionale7 invariato; K1500/20, centrale2700, tre pad500, FFI/SOCP invariati.\n'
        'Simulazione completa con impedenza interna approssimata; confronto a stesso passo fisico per ciascun pezzo.',fontsize=12)
    fig.savefig(DEST/'01_D60_D80_forze.png');plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(14,8),layout='constrained')
    for i,piece in enumerate('ZL'):
        for damping,(d,r,name) in zip([60,80],datasets[piece]):
            t=timebase(d);mask=np.isin(d['phase'],ACTIVE)
            axes[i,0].plot(t[mask],(d['mean_normal']-d['target'])[mask],c=COL[damping],lw=.9,label='D'+str(damping))
            axes[i,1].plot(t[mask],np.maximum(np.linalg.norm(d['L_measured'][mask,3:],axis=1),np.linalg.norm(d['R_measured'][mask,3:],axis=1)),c=COL[damping],lw=.9,label='D'+str(damping))
        axes[i,0].axhline(0,c='.4',lw=.7);axes[i,0].set_ylabel(piece+' — errore medio [N]');axes[i,1].set_ylabel(piece+' — max momento fra i polsi [Nm]')
        for ax in axes[i]:ax.grid(alpha=.2);ax.legend();ax.set_xlabel('Tempo dalla partenza [s]')
    fig.suptitle('Effetto del damping su errore di forza e momenti\nStesso modello approssimato, stessi limiti: nessun filtro aggiuntivo nei grafici.',fontsize=12)
    fig.savefig(DEST/'02_D60_D80_errori_coppie.png');plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True,layout='constrained')
    for damping,(d,r,name) in zip([60,80],datasets['Z']):
        t=timebase(d);m=(t>=12)&(t<=17)
        axes[0].plot(t[m],d['mean_normal'][m],c=COL[damping],lw=1,label='D'+str(damping))
        axes[1].plot(t[m],(d['mean_normal']-d['target'])[m],c=COL[damping],lw=1,label='D'+str(damping))
    d=datasets['Z'][0][0];t=timebase(d);m=(t>=12)&(t<=17);axes[0].plot(t[m],d['target'][m],'k--',lw=1,label='SOCP D60')
    axes[0].set_ylabel('Forza media mani [N]');axes[1].set_ylabel('Errore sul proprio target [N]');axes[1].set_xlabel('Tempo dalla partenza [s]')
    for ax in axes:ax.grid(alpha=.2);ax.legend();ax.set_xlim(12,17)
    fig.suptitle('Zoom Z: D60 contro D80, impedenza approssimata\nIl damping può modificare la sequenza dei contatti: non è solo un filtro della stessa curva.',fontsize=12)
    fig.savefig(DEST/'03_D60_D80_zoom_Z.png');plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True,layout='constrained')
    for damping,(d,r,name) in zip([60,80],datasets['Originale']):
        fb=np.load(OUT/'runs'/name/'kortex_feedback_1kHz.npz');x=fb['data'];cols=fb['columns'].tolist()
        vel=[i for i,c in enumerate(cols) if c.startswith('dq_raw')]
        left=x[x[:,1]==0];right=x[x[:,1]==1]
        axes[0].plot(left[:,0]*1000,np.max(abs(left[:,vel]),axis=1),c=COL[damping],label='D'+str(damping)+' L')
        axes[0].plot(right[:,0]*1000,np.max(abs(right[:,vel]),axis=1),c=COL[damping],ls='--',label='D'+str(damping)+' R')
        axes[1].plot(d['t']*1000,np.maximum(np.linalg.norm(d['L_measured'][:,:3],axis=1),np.linalg.norm(d['R_measured'][:,:3],axis=1)),c=COL[damping],label='D'+str(damping))
    axes[0].axhline(2,c='.3',ls=':',label='Watchdog:2rad/s per20tick')
    axes[0].set_ylabel('Max |velocità giunti| [rad/s]');axes[1].set_ylabel('Max forza fra i polsi [N]');axes[1].set_xlabel('Tempo simulato [ms]')
    for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=8,ncol=2)
    fig.suptitle('Legge originale del laboratorio: D60 e D80 nel plant URDF\nConfronto separato dalle inserzioni con impedenza approssimata; mostra fin dove arriva la prova.',fontsize=12)
    fig.savefig(DEST/'04_originale_D60_D80.png');plt.close(fig)
    result=dict(comparisons=comparisons,matched_checks=checks,
        interpretation='D-only change at identical physics/servo/FFI limits and offline profile. Full approximate-model insertion and original-source early trial kept separate.')
    (DEST/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    with (DEST/'metrics.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['piece','D','RMSE_N','RMS_force_08_25_N','RMS_error_08_25_N','RMS_force_25_10_N','route_overshoot_N','seat_overshoot_N','final_worst_cell_mm'])
        for x in comparisons[:2]:
            for damping in [60,80]:
                m=x['D'+str(damping)];w.writerow([x['case'],damping,*[m.get(k) for k in ['route_rmse_N','route_force_band_08_25_N','route_error_band_08_25_N','route_force_band_25_10_N','route_positive_error_peak_N','seat_positive_error_N','final_worst_cell_mm']]])
    lines=['# Damping60 → 80 nei Gen3 simulati','',
        'Si cambia solo D traslazionale da60 a80Ns/m. D rotazionale7, K1500/20, centrale2700 + tre pad500, '
        'FFI, limiti2mm/s e±12mm, masse, rigidezze/smorzamenti meccanici e passo fisico rimangono identici '
        'nel confronto di ciascun pezzo. Verificati byte identici di URDF e profilo SOCP offline.','',
        'Le inserzioni complete usano lo stesso servo implicito approssimato già mostrato. '
        'Separatamente si cambia D anche nella legge originale del laboratorio, senza modificarne le classi.','',
        '**Esito: D80 non è risolutivo nel modello. RMSE, banda lenta e overshoot restano quasi uguali.** '
        'La banda della forza 2.5–10 Hz diminuisce del '
        f"{100*(1-comparisons[0]['D80']['route_force_band_25_10_N']/comparisons[0]['D60']['route_force_band_25_10_N']):.1f}% sulla Z e del "
        f"{100*(1-comparisons[1]['D80']['route_force_band_25_10_N']/comparisons[1]['D60']['route_force_band_25_10_N']):.1f}% sulla L. "
        'La legge originale continua a fermarsi a0.12s nel plant URDF; l’aumento traslazionale non elimina quello stop.','',
        '| Pezzo | D | RMSE route N | RMS forza0.8–2.5Hz N | RMS errore0.8–2.5Hz N | Overshoot route N | Overshoot seduta N | Errore finale mm |',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for x in comparisons[:2]:
        for damping in [60,80]:
            m=x['D'+str(damping)];lines.append(f"| {x['case']} | {damping} | {m['route_rmse_N']:.3f} | {m['route_force_band_08_25_N']:.3f} | {m['route_error_band_08_25_N']:.3f} | {m['route_positive_error_peak_N']:.3f} | {m['seat_positive_error_N']:.3f} | {m['final_worst_cell_mm']:.2f} |")
    lines+=['','Le bande sono RMS della media delle mani, non picco-picco. La banda della forza '
        'contiene anche variazioni del riferimento: per questo si riporta la banda dell’errore. '
        'L’errore finale è la peggiore cella dopo rilascio/ritirata, non il flag seat_found. '
        'Il tempo della prima finestra200ms entro±0.1N dopo l’inizio rotta è in results.json: '
        'è un descrittore del transitorio su target variabile, non un tempo di assestamento da step ideale.','',
        '## Codice originale','']
    for damping in [60,80]:
        x=comparisons[2]['D'+str(damping)];lines.append(f"- D{damping}: stop a{x['end_s']:.3f}s, {x['error']}; eventi{json.dumps(x['events'])}.")
    lines+=['','Questo confronto non modifica il damping rotazionale e non identifica i limiti '
        'del plant URDF. Nessuna conclusione automatica sul robot reale.','',
        '![Forze](01_D60_D80_forze.png)','', '![Errori e momenti](02_D60_D80_errori_coppie.png)','',
        '![Zoom](03_D60_D80_zoom_Z.png)','', '![Originale](04_originale_D60_D80.png)','']
    (DEST/'RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    manifest=[dict(path=p.name,sha256=sha(p)) for p in sorted(DEST.iterdir()) if p.is_file() and p.name!='SHA256.json']
    (DEST/'SHA256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
