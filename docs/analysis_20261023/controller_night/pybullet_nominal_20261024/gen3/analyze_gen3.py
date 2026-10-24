"""Report saved articulated trials; no model fitting or controller tuning."""
import sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parent))
from analyze_bullet import *
from compare_real import real,metric

def main():
    cases=[]
    for folder in sorted((OUT/'runs').glob('Gen3_*')):
        if not (folder/'COMPLETED.json').exists():continue
        d,r=load(folder)
        if not np.any(d['phase']=='route'):continue
        jt=np.load(folder/'joint_telemetry.npz');j=jt['data']
        diagnostics=r['lab_scene_checks'];limits=np.array([x['torque_limits'] for x in diagnostics])
        q=np.stack([j[:,1:8],j[:,22:29]],axis=1)
        dq=np.stack([j[:,8:15],j[:,29:36]],axis=1)
        tau=np.stack([j[:,15:22],j[:,36:43]],axis=1)
        ratios=abs(tau)/limits[None,:,:]
        m=metrics(d,r)
        summary=dict(run=folder.name,**m,dt_ms=r['dt']*1000,
            servo=r['servo_discretization'],planning=r['planning'],
            max_abs_joint_velocity_rad_s=float(abs(dq).max()),
            max_effort_fraction=float(ratios.max()),
            stream_sample_any_joint_saturated_fraction=float(np.mean(np.any(ratios>=.999,axis=(1,2)))),
            telemetry_note='Joint saturation sampled at100Hz; not every1kHz command',
            max_recorded_FK_error_m=max(x['recorded_tool_position_error_m'] for x in diagnostics))
        cases.append(summary)
        fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True,layout='constrained')
        for side,s in enumerate('LR'):
            for k in range(7):
                axes[0,side].plot(j[:,0],np.degrees(q[:,side,k]),lw=.7,label=str(k+1))
                axes[1,side].plot(j[:,0],dq[:,side,k],lw=.7)
                axes[2,side].plot(j[:,0],tau[:,side,k],lw=.7)
            axes[0,side].set_title('Gen3 '+s);axes[0,side].legend(ncol=7,fontsize=7)
            axes[0,side].set_ylabel('Giunti [deg]');axes[1,side].set_ylabel('Velocità [rad/s]')
            axes[2,side].set_ylabel('Coppia comandata [Nm]');axes[2,side].set_xlabel('Tempo simulato [s]')
        for ax in axes.flat:ax.grid(alpha=.2)
        fig.suptitle(folder.name+' — 14 giunti articolati, impedenza interna dichiarata')
        fig.savefig(folder/'joints.png');plt.close(fig)
    savepath=HERE/'metrics.json';savepath.write_text(json.dumps(cases,indent=2)+'\n')
    primary=[c for c in cases if c['run'].endswith('implicit_dt05')]
    if not primary:return
    fig,axes=plt.subplots(len(primary),2,figsize=(14,4*len(primary)),squeeze=False,layout='constrained')
    comparisons=[]
    for row,c in enumerate(primary):
        folder=OUT/'runs'/c['run'];sim,r=load(folder);piece=c['run'].split('_')[1]
        observed,path=real(piece)
        comparisons.append(dict(run=c['run'],real_source=str(path.relative_to(ROOT)),real=metric(observed),simulation=metric(sim)))
        for ax,d,title in zip(axes[row],[observed,sim],['Run reale di sviluppo','PyBullet: due Gen3 + FFI originale']):
            start=np.flatnonzero(d['phase']=='to_mouth')[0];t=d['t']-d['t'][start]
            active=np.isin(d['phase'],['to_mouth','route','slow_seat','seat_by_feel'])
            for s,col in [('L','#2563eb'),('R','#e87924')]:ax.plot(t[active],d[s+'_normal'][active],c=col,lw=.8,label='Mano '+s)
            ax.plot(t[active],d['target'][active],'k--',lw=1.1,label='SOCP / mano')
            ax.set_title(piece+' — '+title);ax.set_ylabel('Forza normale [N]');ax.set_xlabel('Tempo dalla partenza [s]')
            ax.grid(alpha=.2);ax.legend(ncol=3,fontsize=8)
    fig.suptitle('Confronto descrittivo, senza replay delle forze\nMolla centrale2700 + tre pad500 =4200N/m. Servo interno simulato, contatti non calibrati.')
    fig.savefig(HERE/'real_vs_gen3.png');plt.close(fig)
    (HERE/'real_vs_gen3_metrics.json').write_text(json.dumps(comparisons,indent=2)+'\n')
    refinement=[]
    by={c['run']:c for c in cases}
    for coarse in primary:
        fine=by.get(coarse['run'].replace('dt05','dt025'))
        if not fine:continue
        a=json.loads((OUT/'runs'/coarse['run']/'result.json').read_text())
        b=json.loads((OUT/'runs'/fine['run']/'result.json').read_text())
        keys=['mechanical_pad_K','central_C_linear','central_C_rotational','pad_C_linear','cart_k','cart_d','tool_mass','plate_mass','servo_discretization','torque_control_hz']
        assert all(a[k]==b[k] for k in keys),'Physical/servo parameter changed across refinement'
        refinement.append(dict(coarse=coarse['run'],fine=fine['run'],same_physical_and_servo_parameters=True,
            delta_route_rmse_N=abs(coarse['route_rmse_N']-fine['route_rmse_N']),
            delta_final_worst_cell_mm=abs(coarse['final_worst_cell_mm']-fine['final_worst_cell_mm'])))
    (HERE/'refinement.json').write_text(json.dumps(refinement,indent=2)+'\n')
    lines=['# Due Kinova Gen3 in PyBullet, molla centrale2700','',
        '**Archivio preliminare con impedenza approssimata. Per la legge originale fornita da Lorenzo leggere [LAB_IMPEDANCE_RESULTS.md](LAB_IMPEDANCE_RESULTS.md).**','',
        'Eseguito il controller esterno FFI originale, con piano SOCP offline ricalcolato, '
        'su due Gen3 articolati e contatti dinamici. Questa è una simulazione autonoma, non un replay delle forze.','',
        '## Geometria e parametri','',
        'Mesh ufficiali Kinova, revisione fissata in mesh_provenance.json; cinematica e inerzie '
        'da urdf/gen3_ati_tool.urdf, copia derivata locale. Le basi sono poste nel frame scaffale '
        'con le shelf_in_base delle run: L=(0.7525,-0.3025,0.030)m, R=(0.7525,0.2975,0.030)m; '
        'distanza0.600m e rotazione180° attorno a z. Scaffale, pezzo, vicini, grasp e rotta '
        'provengono dalla configurazione nominale. Non è stata eseguita una nuova calibrazione fisica del lab.','',
        'FK indipendente e posizione tool della prima riga reale sono controllate prima della '
        'simulazione; gli angoli reali iniziali servono come seed IK, non come traiettoria imposta. '
        'Dopo inizializzazione i giunti evolvono sotto coppie, gravità e collisioni.','',
        'Centrale assiale2700N/m, tre pad500N/m ciascuno: totale4200N/m. Massa tool150g '
        '(110g rigidi +40g piastra/pad), come metadata; inerzia del supporto ancora approssimata. '
        'Altre rigidezze e smorzamenti del modello originale, C fissato al riferimento0.25ms. '
        'Contatto nominale8000N/m,207Ns/m; attrito pad-pezzo1. Proprietà non identificate sul banco.','',
        'FFI Ki0.0016m/(Ns) sull’errore sommato, k_eff680N/m per mano, trim±3mm, '
        'totale±12mm, velocità2mm/s; pickup, sentinelle e sequenza originali. K cartesiano1500/20 '
        'e D60/7; nessun tuning o anello esterno PD aggiunto.','',
        '## Servo interno: limite importante','',
        'Queste prove sono state costruite prima di trovare il codice del servo nei Download. Si realizza un’impedenza '
        'cartesiana nominale in coppia, a1kHz, con gravità/Coriolis, limiti di coppia URDF '
        'e damping nullo proiettato0.2. La discretizzazione localmente implicita include '
        'la reazione FT corrente nel predittore di accelerazione; non usa forza futura. '
        'I gain continui restano invariati, ma questa realizzazione non è una replica verificata '
        'del firmware reale.','',
        'La prima realizzazione esplicita è abortita in aria: piccole inerzie dei polsi e '
        'D rotazionale7 hanno prodotto instabilità numerica e saturazione. Il fallimento, '
        'il probe e i sorgenti sono conservati; non è un risultato del robot. La variante '
        'implicita è stata prima verificata in posa ferma e poi sull’inserimento completo.','',
        '## Risultati','',
        'RMSE e bande sono della forza media delle due mani nella fase route. Il flag '
        'seat_found è la sentinella del controller; l’errore geometrico usa invece la posa '
        'vera dopo rilascio e ritirata, mai fornita al controller.','',
        '| Caso | dt ms | RMSE N | RMS forza0.8–2.5Hz N | RMS errore0.8–2.5Hz N | Errore cella peggiore mm |',
        '|---|---:|---:|---:|---:|---:|']
    for c in cases:
        lines.append(f"| {c['run']} | {c['dt_ms']:g} | {c.get('route_rmse_N',float('nan')):.3f} | {c.get('route_force_band_08_25_N',float('nan')):.3f} | {c.get('route_error_band_08_25_N',float('nan')):.3f} | {c['final_worst_cell_mm']:.2f} |")
    lines+=['','Raffinamento a identici K/C, masse, servo e frequenza di controllo:','']
    for x in refinement:lines.append(f"- {x['coarse']} → {x['fine']}: ΔRMSE{x['delta_route_rmse_N']:.4f}N, Δerrore geometrico{x['delta_final_worst_cell_mm']:.2f}mm.")
    if not refinement:lines.append('Non ancora disponibile: non si dichiara convergenza numerica.')
    lines+=['','## Misure e interpretazione','',
        'Un giunto fisso FT misura la reazione trasmessa dalle molle al supporto al polso. '
        'Si rimuove la gravità dei40g mobili e si conserva la loro inerzia; il supporto rigido '
        'a monte del giunto sensore contribuisce alla dinamica del robot ma non alla misura FT. '
        'Questo non riproduce esattamente la collocazione e ogni inerzia dell’ATI fisico. '
        'Sono conservati sei componenti per mano a1kHz, blocchi mediati5ms e misura usata '
        'dal controller (ultimi due blocchi); contatti/pad sono registrati separatamente.','',
        'Il SOCP alloca il wrench completo; il controller FFI delle run controlla solo '
        'la normale. Le coppie e forze tangenziali rimangono misure, non target inseguiti '
        'da un nuovo regolatore. Zero solve durante il moto significa SOCP offline già eseguito.','',
        'La presenza di oscillazioni o di errori geometrici in questo modello dimostra '
        'che possono emergere anche senza rumore ATI o cambiamenti hardware. La loro '
        'assenza non dimostrerebbe che il reale deve essere regolare: servo, contatti, '
        'inerzie e latenza restano da validare. Il confronto reale/simulato è descrittivo '
        'e non attribuisce una causa certa all’hardware.','',
        '![Setup](../runs/'+primary[0]['run']+'/lab_setup.png)','',
        '![Reale e Gen3 simulati](real_vs_gen3.png)','',
        'Per ogni caso: force_tracking.png, wrist_wrench.png, socp_wrench.png, joints.png, '
        'trace.npz/CSV, joint_telemetry.npz, profile.csv, socp_allocations.json, robot_used.urdf '
        'e runner_source.py. GIF nel caso Z a0.5ms. Verifiche del sensore in '
        'articulated_sensor_checks.json; integrità originali in ../input_integrity_and_profile_parity.json.','']
    (HERE/'RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    print(json.dumps(dict(cases=cases,refinement=refinement),indent=2))

if __name__=='__main__':main()
