"""Evidence and conclusions for the original laboratory controller, including aborts."""
import sys,json,hashlib,ast
from pathlib import Path
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parent))
from analyze_bullet import *
from datetime import datetime,timezone

def main():
    cases=[]
    for folder in sorted((OUT/'runs').glob('Gen3Lab_*')):
        if not (folder/'COMPLETED.json').exists():continue
        d,r=load(folder);loop=np.load(folder/'laboratory_loop_1kHz.npz')['data'];cols=np.load(folder/'laboratory_loop_1kHz.npz')['columns'].tolist()
        idx=lambda prefix:[i for i,x in enumerate(cols) if x.startswith(prefix)]
        warm='warm' in folder.name
        info=json.loads((folder/'laboratory_controller.json').read_text())
        m=dict(run=folder.name,dt_ms=r['dt']*1000,seconds=r['truth']['seconds'],error=r['error'],
            last_phase=str(d['phase'][-1]),ever_route=bool(np.any(d['phase']=='route')),
            events=r['laboratory_events'],planning=r['planning'],
            warmup=info['startup'],all_normal_correction_limits_unchanged=r['ffi'],
            max_abs_Kalman_velocity_rad_s=float(abs(loop[:,idx('dq_kalman')]).max()),
            loop_sample_any_torque_clipped_fraction=float(np.mean(loop[:,cols.index('torque_clipped')])),
            loop_sample_slew_active_fraction=float(np.mean(loop[:,cols.index('slew_active')])) )
        if (folder/'kortex_feedback_1kHz.npz').exists():
            fb=np.load(folder/'kortex_feedback_1kHz.npz');raw=fb['data'];fc=fb['columns'].tolist()
            qd=[i for i,x in enumerate(fc) if x.startswith('dq_raw')]
            m['max_abs_raw_velocity_rad_s']=float(abs(raw[:,qd]).max())
            # Kortex reaction signal is minus the previously applied command.
            errors=[];slew=[]
            for s in [0,1]:
                x=loop[loop[:,1]==s];y=raw[raw[:,1]==s]
                command=x[:,idx('tau_applied')];sensor=y[:,[i for i,c in enumerate(fc) if c.startswith('tau_sensor')]]
                errors.append(float(abs(sensor[1:]+command[:-1]).max()))
                slew.append(float(abs(np.diff(command,axis=0)).max()))
            assert max(errors)<1e-12
            assert max(slew)<=.5000000001
            m['feedback_negative_previous_torque_max_error']=max(errors);m['max_abs_torque_step_Nm']=max(slew)
        cases.append(m)
        fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
        for s,side,col in [(0,'L','#2563eb'),(1,'R','#e87924')]:
            x=loop[loop[:,1]==s];t=x[:,0]*1000
            axes[0,0].plot(t,np.max(abs(x[:,idx('dq_kalman')]),axis=1),color=col,label=side+' Kalman')
            axes[0,0].plot(t,np.max(abs(x[:,idx('dq_RC')]),axis=1),color=col,ls='--',alpha=.7,label=side+' filtro RC')
            if (folder/'kortex_feedback_1kHz.npz').exists():
                y=raw[raw[:,1]==s];axes[0,0].plot(y[:,0]*1000,np.max(abs(y[:,qd]),axis=1),color=col,lw=.8,alpha=.45,label=side+' raw')
            axes[0,1].plot(t,x[:,idx('tau_applied')[-1]],color=col,label=side+' giunto7')
            axes[1,0].plot(d['t']*1000,1000*np.linalg.norm(d[side+'_position']-d[side+'_command'],axis=1),color=col,label=side)
            axes[1,1].plot(d['t']*1000,np.linalg.norm(d[side+'_measured'][:,:3],axis=1),color=col,label=side)
            for event in r['laboratory_events']:
                if event['side']==side:
                    for ax in axes.flat:ax.axvline(event['t']*1000,c=col,ls=':',lw=1)
        axes[0,0].axhline(2,color='.2',ls=':',label='Soglia watchdog sul raw')
        axes[0,0].set_ylabel('Max |velocità giunti| [rad/s]');axes[0,1].set_ylabel('Coppia comandata giunto7 [Nm]')
        axes[1,0].set_ylabel('Errore flangia dal comando [mm]');axes[1,1].set_ylabel('Modulo forza polso simulato [N]')
        for ax in axes.flat:ax.grid(alpha=.2);ax.legend(fontsize=7);ax.set_xlabel('Tempo simulato [ms]')
        fig.suptitle(folder.name+' — legge originale, dinamica URDF non validata\n'
                     'Watchdog prima del pickup; questo non valuta la qualità dell’inserimento reale',fontsize=12)
        fig.savefig(folder/'laboratory_diagnostics.png');plt.close(fig)
    (HERE/'lab_metrics.json').write_text(json.dumps(cases,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(10,5),layout='constrained')
    for label,suffix in [('Kalman appena inizializzato',''),('Kalman dopo2s in posa ferma','_warm')]:
        for number,ls in [(250,'-'),(125,'--')]:
            x=np.load(HERE/f'rigid_pin_hold_{number}{suffix}.npz')['data']
            ax.plot(x[:,0]*1000,np.max(abs(x[:,8:15]),axis=1),ls=ls,
                    label=label+f', dt={number/10:g}µs',lw=1.2)
    ax.axhline(2,c='k',ls=':',lw=1,label='Soglia2rad/s (richiesti20tick)')
    ax.set_xlabel('Tempo simulato [ms]');ax.set_ylabel('Max |velocità giunti| [rad/s]');ax.grid(alpha=.2);ax.legend(fontsize=8)
    ax.set_title('Controllo incrociato: plant rigido URDF in Pinocchio ABA\nLegge originale a1kHz; nessun pad/contatto; perturbazione iniziale10⁻⁷rad al giunto7')
    fig.savefig(HERE/'rigid_pin_hold_comparison.png');plt.close(fig)
    model=json.loads((HERE/'lab_model_checks.json').read_text());checks=model['checks']
    worst_fk=max(x['position_error_m'] for c in checks for x in c['tests'])
    worst_jac=max(x['jacobian_error'] for c in checks for x in c['tests'])
    worst_g=max(x['gravity_error_Nm'] for c in checks for x in c['tests'])
    reference=json.loads((HERE/'user_reference_check.json').read_text())
    assert all(reference['executable_method_AST_equal'].values())
    latest=[x for x in cases if 'warm' in x['run']]
    assert latest and all(not x['ever_route'] for x in latest)
    verified=dict(created_utc=datetime.now(timezone.utc).isoformat(),reference=reference,
        original_core_classes_verbatim=True,model_FK_max_error_m=worst_fk,model_J_max_error=worst_jac,
        model_gravity_max_error_Nm=worst_g,initial_robot_contacts=model['initial_robot_contacts'],
        latest_cases=latest,interpretation='Original controller executes; nominal URDF plant fails stationary hold; no valid insertion verdict')
    (HERE/'lab_verification.json').write_text(json.dumps(verified,indent=2)+'\n')
    lines=['# Impedenza originale del laboratorio — prova autonoma','',
        '**Legge originale integrata ed eseguita. Il plant nominale non supera il mantenimento in aria: '
        'non è ancora un simulatore validato per giudicare l’inserimento reale.**','',
        '## Cosa è identico al riferimento fornito','',
        'La copia trovata nei Download ha2092 righe e SHA-256 `'+reference['original_source_sha256']+'`. '
        'È conservata integralmente in lab_source/kinova_controller_original.py. ControlMode, '
        'angle_diff e le quattro classi matematiche vengono estratti testualmente, senza importare ROS/Kortex. '
        'Il corpo eseguibile di compute_torque e estimate_wrench coincide con il codice VERBATIM '
        'incollato da Lorenzo: confronto AST in user_reference_check.json. Questo conferma i metodi '
        'forniti, mentre l’hash dell’intero file effettivamente sulla NVIDIA resta non verificato.','',
        'Si usa Pinocchio reale4.1.0 per FK, Jacobiano e gravità del modello originale150g. '
        'Stessi K1500/20, D60/7, joint damping0.001, clip35Nm sui primi4 e12Nm sugli ultimi3, '
        'filtro RC100Hz, Kalman con r_q=1e-6,r_dq=2.5e-5,q_ddq=1, slew0.5Nm/tick e '
        'watchdog2rad/s per20tick. Null stiffness0. Friction abilitata ma scale0, come default '
        'non sovrascritto dal launch: contributo nullo. Non si inventano coefficienti per il '
        'file gen3_bare.urdf mancante.','',
        'Il wrapper riproduce feedback in gradi/radianti, torque_sensor=-tau_motor applicata '
        'nel tick precedente, sequenza Kalman→compute_torque→slew e pubblicazione250Hz '
        'di pose e wrench stimato. Le pose usate dall’outer loop sono mantenute fra le '
        'pubblicazioni. Il segnale ATI simulato resta separato dal wrench stimato dalle coppie. '
        'Entrambi sono salvati; nessuno viene sostituito con il target SOCP.','',
        '## Cosa è un modello','',
        'Due Gen3 articolati, nella geometria nominale dei metadata del lab; mesh ufficiali '
        'e URDF locale, gravità, pezzo/vicini dinamici e tri-pad. Centrale2700N/m, tre pad500N/m '
        'ciascuno: totale4200N/m. Tool150g, di cui40g mobili. FFI e SOCP originali, nessun tuning. '
        'Il piano offline è ricalcolato e salvato anche quando il robot simulato abortisce prima del pickup.','',
        'Il nodo ROS intero e il trasporto Kortex non vengono avviati: si eseguono le classi '
        'matematiche e il contratto del loop in tempo virtuale deterministico1kHz. Nessun '
        'jitter/rumore di rete inventato. L’homing di8s e lo spostamento fino alla presa non '
        'sono simulati: il setup inizializza la posa aperta della presa. Si distingue un Kalman '
        'appena inizializzato da una covarianza assestata con2s di feedback costante. '
        'Quest’ultimo controlla l’effetto dell’inizializzazione, senza modificare i gain.','',
        '## Verifiche prima di interpretare il fallimento','',
        f'- 40 pose: massimo errore FK Pin/Bullet {worst_fk:.3g} m; Jacobiano {worst_jac:.3g}; gravità {worst_g:.3g} Nm. La gravità confronta correttamente 110 g rigidi + 40 g nominali con il tool 150 g originale.',
        '- Nessun contatto iniziale dei bracci con altri corpi nella posa di prova.',
        '- Sensore meccanico verificato separatamente in aria e in compressione statica: articulated_sensor_checks.json.',
        '- Nelle prove aggiornate si verifica il segno del feedback di coppia e il massimo incremento 0.5 Nm/tick direttamente dalle tracce.',
        '- estimated_wrench_sign_check.json verifica wrench nullo con torque_sensor=-g e recupero delle sei componenti di un carico esterno noto (errore massimo inferiore a2e-15 nelle rispettive unità).','',
        '## Esito delle prove con la legge originale','',
        '| Caso | dt fisico ms | Fine s | Primo watchdog s | Ultima fase |',
        '|---|---:|---:|---:|---|']
    for x in cases:
        first=min((e['t'] for e in x['events'] if 'watchdog' in e['event']),default=float('nan'))
        lines.append(f"| {x['run']} | {x['dt_ms']:g} | {x['seconds']:.3f} | {first:.3f} | {x['last_phase']} |")
    lines+=['','Le prove aggiornate con filtro assestato si fermano prima del pickup: '
        'il watchdog passa a sola gravità e la sentinella esterna interrompe per errore di posa. '
        'Le run più vecchie conservano inizializzazione del filtro fresca e meno telemetria; '
        'non vengono rietichettate come prove aggiornate. **Nessuna di queste prove misura '
        'RMSE di inserimento, overshoot di inserimento o probabilità di successo.**','',
        'Il confronto indipendente usa il plant rigido dello stesso URDF in Pinocchio ABA, '
        'senza Bullet, pad o contatti. Con perturbazione iniziale di10⁻⁷rad al giunto7, il '
        'watchdog scatta a60ms con Kalman fresco e90ms con covarianza assestata, sia con '
        'dt25µs sia con12.5µs. Questo esclude che basti attribuire il fenomeno ai contatti '
        'dei pad o al solo solver Bullet; non prova che il robot reale abbia tale instabilità.','',
        '![Diagnostica indipendente](rigid_pin_hold_comparison.png)','',
        '## Conclusione utilizzabile','',
        'Il controller originale ora è nel simulatore, ma **stesso controller non rende '
        'automaticamente uguale il plant**. Il modello URDF rigido e la realizzazione ideale '
        'dell’attuatore non riproducono ancora il mantenimento che il robot reale sa fare. '
        'Occorre verificare configurazione effettiva e dinamica degli attuatori, inerzie '
        'riflesse/trasmissione, attrito/damping fisico e filtri interni Kortex. Dai dati di '
        'questa prova non si identifica quale contributo sia responsabile. Non si aggiunge '
        'damping o inerzia arbitraria per far riuscire il grafico.','',
        'Le precedenti inserzioni con impedenza implicita approssimata rimangono confronti '
        'preliminari: Z passa da77.2mm a4.4mm raffinando dt0.5→0.25ms; L ha4.8mm a0.5ms. '
        'Quindi né il primo fallimento Z né la successiva riuscita approssimata possono '
        'essere attribuiti al controller originale o a cambiamenti hardware.','',
        'Dati minimi per proseguire con una previsione affidabile: parametri e versione '
        'effettivi del nodo; breve tratto reale in aria di q,dq,coppia misurata/coppia '
        'comandata e timing; informazioni disponibili su attuatore/trasmissione e filtri. '
        'La prima verifica deve riprodurre il mantenimento e una piccola variazione libera; '
        'solo dopo ha senso valutare oscillazioni e overshoot dell’inserimento.','',
        '## File','',
        '- run_lab_impedance.py; lab_source/; user_reference_check.json; lab_model_checks.json; lab_verification.json.',
        '- ../runs/Gen3Lab_*/: trace.npz/CSV, laboratory_loop_1kHz.npz, robot_used.urdf, sorgente per run, piano e allocazioni SOCP, diagnosi e GIF quando richiesta.',
        '- Prove aggiornate: kortex_feedback_1kHz.npz e node_publish_250Hz.npz, quest’ultimo con wrench stimato in frame EE locale e posa tool pubblicata.',
        '- rigid_pin_hold*.npz/json: diagnostica separata del plant rigido; pin_runtime_provenance.json per il runtime isolato.',
        '- Originali verificati in ../input_integrity_and_profile_parity.json. Nessun ROS, robot, deploy, push o modifica di produzione.','']
    (HERE/'LAB_IMPEDANCE_RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    print(json.dumps(verified,indent=2))
if __name__=='__main__':main()
