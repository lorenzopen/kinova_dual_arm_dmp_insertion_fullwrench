"""Consolidate saved central2700 results and numerical checks, without tuning."""
from analyze_bullet import *
import hashlib
from datetime import datetime,timezone

def main():
    rows=json.loads((OUT/'metrics.json').read_text());by={r['run']:r for r in rows}
    primary=[r for r in rows if '_central2700_' in r['run'] and not r['run'].startswith(('Gen3_','Gen3Lab_'))]
    assert primary,'No completed requested cases'
    checks=[];refinements=[]
    for r in primary:
        folder=OUT/'runs'/r['run'];result=json.loads((folder/'result.json').read_text())
        assert result['central_axial_N_per_m']==2700
        assert result['pad_each_axial_N_per_m']==500
        assert result['pad_axial_N_per_m']==4200
        assert result['mechanical_pad_K'][0]==4200
        assert result['cart_k']==[1500]*3+[20]*3
        assert result['cart_d']==[60]*3+[7]*3
        assert result['ffi']['ki_m_per_N_s']==.0016 and result['feedforward_k_eff']==680
        assert result['spring_damping_reference_dt']==.00025
        assert r.get('max_normal_correction_speed_mm_s',0)<=2.000001
        assert r.get('max_abs_normal_correction_mm',0)<=12.000001
        checks.append(dict(run=r['run'],central2700_and_total4200_verified=True,
                           unchanged_controller_parameters_verified=True,controller_abort=result['error']))
    for piece in 'ZL':
        names=[f'{piece}_central2700_D60_dt05',f'{piece}_central2700_D60_dt025',f'{piece}_central2700_D60_dt0125']
        present=[n for n in names if n in by and by[n]['error'] is None]
        for n0,n1 in zip(present,present[1:]):
            r0=json.loads((OUT/'runs'/n0/'result.json').read_text());r1=json.loads((OUT/'runs'/n1/'result.json').read_text())
            for key in ['mechanical_pad_K','central_C_linear','central_C_rotational','pad_C_linear']:
                assert r0[key]==r1[key],f'Physical {key} changed across refinement'
            a,b=by[n0],by[n1]
            refinements.append(dict(coarse=n0,fine=n1,physical_K_C_identical=True,
                route_rmse_difference_N=abs(a['route_rmse_N']-b['route_rmse_N']),
                force_band_difference_N=abs(a['route_force_band_08_25_N']-b['route_force_band_08_25_N']),
                final_worst_cell_difference_mm=abs(a['final_worst_cell_mm']-b['final_worst_cell_mm'])))
    integrity=json.loads((OUT/'input_integrity_and_profile_parity.json').read_text())
    relevant_profiles=[p for p in integrity['profiles'] if '_central2700_' in p['run']]
    assert all(p['same_rows'] and p['same_phases'] and p['max_abs_numeric_difference']<.001 for p in relevant_profiles)
    qa=dict(created_utc=datetime.now(timezone.utc).isoformat(),primary_parameter_checks=checks,
            refinements=refinements,originals_identical=integrity['NVIDIA_originals_identical'],
            profile_parity=relevant_profiles,scope='Software/parameter/integrity checks, not hardware model validation')
    (OUT/'central2700_verification.json').write_text(json.dumps(qa,indent=2)+'\n')
    lines=['# FFI in PyBullet — centrale 2700 N/m','',
        '**Primo backend flottante. La successiva prova con Gen3 articolati e legge originale del laboratorio è in [gen3/LAB_IMPEDANCE_RESULTS.md](gen3/LAB_IMPEDANCE_RESULTS.md).**','',
        '**Prova fisica autonoma eseguita; nessun nuovo controller o parametro hardware validato.**','',
        'La configurazione richiesta è centrale assiale **2700 N/m**, tre molle pad **500 N/m ciascuna**: '
        'totale assiale del tri-pad **4200 N/m**. Le precedenti prove con totale2700/centrale1200 '
        'sono conservate e superate; non vengono rietichettate.','',
        'Si importa il controller delle run senza modificarlo: FFI Ki0.0016 sul totale delle forze, '
        'k_eff680, trim±3mm, totale±12mm, 2mm/s; pickup I e sequenza completa originali. '
        'K cartesiano1500/20, D60/7, scala/masse/rotte/config dai metadata di sviluppo. '
        'Nessuna forza reale viene fornita al motore.','',
        '## Risultati della configurazione richiesta','',
        'Normale e bande si riferiscono alla **media delle due mani**, non al picco-picco. '
        'RMSE e bande sono calcolate sulla fase route; i transitori di cambio fase sono riportati '
        'nelle figure. La banda della forza comprende anche le variazioni del target: '
        'si conserva separatamente la banda dell’errore.','',
        '| Caso | dt ms | RMSE route N | RMS forza .8–2.5Hz N | RMS errore .8–2.5Hz N | Errore finale cella peggiore mm | seat_found |',
        '|---|---:|---:|---:|---:|---:|---|']
    for r in primary:
        result=json.loads((OUT/'runs'/r['run']/'result.json').read_text())
        if r['error']:
            lines.append(f"| {r['run']} | {r['dt']*1000:g} | abort | — | — | {r['final_worst_cell_mm']:.2f} | — |")
        else:
            lines.append(f"| {r['run']} | {r['dt']*1000:g} | {r['route_rmse_N']:.3f} | {r['route_force_band_08_25_N']:.3f} | {r['route_error_band_08_25_N']:.3f} | {r['final_worst_cell_mm']:.2f} | {result['controller_result'].get('seat_found')} |")
    lines+=['','La posa vera è letta esclusivamente per la valutazione: massima distanza dei quattro '
        'centri cella dal target dopo rilascio e ritirata. Il flag seat_found proviene invece '
        'dalla sentinella basata sul wrench: può dichiarare appoggio senza garantire precisione '
        'geometrica. Un singolo replay nominale non misura una probabilità di successo.','',
        '## Verifica del passo numerico','',
        'I coefficienti di smorzamento delle molle sono congelati al riferimento0.25ms: '
        'si è verificata identità di K e C tra i raffinamenti della serie centrale2700. '
        'Non si cambia damping fisico per far migliorare il risultato.','']
    for r in refinements:
        lines.append(f"- {r['coarse']} → {r['fine']}: ΔRMSE={r['route_rmse_difference_N']:.5f}N, "
                     f"ΔRMSbanda={r['force_band_difference_N']:.5f}N; Δerrore geometrico finale={r['final_worst_cell_difference_mm']:.2f}mm.")
    sensitive=any(r['final_worst_cell_difference_mm']>5 for r in refinements)
    if sensitive:
        lines+=['','**L’esito geometrico è sensibile al passo in almeno un confronto.** '
            'Non si può promuovere quel caso a previsione robusta dell’inserimento reale. '
            'Una forza media simile non garantisce la stessa sequenza di contatti o posa finale.']
    lines+=['','La prova iniziale a1ms con meccanica originale è abortita in aria e rimane conservata. '
        'Le prime prove originale21500 utilizzavano il cap di C legato a dt; non costituiscono '
        'un raffinamento a identica fisica. Il loro cambiamento della posa Z non viene usato '
        'per attribuire una causa hardware.','',
        '## SOCP e wrench ai polsi','',
        'Il SOCP completo è ricalcolato offline come nelle run: input, forze per pad, margini '
        'e residui sono in socp_allocations.json; profilo di entrambi i wrench in profile.csv. '
        'Le statistiche SOCP durante il moto sono zero perché il piano è già risolto. '
        'Il confronto con i profili originali conserva uguali righe e fasi; differenze '
        'numeriche massime di tutti i componenti inferiori a0.001 nelle unità rispettive.','',
        'La misura FT è la reazione delle molle sulla flangia, con gravità del tool rimossa '
        'e inerzia conservata. Sono registrati sei componenti per mano, forze di contatto, '
        'normali dei tre pad, deformazioni, pose e diagnostiche FFI. La misura non è il '
        'wrench richiesto dal SOCP. Il controller originale regola solo la normale: '
        'nessun anello di momento/tangenziale è stato aggiunto.','',
        '## Cosa conclude questo confronto','',
        'La simulazione permette di verificare il codice, la fattibilità del piano e i '
        'transitori nel modello dichiarato. Non dimostra che ogni differenza dal reale '
        'sia dovuta a cambiamenti hardware: il modello non è stato identificato/validato '
        'sui dati reali in questa nuova prova e il servo interno Kinova non è disponibile. '
        'Eventuali oscillazioni o overshoot nominali sono compatibilità del modello, '
        'non ricostruzioni causali certe delle run.','',
        'Restano dichiarati: impedenza cartesiana flottante, non sette giunti Kinova; '
        'masse originali del tool (140g oltre il polso, flangia50g), inerzia di flangia0.05kgm²; '
        'rigidezze laterali e torsionali originali, contatto8000N/m–207Ns/m, pezzi/vicini50g, '
        'nessun rumore o latenza di rete. Centrale2700 non identifica da sola queste proprietà. '
        'Il k_eff del controller rimane680 anche se la serie elastica nominale '
        '1500–4200 dà circa1105N/m prima del contatto.','',
        '## Grafici e riproducibilità','',
        '![Confronto reale e PyBullet](real_vs_bullet_central2700.png)','',
        'Per ogni run: force_tracking.png, wrist_wrench.png (sei componenti locali), '
        'socp_wrench.png (momenti della misura al centro attuale, target al centro nominale), '
        'trace.npz e CSV. La GIF Z è la registrazione del caso a0.5ms, non una previsione '
        'del robot. Tutti i source dei runner sono conservati per run.','',
        'Verifiche: backend_checks_central2700.json; central2700_verification.json; '
        'input_integrity_and_profile_parity.json; runtime_provenance.json. '
        f"Originali verificati SHA-256: {integrity['NVIDIA_files']}file NVIDIA, {integrity['NVIDIA_bytes']}byte, "
        f"{integrity['unchanged_runtime_files']} sorgenti runtime invariati. Baseline ZIP intatto. "
        'Nessun ROS, robot, deploy, push o modifica di produzione.','']
    (OUT/'RESULTS.md').write_text('\n'.join(lines),encoding='utf-8')
    print(json.dumps(qa,indent=2))
if __name__=='__main__':main()
