"""Generate a source-linked report from the bounded damping experiment outputs."""
import json
from pathlib import Path
import numpy as np

OUT=Path(__file__).resolve().parent


def main():
    load=lambda n:json.loads((OUT/n).read_text())
    theory=load('damping_frequency_summary.json');validation=load('urdf_model_validation.json')
    observed=load('model_observational_consistency.json')['runs'];dev=[r for r in observed if r['role']=='development']
    refine=load('damping_numerical_validation.json');axis=load('damping_axis_summary.json')
    rows=load('damping_closedloop_metrics.json');postures=load('urdf_postures_development.json')
    lags=[r['command_to_motion_observational_alignment']['delay_s']*1000 for r in dev]
    lines=['# Damping interno: risultati della prima verifica fisica', '',
           '**Conclusione provvisoria: non è dimostrato che basti aumentare cart_d.** '
           'Un modello meccanico coerente con URDF e posture consente di studiare il damping, ma non descrive '
           'l’onda e il ritardo delle run. I numeri dello sweep non sono una previsione quantitativa del robot.', '',
           '## Che cosa è stato verificato', '',
           '- Sorgenti del controllo interno `kinova_control/kinova_controller.py` non trovati nella ricerca mirata sul Desktop. '
           'Il repository contiene launch, parametri e URDF, non l’implementazione del servo. '
           'Non si presume che parametri e legge interna coincidano con un altro robot.',
           '- Solo 8 run FFI di sviluppo, 3 istanti per route e 2 bracci = 48 campioni di postura. '
           'Le 4 run riservate non sono state lette da questo studio. Due ulteriori Z a I quasi spenta sono diagnostiche.',
           f"- Cinematica URDF confrontata con le pose misurate dei CSV: massimo errore posizione {max(v['fk_position_error_m'] for v in validation['samples'])*1e6:.2f} µm; "
           f"orientamento {max(v['fk_orientation_error_rad'] for v in validation['samples']):.3g} rad. Questi scarti sono compatibili con pose/quaternioni CSV arrotondati a 5 decimali.",
           f"- Jacobiano geometrico confrontato con differenze finite su 16 posture centrali: errore massimo {max(v['finite_difference_jacobian_max_error'] or 0 for v in validation['samples']):.3g}. "
           'Matrice di massa simmetrica positiva definita in tutte le posture.',
           '- 384 risposte in frequenza: 48 posture × 2 rigidezze di contatto × D=60/80/100/120. '
           'Poi 192 scenari autonomi FFI a due mani e 192 raffinamenti numerici.', '',
           '## Modello fisico dichiarato', '',
           'Matrice articolare M costruita sommando energie cinetiche di link e payload URDF, '
           'Jacobiano J al flange, inerzia cartesiana Λ=(J M⁻¹ Jᵀ)⁻¹. '
           'Il modello conserva gli accoppiamenti delle sei coordinate cartesiane; '
           'non assegna una massa scalare arbitraria a ogni asse.', '',
           '`Λ ẍ + D ẋ + (K + kc b bᵀ)x = K b u`, con `F = kc bᵀ x`.', '',
           'K=diag(1500,1500,1500,20,20,20); D=diag(d,d,d,7,7,7). '
           'b è la normale del tool registrata nella base del rispettivo braccio, con componenti rotazionali nulle. '
           'Contatto normale lineare che agisce lungo l’asse flange-pad, senza rigidezza di piegamento o vincoli tangenziali. '
           'Il pezzo è idealizzato come supporto fisso per ogni braccio. La dinamica del pezzo libero, i contatti con la board '
           'e la flessibilità della molla fuori asse non sono incluse.', '',
           'Due ipotesi di contatto: kc=1243,9 N/m per ottenere k_eff=680 N/m dalla serie con K=1500; '
           'kc=2700 N/m è il valore nominale della molla documentato, che porta k_eff=964,3. '
           'Il primo valore è ricavato dal nominale del controller, **non** una nuova calibrazione indipendente del contatto.', '',
           'Nessuna inerzia riflessa dei rotori, attrito, filtraggio velocità, servo o ritardo hardware è identificata qui. '
           'Il contatto normale lineare può ammettere forze negative: nello sweep le forze rimangono positive, '
           'ma questo non verifica unilateralità/slip o stabilità dei sei pad.', '',
           'La necessità di considerare inerzia, Jacobiano e accoppiamenti nella scelta del damping è trattata nel lavoro primario '
           '[Kao e Saldarriaga, 2021](https://link.springer.com/article/10.1186/s40648-021-00199-0). '
           'Quel lavoro supporta il metodo generale, non identifica i parametri di questo Kinova.', '',
           '## Quanto attenua il solo impianto ideale', '',
           'La tabella riguarda **ampiezza della risposta forzata a un comando sinusoidale di posizione**, '
           'non RMS della forza reale né smorzamento di un ciclo limite autoeccitato. '
           'Intervalli sulle mediane delle posture delle due ipotesi di contatto.', '',
           '| D traslazionale | Attenuazione aggiuntiva a 1 Hz vs D60 | A 2 Hz | Fase equivalente a 1 Hz |',
           '|---|---:|---:|---:|']
    for damping in [60,80,100,120]:
        rr=[r for r in theory if r['damping']==damping]
        a1=[100*(1-r['gain_ratio_vs_D60_1Hz_min_median_max'][1]) for r in rr]
        a2=[100*(1-r['gain_ratio_vs_D60_2Hz_min_median_max'][1]) for r in rr]
        lag=[r['phase_lag_ms_1Hz_min_median_max'][1] for r in rr]
        interval=lambda x:f'{min(x):.2f}–{max(x):.2f}'
        lines.append(f'| {damping} | {interval(a1)}% | {interval(a2)}% | {interval(lag)} ms |')
    lines+=['', 'Il damping riduce la risonanza del modello soprattutto a frequenze più alte. '
            'A 1 Hz D80 ha un effetto piccolo sull’ampiezza forzata. '
            'Il ritardo equivalente è −fase/(2πf), non latenza pura né tempo di salita.', '',
            '![Risposta meccanica](02_damping_modello_fisico.png)', '',
            '## FFI invariato: esperimenti autonomi', '',
            'Si applicano step 4→6→4 N/mano e un disturbo additivo +0,5 N/mano tra 15 e 19 s. '
            'FFI con I=1,6 mm/(N s), nominale k_eff=680, priorità e limiti originali. '
            'Una postura centrale della prima run di sviluppo per ciascuno dei quattro pezzi; '
            'le risposte dei due bracci sono simulate separatamente e sommate nel feedback. '
            'Per ogni coppia: due contatti × inerzia URDF 1×/2× × ritardo aggiunto 0/120/230 ms × quattro D. '
            'Inerzia 2× è stress ipotizzato, non massa misurata; ritardo aggiunto è mantenuto uguale per tutte le D, '
            'senza inventare una mappa D→ritardo. Nessun rumore aggiunto in questo sweep.', '',
            'Esempio Z, contatto k_eff=680, inerzia 1×, ritardo aggiunto 120 ms:', '',
            '| D | Sovraelongazione salita | RMSE, inclusi transitori | Tempo al 90% dallo step |',
            '|---|---:|---:|---:|']
    for r in rows:
        if r['piece']=='Z' and r['contact_stiffness_N_m']<2000 and r['inertia_scale']==1 and r['extra_command_delay_s']==.12:
            lines.append(f"| {r['damping']} | {r['up_overshoot_N_hand']:.4f} N | {r['rmse_N_hand']:.4f} N | {r['rise_to_90_s']:.3f} s |")
    lines+=['', 'Qui più D aumenta leggermente il picco dell’anello FFI: il minore picco risonante del solo impianto '
            'non implica automaticamente minore overshoot dell’intero anello integrale ritardato. '
            'Questo non prova che D80 peggiorerà il robot; dimostra che non si può scegliere D dalla sola intuizione “più damping”. '
            'Le tracce complete e tutti i minimi per mano sono nei CSV/NPZ.', '',
            '![FFI e damping](03_damping_FFI_sintetico.png)', '',
            '## Il limite decisivo: confronto con i dati', '',
            f'Sulle 8 FFI di sviluppo, il migliore allineamento in banda comando→moto è {min(lags):.0f}–{max(lags):.0f} ms. '
            'È un fit osservazionale di ritardo e guadagno, non una latenza causale identificata. '
            'La qualità R² varia circa 0,28–0,75, quindi un ritardo puro non descrive tutto il moto.', '',
            'Sullo stesso segnale di comando, il modello meccanico ideale produce invece un allineamento di circa '
            '15–25 ms. È stato confrontato sia con solo comando normale, sia usando tutti i sei comandi cartesiani registrati '
            '(posizione e piccolo angolo base rispetto alla postura centrale). Nessun guadagno/ritardo è tarato '
            'nel confronto diretto modello→moto misurato.', '',
            '| Ipotesi di contatto | R² modello con comandi completi, min–max sulle 8 run |',
            '|---|---:|']
    for soft in [True,False]:
        vv=[v['full_6D_command_model_motion_r2'] for r in dev for v in r['normal_only_mechanical_model_diagnostic']
            if (v['contact_stiffness_N_m']<2000)==soft]
        lines.append(f"| k_eff {'680' if soft else '964'} N/m | {min(vv):.3f}–{max(vv):.3f} |")
    lines+=['', '**Questo modello è insufficiente come previsore del moto registrato**. '
            'Sono state verificate cinematica e implementazione numerica; non è stata validata la dinamica del robot. '
            'Il confronto diagnostico può essere influenzato dal supporto del pezzo, dalla linearizzazione a postura fissa '
            'e dai contatti omessi: non isola da solo il servo come unica causa.', '',
            'Nelle due Z con I quasi spenta, l’allineamento comando→moto ha R² soltanto 0,09–0,14: '
            'non è sensato usare quei fit per identificare una nuova latenza precisa. '
            'Il moto osservato e la forza restano invece fortemente associati. '
            'La coordinata è la chiusura della coppia proiettata su una normale fissa, '
            '**non** una misura indipendente della compressione dei pad. '
            'Le prove hanno forze iniziali diverse e sono state interrotte; non sono confronti controllati del solo gain.', '',
            '![Modello contro dati](04_verifica_modello_vs_log.png)', '',
            'Quindi la piccola attenuazione forzata prevista per D80 **non esclude** che il damping possa ridurre '
            'un’oscillazione reale autoeccitata, dovuta per esempio a dinamiche o attrito non rappresentati. '
            'Manca il modello di quel meccanismo: trasformare lo sweep in una percentuale di miglioramento reale sarebbe scorretto.', '',
            '## Una variante più mirata da confrontare', '',
            'Nelle 48 posture, la normale di chiusura è quasi parallela all’asse **y della base di ciascun braccio** '
            f"(|n_y| minimo {min(abs(p['normal_base'][1]) for p in postures):.3f}). "
            'La configurazione documenta K/D nelle basi dei rispettivi robot. '
            'Si può quindi confrontare anche `60 80 60 7 7 7` con `80 80 80 7 7 7`, '
            'anziché aumentare automaticamente tutti gli assi. Questa è una proposta di confronto, non una modifica applicata.', '',
            'Nel modello ideale l’aumento solo su y conserva quasi tutta la variazione di risposta normale '
            'dell’aumento isotropo, evitando quasi tutta la fase aggiunta alla risposta verticale. '
            'A 1 Hz: extra ritardo verticale mediano circa '
            f"{axis[0]['vertical_extra_phase_lag_ms_1Hz_min_median_max'][1]:.2f} ms isotropo contro "
            f"{axis[1]['vertical_extra_phase_lag_ms_1Hz_min_median_max'][1]:.4f} ms solo y. "
            'Anche questi sono risultati del modello, non misure reali.', '',
            '## Protocollo minimo ancora necessario', '',
            'Vedere DAMPING_AB_PROTOCOL.md: confronti ripetuti a FFI/K/target invariati, '
            'registrando insieme forza, moto e timestamp. Non è stato eseguito alcun comando al robot. '
            'L’assenza del sorgente interno e di un confronto D-only impedisce una raccomandazione quantitativa affidabile '
            'di un nuovo damping. I dati attuali giustificano un esperimento mirato, non il valore “giusto”.', '',
            '## Verifiche numeriche e riproducibilità', '',
            'Discretizzazione esatta ZOH del sistema lineare a 5 ms, ripetuta a 2,5 ms per tutti i 192 scenari; '
            'tick FFI 25 ms identico. Massima differenza RMSE: '
            f"{refine['max_metric_difference_N']['rmse_N_hand']:.6f} N; overshoot "
            f"{refine['max_metric_difference_N']['up_overshoot_N_hand']:.6f} N. Limite comando 2 mm/s verificato in tutti i casi. "
            'I campioni di forza immediatamente allo step di disturbo sono discontinui per costruzione: '
            'un damping finito non rimuove una discontinuità additiva imposta all’uscita.', '',
            'Sorgenti: rigid_model.py, damping_physics.py, model_consistency.py, damping_axis.py, damping_report.py. '
            'Input fisico URDF con SHA-256 in urdf_model_validation.json; q/posture in urdf_postures_development.json. '
            'Metriche complete damping_frequency_metrics.csv/json, damping_closedloop_metrics.csv/json, '
            'model_observational_consistency.json e damping_axis_metrics.json. '
            'Le figure sono state ispezionate. NVIDIA_RUNS e la baseline conservata sono immutate.', '']
    (OUT/'DAMPING_REPORT.md').write_text('\n'.join(lines),encoding='utf-8')
    print('Wrote DAMPING_REPORT.md',flush=True)


if __name__=='__main__':main()
