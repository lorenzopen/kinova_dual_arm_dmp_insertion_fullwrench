"""Report empirical cross-run checks and conditional controller screening."""
import json
from pathlib import Path
import numpy as np

OUT=Path(__file__).resolve().parent


def main():
    load=lambda name:json.loads((OUT/name).read_text())
    empirical=load('empirical_crossrun_summary.json');design=load('empirical_design.json')
    actual=load('empirical_crossrun_metrics.json');known=load('empirical_software_validation.json')
    conditional=load('conditional_controller_metrics.json');ranking=load('conditional_controller_ranking.json')
    shortlist=load('conditional_shortlist_development_only.json')['shortlist']
    checks=load('conditional_replay_validation.json');numerical=load('conditional_numerical_refinement.json')
    baseline=next(r for r in ranking if r['id']=='I_P0_I1.6_D0')
    lines=['# Modelli empirici e confronto dei controllori — risultati di sviluppo', '',
           '**Nessun nuovo controllore è ancora validato sul robot.** '
           'I modelli empirici migliorano la descrizione rispetto al solo URDF, ma non ricreano la maggior parte '
           'dell’onda partendo dai comandi. Nel replay condizionato, la FFI attuale e un PI più lento risultano '
           'praticamente alla pari nel criterio robusto; P/PD da soli lasciano offset importanti. '
           'Nessuna famiglia soddisfa tutti gli obiettivi provvisori nelle ipotesi esplorate.', '',
           '## 1. Capacità di descrivere una run usando le altre', '',
           'Si usano le otto run di sviluppo già congelate: due Z, due L, due S, due T. '
           'In ogni fold si stimano parametri sulle altre sette e si calcola l’ottava senza aggiustarne '
           'guadagno o ritardo. Quattro run riservate non sono usate in questo studio. '
           'La cross-validation è retrospettiva, nello stesso regime FFI; le run non sono esperimenti con '
           'eccitazione indipendente né un campione di tutti i tentativi.', '',
           'Il comando e il moto sono la chiusura della coppia proiettata su una normale opposta fissa, '
           'definita dalla **prima** posa osservata della route. Non si usa la posa futura di metà route '
           'per definire il sistema di coordinate di questo studio. Non si tratta di una misura diretta '
           'della compressione dei singoli pad.', '',
           'Campionamento ZOH causale a 100 Hz; filtro causale Butterworth ordine 2, banda 0,3–8 Hz, '
           'stato iniziale zero dopo sottrazione del primo campione. Valutazione da 3 s a fine route meno 0,5 s. '
           'Questa banda e questa coordinata differiscono dalla verifica meccanica precedente: '
           'i due R² non vanno confrontati come fossero la medesima prova.', '',
           f"Provati {len(design['models'])} modelli LTI a ritardo/polo/due poli e 54 configurazioni ARX a due poli con regolarizzazione. "
           'Ogni famiglia è selezionata per errore di **simulazione libera** sui dati di stima, con peso '
           'normalizzato uguale per run. Poli ARX instabili sono esclusi. Non si seleziona sul punteggio '
           'della run di verifica. Il rapporto forza/moto è stimato solo sulle sette run di training e '
           'poi applicato alla run di verifica.', '',
           '| Famiglia | R² moto libero: min / mediana / max | R² forza da comandi: min / mediana / max | RMSE forza da comandi, mediana |',
           '|---|---:|---:|---:|']
    triple=lambda values:' / '.join(f'{v:.3f}' for v in values)
    for row in empirical:
        lines.append(f"| {row['family']} | {triple(row['motion_free_r2_min_median_max'])} | {triple(row['force_free_r2_min_median_max'])} | {row['force_free_rmse_min_median_max'][1]:.3f} N/mano |")
    constit=empirical[0]['force_given_measured_motion_r2_min_median_max']
    lines+=['', 'Tutti i modelli falliscono la soglia esplorativa dichiarata prima dello studio: mediana R² della '
            'forza ≥0,7 e peggiore run ≥0,5. Questa soglia è un filtro di qualità provvisorio della ricerca, '
            'non un criterio di accettazione della tesi.', '',
            f"Se invece si fornisce il **moto già misurato**, il rapporto forza/moto della componente dinamica dà R² "
            f"{constit[0]:.3f}–{constit[2]:.3f}, mediana {constit[1]:.3f}. "
            'Questo sostiene l’associazione della variazione di forza con il moto reale e la deformazione, '
            'ma non identifica quale meccanismo generi quel moto. Non dimostra che il rumore ATI sia nullo, '
            'né che la dinamica dei sei pad sia scalare.', '',
            '![Cross-validation tra run](05_modelli_empirici_crossrun.png)', '',
            '### Perché il fit a un passo può ingannare', '']
    arx=next(row for row in empirical if row['family']=='ARX2')
    lines+=[f"ARX: mediana R² del moto **a un passo** {arx['motion_one_step_r2_min_median_max'][1]:.3f}, "
            f"contro {arx['motion_free_r2_min_median_max'][1]:.3f} in **simulazione libera**. "
            'Il primo usa il moto misurato passato a ogni campione; il secondo usa solo il comando e lo stato iniziale. '
            'Un segnale molto autocorrelato può quindi essere previsto bene a 10 ms senza avere un buon modello '
            'della risposta al comando.', '',
            'La distinzione tra predizione con misure passate e simulazione con soli ingressi è spiegata nella '
            '[documentazione primaria MathWorks](https://www.mathworks.com/help/ident/ug/definition-simulation-and-prediction.html). '
            'Una buona verifica nello stesso regime non dimostrerebbe comunque la risposta causale a un nuovo controllore.', '',
            '![Predizione e simulazione](06_predizione_vs_simulazione.png)', '',
            '### Verifiche software', '',
            f"Su un sistema noto generato separatamente, ARX recupera il ritardo 8 campioni e i coefficienti "
            f"con errore massimo {known['max_coefficient_error']:.3g}; simulazione su una nuova sequenza di ingresso R²=1. "
            'Sono verificati anche i prefissi causali: alterare i campioni futuri non modifica il passato del filtro '
            'o dei 316 modelli. Queste verifiche controllano il software, **non validano il robot**.', '',
            '## 2. Cambiare il controllore senza fingere che il modello sia già validato', '',
            'Eseguiti **5.888 replay condizionati**: 8 run × 46 candidati × 16 modelli ipotizzati. '
            'Dato il fallimento della verifica predittiva, questo è un confronto di sensibilità sotto ipotesi esplicite, '
            'non una previsione della forza che il robot produrrà.', '',
            '`F_alt_sum = F_registrata_sum + G_modello[s_alt − s_FFI_registrata]`.', '',
            'Il residuo registrato resta fisso anche quando cambia il controllore. Il modello locale del delta-comando '
            'esplora k=480/680/964/1500 N/m/mano e ritardi 120/180/230 ms; 1500 è uno stress di guadagno, '
            'non una nuova rigidezza identificata. Si aggiungono due primi ordini e due modelli risonanti ipotetici. '
            'Questi scenari non rappresentano cambiamenti del parametro fisico cart_d.', '',
            'Target e feedforward originali, I su errore somma, trim ±3 mm, totale ±12 mm, velocità trim/totale '
            '2 mm/s e priorità al nominale sono conservati. P e D sono correzioni assolute di posizione, '
            'non incrementi accumulati; P/PD non acquisiscono una I nascosta con l’anti-windup. '
            'D sulla forza somma misurata con filtro causale da 50 ms. L’inizializzazione mantiene lo stesso '
            'FF/trim/output registrato, con bias fisso per P/PD e I inizializzata senza salto per PI/PID.', '',
            'Il replay della misura tra tick è ZOH, senza anticipare il campione successivo. '
            'I e limiti usano ffi_dt_ms registrato, già limitato; D usa gli intervalli effettivi del clock. '
            'Il segnale D ha però il rate delle letture FFI, non tutti i messaggi ATI 200 Hz: '
            'la sensibilità ad alta frequenza deve ancora essere stressata separatamente.', '',
            '### Risultato robusto sullo spazio iniziale di parametri', '',
            'Punteggio dichiarato prima del calcolo: media sulle 8 run del peggior modello della somma '
            '`RMSEroute/0,2 + RMSbanda/0,05 + maxerrorepositivo_primo_seat/0,25 + p95errore_route/0,5`. '
            'Se la forza media delle mani diventa ≤0, il caso viene penalizzato. Questo non verifica il contatto '
            'su ciascuna mano. I pesi riflettono obiettivi esplorativi, non un ottimo universale.', '',
            '| Famiglia | P [mm/N] | I [mm/(N s)] | D [mm s/N] | Punteggio robusto | Variazione vs FFI |',
            '|---|---:|---:|---:|---:|---:|']
    for row in shortlist:
        ratio=(row['mean_run_worst_model_score']/baseline['mean_run_worst_model_score']-1)*100
        lines.append(f"| {row['family']} | {row['kp_m_N']*1000:g} | {row['ki_m_N_s']*1000:g} | {row['kd_m_s_N']*1000:g} | {row['mean_run_worst_model_score']:.4f} | {ratio:+.2f}% |")
    lines+=['', '**FFI attuale e PI sono praticamente alla pari**: la differenza del punteggio è circa 0,03%, '
            'troppo piccola per una scelta robusta rispetto a incertezza di modello e discretizzazione. '
            'PD senza I non elimina il compromesso di offset/forza minima. '
            'La griglia iniziale ha P fino a 0,2 mm/N e D fino a 0,015 mm s/N: '
            'non esclude compensatori con anticipo più forte e roll-off, da studiare come fase separata.', '',
            'Nel modello centrale (680 N/m/mano e 180 ms), medie sulle 8 run per route e sulle due L per il picco finale:', '',
            '| Famiglia | RMS forza 0,8–2,5 Hz | RMS forza 2,5–10 Hz | RMSE forza–target route | Picco finale L |',
            '|---|---:|---:|---:|---:|']
    for row in shortlist:
        rr=[r for r in conditional if r['candidate']==row['id'] and r['model']=='delay_k680_d0.18']
        vals=[float(np.mean([r[key] for r in rr])) for key in ['route_band_rms_N','route_high_band_rms_N','route_rmse_N']]
        peak=float(np.mean([r['first_seat_peak_N'] for r in rr if r['piece']=='L']))
        lines.append(f"| {row['family']} | {vals[0]:.3f} N | {vals[1]:.3f} N | {vals[2]:.3f} N | {peak:.3f} N |")
    satisfying=[]
    for candidate in ranking:
        rr=[r for r in conditional if r['candidate']==candidate['id']]
        if all(r['route_band_rms_N']<=.05 and r['route_rmse_N']<=.2 and r['first_seat_max_positive_error_N']<=.25 for r in rr):
            satisfying.append(candidate['id'])
    lines+=['', f"Candidati che soddisfano **tutte** le soglie provvisorie su **tutte** le run/modelli: {len(satisfying)}. "
            'Questo esito riguarda questo insieme di ipotesi e parametri; non dimostra impossibilità di ogni altra legge.', '',
            '![Replay condizionato](07_controllori_replay_condizionato.png)', '',
            '### Che cosa il replay non può dire', '',
            'Non cambia contatto, geometria, residuo, ordine o durata delle fasi, eventi di arresto e successo. '
            'Non prevede perdita di presa, slip, saturazione coppie o risposta differente dell’onda autonoma al nuovo feedback. '
            'Non simula indipendentemente le due mani; le forze minime per mano osservate sono salvate come dati reali '
            'in conditional_replay_validation.json, mentre i minimi dei casi alternativi sono solo medie delle mani. '
            'Le curve a gradini riflettono lo ZOH delle letture FFI: non sono nuove misure del rumore ad alta frequenza.', '',
            '### Verifiche numeriche', '',
            f"Baseline identica al controller effettivo: massimo errore forza {max(r['baseline_force_replay_max_error_N'] for r in checks):.3g} N, "
            f"comando {max(r['baseline_command_replay_max_error_mm'] for r in checks):.3g} mm. "
            'Tutti i casi rispettano 2 mm/s e i limiti di corsa. La riproduzione della baseline è un controllo '
            'dell’implementazione: il residuo è imposto, quindi non è validazione del modello.', '',
            f"Raffinamento 5→2,5 ms su {numerical['scenarios']} casi selezionati, un run per pezzo e quattro modelli "
            'inclusi stress: massima differenza RMSE '
            f"{numerical['maximum_absolute_metric_difference_N']['route_rmse_N']:.4f} N, banda "
            f"{numerical['maximum_absolute_metric_difference_N']['route_band_rms_N']:.4f} N, picco finale "
            f"{numerical['maximum_absolute_metric_difference_N']['first_seat_peak_N']:.4f} N. "
            'Quattro run riservate ancora escluse dalla selezione e dal punteggio.', '',
            '## 3. Conseguenza pratica per la ricerca', '',
            'Non emerge una sostituzione P/PD semplice che risolva il problema. La FFI congelata resta il riferimento '
            'più solido tra i casi attuali; un PI con I più lenta merita ulteriori confronti, senza un vantaggio '
            'già dimostrato. Occorre separare il transitorio del piano dal rigetto dell’onda e verificare '
            'le richieste di forza incompatibili con il limite di movimento.', '',
            'Prossime tranche: compensatori con anticipo e roll-off dichiarati; gestione del budget nominale/feedback; '
            'preview del target disponibile realmente nel profilo offline e pianificazione sotto vincoli. '
            'Le metriche resteranno riferite al **target originale**, senza spostarlo nel grafico per ridurre artificialmente l’errore. '
            'Il clock delle fasi e gli eventuali eventi precoci richiedono attenzione per non introdurre una preview “oracolo”. '
            'Solo dopo una selezione congelata si valuteranno le quattro run riservate, con i limiti retrospettivi dichiarati.', '',
            'File riproducibili: empirical_models.py, empirical_validation.py, conditional_controllers.py, '
            'conditional_refinement.py e questo generatore. Disegni congelati in empirical_design.json e '
            'conditional_controller_design.json. Metriche, parametri, controllori scartati e tracce NPZ conservati.', '']
    (OUT/'EMPIRICAL_CONTROL_REPORT.md').write_text('\n'.join(lines),encoding='utf-8')
    print('Wrote EMPIRICAL_CONTROL_REPORT.md; all-threshold candidates',len(satisfying),flush=True)


if __name__=='__main__':main()
