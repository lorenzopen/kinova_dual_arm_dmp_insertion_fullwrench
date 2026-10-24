"""Independent numerical refinement and compact result tables/report."""
import simulate as s
import numpy as np
import json


def main():
    rows=json.loads((s.OUT/'simulation_metrics.json').read_text())
    recs=json.loads((s.REVIEW/'run_catalog.json').read_text())
    recs=[r for r in recs if r['status']=='curated_seat_found' and '_ffi_' in r['id']]
    bases={r['id']:r for r in rows if r['window_ms']==0 and r['stiffness_N_m']==680 and r['delay_ms']==170}
    summary=[]
    for piece in 'ZLST':
        for w in s.WINDOWS:
            rr=[r for r in rows if r['piece']==piece and r['window_ms']==w*1000]
            central=[r for r in rr if r['stiffness_N_m']==680 and r['delay_ms']==170]
            row=dict(piece=piece,window_ms=w*1000)
            for key in ['route_band_physical_N','route_tracking_rmse_N','first_seat_peak_physical_N']:
                v=np.array([r[key] for r in rr]);ratio=np.array([r[key]/bases[r['id']][key] for r in rr])
                row[key]=dict(central_mean=float(np.mean([r[key] for r in central])),
                               scenario_ratio_percent_min_median_max=(np.percentile(ratio,[0,50,100])*100).tolist())
            summary.append(row)
    s.dump('summary.json',summary)
    # Halve the simulation grid on all 4 pieces, both mild/long windows,
    # central and high stiffness/high delay scenarios. No physical claims from
    # reproducing baseline: that's just controller implementation validation.
    numerical=[]
    for piece in 'ZLST':
        rec=next(r for r in recs if r['piece']==piece)
        s.DT=.0025;p=s.prepare(rec)
        for w in [.1,.5]:
            for stiffness,delay in [(680,.17),(964,.23)]:
                sim=s.simulate(p,w,stiffness,delay,step=s.DT);m=s.metrics(p,sim)
                orig=next(r for r in rows if r['id']==rec['id'] and r['window_ms']==w*1000
                          and r['stiffness_N_m']==stiffness and r['delay_ms']==delay*1000)
                diff={k:float(m[k]-orig[k]) for k in m}
                assert abs(diff['route_band_physical_N'])<.01
                assert abs(diff['route_tracking_rmse_N'])<.03
                assert abs(diff['first_seat_peak_physical_N'])<.08
                numerical.append(dict(id=rec['id'],window_ms=w*1000,stiffness_N_m=stiffness,
                                      delay_ms=delay*1000,metric_difference_2p5ms_minus_5ms=diff))
    s.DT=.005
    s.dump('numerical_refinement.json',numerical)
    physical_min=min(r['min_force_mean_hand_N'] for r in rows)
    assert physical_min>0, physical_min
    validation=json.loads((s.OUT/'replay_validation.json').read_text())
    print('Replay max error mm',max(v['replay_max_output_error_mm'] for v in validation))
    print('Physical minimum across scenarios',physical_min)
    print('Max grid refinement differences', {k:max(abs(v['metric_difference_2p5ms_minus_5ms'][k]) for v in numerical)
          for k in ['route_band_physical_N','route_tracking_rmse_N','first_seat_peak_physical_N']})
    for r in summary:
        if r['window_ms'] in [0,100,500]:print(json.dumps(r),flush=True)
    lines=['# Moving average ATI: replay e simulazione condizionata', '',
           'Analisi del 23/10/2026. Sorgenti NVIDIA_RUNS lette soltanto; nessun controllo del robot, nessuna modifica al controller.', '',
           '## Risultato', '',
           'Un MA da 25–100 ms attenua poco le onde lente osservate. Finestre da 250–500 ms attenuano il segnale ma ritardano la misura; '
           'nel modello possono ridurre parte della componente fisica 0,8–2,5 Hz, con un peggioramento del tracking. '
           'Il beneficio non è uniforme tra i modelli: non emerge una finestra che risolva da sola oscillazione e transitori.', '',
           '### Calcolo del solo filtro (200 Hz)', '',
           '| Finestra aggiunta | Campioni | Ritardo di gruppo | Riduzione ampiezza a 1 Hz | A 2 Hz |',
           '|---|---:|---:|---:|---:|']
    for r in json.loads((s.OUT/'filter_theory.json').read_text()):
        lines.append(f"| {r['window_ms']:g} ms | {r['samples']} | {r['delay_ms']:g} ms | {r['attenuation_amplitude_1Hz_percent']:.2f}% | {r['attenuation_amplitude_2Hz_percent']:.2f}% |")
    lines+=['', 'È un filtro **causale** sugli ultimi N campioni, con storia iniziale costante; nessun filtro centrato o filtfilt nella legge di controllo. '
            'La durata indicata è N/fs; il ritardo di gruppo è (N−1)/(2fs). '
            'La risposta in frequenza è |sin(πfN/fs)/(N sin(πf/fs))|. '
            '[Riferimento primario: UC Berkeley](https://ptolemy.berkeley.edu/eecs20/week12/freqResponseRA.html).', '',
            '### Risultati simulati: media delle tre run di ogni pezzo nel modello centrale', '',
            'Tutte le forze sono la **media delle due mani**, in N; target in N/mano. La banda è una componente del segnale, '
            'non una stima indipendente del rumore sensoriale. Modello centrale: 680 N/m per mano, 170 ms di ritardo.', '',
            '| Pezzo | MA aggiunto | RMS fisica 0,8–2,5 Hz | RMSE forza–target route | Picco primi 1,6 s dopo route |',
            '|---|---:|---:|---:|---:|']
    for r in summary:
        if r['window_ms'] in [0,100,250,500]:
            v=[r[k]['central_mean'] for k in ['route_band_physical_N','route_tracking_rmse_N','first_seat_peak_physical_N']]
            lines.append(f"| {r['piece']} | {r['window_ms']:g} ms | {v[0]:.3f} N | {v[1]:.3f} N | {v[2]:.3f} N |")
    lines+=['', 'Il picco della L resta importante anche con 500 ms; il filtro modifica anche la forza presente **prima** del picco. '
            'Una riduzione del picco nel modello non dimostra che il filtro abbia rimosso la causa della sovraelongazione.', '',
            '### Sensibilità: intervallo min–max del rapporto rispetto alla base', '',
            'Tre run per pezzo × 9 scenari; questi intervalli **non sono intervalli di confidenza**. '
            '100% = controller attuale. Si valutano rigidità 480, 680 e 964 N/m/mano e ritardi puri 120, 170 e 230 ms; '
            'non sono nuovi parametri identificati dai dati. 680 è il nominale del controller, 964 un riferimento alternativo già documentato, '
            '480 è una perturbazione inferiore scelta per lo studio. I ritardi coprono il range osservazionale già riportato, '
            'che non equivale a identificare un puro ritardo causale.', '',
            '| Pezzo | Finestra | RMS fisica: min / mediana / max | RMSE: min / mediana / max |',
            '|---|---:|---:|---:|']
    for r in summary:
        if r['window_ms'] in [100,250,500]:
            fmt=lambda key:' / '.join(f'{v:.1f}%' for v in r[key]['scenario_ratio_percent_min_median_max'])
            lines.append(f"| {r['piece']} | {r['window_ms']:g} ms | {fmt('route_band_physical_N')} | {fmt('route_tracking_rmse_N')} |")
    lines+=['', '## Che cosa è stato simulato', '',
            '1. **Bag registrate:** normale ricostruita con unità, tara, montaggi e orientamenti misurati, a 200 Hz. '
            'Si riproduce approssimativamente la media backend degli ultimi due messaggi, poi si aggiunge il MA. '
            'Le curve mostrano una misura filtrata della stessa forza già avvenuta: nessun comportamento alternativo del robot.', '',
            '2. **Anello chiuso condizionato:** stessi target nominali SOCP offline, KI=0,0016 m/(N s) su errore somma, '
            'limiti originali di corsa/velocità e priorità del nominale. Le classi FFISettings/NominalIntegral sono estratte dal codice effettivo '
            'con AST senza importare ROS. Misura effettivamente usata dall’FFI ricavata da target e ffi_error_N. '
            'Tra aggiornamenti è interpolata a 200 Hz; il modello non risolve la dinamica oltre la banda utile dei tick FFI (~35–40 Hz).', '',
            'La forza somma alternativa è **y(t)=y₀(t)+2k·[s_alt(t−τ)−s_base(t−τ)]**. '
            'Il modello descrive solo la variazione locale rispetto alla registrazione; la componente residua del moto e del contatto '
            'originale resta fissa. Non vengono trasformate in previsioni causali le correlazioni forza/chiusura registrate in anello chiuso. '
            'Il MA si applica alla sola normale di feedback, dopo compensazione: non si filtrano qui tutti i sei assi ATI prima di gravità/tara.', '',
            'Il primo stato registrato può avere ffi_seq=2: tutte le alternative partono dallo stesso stato registrato. '
            'Si mantengono gli istanti degli aggiornamenti, arrotondati alla griglia di 5 ms, e gli intervalli ffi_dt_ms originali '
            'per integrazione e limiti. I clamping di dt sono così conservati. Il filtro parte dal primo stato osservato; '
            'la presa iniziale non è simulata. Target ricavato da ffi_ff_want_mm per rispettare quello realmente usato dal controller.', '',
            'L’andamento della forza di riferimento viene imposto dai log. Riprodurre il caso senza filtro verifica '
            'l’implementazione del replay, **non valida il modello del robot**. La simulazione non identifica attrito, '
            'elasticità o servo interni e non prevede slittamento, perdita di presa o cambi di contatto. '
            'Le condizioni di arresto sono conservate nel tempo dei log, non ricalcolate: nessuna previsione di successo dell’inserzione.', '',
            '## Verifiche', '',
            f"- 12 run FFI selezionate riuscite, 6 finestre incl. base, 9 modelli = {len(rows)} scenari.",
            f"- Riproduzione stati FFI senza filtro: errore massimo {max(v['replay_max_output_error_mm'] for v in validation):.3g} mm (precisione numerica).",
            '- Il caso senza filtro restituisce esattamente la forza di base per tutti i 9 modelli.',
            '- Raffinamento 5 → 2,5 ms su un caso per pezzo, finestre 100/500 ms, modello centrale e massimo k/ritardo: 16 confronti.',
            f"- Differenza massima RMS banda: {max(abs(v['metric_difference_2p5ms_minus_5ms']['route_band_physical_N']) for v in numerical):.4f} N; "
            f"RMSE tracking: {max(abs(v['metric_difference_2p5ms_minus_5ms']['route_tracking_rmse_N']) for v in numerical):.4f} N; "
            f"picco: {max(abs(v['metric_difference_2p5ms_minus_5ms']['first_seat_peak_physical_N']) for v in numerical):.4f} N.",
            f'- Forza media mani minima nel modello: {physical_min:.3f} N. Questo non garantisce contatto positivo su ogni singola mano.',
            '- NVIDIA_RUNS verificata separatamente mediante manifest SHA-256: vedere ../archive_review/archive_integrity.json.', '',
            'La metrica di banda usa Butterworth zero-phase 0,8–2,5 Hz soltanto **nell’analisi**, con detrending cubico '
            'e 1,5 s esclusi ai bordi; non viene usata dall’integratore. Le metriche route precedono slow_seat. '
            'Le curve del replay mostrano le misure agli aggiornamenti, quindi i picchi possono differire lievemente dal massimo CSV a 100 Hz.', '',
            '## Come procederei', '',
            'Una finestra breve 25–50 ms è un candidato da valutare solo se si vuole togliere componenti più veloci: '
            'non è una soluzione dimostrata per la banda 0,8–2,5 Hz. Non sceglierei 250–500 ms come correzione principale dei transitori. '
            'Prima renderei coerente l’evoluzione nominale con k_eff × velocità massima = **1,36 N/s per mano**, '
            'studiando anche la continuità route → slow_seat e il budget condiviso con I. '
            'La traiettoria va anticipata/ripianificata se serve rispettare contemporaneamente forza minima e velocità; '
            'smussare il target dopo il calcolo SOCP non dimostra che i margini di presa restino validi.', '',
            'Per predire quantitativamente il robot serve una identificazione dedicata del trasferimento comando → forza, '
            'in condizioni di contatto controllate e ripetibili, poi una verifica su dati separati. '
            'Questa analisi fornisce una scelta ragionata dei prossimi esperimenti, non parametri già validati.', '',
            '## File', '',
            '- 01_risposta_filtro.png: risposta teorica e ritardo.',
            '- 02_bag_filtrate.png: filtro aggiunto alle bag Z/L.',
            '- 03_simulazione_forze.png: forza fisica simulata vs misura vista da I, Z/L.',
            '- 04_sensibilita_12_run.png: tutti i modelli e le run.',
            '- simulation_metrics.csv/json, summary.json: numeri completi.',
            '- *_central.npz: tracce complete delle 12 run nel modello centrale.',
            '- replay_validation.json, numerical_refinement.json: verifiche.',
            '- simulate.py e check_and_report.py: codice riproducibile.', '']
    (s.OUT/'REPORT.md').write_text('\n'.join(lines),encoding='utf-8')


if __name__=='__main__':main()
