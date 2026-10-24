# Moving average ATI: replay e simulazione condizionata

Analisi del 23/10/2026. Sorgenti NVIDIA_RUNS lette soltanto; nessun controllo del robot, nessuna modifica al controller.

## Risultato

Un MA da 25–100 ms attenua poco le onde lente osservate. Finestre da 250–500 ms attenuano il segnale ma ritardano la misura; nel modello possono ridurre parte della componente fisica 0,8–2,5 Hz, con un peggioramento del tracking. Il beneficio non è uniforme tra i modelli: non emerge una finestra che risolva da sola oscillazione e transitori.

### Calcolo del solo filtro (200 Hz)

| Finestra aggiunta | Campioni | Ritardo di gruppo | Riduzione ampiezza a 1 Hz | A 2 Hz |
|---|---:|---:|---:|---:|
| 25 ms | 5 | 10 ms | 0.10% | 0.39% |
| 50 ms | 10 | 22.5 ms | 0.41% | 1.62% |
| 100 ms | 20 | 47.5 ms | 1.63% | 6.44% |
| 250 ms | 50 | 122.5 ms | 9.96% | 36.33% |
| 500 ms | 100 | 247.5 ms | 36.34% | 100.00% |

È un filtro **causale** sugli ultimi N campioni, con storia iniziale costante; nessun filtro centrato o filtfilt nella legge di controllo. La durata indicata è N/fs; il ritardo di gruppo è (N−1)/(2fs). La risposta in frequenza è |sin(πfN/fs)/(N sin(πf/fs))|. [Riferimento primario: UC Berkeley](https://ptolemy.berkeley.edu/eecs20/week12/freqResponseRA.html).

### Risultati simulati: media delle tre run di ogni pezzo nel modello centrale

Tutte le forze sono la **media delle due mani**, in N; target in N/mano. La banda è una componente del segnale, non una stima indipendente del rumore sensoriale. Modello centrale: 680 N/m per mano, 170 ms di ritardo.

| Pezzo | MA aggiunto | RMS fisica 0,8–2,5 Hz | RMSE forza–target route | Picco primi 1,6 s dopo route |
|---|---:|---:|---:|---:|
| Z | 0 ms | 0.117 N | 0.321 N | 4.765 N |
| Z | 100 ms | 0.115 N | 0.326 N | 4.760 N |
| Z | 250 ms | 0.106 N | 0.332 N | 4.789 N |
| Z | 500 ms | 0.095 N | 0.338 N | 4.900 N |
| L | 0 ms | 0.111 N | 0.286 N | 6.410 N |
| L | 100 ms | 0.111 N | 0.299 N | 6.368 N |
| L | 250 ms | 0.104 N | 0.325 N | 6.262 N |
| L | 500 ms | 0.092 N | 0.386 N | 5.989 N |
| S | 0 ms | 0.069 N | 0.517 N | 5.521 N |
| S | 100 ms | 0.069 N | 0.523 N | 5.480 N |
| S | 250 ms | 0.065 N | 0.532 N | 5.401 N |
| S | 500 ms | 0.057 N | 0.566 N | 5.329 N |
| T | 0 ms | 0.103 N | 0.272 N | 4.434 N |
| T | 100 ms | 0.102 N | 0.285 N | 4.445 N |
| T | 250 ms | 0.093 N | 0.307 N | 4.441 N |
| T | 500 ms | 0.081 N | 0.347 N | 4.414 N |

Il picco della L resta importante anche con 500 ms; il filtro modifica anche la forza presente **prima** del picco. Una riduzione del picco nel modello non dimostra che il filtro abbia rimosso la causa della sovraelongazione.

### Sensibilità: intervallo min–max del rapporto rispetto alla base

Tre run per pezzo × 9 scenari; questi intervalli **non sono intervalli di confidenza**. 100% = controller attuale. Si valutano rigidità 480, 680 e 964 N/m/mano e ritardi puri 120, 170 e 230 ms; non sono nuovi parametri identificati dai dati. 680 è il nominale del controller, 964 un riferimento alternativo già documentato, 480 è una perturbazione inferiore scelta per lo studio. I ritardi coprono il range osservazionale già riportato, che non equivale a identificare un puro ritardo causale.

| Pezzo | Finestra | RMS fisica: min / mediana / max | RMSE: min / mediana / max |
|---|---:|---:|---:|
| Z | 100 ms | 90.9% / 98.8% / 107.9% | 100.0% / 101.6% / 104.9% |
| Z | 250 ms | 77.4% / 90.6% / 105.8% | 99.4% / 103.4% / 113.6% |
| Z | 500 ms | 71.8% / 84.4% / 91.3% | 98.4% / 106.6% / 136.5% |
| L | 100 ms | 92.8% / 99.8% / 110.6% | 103.0% / 104.2% / 107.0% |
| L | 250 ms | 80.1% / 94.2% / 114.2% | 109.8% / 113.8% / 120.4% |
| L | 500 ms | 71.8% / 84.8% / 93.6% | 121.1% / 135.9% / 166.5% |
| S | 100 ms | 93.2% / 100.3% / 109.9% | 100.4% / 100.9% / 102.1% |
| S | 250 ms | 83.4% / 94.2% / 118.3% | 101.1% / 102.7% / 105.9% |
| S | 500 ms | 77.2% / 86.7% / 94.4% | 103.1% / 108.8% / 140.7% |
| T | 100 ms | 88.9% / 99.0% / 109.0% | 102.9% / 104.4% / 107.6% |
| T | 250 ms | 73.9% / 91.8% / 109.0% | 107.1% / 111.6% / 125.2% |
| T | 500 ms | 69.1% / 82.7% / 90.9% | 113.5% / 123.8% / 172.7% |

## Che cosa è stato simulato

1. **Bag registrate:** normale ricostruita con unità, tara, montaggi e orientamenti misurati, a 200 Hz. Si riproduce approssimativamente la media backend degli ultimi due messaggi, poi si aggiunge il MA. Le curve mostrano una misura filtrata della stessa forza già avvenuta: nessun comportamento alternativo del robot.

2. **Anello chiuso condizionato:** stessi target nominali SOCP offline, KI=0,0016 m/(N s) su errore somma, limiti originali di corsa/velocità e priorità del nominale. Le classi FFISettings/NominalIntegral sono estratte dal codice effettivo con AST senza importare ROS. Misura effettivamente usata dall’FFI ricavata da target e ffi_error_N. Tra aggiornamenti è interpolata a 200 Hz; il modello non risolve la dinamica oltre la banda utile dei tick FFI (~35–40 Hz).

La forza somma alternativa è **y(t)=y₀(t)+2k·[s_alt(t−τ)−s_base(t−τ)]**. Il modello descrive solo la variazione locale rispetto alla registrazione; la componente residua del moto e del contatto originale resta fissa. Non vengono trasformate in previsioni causali le correlazioni forza/chiusura registrate in anello chiuso. Il MA si applica alla sola normale di feedback, dopo compensazione: non si filtrano qui tutti i sei assi ATI prima di gravità/tara.

Il primo stato registrato può avere ffi_seq=2: tutte le alternative partono dallo stesso stato registrato. Si mantengono gli istanti degli aggiornamenti, arrotondati alla griglia di 5 ms, e gli intervalli ffi_dt_ms originali per integrazione e limiti. I clamping di dt sono così conservati. Il filtro parte dal primo stato osservato; la presa iniziale non è simulata. Target ricavato da ffi_ff_want_mm per rispettare quello realmente usato dal controller.

L’andamento della forza di riferimento viene imposto dai log. Riprodurre il caso senza filtro verifica l’implementazione del replay, **non valida il modello del robot**. La simulazione non identifica attrito, elasticità o servo interni e non prevede slittamento, perdita di presa o cambi di contatto. Le condizioni di arresto sono conservate nel tempo dei log, non ricalcolate: nessuna previsione di successo dell’inserzione.

## Verifiche

- 12 run FFI selezionate riuscite, 6 finestre incl. base, 9 modelli = 648 scenari.
- Riproduzione stati FFI senza filtro: errore massimo 4.34e-15 mm (precisione numerica).
- Il caso senza filtro restituisce esattamente la forza di base per tutti i 9 modelli.
- Raffinamento 5 → 2,5 ms su un caso per pezzo, finestre 100/500 ms, modello centrale e massimo k/ritardo: 16 confronti.
- Differenza massima RMS banda: 0.0006 N; RMSE tracking: 0.0025 N; picco: 0.0105 N.
- Forza media mani minima nel modello: 1.746 N. Questo non garantisce contatto positivo su ogni singola mano.
- NVIDIA_RUNS verificata separatamente mediante manifest SHA-256: vedere ../archive_review/archive_integrity.json.

La metrica di banda usa Butterworth zero-phase 0,8–2,5 Hz soltanto **nell’analisi**, con detrending cubico e 1,5 s esclusi ai bordi; non viene usata dall’integratore. Le metriche route precedono slow_seat. Le curve del replay mostrano le misure agli aggiornamenti, quindi i picchi possono differire lievemente dal massimo CSV a 100 Hz.

## Come procederei

Una finestra breve 25–50 ms è un candidato da valutare solo se si vuole togliere componenti più veloci: non è una soluzione dimostrata per la banda 0,8–2,5 Hz. Non sceglierei 250–500 ms come correzione principale dei transitori. Prima renderei coerente l’evoluzione nominale con k_eff × velocità massima = **1,36 N/s per mano**, studiando anche la continuità route → slow_seat e il budget condiviso con I. La traiettoria va anticipata/ripianificata se serve rispettare contemporaneamente forza minima e velocità; smussare il target dopo il calcolo SOCP non dimostra che i margini di presa restino validi.

Per predire quantitativamente il robot serve una identificazione dedicata del trasferimento comando → forza, in condizioni di contatto controllate e ripetibili, poi una verifica su dati separati. Questa analisi fornisce una scelta ragionata dei prossimi esperimenti, non parametri già validati.

## File

- 01_risposta_filtro.png: risposta teorica e ritardo.
- 02_bag_filtrate.png: filtro aggiunto alle bag Z/L.
- 03_simulazione_forze.png: forza fisica simulata vs misura vista da I, Z/L.
- 04_sensibilita_12_run.png: tutti i modelli e le run.
- simulation_metrics.csv/json, summary.json: numeri completi.
- *_central.npz: tracce complete delle 12 run nel modello centrale.
- replay_validation.json, numerical_refinement.json: verifiche.
- simulate.py e check_and_report.py: codice riproducibile.
