# Guida ai risultati della campagna

Termine della campagna: 24/10 ore 09:00 Singapore. I confronti offline e la
sintesi sono completati; nessun nuovo controllore è validato sul
robot. Questo indice distingue la conclusione attuale dalle note storiche
dei report progressivi, senza modificare artifact congelati.

| Documento | Contenuto e ruolo |
| --- | --- |
| [THESIS_SYNTHESIS.md](THESIS_SYNTHESIS.md) | Punto di ingresso:12run osservate, confronto storico descrittivo, testo adattabile e limiti |
| [THESIS_EVIDENCE.json](THESIS_EVIDENCE.json) | Dieci affermazioni, livello di evidenza, fonti hash e limiti |
| [HOLDOUT_REPORT.md](HOLDOUT_REPORT.md) | Confronto finale congelato448casi, nessun tuning successivo; holdout retrospettivo |
| [DAMPING_REPORT.md](DAMPING_REPORT.md) | Priorità damping: modello fisico dichiarato, verifica predittiva fallita, nessun nuovo D scelto |
| [DAMPING_AB_PROTOCOL.md](DAMPING_AB_PROTOCOL.md) | Futuro D-only a FFI fissa; preparato e non eseguito |
| [MINIMUM_IDENTIFICATION_PROTOCOL.md](MINIMUM_IDENTIFICATION_PROTOCOL.md) | Dati sincronizzati e prove minime necessarie |
| [SIGNAL_PIPELINE_REPORT.md](SIGNAL_PIPELINE_REPORT.md) | Media ATI già presente, timestamp e limiti del log; filtri ideali separati dal robot |
| [EMPIRICAL_CONTROL_REPORT.md](EMPIRICAL_CONTROL_REPORT.md) | Free simulation vs one-step e primo studio controllori su sviluppo |
| [ADVANCED_CONTROL_REPORT.md](ADVANCED_CONTROL_REPORT.md) | PD/PID/allocazione e stress autonomi; fallimenti di raffinamento conservati |
| [PROFILE_PREVIEW_REPORT.md](PROFILE_PREVIEW_REPORT.md) | Preview disponibile e fattibilità del solo comando nominale |
| [PLANNED_REFERENCE_REPORT.md](PLANNED_REFERENCE_REPORT.md) | Piani/riferimenti, raffinamento fallito, verifica parziale a eventi esatti |
| [LINEAR_ROBUSTNESS_REPORT.md](LINEAR_ROBUSTNESS_REPORT.md) | Poli e sensibilità solo del modello locale non limitato; non certifica il robot |
| [SAVED_EVIDENCE_AUDIT.md](SAVED_EVIDENCE_AUDIT.md) | Coerenza tracce/metriche/CSV/hash e distinzione delle tranche storiche |
| [RESULTS.md](RESULTS.md) | Registro narrativo progressivo, aggiornamenti recenti in apertura |

Per stato operativo usare [STATE.json](STATE.json); per storia delle esecuzioni
e verifiche [NIGHT_LOG.md](NIGHT_LOG.md). La configurazione delle sette leggi,
split e score non vanno ricavati da una vecchia shortlist: la fonte finale è
[FINAL_COMPARISON_FREEZE.json](FINAL_COMPARISON_FREEZE.json) con il suo manifest
e supplemento sorgenti. Le righe storiche «prossimo batch holdout» sono state
superate dal marker [HOLDOUT_EVALUATION_COMPLETED.json](HOLDOUT_EVALUATION_COMPLETED.json),
non sono ordini di rilanciarlo. Non scegliere nuovi parametri su quelle run.

I controlli software e SHA possono passare mentre il modello fisico non è
adeguato: non sommare queste due nozioni in un unico «tutto verificato».
Le soglie0,05/0,20/0,25N sono esplorative, non criteri di sicurezza/tesi.
Le12run selezionate non sono un tasso di successo12/12; `believed_mm` non è
metrologia esterna; la positività della media non certifica la presa bilaterale.

NVIDIA_RUNS, snapshot protetto, sorgenti di produzione e analisi precedenti
rimangono in sola lettura. Fonti e risultati della campagna sono conservati;
nessuna azione robot/ROS/deploy/push è stata eseguita dalla ricerca notturna.
