# Coerenza degli output salvati

Verifica 2026-10-23T19:20:20.991559+00:00. **Nessuna nuova traiettoria simulata, analisi delle
forze originali, selezione o tuning.** Il diagnostico del clock ha letto soltanto
ffi_seq/t_wall/ffi_dt_ms nei quattro CSV riservati. I controlli riguardano coerenza software
e conservazione dei risultati, non validazione fisica del Kinova.

Verificati i quattro manifest di consegna e i relativi artifact,14 fingerprint
congelati, tutti gli input hash del registro tesi, 8 SHA dei
checkpoint e 47 collegamenti locali negli11 report principali.
I link web già citati non sono stati ricontrollati; non si aggiungono nuove
affermazioni bibliografiche. Nessun report immutabile precedente modificato.

## Corrispondenza tra JSON, CSV e tracce

Tutti448 casi hanno identità univoca e corrispondono alle7leggi ×16modelli
×4run del freeze. La data del freeze precede START e COMPLETED. Dal solo NPZ
salvato sono ricalcolate le **stesse9 riduzioni già pubblicate** (RMSE/p95/errori
route, picchi/eccessi/difetti seat, minimo medio, corsa comando), senza invocare
il simulatore. 4032 confronti scalari,
massima differenza 0.00e+00 nelle unità dichiarate.
Confrontati tutti i campi JSON↔CSV, formula dello score con penalità originale,
aggregazione peggiore modello per run e obiettivi provvisori.0/448 con tutti
gli obiettivi rimane l'esito già consegnato. Limiti corsa/velocità e baseline
salvate sono coerenti. Non è una seconda valutazione delle alternative sul robot.

## Un controllo fallito: clock del log scambiato per dt FFI

Il primo audit è fallito perché divideva gli incrementi della correzione
normale per gli intervalli `event_time` salvati (t_wall della scrittura CSV),
chiamandoli impropriamente «actual tick». I sorgenti usano invece ffi_dt_ms,
che è misurato all'update e può essere clamped. Codice e log del controllo
fallito sono conservati in audit_saved_evidence_attempt1.py e
SAVED_EVIDENCE_ATTEMPT1_FAILURE.json; il criterio non è stato allargato.

Riletti soltanto i campi temporali, verificati gli hash originali e salvati
clock ausiliari separati: nessuna modifica alle tracce congelate. Gli incrementi
divisi per il **dt effettivamente usato dalla legge FFI** rispettano2mm/s in
tutti448casi, in accordo con le metriche originali. Dividendo invece per il
clock del log i massimi sono circa2,139/2,127/2,045/2,074mm/s (Z/L/S/T).
Le differenze fra intervalli raggiungono3,44/11,30/2,89/1,88ms; includono
diversa posizione della timestamp e possibile clamp, non una misura isolata
di latenza o jitter. Il diagnostico non identifica il consumo del target nel
servo o la velocità reale dei tool. Il limite riguarda la correzione normale
software, non tutta la velocità cartesiana dovuta anche alla route.

È una precisazione del dominio della verifica, non una nuova superiorità di
un controllore o la dimostrazione di una violazione hardware. Il replay usa
il clock dei log per collocare gli eventi e ffi_dt_ms per gli incrementi,
come dichiarato nel sorgente congelato: l'incertezza temporale rimane. Non
sono stati spostati eventi o rifatti i confronti per migliorare lo score.

## I fallimenti restano fallimenti

| Verifica di sviluppo conservata | Casi | ΔRMSE max [N] | Δbanda max [N] | Δpicco max [N] |
| --- | --- | --- | --- | --- |
| advanced_numerical_refinement.json | 128 | 0.0265 | 0.0180 | 0.0555 |
| advanced_numerical_refinement_1p25ms.json | 128 | 0.0391 | 0.0188 | 0.0627 |
| planned_reference_refinement.json | 608 | 0.0029 | 0.0012 | 0.1839 |

Le tolleranze lette dall'AST dei sorgenti delle verifiche sono 0,03 N RMSE,
0,01 N banda e 0,05 N picco. La verifica dei piani fallisce sul picco. Non sono
state allargate. Il completamento di un batch o l'identità della baseline non
risolvono tali fallimenti. Le successive verifiche a eventi esatti coprono solo
una parte dei modelli, come dichiarato nei relativi report.
La seconda bozza dell'audit aveva trascritto 0,02 N per RMSE: corretta a 0,03 N
dal sorgente, conservando bozza/log. Gli esiti falliti restano identici; nessuna
tolleranza della campagna o dato di raffinamento è stato modificato.

## Come leggere documenti scritti in tranche diverse

Le sezioni «prossima tranche», «holdout non ancora analizzato» o «verifiche da
fare» nei vecchi report descrivono il momento di quella tranche. Non annullano
i risultati successivi. L'ordine e l'esito attuale sono in [READING_GUIDE.md](READING_GUIDE.md)
e STATE.json. I report congelati non vengono riscritti retroattivamente per
nascondere cosa fosse noto prima del holdout o di un fallimento numerico.

Rimangono tre limiti decisivi: modelli comando→moto→forza insufficientemente
predittivi, nessun confronto D-only matched con FFI invariata, nessuna garanzia
sui contatti individuali da un replay scalare. La [sintesi tesi](THESIS_SYNTHESIS.md)
mantiene distinte osservazione, replay condizionato e simulazione autonoma.
