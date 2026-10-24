# Inserimento autonomo PyBullet — FFI delle run, molla CENTRALE 2700 N/m

Nuova richiesta esplicita di Lorenzo del 24 ottobre, successiva alla campagna
notturna. Tutti i nuovi file rimangono qui. I dati NVIDIA e i controller reali
sono in sola lettura. Il confronto non utilizza forze reali come ingresso.

**Estensione successiva richiesta: due Gen3 articolati nella geometria del lab.**
Vedere [gen3/LAB_IMPEDANCE_RESULTS.md](gen3/LAB_IMPEDANCE_RESULTS.md) per la
legge originale fornita da Lorenzo e le verifiche. [gen3/RESULTS.md](gen3/RESULTS.md)
conserva il precedente confronto con servo approssimato.
Le sezioni sotto descrivono il primo backend flottante e rimangono distinte:
masse e dinamica del polso flottante non sono quelle dei bracci articolati.

## Cosa viene eseguito

- Codice reale importato: `FFIProfileInsertion`, `NominalIntegral`, pickup I,
  `NominalPlanner`, `GraspSOCP`, prior TP-GMM e sequenza completa fino al rilascio.
- Parametri FFI delle run: Ki 0.0016 m/(N s) sull'errore sommato, k_eff 680 N/m
  per mano, trim ±3 mm, totale ±12 mm, velocità normale 2 mm/s. Nessun tuning.
- Parametri cartesiani nominali: K traslazionale 1500 N/m, rotazionale 20 Nm/rad;
  D traslazionale 60 Ns/m, rotazionale 7. Streaming 100 Hz, FFI 50 Hz nominali.
- SOCP offline ricalcolato **prima** della dinamica, come nelle run FFI reali.
  `socp_allocations.json` conserva input, forze per punto, residui, margini e
  solver; `profile.csv` conserva i due wrench completi. Gli zero nelle statistiche
  SOCP durante l'esecuzione significano piano offline, non SOCP omesso.
- Pezzo e vicini dinamici nello scaffale, geometrie non convesse a colonne
  smussate dal codice originale. Fixture solo prima del rilascio dell'operatore.
- Sensoristica al polso ricavata dalla reazione delle molle applicata alla
  flangia: sei componenti, gravità tool rimossa, inerzia conservata. Il controller
  riceve la media degli ultimi due blocchi da 5 ms. Nessun rumore o ritardo
  di rete inventato. Contatti veri su ogni pad/piastra registrati separatamente.

## Che cosa significa “molla centrale 2700”

Il modello `floating_plate.py` trovato nel progetto TESI_antiG ha tre molle
assiali da 500 N/m e una centrale da 20000: totale 21500 N/m. La prima serie
`*_D60_dt05` conserva quella meccanica, con geometria e gain delle run reali.
Non va confusa con la molla reale da 2700 N/m.

Lorenzo ha precisato: **CENTRALE 2700**, non totale. La serie richiesta è
`*_central2700_D60_*`: centrale **2700**, tre molle pad **500 ciascuna**, totale
assiale **4200 N/m**. Le prove precedenti `*_axial2700_D60_*` usavano centrale
1200 per un'interpretazione errata del totale: restano conservate, sono superate
e non vengono presentate come le prove richieste. La topologia viene conservata;
rigidezze laterali e torsionali restano quelle nominali del modello originale.
Questo non identifica tutte le proprietà della molla reale libera. In particolare,
2700 N/m sulla centrale da solo non determina flessione, torsione, attriti,
smorzamento o isteresi. Con K braccio 1500 e totale assiale 4200, la serie
elastica ideale è circa 1105 N/m prima di aggiungere la compliance di contatto;
il k_eff 680 usato dal controller rimane quello delle run, senza ritararlo.

Il pad model conserva le proprie masse nominali (piastra 80 g + 3 sfere 20 g,
flangia 50 g) e l'inerzia di flangia aumentata a 0.05 kgm² dal simulatore originale.
Non sono le masse/inerzie effettive dei sette giunti Kinova. Il backend è
un'impedenza cartesiana ideale applicata al modello flottante, non una replica
certificata del servo interno Kinova. Contatto pezzi nominale 8000 N/m e 207 Ns/m,
attrito pad-pezzo 1, pezzo-pezzo .5, pavimento .3; proprietà non calibrate sul banco.

## Verifiche e limiti numerici

`backend_checks.json`: confronto indipendente tra algebra del sensore e
reazione effettivamente applicata dal codice molle, deriva in aria, equilibrio
statico, segni e attrito letto da Bullet. `input_integrity_and_profile_parity.json`:
SHA-256 di tutti gli originali, sorgenti runtime e copie del modello; confronto
numerico con i piani SOCP realmente archiviati.

La prova originale a 1 ms è abortita in aria: è un fallimento numerico conservato,
non un risultato del robot. Il costruttore delle molle applica un cap di damping
dipendente da dt: per la nuova serie 2700 i coefficienti sono inizializzati con
un riferimento **fisso 0.25 ms**, anche quando il passo fisico è 0.5 ms. Così il
raffinamento non cambia di nascosto lo smorzamento meccanico.
Le prime prove 21500 usavano il cap originale legato a dt e non sono una verifica
di convergenza a parametri fisici identici. Il forte cambiamento della posa finale
Z fra quelle prove viene mantenuto come evidenza di sensibilità, non nascosto.

## File da leggere

- `RESULTS.md`: conclusioni, tabella metriche, convergenza e limiti.
- `real_vs_bullet_central2700.png`: confronto descrittivo fra prime run reali di sviluppo
  e simulazione, ciascuna sul proprio clock dalla partenza; nessun fitting.
- `runs/<caso>/force_tracking.png`, `wrist_wrench.png`, `socp_wrench.png`.
- `runs/<caso>/trace.npz`: posizioni, orientamenti, velocità oggetto, wrench al
  polso, wrench contatto, normali per pad, deformazioni, FFI e wrench desiderati.
- `trace.csv` e `wrist_wrench_full.csv`: sei componenti ai polsi in assi scaffale,
  N e Nm. Le figure locali tool usano x lungo la direzione di spinta di ogni mano.
- `raw_wrist_1kHz`, quando presente: timestamp pre-step, 12 componenti raw L/R,
  12 componenti compensate. `sensor_block_average_200Hz`: timestamp disponibilità,
  12 componenti dei blocchi mediati. `sensor_200Hz` storico è un campionamento
  dell'ultimo wrench istantaneo, non il blocco medio; il controller usa sempre
  i blocchi mediati. `L/R_measured` è il valore effettivamente usato a 100 Hz.
- `runner_source.py`: sorgente esatto salvato per la singola esecuzione.

“seat_found” è una decisione del controller basata sul wrench: non equivale
all'errore geometrico vero. Si riporta separatamente la massima distanza dei
quattro centri cella dal target **dopo** rilascio e ritirata. Una riuscita nominale
o una differenza dal reale non dimostra che il problema sia dovuto a cambiamenti
hardware; può dipendere anche da proprietà di contatto/servo mancanti nel modello.

Runtime Bullet da binari ufficiali conda-forge isolati, SHA in
`runtime_provenance.json`; nessuna installazione di sistema. Riproduzione con
il Python indicato nel protocollo e `run_bullet.py --piece Z --central-k 2700
--dt .0005 --tag NUOVO_NOME`; ogni directory di run è esclusiva, non sovrascritta.
