# Ricerca notturna controllore normale — 23/24 ottobre 2026

Autorizzazione: Lorenzo chiede di conservare codice e simulazioni e lavorare
durante la notte cercando un controllo con meno oscillazioni/sovraelongazioni e
buon inseguimento. Campagna locale fino alle **09:00 del 24/10 Asia/Singapore**
(2026-10-24 01:00 UTC). Nessuna esecuzione sul robot autorizzata da questa ricerca.

## Conservazione

- `NVIDIA_RUNS/` è strettamente in sola lettura. Non cancellare, spostare,
  rinominare, sovrascrivere o generare alcun file al suo interno.
- `baseline_20261023/source_and_simulations.zip` e il suo manifest SHA-256 sono
  una copia conservata: non modificarli. Verificata corrispondenza dei tre moduli
  controller/FFI/backend con tutte le 12 revisioni registrate nelle run.
- Nuovo codice esclusivamente in questa directory; mantenere i controller reali
  e le vecchie analisi intatti. Niente push, deploy, ROS o comandi al robot.

## Domande da risolvere, in ordine

Steering successivo di Lorenzo: leggere prima **DAMPING_STEERING.md** e trattare
il damping del controllo cartesiano come candidato prioritario, mantenendo
fermo il controllore FFI nel confronto. Non limitare lo studio al PID esterno.

1. **Fattibilità fisica:** con k=680 N/m per mano e velocità massima 2 mm/s,
   il modello quasi statico consente 1,36 N/s. Una variazione di 2 N richiede
   almeno 1,47 s di moto, più il ritardo: documentare quali richieste di rapidità
   sono incompatibili con i vincoli, senza promettere assenza assoluta di onde.
2. **Modelli:** separare dinamica comando→movimento da movimento→forza.
   Confrontare ritardo puro, primo ordine+ritardo e un modello risonante.
   Cercare anche dipendenza dal verso del moto, saturazione, isteresi.
   Le correlazioni in anello chiuso non identificano da sole un impianto causale.
3. **Validazione:** usare le prime due run riuscite FFI per pezzo come sviluppo;
   la terza cronologica di Z/L/S/T è holdout. La selezione va congelata prima
   del confronto e del tuning. Le prove Z con I quasi spenta e le vecchie
   online possono confutare modelli, ma differiscono per condizioni/gain:
   non chiamarle esperimenti controllati. Non trattare riproduzione esatta del
   replay con residuo imposto come validazione indipendente del modello.
   Le terze run sono già state viste nelle precedenti analisi descrittive e
   nello studio moving average: questo è un holdout per il nuovo tuning, non
   una verifica prospettica indipendente. Dichiararlo nella consegna.
4. **Controllori:** baseline FFI reale; FF+P; FF+PI; FF+PD filtrato sulla misura;
   FF+PID filtrato; compensazione del ritardo se i modelli la sostengono.
   Stessi limiti di corsa e velocità per confronti principali. Verificare
   separatamente gestione della priorità nominale/feedback e pianificazione
   anticipata del target con preview realmente disponibile nel SOCP offline.
   Nessuna anticipazione della forza futura misurata. Non filtrare target SOCP
   dichiarando ancora garantiti i margini: verificare i minimi e documentare
   il ritardo rispetto al requisito originale.
5. **Robustezza:** non scegliere solo sul caso centrale. Esplorare mismatch di
   rigidezza/ritardo, rumore, jitter, disturbi, contatto e limiti. Riportare
   entrambi i versi degli step e offset persistenti. Modelli semplici non
   dimostrano stabilità reale del contatto o successo di inserzione.

## Metriche e soglie esplorative

Media mani in N; errore del controllo è sulla somma. Forza sinistra/destra e
minimi per mano vanno riportati quando disponibili, senza inferire contatto
bilaterale dalla sola media. Metriche: banda 0,8–2,5 Hz, RMSE rispetto al target
originale, massimo errore positivo/negativo, picco finale L, tempo di salita
10–90%, assestamento, saturazione e corsa/comando. Riportare anche oscillazioni
in altre bande per non spostare semplicemente il problema fuori dalla metrica.

Obiettivi **provvisori**, non criteri di accettazione concordati né limiti di
sicurezza: RMS banda ≤0,05 N/mano, overshoot ≤0,25 N/mano, RMSE route ≤0,20 N/mano.
Misurare il tempo di risposta ottenibile sotto vincoli, senza imporre una soglia
di velocità impossibile. Se nessun candidato li soddisfa, riportarlo.

## Riproducibilità e avanzamento

Runtime Python:
`C:/Users/Lorenzo/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe`.
Dipendenze locali in `tmp/force_plot_deps` e `tmp/bag_analysis_deps`.
Riutilizzabili `../moving_average_sim/simulate.py`, `../archive_review/analyze.py`,
`bags.py`, `integrator_probe.py`, `transient_diagnosis.py`. Per leggere dati usare
catalogo già generato; non rieseguire script dell'archivio NVIDIA_RUNS.

Prima di lanciare un batch verificare i processi della sola campagna tramite
command line, niente worker duplicati o interferenze con altri esperimenti.
Un solo worker di questa campagna, BLAS a un thread; salvare risultati e stato
tra batch, lavorando a tranche bounded. Non occupare la notte con sleep/poll.
Su questo host Get-CimInstance Win32_Process restituisce access denied; usare
Get-Process più PID/checkpoint del proprio worker come ripiego. Non terminare
processi non attribuibili con certezza alla campagna; evitare escalation inutili.
Ogni turno leggere `STATE.json` e `NIGHT_LOG.md`, svolgere il prossimo lavoro
utile, aggiornare numeri/limiti e salvare figure leggibili e codice verificato.
Distinguere sempre dati reali, replay condizionato e simulazione autonoma.

Consegna: `RESULTS.md` con grafici, confronti riproducibili, candidato se emerge,
parametri e intervallo di validità, problemi irrisolti e prossimo esperimento
minimo necessario. Non dichiarare pronto al robot un candidato solo simulato.
Verificare manifest dell'archivio alla consegna. Notificare soltanto risultati
significativi, completamento o un impedimento che richiede il proprietario.

Entro il termine produrre una sintesi onesta anche se non esiste un vincitore.
Non continuare costi/simulazioni oltre il termine della notte; se il primo
risveglio avviene dopo il termine, consolidare soltanto quanto già ottenuto.
