# Tre run FFI riuscite: valutazione del 23 ottobre 2026

Pull fast-forward eseguito da `1c25bd7` a `c80dd78` sul branch
`codex/feedforward-i-20261023`. Analisi dei CSV reali delle 17:43, 17:51 e 17:56,
con riferimenti reali del 22/10 alle 20:20 e del 23/10 alle 12:32.
Nessuna modifica al controllo e nessun comando ai robot.

**Giudizio:** le tre prove dimostrano il completamento ripetuto di questa route Z
e un buon recupero della forza media richiesta. Il sistema conserva oscillazioni
residue e transitori importanti. Sono risultati utilizzabili per una tesi sulla
pianificazione/esecuzione dell'inserimento con limiti sperimentali espliciti;
non dimostrano un inseguimento della forza uniforme e preciso in tutte le fasi.

## Cosa è migliorato

Le tre prove usano gli stessi parametri: feedforward SOCP offline + I ATI,
k_eff 680 N/m per mano, K_I 0,0016 m/(N s) sull'errore della somma, velocità
massima nominale+trim 2 mm/s per mano, trim ±3 mm. I due nodi registrano
cart_k 1500/20, cart_d 60/7, friction/Kalman attivi, 1 kHz. I tre profili
offline locali hanno lo stesso hash: è ripetuto lo stesso piano.

| Metrica | FFI 17:43 | FFI 17:51 | FFI 17:56 | Online 22/10 20:20 |
|---|---:|---:|---:|---:|
| Esito registrato | entrata, no JAM | entrata, no JAM | entrata, no JAM | entrata, no JAM |
| RMS oscillazione sx/dx, 0,8–2,5 Hz [N] | 0,120 / 0,119 | 0,124 / 0,123 | 0,112 / 0,110 | 0,155 / 0,156 |
| RMS oscillazione somma [N] | 0,239 | 0,246 | 0,222 | 0,311 |
| RMS banda / forza media [%] | 2,89 | 2,97 | 2,70 | 3,57 |
| RMS errore media delle mani, intera route [N] | 0,304 | 0,339 | 0,323 | 0,256 |
| RMS errore media delle mani, esclusi 1,5 s a ciascun bordo [N] | 0,175 | 0,191 | 0,187 | 0,215 |
| Bias della media delle mani, intera route [N] | −0,026 | −0,011 | −0,031 | −0,014 |
| Minimo normale di una mano, intera route [N] | 2,95 | 2,91 | 3,12 | 3,27 |
| Massimo assoluto del trim in route [mm/mano] | 0,843 | 1,048 | 0,660 | — |
| Aggiornamenti con limite di velocità in route [%] | 6,84 | 8,79 | 7,68 | — |
| Errore in sede *stimato dal robot* [mm] | −0,45 | −0,05 | +0,20 | +0,39 |

L'oscillazione in banda è circa il 21–29% inferiore al riferimento online in
valore assoluto; la riduzione relativa alla forza media è circa il 17–25%.
Il confronto è descrittivo: cambiano controllore, pianificazione e condizioni
di presa, e il riferimento è una sola prova storica. Non isola un effetto causale.

Rispetto al feedforward puro delle 12:32, la forza non rimane a circa metà
target: i rapporti mediani misurata/richiesta sono circa 1 in tutte le nuove prove.
Il feedforward puro aveva 0,179 N RMS sulla somma, ma 4,15% della forza media
e 2,02 N RMS di errore della media delle mani rispetto al target. Non è un
riferimento migliore solo perché ha un'onda più piccola in newton.

## Cosa resta da risolvere

**Le oscillazioni sono ancora visibili.** Il picco dello spettro della somma è
a circa 1,20, 1,10 e 1,00 Hz nelle tre run, con ampiezza 0,170, 0,148 e 0,176 N.
Questa ampiezza spettrale non coincide con il massimo del segnale né con l'RMS.
Nello zoom 5–10 s le forze registrate di una mano coprono circa 1,0–1,2 N fra
minimo e massimo: la finestra comprende più cicli e una variazione del target,
quindi il suo range non va presentato come ampiezza di ogni oscillazione.

Anche la banda 2,5–10 Hz contiene forza variabile: RMS della somma 0,128,
0,143 e 0,131 N, contro 0,136 N del riferimento online. Il miglioramento
si concentra nella banda bassa analizzata; non c'è un abbattimento generale
delle componenti più veloci. L'RMS di una singola banda non sostituisce l'errore
rispetto al target e non certifica stabilità.

**I transitori peggiorano la metrica completa.** Gli RMS di 0,42 / 0,54 / 0,51 N
sulla somma citati nei README delle run escludono gli ultimi due secondi.
Ricalcolando l'intera route diventano 0,609 / 0,679 / 0,645 N. La differenza
non è un conflitto fra analisi: cambia la finestra. Senza 1,5 s iniziali e finali
il tracking centrale migliora rispetto al riferimento; includendo tutto,
il tracking RMS è peggiore. Vanno mostrati entrambi.

**Nel finale il riferimento non rispetta il limite dinamico del nominale.**
Il piano offline comune passa da 3,636 a 5,659 N/mano tra 18,76 e 18,98 s:
circa 2,022 N in 0,22 s. Per la legge implementata:

\[
\Delta s_{ff}=\frac{\Delta N^*}{680}=2,97\ \mathrm{mm/mano},\qquad
T_{min}=\frac{2,97}{2}=1,49\ \mathrm{s}.
\]

La richiesta nominale sale quindi molto più rapidamente del nominale applicabile.
Il rate limit è visibile nei log; durante questa salita il feedforward ha priorità
nel budget di velocità condiviso con l'I. Poco dopo, il target ritorna a circa
3,62 N/mano mentre la forza misurata sta ancora salendo. È un contributo concreto
all'errore e alla sovraelongazione del finale. Non basta aumentare K_I per far
eseguire al comando una variazione di 3 mm in 0,22 s mantenendo 2 mm/s.

Il calcolo riguarda il modello nominale del software, non una misura indipendente
di rigidezza della presa. Non spiega da solo l'onda nella parte centrale della route.
All'inizio esiste un altro raccordo rilevante: pickup a circa 5,8–6,1 N/mano,
target to_mouth circa 3,62 N, poi target route circa 4,5 N. I grafici delle fasi
mostrano anche questa risposta, non soltanto il tratto centrale più regolare.

**Non emerge saturazione di corsa del trim.** Il trim rimane entro circa ±1,05 mm
contro ±3 mm disponibili e non si registra un comando al limite di posizione.
`ffi_i_limited` significa che l'incremento I è stato limitato anche da vincoli
di velocità; non significa automaticamente trim arrivato a ±3 mm.
Le percentuali di saturazione usano aggiornamenti distinti `ffi_seq`, non righe CSV
che ripetono lo stesso stato. Non risultano aggiornamenti intermedi saltati dal log.

La mediana dell'intervallo di controllo FFI è circa 28 ms; il CSV è circa 100 Hz.
C'è un solo aggiornamento con dt clamped nella prima run, all'ingresso in route;
nelle altre due non ce ne sono in route. Il solo aumento della frequenza del log
non interviene sulla dinamica responsabile delle forze misurate.

## Valutazione per la tesi

Una formulazione sostenuta dai dati è:

> In tre ripetizioni della medesima traiettoria Z, l'esecuzione con forza nominale
> SOCP e correzione integrale ha completato l'inserimento senza JAM, con rapporto
> mediano fra forza misurata e richiesta prossimo all'unità e oscillazione nella
> banda 0,8–2,5 Hz pari a circa 2,7–3,0% della forza media. Persistono oscillazioni
> residue e transitori di inseguimento, particolarmente in corrispondenza di
> variazioni del riferimento incompatibili con il limite di velocità nominale.

Non sono sostenute le affermazioni «oscillazioni eliminate», «tracking preciso in
ogni fase», «stabilità robusta dimostrata», «accuratezza di posa certificata entro
0,5 mm» o «successo garantito». Tre riuscite sono evidenza di ripetibilità locale
su questa route; le quote `believed_mm` derivano dalla stima del robot e richiedono
metrologia indipendente per una conclusione sull'accuratezza del pezzo. Le normali
restano positive, ma questo non misura indipendentemente lo slip né dimostra
che tutti i vincoli di attrito/momento siano rispettati nel contatto reale.

Separare completamento del compito, tracking di forza e transitori è coerente con
il benchmark [NISTIR 8097](https://nvlpubs.nist.gov/nistpubs/ir/2015/NIST.IR.8097.pdf),
che distingue metriche di assestamento/disturbo e metriche funzionali di assembly.
Non sto usando i numeri di altri robot come soglia universale per questo setup.

## Il prossimo passo con una motivazione verificabile

1. **Verificare/raccordare il piano prima di ritoccare i gain.** Investigare perché
   il piano contiene il rialzo terminale e valutare una transizione anticipata o
   una temporizzazione compatibile con la velocità nominale. Non basta abbassare o
   filtrare un target necessario alla presa: vanno preservati i vincoli SOCP e il
   sostegno del pezzo lungo la transizione. Il piano è ammissibile staticamente;
   manca una garanzia che sia seguibile dinamicamente dall'esecuzione.
2. **Separare l'onda dell'anello dai disturbi del movimento con una prova dedicata.**
   Con la stessa presa, posa e target costanti, confrontare correzione I attiva
   e chiusura comandata mantenuta fissa, partendo da forza effettiva simile.
   Se l'onda rimane anche senza correzione I, cambiare solo K_I non ne rimuove la
   sorgente; se compare principalmente con I, è motivata l'identificazione e
   taratura di quell'anello. La comparazione va eseguita mantenendo i controlli
   esistenti di forza/tracking; questa analisi non ha eseguito prove hardware.
3. **Ripetere il confronto con condizioni controllate**, contando tutti i tentativi
   e verificando la sede del pezzo con una misura esterna. Non cambiare assieme
   K_I, impedenza, filtri e piano: altrimenti non si attribuisce il miglioramento.

Le dinamiche del servo, la flessibilità e i filtri sono fattori reali nel progetto
di anelli forza→movimento: [Keemink et al., IJRR 2018](https://journals.sagepub.com/doi/10.1177/0278364918768950)
li analizzano in un quadro di admittance control. Questo è contesto progettuale,
non un'identificazione del nostro impianto. Da queste sole run non attribuisco
l'onda a friction, Kalman, filtro ATI o ritardo puro, né ricavo un margine di fase.

## Grafici, metodo e riproduzione

- [Forze, zoom e oscillazioni delle tre run](01_tre_run_forze.png).
- [Tutte le fasi e i termini di controllo](02_transitori_e_comando.png).
- [Confronto storico con RMS assoluto, relativo e tracking](03_confronto_storico.png).
- [Zoom del transitorio finale e del limite nominale](04_zoom_finale.png).
- [Metriche complete e hash dei CSV](metrics.json), [tabella CSV](summary.csv).

Le forze delle singole mani sono proiettate sugli assi +x delle flange misurate,
con la stessa convenzione della normale registrata. La differenza massima rispetto
alla somma nel log è inferiore a 0,025 N. I grafici di forza/tracking non hanno
filtri temporali aggiunti. RMS banda: ricampionamento al dt mediano dei timestamp
reali, detrend cubico, Butterworth ordine 2 SOS avanti/indietro, bordi di 1,5 s
esclusi. Spettro: ricampionamento, detrend cubico, finestra Hann. La banda 0,2–0,8 Hz
nel JSON è solo diagnostica: su 19 s le frequenze più basse hanno pochi cicli.

Gli errori sono forza misurata meno target, senza traslare il target per compensare
il ritardo. Per «media delle mani» si intende `(N_L+N_R)/2`; non è una media temporale.
Il suo errore RMS è metà dell'errore RMS della somma. I valori di ogni singola mano
sono riportati separatamente nel JSON. Le statistiche route usano tutte le righe
registrate; le finestre parziali sono indicate esplicitamente.

Esecuzione riproducibile da root con Python e numpy/scipy/matplotlib:

```text
python docs/analysis_20261023/ffi_three_successes/review.py
```

Lo script riconosce anche le dipendenze locali in `tmp/force_plot_deps`, se presenti.
