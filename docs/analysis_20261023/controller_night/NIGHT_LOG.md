# Registro ricerca notturna

## 23/10, avvio verso le 22:20 Singapore

Autorizzata conservazione baseline e ricerca notturna sui controllori.
Conservati 431 file in ZIP con verifica di ogni hash e del contenitore.
I tre moduli runtime principali coincidono con le 12 revisioni delle run FFI.
Il protocollo dichiara holdout cronologico, confronto sotto gli stessi limiti,
separazione tra replay e modello predittivo e termine alle 09:00 Singapore.
Non è ancora stato selezionato né validato un nuovo controllore.

## Prima tranche completata

Automazione heartbeat attiva ogni 30 minuti con termine 24/10 01:00 UTC.
Congelati split cronologico, 46 candidati e 27 modelli in frozen_design.json.
Eseguiti 1.242 test autonomi con step nei due versi e disturbo additivo.
Verificati identità update FFI su 1.000 tick e limite 2 mm/s su tutti i casi.
Salvati metriche complete, shortlist per famiglia e figura ispezionata.
Nessun vincitore validato: P/PD hanno offset, I più sovraelongazione in mismatch;
PI/PID migliorano parte del compromesso, alcuni P migliori sono al bordo griglia.
La shortlist è solo sintetica e non va chiamata raccomandazione per il robot.
Prossimo lavoro utile: capacità predittiva dei modelli e confronto su sviluppo.
Accesso a Win32_Process via CIM negato; Get-Process e propri checkpoint come
ripiego. Nessun processo persistente della campagna lasciato vivo alla fine della tranche.

## Steering: damping cartesiano interno

Lorenzo propone che potesse bastare più damping. Aggiunta priorità esplicita in
DAMPING_STEERING.md: confronto D interno mantenendo FFI invariato, separato dal
D esterno sulla forza. Audit metadata: 34 record confermati a 60/7, due a 80/7;
le 12 FFI sono a 60/7. La Z online gain0,5 del 23/10 12:18 con D80 conserva banda
0,1583 N RMS media mani: non è una prova sul beneficio di D80 nella FFI.
Nessun valore nuovo impostato al robot o al launch di produzione.

## Heartbeat 23/10 22:44 Singapore — tranche damping

Ricerca mirata sul Desktop del sorgente kinova_controller: nessuna copia trovata.
Creati rigid_model.py e damping_physics.py: inerzie rigid-body/Jacobiani dall'URDF,
inerzia cartesiana 6D con accoppiamenti, K e D fisici separati. Solo 8 run di
sviluppo, 48 posture (3 istanti ×2 bracci×8 run). Verifica FK entro arrotondamenti
CSV: 7,48 µm posizione e 1,48e-5 rad orientamento; Jacobiano differenze finite
errore 2,38e-10. Il primo assert orientamento a 1e-5 è stato corretto a 3e-5 dopo
verifica della precisione CSV a 5 decimali, conservando il motivo della correzione.

384 sweep frequenza (2 contatti, 4 damping), 192 scenari autonomi FFI e 192
raffinamenti 5→2,5 ms. Limite 2 mm/s rispettato; differenza massima RMSE 0,000643 N,
overshoot 0,000245 N. Discretizzazione ZOH esatta, tick FFI25ms identico.
I modelli includono contatto ancorato ideale, masse URDF1×/2× e ritardo aggiunto
0/120/230ms fisso rispetto a D. Non rappresentano attrito/rotori/servo/board/slip.

Il confronto diagnostico model_consistency.py usa 8 development più le due Z a
I quasi spenta, non le 4 riservate. Comando/moto proiettati sulla normale fissa
opposta di metà route, non distanza euclidea né compressione diretta dei pad.
Fit osservazionale comando→moto FFI140–215ms, R²0,28–0,75. Modello15–25ms.
La previsione diretta con tutti i 6 comandi ha R² -1,093..0,070 (kc morbido) o
-0,506..0,136 (kc2700). Il modello ideale è quindi inadeguato come previsore,
nonostante FK e numerica corretti. Le diagnostiche I quasi off hanno fit
comando→moto debole R²0,09–0,14: non identificare una latenza da quei numeri.

damping_axis.py confronta D isotropo80 e solo base-y80. Normali |ny|>=0,990,
effetto normale simile nel modello e minore fase verticale per solo y.
Questo motiva soltanto una variante del futuro confronto sperimentale.
Preparati DAMPING_REPORT.md e DAMPING_AB_PROTOCOL.md. Figure 02/03/04 ispezionate;
nessun controllo reale cambiato, nessun processo persistente lasciato vivo.

Prossima tranche: modelli empirici comando/moto/forza con verifica tra run di
sviluppo e confronto di controllori sotto incertezza, mantenendo distinta
la calibrazione osservazionale dalla causalità e dal replay a residuo imposto.

## Heartbeat 23/10 23:14 Singapore — modelli empirici e replay dei controllori

Congelato empirical_design.json prima del calcolo: 316 forme LTI (ritardo,
primo e secondo ordine) e 54 ARX2 con ridge. Input/output ZOH100Hz, normale
opposta della prima posa route e filtro causale0,3–8Hz. Otto fold train7/test1
sulle sole 8 development. Nessuna riscalatura o riallineamento su test.
R² forza libera mediana0,36–0,40, peggiore0,27–0,29: nessuna famiglia supera
il gate provvisorio. Forza dato il moto misurato R² mediana0,926. ARX un passo
R²0,985 contro libero0,412: un fit locale alto non dimostra una buona dinamica.
Verificato recupero di un impianto noto (ritardo8 campioni, errore coefficienti
6,22e-15) e invarianza dei prefissi rispetto a campioni futuri alterati.

Congelato conditional_controller_design.json: 46 candidati ×16 modelli ×8run
=5.888 replay. Il residuo delle registrazioni resta imposto, non si dichiara
questa una previsione del robot. P/D in posizione assoluta, nessuna I nascosta
su P/PD; D filtrato50ms alla cadenza FFI, non a200Hz. Stessi limiti e priorità
nominale. Baseline forza errore1,78e-15N, comando1,73e-15mm; tutti≤2mm/s.
Nella griglia iniziale FFI I1,6 e PI(P0,05/I0,8 in mm,N,s) sono quasi pari
(11,8874 vs11,8907, circa0,03%). PD senzaI score peggiore32,7%. Nessun candidato
soddisfa tutte le soglie provvisorie in tutti i casi. Non chiamare I ottimale
fuori dalla griglia; anticipatori più forti/roll-off restano da provare.
Raffinati80 casi su4pezzi e4modelli: RMSE maxdiff0,00105N, banda0,000567N,
picco finale0,0218N. Figure05/06/07 ispezionate. Report e dati salvati.

Letto insert_open.Profile: profilo completo con tempi relativi per fase,
campionamento lineare con endpoint hold. Controller.follow è wall-clock paced
ma watch_jam/lands_high possono troncare fasi. Una preview basata sui futuri
timestamp osservati sarebbe un oracolo di scheduling: usare il profilo reale
e dichiarare ogni ipotesi di transizione; valutare sempre contro target originale.

Prossimo lavoro utile: ampliare separatamente anticipo/roll-off e gestione del
budget totale, poi preview/pianificazione vincolata. Non usare ancora le quattro
run riservate finché il confronto e i parametri non sono congelati. Nessun
processo persistente della campagna rimane vivo a fine tranche.

Checkpoint esclusivo verificato: checkpoints/tranche_20261023_2314.zip, 58 file, 3.804.686 byte; include sorgenti, report, JSON e figure di questa tranche. CSV/NPZ completi restano separati. NVIDIA_RUNS765 e sorgenti runtime332 verificati invariati.

## Heartbeat 23/10 23:44 Singapore — PD forte, allocazione e stress autonomi

Letti protocollo/stato/registro/risultati/steering e audit damping. Nessun Python
vivo prima del batch. Nessun accesso robot/ROS/deploy/push. Tutti i nuovi file
sono nella campagna. Il kernel conditional_controllers.py è esteso con callback
di allocazione e roll-off P+D opzionali; la versione precedente resta nel
checkpoint2314. Con opzioni disattivate, quattro metriche su tutte le5.888
combinazioni precedenti coincidono esattamente (maxdiff0).

Congelato advanced_controller_design.json prima del tuning:112 candidati,
16 modelli,8 development =14.336 casi. P esteso fino0,8mm/N, D0,1mm*s/N,
I0/0,4/0,8; D su misura50ms e roll-off P+D30/100ms. Baseline e griglia precedente
incluse. Sei leggi fissate hanno anche varianti feedback-prima e somma-proiettata,
senza cambiare limiti componenti/somma. Verificate240.000 proiezioni casuali.
Il migliore PI P0,1/I0,8/roll-off100ms score11,844729 vsFFI11,887384: -0,36%,
non un miglioramento dimostrato. Nessuno supera tutte le soglie provvisorie.
La priorità feedback può ridurre un picco e peggiorare minimi/offset altrove.

Raffinati128 casi su4 development,4 modelli,7 finalisti provvisori più un PD
forte aggiuntivo. Il test complessivo5→2,5ms FALLISCE: maxdiffRMSE0,02649N,
banda0,01800N, picco0,05553N. Ulteriore5→1,25ms FALLISCE:0,03905/0,01881/0,06273N.
Valori/script/assert conservati senza cambiare tolleranze. Il caso critico è
PD P0,3/D0,1/roll-off100ms con1500N/m e230ms. Senza quel caso, il sottoinsieme
moderato5→2,5ms ha RMSE0,00248N, banda0,000567N, picco0,02179N. I timestamp FFI
vengono arrotondati alla griglia: questo cambia il timing degli aggiornamenti
oltre alla propagazione dell'impianto. Non attribuire la differenza soltanto
a instabilità continua del PD. Per riesaminarlo serve replay a eventi esatti.

Congelato advanced_autonomous_design.json:8 ipotesi selezionate,16 impianti,
3 varianti =384 stress autonomi, senza residuo registrato. Target4→6→4N/mano,
rumore0,013N/mano, jitter15–35ms, onda fisica0,15N/mano a1,2Hz da12s e bias
+0,5N/mano16–22s. Rumore comune e causale, limiti invariati. Altri384 casi a
1,25ms con identici eventi/rumore: maxdiffRMSE0,000193N, overshoot0,000005N,
banda0,000312N. Alcuni modelli con PD forte conservano banda≈0,28N senza rumore;
è un rischio del guadagno ipotizzato, non una spiegazione causale del robot.

Verifica numerica indipendente con la classe reale NominalIntegral estratta
via AST, senza ROS:48 scenari,61.648 update, errore comando0. Il primo worker
di verifica rileggeva/decomprimeva NPZ a ogni update: fermato solo il proprio
PID19452, attribuito da assenza di altri Python, path e ora di avvio23:53.
Corretto il caching in RAM e rieseguito in1,8s con esito positivo. Nessun batch
duplicato o processo estraneo terminato. BLAS/OMP/MKL sempre a un thread.

ADVANCED_CONTROL_REPORT.md, CSV/JSON/NPZ, figure08/09 prodotti e ispezionati;
stessa scala verticale nei pannelli a regime per distinguere rumore piccolo e
onda fisica. Nessun holdout letto dal tuning. Prossimo: preview del vero profilo
offline/pianificazione fattibile, senza forza futura né scheduling preso dai
timestamp futuri dei log. Damping interno resta confronto distinto D-only:
nessun nuovo dato o sorgente interno trovato in questa tranche.

Conservazione verificata alle15:56:29UTC: NVIDIA_RUNS identico su765 file,
4.080.005.175byte; SHA-256 ZIP baseline e431 elementi verificati,332 sorgenti
runtime invariati. Nessun worker persistente a fine tranche. La campagna resta
attiva fino al termine previsto, prossimo studio descritto in STATE.json.

Checkpoint esclusivo tranche_20261023_2344.zip creato e verificato:79 file,
6.879.174byte. Include disegni congelati, codice, report, metriche JSON e figure;
CSV/NPZ restano nella campagna. Git diff e stato dei file tracciati vuoti.

## Heartbeat 24/10 00:14 Singapore — preview del profilo e limite del nominale

Letti istruzioni/stato/report/registro/steering/audit e log precedenti; nessun
Python vivo. SHA-256 checkpoint2344 verificato prima del batch. Nuovo lavoro
solo nella campagna, BLAS/OMP/MKL/NUMEXPR un thread. Nessun robot/ROS/deploy/push.
Kernel conditional_controllers.py esteso con nominal_wants opzionale; versione
precedente nel checkpoint2344, metriche baseline I/PI esattamente identiche.

Congelato profile_preview_design.json prima dei calcoli: due leggi già fissate
(FFI originale e PI P0,1/I0,8/roll-off100ms), shift oppure inviluppo solo rialzi,
h150/300/600/1200ms e clockoffset-40/0/+40ms:50 candidati x16 modelli x8run=6.400
replay. Tutti i profili veri verificati con hash raw dei metadati:8 file,
4 contenuti unici. Preview solo nella fase corrente con endpoint hold, niente
forza futura né orari futuri osservati delle fasi. Il phase_t0 vero non è nei
log: elapsed ricostruito causalmente dalla prima riga della fase. Audit target
RMSE~0,01-0,03N, max~0,22-0,65N sui tratti ripidi. Offset sono ipotesi, non bounds
identificati. Delta-preview ancorata al nominale esatto per conservare baseline;
non si presenta come ricostruzione assoluta di un nuovo controller eseguito.

Raggruppati gli offset nel caso peggiore, senza selezionarli a favore. Nessuna
preview della griglia migliora lo score rispetto alla sua legge senza preview:
PI solo-rialzi150ms migliore +6,08%; FFI solo-rialzi150ms +8,83%. Nessun candidato
supera tutte le soglie. Il feedback rimane sul target originale, limiti identici.
Softwarechecks endpoint hold/nessun cambio fase futuro; prefissi invarianti
cambiando futuri phase/time/nominal nei log e tutte le misure di forza.
Raffinamento240 casi5→2,5ms PASSA: ΔRMSE0,001496N, banda0,001128N, picco0,013674N.

profile_feasibility.py valuta il vero comando nominale ff=(target-pickup)/680:
32 casi fase e16 transizioni. Picchi richiesti Z87,8/L143,0/S58,8/T109,0mm/s
contro2mm/s; frazione tempo route sopra limite3,04/6,47/5,59/3,59%. Calcolato
fit minimax Lipschitz con tutta la preview della fase, estremi liberi e nessun
ritardo: inevitabile massimo scarto nominale Z1,294/L1,410/S1,347/T1,070mm/mano.
NON è un limite inferiore dell'errore di forza reale: geometria/board/contatto
possono cambiare la forza più rapidamente. Verificato bound analitico e50
griglie casuali contro tutti i vincoli a coppie, pendenza e maggiorazione.
Salvati fit minimax e inviluppo superiore; non sono nuovi piani SOCP validati.

Letto il sorgente del planner: SOCP100ms, traccia controllo20ms, Profile.sample
interpola le righe. Un nuovo SOCP può quindi diventare rampa20ms e il primo
target della fase può provenire dalla precedente soluzione. La prima tabella
endpoint-only dava salto0 per Z/S/T: espansa, prima della consegna, anche al
minimo dei primi150ms slow_seat. Riduzioni Z1,86N/1,37s, L1,566N/1,152s,
S1,11N/0,82s, T0,26N/0,19s nel SOLO nominale a2mm/s. Nessuna causalità delle onde
attribuita al campionamento da questo riscontro. cart_d interno non alterato.

PROFILE_PREVIEW_REPORT.md, figure10/11 e CSV/JSON/NPZ prodotti e ispezionati;
riportati anche minimi individuali reali (le alternative predicono solo media).
Footer riposizionati dopo ispezione per evitare taglio. Quattro holdout ancora
esclusi. Prossimo lavoro: pianificazione scalare a velocità limitata distinta
dal riferimento del feedback e verifica dei compromessi/margini, poi congelare
finalisti e fare una sola valutazione delle quattro run riservate.

Conservazione verificata alle16:26:47UTC: archivio identico765 file,
4.080.005.175byte; ZIP baseline e431 elementi verificati,332 sorgenti runtime
immutati. Git diff/stato tracciati vuoti, nessun Python persistente vivo.
Checkpoint nuovo include anche i log di processo; i vecchi checkpoint sono
immutati. Campagna ancora entro termine e attiva secondo STATE.json.

Checkpoint esclusivo tranche_20261024_0014.zip creato/verificato:101 file,
8.652.037byte. CSV/NPZ completi restano separati nella campagna.

## Heartbeat 24/10 00:44 Singapore — piano e feedback, picchi a eventi esatti

Istruzioni/protocollo/stato/report/registro/steering/audit/log letti, nessun Python
vivo. Verificato checkpoint0014 prima di lanciare il worker. BLAS/OMP/MKL/NUMEXPR
un thread, batch sequenziali, output solo nella campagna. Nessun robot/ROS/deploy/push.
Kernel conditional_controllers.py esteso con feedback_targets opzionale, errore
iniziale corretto per ingresso bumpless; versione precedente nel checkpoint0014.

Congelato planned_reference_design.json:38 configurazioni x16 modelli x8 run
=4.864 replay. Due leggi già fissate, piani originale/maggiorante/minimax, feedback
originale/pianificato, offsetclock±40/0ms. Piani campionati assolutamente e incluso
lookup dell'originale come controllo del clock, oltre alle baseline log esatte.
La maggiorante non abbassa il profilo scalare allo stesso tempo; minimax può
abbassarlo e non è candidato con margini vecchi invariati. Nessun piano nuovo
migliora lo score provato rispetto al controllo del clock equivalente. Non
è previsione reale: residuo registrato imposto, impianti non validati.

Congelato planned_reference_autonomous_design.json:14 configurazioni a offset0
(inclusi alias baseline) x16 modelli x2 condizioni x4 forme piano=1.792 scenari
autonomi e1.792 raffinamenti. Precarico al primo target route, prelude2s e fasi
su calendario sintetico noto, non orari futuri dei log; niente preview fuori
dalla fase corrente. Variante rumore0,013N media mani,jitter15-35ms,onda fisica
0,15N/mano1,2Hz. Sempre stessi limiti. Raffinamento2,5→1,25ms PASSA:
ΔRMSE0,000428N, banda0,000264N, picco0,001411N. Salvataggi separati di bande
dell'errore0,2-0,8/0,8-2,5/2,5-10Hz, senza cambiare score; la banda grezza
include anche variazioni volute del target. Alias baseline esatti.

planned_reference_validation:41.240 aggiornamenti della classe I reale,
maxerrorecomando2,22e-15mm; causalità dei prefissi e maggiorazione scalare
verificate, pendenze dentrofase≤2mm/s. Raffinamento608 casi5→2,5ms FALLISCE
il criterio picco0,05N: maxpiccoΔ0,183896N, RMSE0,002857N, banda0,001167N.
Log/JSON/assert conservati. Non si è allargata la soglia.

Creato event_exact_delay.py per il solo ritardo puro: nessun arrotondamento
dei timestamp di controllo; unione tempi misure e comandi ritardati, RMSE
pesato sulla durata e picchi di intervallo; solo bande campionate2,5ms.
456 casi: tutte38 varianti,3 modelli480/.12-680/.18-1500/.23, prima development
per pezzo. Baseline esatta, input ritardato sempre precedente al controllo,
limiti verificati. Identità indipendente con vera classe FFI su123.720 update,
maxerrore2,66e-15mm. Critico Z PI+maggiorante nel feedback+offset40ms,k1500/.23:
picco5ms6,804285N,2,5ms6,988181N,eventi6,981272N. La griglia5ms perdeva un massimo
molto breve. Residuo ZOH fra misure ancora ipotizzato: non è prova della vera
forma fisica del picco, né validazione dei4.864 casi o dei modelli lag/risonanti.

PLANNED_REFERENCE_REPORT.md e figure12/13 prodotti e ispezionati; etichetta
laterale del grafico del picco riposizionata dopo QA. Full wrench/margini mai
dichiarati preservati dai piani modificati. Damping interno sempre distinto
e invariato, confronto D-only già pronto ma dati/sorgente mancano.

Creato FINAL_COMPARISON_FREEZE.json esclusivamente PRIMA di leggere/analizzare
i CSV riservati:7 confronti FF/I/P/PI/PD/PID,16 modelli,4 run=448 casi attesi,
dt2,5ms, score/diagnostiche/limiti fissi. SHA-256 freeze
c84b3ea52d1e810ebfb431c37266edfd0c4cded16c641b797da25dbdb0707b82.
Scelte da ranking development e una PI semplice quasi pari; nessun candidato
hardware. Manifest e supplemento transitive source/runtime creati esclusivi,
freeze originale immutato. Le quattro run sono holdout retrospettivo per questo
tuning, già viste in vecchie analisi: dichiarazione nel freeze/report.

Prossimo: unico batch holdout bloccato con marker/progresso esclusivo, verifica
hash sorgenti/base/supplemento/CSV, ripresa soltanto run mancanti dopo eventuale
interruzione. Nessun tuning dopo i risultati. Poi robustezza analitica/limiti
dei modelli e protocollo minimo di dati, senza trasformare score simulati in
raccomandazioni sul robot. Holdout ancora NON analizzato in questa tranche.

Conservazione verificata17:02:30UTC: NVIDIA_RUNS identico765 file,
4.080.005.175byte; ZIP baseline e431 elementi verificati,332 sorgenti runtime
invariati. Tutti i fingerprint del freeze e del supplemento corrispondono.
Nessun Python persistente; Git diff/stato tracciati vuoti. Campagna attiva,
termine sempre24/10 09:00Asia/Singapore, nessun obiettivo hardware dichiarato.

Checkpoint esclusivo tranche_20261024_0044.zip creato e verificato:138 file,
10.557.329byte, inclusi freeze finale, supplemento, report, figure e log di
fallimento. Tracce NPZ e tabelle CSV restano separate nella campagna.

## Heartbeat 24/10 01:14 Singapore — holdout congelato completato

Letti AGENTS/PROTOCOL/STATE/NIGHT_LOG/RESULTS, steering damping e audit.
Checkpoint0044 verificato prima del batch, nessun Python vivo; fallback
Get-Process a causa dei precedenti limiti CIM, nessuna interferenza con altri
processi. Un solo worker sincrono, OPENBLAS/OMP/MKL/NUMEXPR=1, niente pycache.

evaluate_locked_holdout.py: prima14 fingerprint sorgenti+versioni runtime,
manifest freeze/supplemento e SHA dei4 CSV; marker esclusivo START alle
17:18:55UTC, completamento17:18:59UTC.7candidati x16modelli x4run=448 casi,
dt2,5ms. Un solo batch senza modificare i parametri. Salvataggi immutabili
per run, NPZ completo, hash e marker completed, progresso complete/PIDnull.
No ROS/robot/push; holdout retrospettivo dichiarato. Identità baseline max
3,11e-15N/2,17e-15mm, limiti2mm/s verificati.

Score peggiore modello per run poi media4: FFI11,00702; PI filtrata10,82928
(-1,6148%); PI semplice10,86160(-1,3211%); PID10,89892(-0,9821%). FFsolo14,80371,
P16,15831,PD15,95527.0/448 supera tutti gli obiettivi esplorativi. Nessun nuovo
controller selezionato per il robot, nessuna nuova identificazione dei tempi
o garanzia di stabilità. Le ipotesi fissano residuo/contatto/geometria/fasi.

Report da soli risultati salvati, senza rieseguire replay. Osservati route
RMSE Z/L/S/T0,322/0,275/0,530/0,277N; banda0,8-2,5Hz0,111/0,104/0,077/0,104N.
Minimi mani attivi Z3,131/3,118,L3,308/3,291,S2,055/3,380,T2,546/3,280N.
Asimmetria slow_seat S/T visibile; delta scalare non predice singoli contatti.
Richiesta comune non è margine full wrench, timestamp non sincroni; ricostruzione
fn_N/wrench differisce fino0,0144N somma. PiccoLbaseline6,610N; PI filtrata
centrale ipotizzata6,329N, ancora2,543N sopra richiesta. Eccesso200ms2,646/2,365N.
Forza grezza/errori separati in3bande, limitazioni di tutti i tick documentate.

Durante QA figurascale individuata penalità sviluppo FFsolo1,25e8: due casi S
del modello k1500 con media≤0, minimo-0,025N, già nel kernel congelato. Non è
nuovo errore o enorme forza misurata. Report aggiunge spiegazione, barra annotata
fuori scala; score/JSON originali conservati.3figure14/15/16 ispezionate,
report/CSV/JSON e manifestconsegna salvati. Nessuna modifica ai14 sorgenti congelati.

Priorità damping ribadita: FFIsempreD60/7, niente esperimentoD-only disponibile,
ζ/ritardo dei replay non sono cart_d. Protocollo futuro già pronto, nessun
nuovo compensatore ritardo ammesso senza modello predittivo adeguato.

Conservazione verificata17:22:08UTC:765file NVIDIA_RUNS identici,
4.080.005.175byte; ZIPbaseline+431contenuti e332sorgenti invariati.
Gitdiff/stato tracciati vuoti, nessun Python persistente. STATE aggiornato:
prossime tranche solo robustezza analitica ipotetica/sintesi/protocollo dati,
nessun ulteriore tuning su holdout. Termine24/10 09:00Singapore invariato.

Checkpoint esclusivo tranche_20261024_0114.zip creato e verificato:156 file,11.907.339 byte. NPZ e CSV conservati separatamente; manifest di consegna include i loro hash.


## Heartbeat24/10 01:44Singapore — robustezza locale, nessun nuovo tuning

LettiAGENTS,PROTOCOL,STATE,NIGHT_LOG,RESULTS,steeringD/audit e processo holdout.
Nessun Python vivo; checkpoint0114 SHA corrisponde f38aee5d3c9105153ad1bc336a398c0c351cfee902fd82d434a7dd92fdb9a44c.
Fallback Get-Process già autorizzato perché CIM negato; nessun processo terminato.
Un solo worker BLAS/OMP/MKL/NUMEXPR1. Codice/nuove uscite solo in campagna.

LINEAR_ROBUSTNESS_DESIGN creato esclusivo prima del calcolo:7leggi già congelate
x16impianti ipotizzati x3tick20/25/30ms=336anelli locali, nominale/target fermi,
limiti inattivi. Non legge CSV né ripete replay/tuning holdout. Plant ZOH esatto
con ritardo frazionario per lag/risonanza, puro gain+delay al campionamento;
C(q) deriva dall'I backward Euler, D sulla misura tau50ms e rolloff già congelati.
Nessuna conversione cart_d->ritardo/ζ. Nessun nuovo candidato o compensatore.

Tutti336 anelli locali hannoρ<1. Verifica radici polinomiali versus matrice
di stato costruita separatamente su tutti336:Δρmax5,493e-13; soglia I+ritardo
nota e transfer lag ZOH verificati, errore frontiera6,66e-16. Non certifica
robot/saturazioni/jitter/contatto, che i modelli non descrivono adeguatamente.
FFI minPMattraversamento24,6° nella griglia; massimo|S|banda0,8-2,5Hz4,204,
PI filtrata1,962,PIsemplice1,559,PID1,426,P1,176,PD1,411,FF1. Questi rapporti
di ampiezza sono ipotesi di disturbo imposto, non previsioni RMS delle run.
P/PD centrali S(0)=0,936; I S(0)=0 solo nell'ideale stabile non limitato.
Conseguenza: onde non implicano instabilità locale; banda e tracking possono
peggiorare/migliorare separatamente. Nessuna attribuzione causale ai log.

LINEAR_ROBUSTNESS_REPORT,JSON/CSV/verifiche e figura17 salvati; figura ispezionata.
MINIMUM_IDENTIFICATION_PROTOCOL aggiunge pacchetto minimo di timestamp nativi,
ricezione/consumo, target applicato, stato, wrench raw per mano e FFI, in futuro
D-only. Ripetizioni e finestre fissate, nessun cherry picking o nuova soglia
di sicurezza. Margine fullwrench distinto dalla richiesta scalare comune.

Originali verificati17:49:13UTC:765file/4.080.005.175byte identici, baselineZIP
e431elementi,332sorgenti runtime invariati. Verificati inoltre tutti14fingerprint
e ciascun artifact del manifestconsegna holdout precedente; nuova consegna
locale esclusiva. STATE aggiornato: eventuale audit sorgenti della pipeline
segnali/clock, niente altro tuning; cutoff24/10 09:00Singapore invariato.

Checkpoint esclusivo tranche_20261024_0144.zip creato e verificato:167 file,12.192.931 byte. Nessun worker persistente; Git tracciato invariato.


## Heartbeat 24/10 02:14 Singapore — audit sorgenti della pipeline

Letti AGENTS/PROTOCOL/STATE/NIGHT_LOG/RESULTS, steering damping/audit e process
log precedente; nessun Python vivo. Checkpoint0144 SHA79ffd358ffca909e1c3738b07e56335225b9039747dacff3f91cb50fb32ef000
verificato. CIM già negato su questo host, fallbackGet-Process e marker PIDnull.
Un solo worker; BLAS/OMP/MKL/NUMEXPR1, no pycache, cache Matplotlib nella campagna.
Alcune ricerche iniziali con glob in argomento path non supportato da rg su
Windows/nonexistent logger.py/cell.yaml corrette usando file trovati e percorsi
espliciti; nessuna modifica o esecuzione di sorgenti ROS. Tutti i read utili riusciti.

signal_pipeline_audit.py legge soltanto sei sorgenti e12metadata originali del
freeze; nessun CSV numerico, replay o tuning. Backendmedia n_avg=round(period
config100Hz x ft_rate200Hz)=2 messaggi su entrambe le mani di tutte12run.
Il topic è ft_data_avg, non il singolo campione raw. Nodo AtiNode acc/n mean
ogni every(default5), launchlo passa conconfigure_sensorfalse; default non
prova runtimeevery o frequenza hardware. raw ft CSV è media in sensoraxes
prima di bias/gravità/frame. Insertion proietta su normali misurate e somma
clippata, FFI aggiunge I senza ulteriore filtro misura. Cart_d è servo diverso.

Gitshow su12revisioni x6file=72confronti; tutti disponibili;22anchor per
revisione=264definizioni AST identiche al corrente. Sorgente fisso/snapshot e
metadata dichiarato non dimostrano eventuali patch non committate sul processo;
limite esplicito. SHAmetadata/sorgenti/anchor/righe salvati nel JSONprovenienza.

Algebra FIR ideale:1kHz raw,5campioni perbloccononoverlap,ultimi2blocchi equivale
a10consecutivi; controlloindipendentecascata entro2e-14 eguadagnoDC1. A1Hz
riduzione0,016284%,a2,5Hz0,101750%,a10Hz1,62065%;ritardofiltro4,5ms relativo
all'ultimo raw incluso, senza doppioconteggio blocchi né coda/consumeage/servo.
Figura18illustra anchefinestreideali100/200ms, NON nuovi controllerottimizzati
o simulazione run. Figura ispezionata, report rigenerato dal soloauditJSON per
chiarire chei4,5ms includono già finestra blocchi; niente riesecuzioneGit/CSV.

Sorgente dimostra sequenza command->wait->read->FFIupdate->log;dsnuovo e target
cartesianogiàpubblicato non sono stessoistante. ft_stampèultimomessaggio,
noncentro/tuttiicampionidellamedia; ft_age/joint_age sono da callback locale,
non acquisizione hardware. ROSstamp pubblicazione nonackconsumo servo.
Nessuna correzione ad hoc ai risultati congelati; limiti timing dichiarati.
Dentrokinova_control mancail sorgente effettivo/semantica, no mappingD->tau/ζ.

SIGNAL_PIPELINE_REPORT, auditJSON,figura18 e manifestconsegna esclusivo salvati;
nuove definizioni/metadata verificati nuovamente prima consegna. Consegne
holdout/analitica precedenti e14fingerprint intatti. Originali SHAverificati
18:19:06UTC:765file4080005175byte identici,ZIPbaseline431elementi,332sorgenti
runtime invariati. STATE:prossima sintesi difendibilepertesi senza nuovegriglie.
Campagna attiva entrocutoff24/10 09:00Singapore, nessun ROS/robot/deploy/push.

Checkpoint esclusivo tranche_20261024_0214.zip creato e verificato:174 file,12.372.491 byte. Git tracciato invariato; nessun worker Python persistente.


## Heartbeat 24/10 02:44 Singapore — interpretazione per la tesi

Letti AGENTS/PROTOCOL/STATE/NIGHT_LOG/RESULTS, steeringD/audit, log pipeline e
marker holdout complete/PIDnull. Nessun Python vivo; checkpoint0214 SHA
16330da496cd70bea9b16e536b112f0da10314388735235f5d4372eba5c104b1 verificato.
Un solo worker con BLAS/OMP/MKL/NUMEXPR1, no pycache, cacheMatplotlib campagna.

thesis_synthesis.py aggrega soltanto metriche già salvate nell'archivio e negli
studi congelati: nessun CSV forza ricaricato, modello rifittato, replay o tuning.
12identità dal freeze,8development/4holdout conservati. Nuova tabella e figura19
distinguono RMS banda/errore totale; riferimento targetCSV arrotondato/metodo
storico dichiarato diverso da metriche precise/ricampionamento del holdout.
Tre ripetizioni per pezzo, no stima inferenziale o curva media che nasconde casi.

RMS banda12FFI0,059-0,123N;routeRMSE0,270-0,554N;LslowseatRMSE1,396-1,521N.
PerZ20:20metrica storica banda0,155582N,RMSE0,256139N versus FFI Zbanda
0,111068-0,123064N eRMSE0,304285-0,339291N. Meno banda non migliora tutto.
Riferimenti storici9 espliciti(Z1,L3,S2,T3), condizioni non matched; metadati
argv/git/esecuzione/runtime disponibili salvati.20:20insert_open --online
gain0.5, non confronto soloSOCPoffline. D runtime20:20 non confermato.

Verificati12flagseat_found senzaJAM nella selezione; catalogo24FFI con13flag
seat_found. Non12/12tentativi né13/24probabilità configurazionefinale. Valori
believed_mm non metrologia esterna epositivitàmedianonmarginefullwrench.
Sorgenti/misure/modelli/counterfactual distinti; no universalità accettazione
tesi. Dieci affermazioni con evidence/limiti registrate, testo tesi adattabile.
Damping ipotesi aperta, primaD-onlyconFFI invariata; nessun nuovoPID/compensatore.

THESIS_SYNTHESIS.md,THESIS_EVIDENCE.json,tabellaCSV,figura19 e codice salvati;
figura ispezionata. Seconda generazione della sola sintesi da metriche salvate
per aggiungere argv/riferimenti e spaziatura; nessuna griglia numerica ripetuta.
Riferimenti locali verificati, tutte fonti hash inledger, consegne precedenti
e14fingerprint congelati invariati. Manifestthesis esclusivo prodotto.

Originali verificati18:49:44UTC:765file4080005175byte identici,ZIPbaseline431
elementi e332sorgenti runtimeinvariati. Git tracciato invariato, nessun worker
persistente. STATE:studi e sintesi offline completi, soltanto ulteriore lavoro
che aggiunga evidenza discriminante o risolva un problema concreto; niente
nuovi sweep senza modello. Termine24/10 09:00Singapore sempre invariato.

Checkpoint esclusivo tranche_20261024_0244.zip creato e verificato:181 file,12.497.035 byte. Nessun worker persistente; Git tracciato invariato.


## Heartbeat24/10 03:14Singapore — coerenza output e distinzione dei clock

LettiAGENTS/PROTOCOL/STATE/NIGHT_LOG/RESULTS,steeringD/audit,log/markerholdout.
Checkpoint0244 SHAe61d6dfd75a3f2639db5815e81dac81a28a13e6ce364c543466889fc22c2d23f
verificato,nessunPythonvivo;CIMgiànegato,fallbackGet-Process. Unsolo worker,
BLAS/OMP/MKL/NUMEXPR1,nopycache. Nessun ROS/robot/deploy/push o codice produzione.

audit_saved_evidence.py controlla solo artifact salvati,4manifestconsegna,
8SHAcheckpoint,link11report. DaNPZ448casi ricalcola leSTESSE9riduzioni metriche
senza invocare replay/fit/controller:4032confronti con differenza esattamente0.
Tutti campiCSV/JSON, combinazioni7x16x4, formula score/penalità/aggregazione
e0/448obiettivi congiunti coerenti. Tracce/originalirisultati non modificati.

Primo audit FALLITO:dividevaΔcomandi per event_time=logt_wall e lo chiamava
actualtick. Conservati audit_saved_evidence_attempt1.py,log originale e
SAVED_EVIDENCE_ATTEMPT1_FAILURE.json. Diagnosi concreta richiede lettura dei
SOLIffi_seq/t_wall/ffi_dt_ms dai4CSVriservati (hash verificati), non forze,
target,nuovi punteggi o tuning. Sorgente distingue p.te da p.clock=cumsumFFIdt.
Nuovi clockausiliari separatiNPZ+SAVED_CLOCK_DIAGNOSTIC,originaliNPZimmutati.

ConFFIdt effettivamenteusato, maxΔds/dt2,00000000000003mm/s su tutti448casi.
Conintervallilog:maxZ2,13852,L2,12735,S2,04525,T2,07439mm/s. Maxdifferenza
intervalli3,438/11,300/2,888/1,876ms:includeclockposizione/clamp,nonlatenza
isolataidentificata. Event_time log resta schedulingreplay,dtFFI limiter;
incertezza dichiarata,nessuna riallocazione tempi per cambiare graduatoria.
2mm/s èlimitesoftwarecorrezione normale,non velocità reale interatool/route.

Seconda bozza audit passava ma riportava erroneamente0,02N tolleranzaRMSE:
conservati bozzaJSON,codiceattempt2,log. Estratte ora soglie direttamente
dall'ASTadvanced_refinement.py/planned_reference_validation.py:0,03N RMSE,
0,01banda,0,05picco. Nessuna sogliaoriginariaallargata; treverifiche sviluppo
restanoFALLITE:advanced128casi 5->2,5ms e5->1,25ms,planned608picco0,1839N.
Terzo audit finalePASSAstessa448/4032,nonnuova simulazione. Fonte/sha criteri
inJSON; logfallimenti eiterazioni QA conservati. Unread iniziale filename
refinement nonesistente risolto con file presente,nessun impatto sull'analisi.

READING_GUIDE risolve prospettiva storica dei vecchi«prossimo holdout» senza
editarli; auditreport chiarisce softwarevsvalidazione fisica. Linknuovi verificati,
esclusivo EVIDENCE_AUDIT_DELIVERY_MANIFEST includefonti/bozze/clocks/fallimenti.
Originali nuovamenteverificati19:20:48UTC:765file4080005175byte identici,
ZIPbaseline431elementi,332runtime e14fingerprintfreeze intatti. Gittracciato
invariato,nessun worker persistente. Stati/limiti/conclusioneD invariati;
termine24/10 09:00Singapore. Nessun nuovocandidato promosso dalla QA.

Checkpoint esclusivo tranche_20261024_0314.zip creato e verificato:197 file,12.547.203 byte. Clock NPZ separati inclusi per hash nel manifest QA; nessun worker persistente o modifica Git tracciata.


## Heartbeat24/10 03:44Singapore — conservazione completa delle tracce numeriche

LettiAGENTS/PROTOCOL/STATE/NIGHT_LOG/RESULTS,steeringD/audit e logQA/marker.
Checkpoint0314 SHAbba11454bc455ca4a1f3ae0b18ae38564adffc1d51f1824e22576ea34b07f761
verificato,nessunPythonvivo;fallbackGet-Process per CIMgiànegato. Un solo worker,
BLAS/OMP/MKL/NUMEXPR1,nopycache. Nessun nuovo fit/simulazione/tuning/ROS/robot.

Verificata copertura conservazione:61NPZ root148.649.220byte,10CSV13.499.803byte;
i checkpoint precedenti del codice escludevano intenzionalmente NPZ. Creata
conservazione aggiuntiva esclusiva in deliveries,senza spostare/cancellare nulla.
preserve_campaign_payload.py usa libreria standard,verifica14sorgentifreeze,
5manifestconsegna precedenti,baselineSHA,poi copia soltanto artifact scientifici
campagna,riferimenti di archive_review e dipendenze sorgenti note. Nessun contenuto
NVIDIA_RUNS incluso; 4GBoriginali rimangono nei percorsi originali in sola lettura.

ZIP controller_night_20261024_0344.zip:383file316.904.324byte noncompressi,
267.468.570byteZIP,SHA4a9f08debd4012a6775cbb3a165163ef69ec6825d4e3510f907a75f1492fda4f.
Include61NPZ/10CSV/48Python della campagna e copia byte-identica snapshotbaseline
con431elementi,senza modificare il file protetto. Cache/checkpoint pregressi
nonduplicati,restano conservati. Loglive/pointermanifest esclusi per evitare
self-reference;4fileoperativi inclusi come snapshot datati con hashZIP esatto.

Verificatore indipendente verify_campaign_payload.py rilegge ogni elemento
ZIP(CRC+SHA+dimensioni),interoSHAZIP e tuttiimmobili workspace;nessuna estrazione
o scrittura da verificatore. Manifestsidecar,esito eCURRENT_PAYLOAD salvati
esclusivi;builder non va rilanciato sullo stesso target. PAYLOAD_README/STATUS
spiegano input/deps ancora necessari:non ambiente portatile o release robot.

Originali verificati19:48:14UTC:765file4080005175byte identici,ZIPbaseline431
elementi e332sorgenti runtime invariati. Stato aggiornato solo dopo confezione:
si verifica poi chegliimmutable rimangano uguali mentre snapshotoperativi
possono cambiare. Nessuna nuova conclusione sul damping o nuovo candidato.
STATE:sintesi/studi/preservazione completi,niente ripetizioni senza nuovo
problema oinput;alcutoff solochiusura/integrità. Termine09:00Singaporeinvariato.

Verifica CLI read-only dopo aggiornamento dei file operativi:383 elementi ZIP validi,379 file immutabili workspace coincidenti,4 snapshot operativi separati; tutte61NPZ coperte. Checkpoint esclusivo tranche_20261024_0344.zip creato e verificato:205 file,12.560.123 byte. Nessun worker persistente o modifica Git tracciata.


## Intake 2026-10-23T20:15:36.945109+00:00
Letti istruzioni/stato/registro/report e steering damping; nessun Python persistente. Controllo bounded: 765 file NVIDIA, aggiunti 0, rimossi 0, metadata cambiati 0; 14 fingerprint sorgenti SHA, cambiati 0. Checkpoint piu recente SHA verificato. Sorgente servo nei layout noti: 0. Nessun nuovo sweep/fit/tuning/payload; archivio solo in lettura. Fast check size/mtime NON sostituisce verifica SHA completa alla chiusura. Nuova evidenza da riesaminare: False.

## Intake 2026-10-23T20:46:15.502739+00:00
Letti istruzioni/stato/registro/report e steering damping; nessun Python persistente. Controllo bounded: 765 file NVIDIA, aggiunti 0, rimossi 0, metadata cambiati 0; 14 fingerprint sorgenti SHA, cambiati 0. Checkpoint piu recente SHA verificato. Sorgente servo nei layout noti: 0. Nessun nuovo sweep/fit/tuning/payload; archivio solo in lettura. Fast check size/mtime NON sostituisce verifica SHA completa alla chiusura. Nuova evidenza da riesaminare: False.

## Intake 2026-10-23T21:14:41.058410+00:00
Letti istruzioni/stato/registro/report e steering damping; nessun Python persistente. Controllo bounded: 765 file NVIDIA, aggiunti 0, rimossi 0, metadata cambiati 0; 14 fingerprint sorgenti SHA, cambiati 0. Checkpoint piu recente SHA verificato. Sorgente servo nei layout noti: 0. Nessun nuovo sweep/fit/tuning/payload; archivio solo in lettura. Fast check size/mtime NON sostituisce verifica SHA completa alla chiusura. Nuova evidenza da riesaminare: False.

## Intake 2026-10-23T21:44:37.606530+00:00
Letti istruzioni/stato/registro/report e steering damping; nessun Python persistente. Controllo bounded: 765 file NVIDIA, aggiunti 0, rimossi 0, metadata cambiati 0; 14 fingerprint sorgenti SHA, cambiati 0. Checkpoint piu recente SHA verificato. Sorgente servo nei layout noti: 0. Nessun nuovo sweep/fit/tuning/payload; archivio solo in lettura. Fast check size/mtime NON sostituisce verifica SHA completa alla chiusura. Nuova evidenza da riesaminare: False.

## Intake 2026-10-23T22:14:38.993682+00:00
Letti istruzioni/stato/registro/report e steering damping; nessun Python persistente. Controllo bounded: 765 file NVIDIA, aggiunti 0, rimossi 0, metadata cambiati 0; 14 fingerprint sorgenti SHA, cambiati 0. Checkpoint piu recente SHA verificato. Sorgente servo nei layout noti: 0. Nessun nuovo sweep/fit/tuning/payload; archivio solo in lettura. Fast check size/mtime NON sostituisce verifica SHA completa alla chiusura. Nuova evidenza da riesaminare: False.

## Intake 2026-10-23T22:44:12.428269+00:00
Letti istruzioni/stato/registro/report e steering damping; nessun Python persistente. Controllo bounded: 765 file NVIDIA, aggiunti 0, rimossi 0, metadata cambiati 0; 14 fingerprint sorgenti SHA, cambiati 0. Checkpoint piu recente SHA verificato. Sorgente servo nei layout noti: 0. Nessun nuovo sweep/fit/tuning/payload; archivio solo in lettura. Fast check size/mtime NON sostituisce verifica SHA completa alla chiusura. Nuova evidenza da riesaminare: False.

## Intake 2026-10-23T23:14:11.740945+00:00
Letti istruzioni/stato/registro/report e steering damping; nessun Python persistente. Controllo bounded: 765 file NVIDIA, aggiunti 0, rimossi 0, metadata cambiati 0; 14 fingerprint sorgenti SHA, cambiati 0. Checkpoint piu recente SHA verificato. Sorgente servo nei layout noti: 0. Nessun nuovo sweep/fit/tuning/payload; archivio solo in lettura. Fast check size/mtime NON sostituisce verifica SHA completa alla chiusura. Nuova evidenza da riesaminare: False.

## Intake 2026-10-23T23:44:13.999915+00:00
Letti istruzioni/stato/registro/report e steering damping; nessun Python persistente. Controllo bounded: 765 file NVIDIA, aggiunti 0, rimossi 0, metadata cambiati 0; 14 fingerprint sorgenti SHA, cambiati 0. Checkpoint piu recente SHA verificato. Sorgente servo nei layout noti: 0. Nessun nuovo sweep/fit/tuning/payload; archivio solo in lettura. Fast check size/mtime NON sostituisce verifica SHA completa alla chiusura. Nuova evidenza da riesaminare: False.

## Intake 2026-10-24T00:14:14.937086+00:00
Letti istruzioni/stato/registro/report e steering damping; nessun Python persistente. Controllo bounded: 765 file NVIDIA, aggiunti 0, rimossi 0, metadata cambiati 0; 14 fingerprint sorgenti SHA, cambiati 0. Checkpoint piu recente SHA verificato. Sorgente servo nei layout noti: 0. Nessun nuovo sweep/fit/tuning/payload; archivio solo in lettura. Fast check size/mtime NON sostituisce verifica SHA completa alla chiusura. Nuova evidenza da riesaminare: False.

## Intake 2026-10-24T00:44:15.594083+00:00
Letti istruzioni/stato/registro/report e steering damping; nessun Python persistente. Controllo bounded: 765 file NVIDIA, aggiunti 0, rimossi 0, metadata cambiati 0; 14 fingerprint sorgenti SHA, cambiati 0. Checkpoint piu recente SHA verificato. Sorgente servo nei layout noti: 0. Nessun nuovo sweep/fit/tuning/payload; archivio solo in lettura. Fast check size/mtime NON sostituisce verifica SHA completa alla chiusura. Nuova evidenza da riesaminare: False.
