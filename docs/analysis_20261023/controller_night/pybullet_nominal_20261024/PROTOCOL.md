# Nuova prova autorizzata da Lorenzo — 24 ottobre 2026

Richiesta successiva alla ricerca notturna: eseguire davvero l'inserimento in
PyBullet con controller FFI delle run, parametri nominali, tri-pad meccanici,
forze/coppie ai polsi e piano SOCP. Non è un replay delle forze reali.

Output esclusivamente in questa sottocartella. NVIDIA_RUNS, baseline, vecchie
analisi e sorgenti di produzione rimangono immutati. Nessun ROS/robot/deploy.
Un solo worker di simulazione, BLAS un thread. Nessuna ricerca di gain.

Si importa senza modifiche FFIProfileInsertion/NominalIntegral, pickup PI,
sequenza completa, pacing, sentinelle, GraspSOCP e TP-GMM del pacchetto reale.
Si ricalcola il piano offline con NominalPlanner e si conservano allocazioni
complete e profilo. Si scelgono le prime run dello sviluppo Z/L/S/T già fissato,
soltanto per config/argv; nessuna forza misurata guida la dinamica simulata.

Impianto iniziale dichiarato: modello floating-plate originale in
TESI_antiG/lorenzo_data_analysis/10_tripad_passivo, copiato per provenienza e
scratch isolato. Geometria reale dei pad dal metadata; K1500/20, D60/7;
meccanica/massa/inerzia nominali del modello PyBullet. Non è il servo Kinova
articolato: il suo sorgente interno resta assente. D80 sarà una variazione del
coefficiente fisico della stessa impedenza ideale, senza ritardi inventati.

Contatto nominale da shelf.py: 8000 N/m, 207 Ns/m; pezzi 50 g, attrito efficace
pad-pezzo 1, pezzo-pezzo .5, pavimento .3. Verificare attriti letti dal motore.
Questi valori e le masse/rigidezze dei pad non sono una calibrazione hardware.
Fixture iniziale tiene il pezzo come l'operatore; viene rimossa soltanto dopo
il pickup misurato. Pezzo e vicini poi sono liberi nella scena.

Il controller riceve la reazione meccanica delle molle al polso, compensata
per gravità del tool; contiene inerzia del tool. Media blocchi 5ms, poi ultimi
due blocchi come idealizzazione della pipeline. Registrare separatamente
contatti veri su pad/piastra e forza/momento al polso, senza sostituire la
forza richiesta dal SOCP alla misura. Posa vera oggetto soltanto per metriche.

Verifiche necessarie: segni e poli del wrench, gravità/statica, azione-reazione,
parametri/solver, limiti FFI, risultato di inserimento geometrico dopo rilascio,
raffinamento dt 1ms→.5ms. Conservare anche abort/fallimenti. Risultati distinti
fra nominale simulato, osservato reale e inferenze. Una differenza simulazione
versus realtà non identifica da sola cambiamenti hardware.

La scadenza della precedente campagna notturna è trascorsa: questa è una nuova
richiesta esplicita. Nessun precedente tuning/holdout viene riaperto.

Steering successivo esplicito: **molla CENTRALE 2700 N/m**, non totale 2700.
Quindi si fissano centrale 2700 e tre pad 500 ciascuno: totale assiale 4200.
La serie principale è `*_central2700_D60_*`. Le prove con totale2700 (centrale
1200) sono superate, conservate separatamente; nessuna rietichettatura dei dati.
Smorzamenti meccanici inizializzati al riferimento fisso .25ms, indipendente
dal passo fisico del raffinamento .5→.25ms. K/D cartesiani e FFI invariati.

Ulteriore steering esplicito: importare entrambi i Kinova Gen3 e posizionarli
come in laboratorio. Il backend gen3/run_gen3.py importa le mesh ufficiali e
la cinematica/inerzie del file URDF locale, riusa shelf_in_base dei metadata,
verifica FK contro catena indipendente e posizione tool registrata, inizializza
con IK dagli angoli iniziali reali. Dopo il setup nessun reset di posa ai giunti.
Modello esplicito delle molle con 40g mobili e supporto rigido 110g, totale150g.
FT da giunto fisso al polso; legge cartesiana in coppia nominale dichiarata,
non codice del servo Kinova. Realizzazione implicita locale a1kHz per evitare
instabilità numerica del PD esplicito sui polsi a bassa inerzia; K/D invariati.
Conservare abort esplicito e verifica statica; nessuna attribuzione hardware.
Confronto principale Z a dt0.5ms e0.25ms; L come secondo caso se eseguibile,
senza variare gain fra le prove. Un solo processo di simulazione per volta.

Steering più recente: usare la legge del pacchetto ROS del laboratorio.
Trovata nei Download una copia kinova_controller.py di2092 righe coerente con
il resoconto fornito: snapshot e SHA-256 in gen3/lab_source. Non importare il
nodo ROS/Kortex: estrarre testualmente soltanto classi matematiche, eseguirle
con Pinocchio reale e sostituire trasporto hardware con giunti/coppie Bullet.
Riutilizzare CartesianImpedanceController, OnlineKalmanState e friction class;
stessi default del nodo per clip35/12Nm, LPF100Hz, joint damping0.001,
slew0.5Nm/tick e watchdog2rad/s per20tick. Null stiffness0. Parametri cartesiani
da launch1500/20 e60/7. Il launch abilita friction ma non sovrascrive scale0:
nominale con scala0, nessuna compensazione nonzero inventata senza gen3_bare.
L'hash della versione effettivamente sulla NVIDIA rimane da confermare.
Le prove precedenti con servo implicito sono preliminari, non prove di questa
legge originale. In particolare Z ha cambiato errore77→4.4mm raffinando dt:
non è una previsione robusta e non va attribuita all'hardware o al controller.
