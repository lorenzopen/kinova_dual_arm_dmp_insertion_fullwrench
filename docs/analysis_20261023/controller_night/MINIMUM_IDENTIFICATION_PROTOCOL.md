# Dati minimi per decidere sul damping e sul controllo di forza

Protocollo preparato dalla campagna; nessuna acquisizione o esecuzione sul robot.
Le sette configurazioni del confronto finale restano congelate. Nessun tuning
ulteriore sulle quattro run riservate.

## Prima domanda: cambia davvero qualcosa aumentando soltanto D?

Il primo confronto rimane quello in [DAMPING_AB_PROTOCOL.md](DAMPING_AB_PROTOCOL.md):
FFI attuale, stesso profilo originale, K, gain I, limiti, precarico, geometria
e condizioni di contatto. A=60/7, B=80/7 traslazionale isotropo, solo dopo
verifica della semantica effettiva del servo. La variante y-only è un confronto
separato e richiede la conferma del frame in cui agisce la matrice D.
Non combinare l'aumento di D con nuovi filtri ATI o gain I/P/D esterni.

Prima di un confronto interpretato come D-only occorrono sorgente e versione
del controllore cartesiano effettivamente caricato, più la configurazione
runtime di ciascun braccio. Da documentare: legge della coppia/azione interna,
frame e unità di K/D, origine e filtro della velocità, discretizzazione,
eventuale nullspace, limiti e compensazioni. Se una voce manca rimane ignota;
non sostituirla con quella di un controllore cartesiano generico.

Pilot in ordine A–B–B–A con ripristino delle condizioni iniziali. Quattro run
sono uno screening; per valutare la ripetibilità il protocollo precedente
prevede almeno tre blocchi o un ordine bilanciato fissato prima dei risultati.
Registrare ogni tentativo e il motivo di interruzione. Z per primo; L per il
transitorio route→slow_seat se il pilot consente di proseguire. Nessuna scelta
delle sole run riuscite o delle finestre più lisce.

## Pacchetto minimo per ciascun tentativo

| Segnale | Contenuto indispensabile | Scopo |
| --- | --- | --- |
| Configurazione | Revisione codice effettiva, hash del profilo SOCP, K/D/gain/limiti runtime per braccio, tool e pezzo, tara/precarico | Dimostrare che cambia solo il fattore dichiarato |
| FFI a ogni update | Sequence ID, fase e clock del profilo, target originale, pickup, ff desiderato/applicato, trim, totale, errore, dt reale/clamped, flag di limite | Separare richiesta, comando raggiungibile e scheduling |
| Comando consumato dal servo | Posa e timestamp dell'ultimo target effettivamente accettato per ciascun braccio | Distinguere pubblicazione da applicazione |
| Stato robot | Posa e twist misurati con timestamp nativo; q/dq se disponibili | Separare comando→moto da moto→forza |
| ATI per mano | Wrench raw, frame/calibrazione, timestamp di acquisizione, eventuale forza filtrata effettivamente usata | Separare rumore, filtro e risposta fisica |
| Consumo della misura | ID/timestamp del campione ATI e dello stato usati in ciascun tick FFI/servo, timestamp di ricezione | Misurare età della misura e ritardi della pipeline |
| Esito | Contatto/posa iniziali, note sulla presa, fase e motivo di stop, eventi di limite/interruzione | Conservare fallimenti e condizioni variabili |

Conservare i segnali al rate nativo disponibile e riportare distribuzione
degli intervalli, campioni mancanti e duplicati. Non ridurre tutto al solo CSV
FFI. Wrench, pose e target campionati a tempi diversi non vanno dichiarati
simultanei solo perché finiscono nella stessa riga. Per studiare il tratto
2,5–10 Hz serve risoluzione temporale adeguata; dichiarare la banda realmente
osservabile dai rate misurati e l'eventuale antialiasing, senza interpolare per
inventare informazione mancante.

Usare un clock monotono comune oppure salvare una mappa fra clock con offset,
deriva e incertezza. Conservare sia i timestamp nativi sia quelli di ricezione
e consumo. Se non è possibile conoscere il comando realmente consumato,
chiamare il ritardo stimato «pubblicazione→moto», con quella limitazione.
Per il D sulla forza conta anche il rate effettivo di consumo FFI: non basta
il rate nominale del sensore ATI.

## Confronto da fissare prima di guardare le nuove forze

Finestre legate alle fasi: route completa e primi 1,6 s di slow_seat, più la
fase slow_seat completa per i minimi. Conservare anche la traccia integrale.
Allineamento sia per tempo di fase sia, quando disponibile, per avanzamento
geometrico; non spostare le curve per massimizzare a posteriori il vantaggio.

Riportare per mano e media: errore rispetto alla richiesta originale, RMS
0,2–0,8 /0,8–2,5 /2,5–10 Hz della forza e dell'errore, picco puntuale e massimo
della media causale di 200 ms, eccesso/difetto, minimi, ritardo/moto residuo dopo
inversione del comando, frazioni di limitazione e successo/interruzione.
I bordi delle finestre e il metodo dei filtri di analisi vanno fissati uguali.
La media di 200 ms è una diagnostica: non sostituisce i dati raw né il picco.

Per A/B confrontare differenze entro blocco e dispersione fra ripetizioni,
mostrando tutte le curve. Meno banda è utile solo insieme a tracking, minimi
per mano e rapidità. La richiesta scalare comune non è il margine della presa:
per dichiarare mantenuti i requisiti occorre valutare il full wrench con i
vincoli/modello d'attrito originali. Le soglie esplorative della notte non
diventano automaticamente criteri di accettazione della tesi.

## Solo se il D-only non discrimina: identificazione separata

Prima di cambiare l'architettura esterna, identificare risposta comando→moto
e moto→forza con un ingresso noto, persistente e distinto dal disturbo.
Ampiezza, banda e condizioni della futura eccitazione richiedono la semantica
del servo e verifica dei margini del caso reale; questa campagna non assegna
valori robot a partire dal modello non validato.

Congelare prima della prova le ripetizioni di sviluppo e quelle riservate,
tenere separati verso di chiusura/apertura e condizioni di contatto, annotare
le saturazioni invece di nasconderle. Una previsione one-step con forza reale
passata disponibile non basta: confrontare simulazione libera su ingressi
noti e run riservate, includendo il transitorio finale e il modello nullo.
Usare solo i dati disponibili all'istante simulato, stato iniziale dichiarato,
nessuna forza futura imposta. Separare risultati su contatto invariato da
transizioni/slittamento/asimmetria delle mani che il modello non descrive.

Prima di studiare un compensatore del ritardo, l'incertezza del modello deve
comprendere frequenze delle onde, fase/ritardo, risposta nei due versi e
previsione libera. Una buona correlazione con il moto già misurato non basta
a prevedere il moto prodotto da un nuovo comando. Se questa verifica fallisce,
riportare il limite e non cercare un vincitore con ulteriori replay condizionati.

Nuovi dati e manifest vanno in una nuova directory autorizzata: **NVIDIA_RUNS
resta integralmente in sola lettura**. Conservare revisioni, tentativi falliti e
SHA-256; nessun ordine di robot, ROS, deploy o push è contenuto in questo file.
