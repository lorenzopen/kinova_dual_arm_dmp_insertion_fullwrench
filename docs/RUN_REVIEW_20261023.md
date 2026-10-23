# Run Z offline e oscillazioni — verifica del 23 ottobre

`git pull --ff-only` su main: già aggiornato a `cd53f2d`. La nuova acquisizione è
nel commit `24438fd` su `origin/fix-ati-mappa`, esaminato sul branch separato
`codex/run-review-20261023`. Nessuna modifica alle leggi di controllo.

## Risultati ricalcolati dai CSV

Errore normale: `fn_N - 2*target_N`; target_N è **per mano**. Normali ricontrollate
proiettando le forze ATI sugli assi delle flange misurate: nella nuova run la massima
differenza dalla somma registrata è 0.014 N, compatibile con l'arrotondamento CSV.

| Run | Modalità | Mediana misurata/chiesta | RMS oscillazione 0.8–2.5 Hz, totale |
|---|---|---:|---:|
| 22/10 19:32 | offline, k_eff 730 | 0.631 | 0.232 N |
| 22/10 19:53 | offline, k_eff 480 | 0.887 | 0.230 N |
| 22/10 20:13 | offline, k_eff 580 | 1.178 | 0.172 N |
| 22/10 20:20 | online, gain 0.5 | 1.000 | 0.311 N |
| 23/10 12:32 | offline, k_eff 964 | 0.529 | 0.179 N |

Intera fase route, tempi reali, ricampionamento all'intervallo mediano, detrend cubico,
Butterworth ordine 2 avanti/indietro, bordi di 1.5 s esclusi dal calcolo RMS. Le run
non sono repliche controllate: cambiano presa, condizioni e durata. La finestra e
la metrica differiscono da alcune vecchie stime di picco per mano.

La run del 23/10 è **feedforward puro**, `squeeze_feedback=false`, non PI e non
`--offline --online`. I parametri letti su entrambi i nodi confermano K=1500/20,
D=60/7, attrito e Kalman attivi. Il suo k_eff 964 è la combinazione nominale in
serie di 1500 e 2700 N/m. È un modello assiale, non una calibrazione della presa.

Primo sesto della nuova route: **8.96 N richiesti, 4.66 N misurati**. Ultimo sesto:
**7.14 N richiesti, 2.93 N misurati**. Minimo totale 2.14 N, minimi per mano circa
1.06/1.03 N. La forza è sotto target già prima della parte finale. Nel finale
crescono i carichi laterali e cala ulteriormente la stretta: compatibile con il
disturbo da contatto riportato dall'operatore, ma il CSV non misura lo slip del pezzo.
La frequenza effettiva CSV è dell'ordine di 100 Hz; massimo intervallo route 20.4 ms:
questa acquisizione non richiede una frequenza più alta per vedere onde da 1–2 Hz.

Il risultato dell'offline non si risolve dimostrabilmente assegnando "k_eff vero=680".
Quel valore è una stima empirica dalla run precedente, assumendo una relazione
proporzionale senza errori di geometria/offset. Non è una misura indipendente di
rigidezza. Nella run con k_eff 480 la prima metà segue bene, poi il rapporto scende
a circa 0.7: il problema non è un semplice fattore di scala costante.

L'offline nuova ha meno oscillazione **assoluta** dell'online delle 20:20, ma lavora
anche con circa metà forza: RMS/forza media circa **4.2% contro 3.6%**. Non basta
quindi per dichiarare l'offline migliore. La prova online delle 12:18 del 23/10 e il
bag a pezzo fermo sono citati nel README della run, ma i relativi dati grezzi **non
sono nel repository ricevuto**; non li tratto come verifiche indipendenti.

## Prova consigliata

Separare la pianificazione offline dall'esecuzione in feedforward. Il modulo
`insert_pi --offline` conserva il piano SOCP completo e regola la stretta normale
con gli ATI; non somma automaticamente un precarico F/K al PI. Gli altri canali
restano quelli esplicitamente selezionati. Nessuna run hardware PI è presente tra
i file ricevuti: la riduzione delle oscillazioni resta da misurare.

**T1:** mantenere K=1500/20 e D=60/7 nel primo confronto, senza cambiare anche
friction/Kalman. **T2:** bag come in GUIDA.md. **T3**, stesso pezzo e stessa board:

```bash
python -m kinova_dual_arm_dmp_insertion.insert_pi check "$W" --arm "$R" --slow 3 --offline
python -m kinova_dual_arm_dmp_insertion.insert_pi insert "$W" --arm "$R" --slow 3 --offline --pi-kp 0.0001 --pi-ki 0.001
```

P=0.1 mm/N, I=1 mm/(N s), errore sulla somma delle mani. Limite 2 mm/s e ±6 mm
di correzione per mano dopo pickup; tempo effettivo, anti-windup. L'integratore
storico a gain 0.5 e 50 Hz equivale a circa 4.17 mm/(N s): la nuova I è circa
quattro volte più lenta. Questo è un punto di partenza per un pilot, non un margine
di stabilità identificato. Non correggere contemporaneamente rho, stiffness, D e gain.

Se rimane un'onda periodica, una seconda prova può dimezzare **solo I** a 0.0005,
controllando anche il calo di forza al contatto. Se la forza recupera troppo piano,
non continuare a ridurre I: confrontare P e risposta del basso livello, distinguendo
il raggiungimento dei limiti di posizione/velocità dai problemi di taratura.
In particolare `pi_rate_limited`/`pi_position_limited` e l'errore persistente mostrano
se il comando sta chiedendo una correzione che i limiti non consentono.

PI è un punto di partenza ragionevole nel feedback di forza; una derivata della
forza rumorosa richiede cautela. Il riferimento generale
[Modern Robotics, §11.5](https://modernrobotics.northwestern.edu/nu-gm-book-resource/11-5-force-control/)
mostra PI con feedforward per controllo diretto in coppia: non è una prova di
stabilità del nostro anello di posizione con tripad.

Non attribuisco dai soli log il ritardo equivalente a un'unica causa (friction,
Kalman, filtro ATI, dinamica o trasporto). La fase di due segnali in anello chiuso
non identifica da sola la funzione di trasferimento né un ritardo puro. Se il PI
non migliora il compromesso, serve una prova dedicata a posa/target costanti per
separare risposta del braccio e disturbi di traiettoria, anziché copiare gain PID.

## Artefatti

- `analysis_20261023/review_new_runs.py`: analisi riproducibile dei cinque CSV.
- `analysis_20261023/new_runs_metrics.json`: valori, bin temporali, hash e metadati.
- `analysis_20261023/new_runs_force_tracking.png`: target, misure, rapporto e carico laterale.

Questo aggiornamento contiene solo analisi e guida. Nessun comando ai robot.
