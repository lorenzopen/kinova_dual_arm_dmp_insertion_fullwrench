# Variante PI della stretta — 23 ottobre 2026

File: `kinova_dual_arm_dmp_insertion/insert_pi.py`. Comandi operativi per T1/T2/T3
in cima a `GUIDA.md`. `main`, `insert_open` e `insert_pick` mantengono la loro legge
di controllo. Il solo supporto condiviso aggiunto è la registrazione diagnostica.

## Legge e unità

Il SOCP e il significato del suo target non cambiano. La stretta regola la somma
delle forze normali delle mani; `target_N` nel CSV resta invece **per mano**.

\[
e_k=2N_k^*-(N_{L,k}+N_{R,k}),\qquad p_k=K_Pe_k
\]
\[
i_k^{trial}=i_{k-1}+K_Ie_k\Delta t_k,\qquad s_k^{trial}=p_k+i_k^{trial}
\]
\[
s_k=\operatorname{clip}\left(s_k^{trial},
\max(s_{min},s_{k-1}-v_{max}\Delta t_k),
\min(s_{max},s_{k-1}+v_{max}\Delta t_k)\right),
\qquad i_k=s_k-p_k.
\]

L'ultima assegnazione è l'anti-windup tramite tracking del comando ottenibile:
se non ci sono saturazioni coincide con l'integrazione normale; se ci sono,
l'integrale non accumula una correzione che il comando non può eseguire.
P è una correzione **assoluta** e non viene integrato ad ogni tick.

La prima misura inizializza `i = s_corrente - P`, senza salto del comando.
Il tempo viene dal clock del backend, non da un `dt` nominale moltiplicato per il numero
di tick. Dopo l'inizializzazione, dt non positivo o oltre 100 ms causa `Abort` prima
di cambiare lo stato PI. Gli stop ATI/joints e gli altri controlli esistenti restano attivi.

## Valori iniziali

| Parametro | Default | Significato |
|---|---:|---|
| `--pi-kp` | 0.0001 m/N | 0.1 mm/N di errore totale |
| `--pi-ki` | 0.001 m/(N s) | 1 mm/s per N di errore totale, se non limitato |
| `--pi-speed-mm-s` | 2 | velocità massima della correzione per mano |
| `--pi-limit-mm` | 6 | correzione massima in ciascun verso dopo la presa |
| pickup travel | 15 mm | chiusura massima aggiuntiva rispetto al contatto di approccio |
| pickup timeout | 6 s | forza entro max(0.3 N, 5% totale) per almeno 0.2 s |

La presa richiede anche almeno metà del target per mano su **ciascuna** mano,
per evitare di accettare il solo valore sommato con un pad scarico. Se non converge,
si interrompe prima della richiesta di lasciare il pezzo. La posa di pickup viene
assorbita nel frame congelato e il PI riparte su `ds=0`, senza trasportare il suo
integrale in un riferimento diverso. Le conferme dell'operatore non sono integrate.

Il vecchio integratore a gain 0.25 e 50 Hz equivale nominalmente a circa
2.08 mm/(N s), senza P. Qui I parte da 1 mm/(N s), con P e limite di velocità
più contenuto. È una scelta iniziale per il pilot, **non una taratura sul robot**.
Non si sostiene che aggiungere P equivalga a smorzare una risonanza.

Non viene introdotta una derivata della forza né un nuovo filtro. La rigidezza e il
damping dei Kinova sono impostati in T1, indipendentemente dal PI; il modulo non
scrive i parametri dei controller. Per regolare la forza via posizione contano anche
la molla del tripad, la risposta del basso livello, la geometria e i ritardi reali.

## SOCP e altri canali

Default: SOCP online e PI normale. `--offline`: stesso pianificatore nominale e
profilo completo a 12 componenti di `insert_open`, con **feedback PI ATI durante
l'esecuzione**, senza solve online. Non è il feedforward puro `insert_open --offline`.
Non si aggiunge un precarico `F/K` al PI: lo spostamento di presa è già nel frame.

`--twist`, `--wrench-channels` e `--no-bending` conservano i significati precedenti.
Il PI non rende indipendentemente controllabili tutti i 12 componenti. Tangenziali
e momenti hanno ancora le leggi già presenti, se selezionati. Nel modulo PI
`--squeeze-gain` agisce solo sulla correzione tangenziale dei modi `forces/contact`;
non modifica P/I normale né il gain torsionale.

## Dati e verifiche

CSV `_pi_`: colonne `pi_seq`, `pi_dt_ms`, `pi_error_N`, `pi_p_mm`, `pi_i_mm`,
`pi_output_mm`, `pi_rate_limited`, `pi_position_limited`. Sono lo stato dell'ultimo
aggiornamento PI, ripetuto alle righe di acquisizione; usare `pi_seq` per selezionare
gli aggiornamenti distinti. Dopo il reset sono vuote/NaN fino al nuovo aggiornamento.
Durante pickup `ds_mm` indica la chiusura di pickup; dopo il freeze torna a zero.
Durante release il loop è disattivato e i diagnostici eventualmente presenti sono
l'ultimo stato, non nuovi aggiornamenti. Nessun termine PI è presente nei vecchi modi.
Il meta salva gain fisici, limiti, tipo PI e modalità SOCP; i vecchi campi del gain
normale diventano null per non confonderli con un parametro attivo del PI.

`tests/test_squeeze_pi.py` controlla legge, dt, P non accumulato, limiti, anti-windup,
reset, input errati, timeout pickup e assenza di solve nell'esecuzione offline.
`docs/analysis_20261023/validate_pi.py` riproduce una matrice di esecuzioni sul toy
backend a 1000/1500 N/m di braccio, con molla nominale in serie, su Z/T/L, con SOCP
online/offline e canali scalari/tangenziali. I CSV in `tmp/pi_validation` sono sintetici.
Queste prove verificano software e sequenza: **non riproducono oscillazioni, slip,
ritardi o contatti dei vicini e non certificano la stabilità sul Kinova**.
