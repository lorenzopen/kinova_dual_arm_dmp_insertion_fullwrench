# Forza nominale SOCP + piccola correzione I

Variante separata: `kinova_dual_arm_dmp_insertion/insert_ffi.py`.
Comandi e terminali in cima a `GUIDA.md`. Nessuna modifica a coni, margini, target
SOCP, leggi di `main`/`insert_open`/`insert_pi` o parametri dei controller Kinova.
Il logger condiviso aggiunge colonne e riconosce il nuovo modo.

## Riferimento della chiusura

La presa iniziale è quella con contatto misurato, eseguita con I a tempo reale,
P=0, I=1 mm/(N s), velocità massima 2 mm/s per mano. Deve raggiungere il target
entro la tolleranza e mantenerlo 0.2 s, con entrambi i pad caricati, entro 6 s.
In caso contrario termina prima della richiesta di lasciare il pezzo. I controlli
di forza, tracking e freschezza dei segnali già esistenti restano attivi.

Alla fine si registra

\[
N_{pick}=\tfrac12(N_L+N_R).
\]

Questa è la forza media misurata alla presa, non una nuova forza imposta. La posa
comandata che l'ha prodotta viene incorporata nel frame congelato. Perciò il
nominale della fase successiva è **relativo** a quella posa:

\[
s_{ff}^{want}=\frac{N^*(t)-N_{pick}}{k_{eff}}.
\]

Non si aggiunge nuovamente l'intero `N*/k_eff` alla chiusura già presente.
Se target e forza di pickup coincidono, il nominale relativo è zero.
La posizione risultante va letta come pickup + variazione nominale + trim.
L'ancoraggio alla presa misurata riduce l'effetto di un offset iniziale sconosciuto;
non identifica la geometria esatta del pezzo né una rigidezza vera.

`k_eff` è uno scalare assiale per mano: valore nominale ottenuto dal cart_k letto
su entrambi i bracci in serie alla molla nominale, oppure override esplicito
`--k-eff`. A K=1500 e molla 2700 è 964.3 N/m. Il modo richiede ATI su entrambi
i bracci. Se non si possono leggere le rigidezze, occorre `--k-eff` esplicito.
Non vengono inferite rigidezze laterali o torsionali da quella assiale.

## Integratore e limiti

\[
e=2N^*-(N_L+N_R),\qquad
i_{trial}=i_{prev}+K_I e\,\Delta t,\qquad s=s_{ff}+i.
\]

La prima lettura mantiene `s_ff=i=s=0`: il comando non salta alla nuova forza.
Poi il nominale viene raggiunto gradualmente. La stessa `s` chiude simmetricamente
ogni mano; la riduzione di distanza totale è `2*s`. L'errore è la **somma** delle
mani, non una forza per mano. Non sono aggiunti P, D, banda morta o filtri ATI.

| Parametro | Default |
|---|---:|
| `--i-ki` | 0.0016 m/(N s), cioè 1.6 mm/(N s) (regola sotto) |
| `--i-limit-mm` | ±3 mm di trim per mano |
| Velocità trim | massimo 2 mm/s per mano |
| Velocità totale nominale + trim | massimo 2 mm/s per mano |
| Escursione totale relativa al pickup | ±12 mm per mano |
| Nominale interno | ±(12 mm − limite trim), default ±9 mm |
| Intervallo dopo la prima lettura | dt oltre 100 ms integrato come 100 ms; Abort oltre 300 ms |

Gli intervalli ammissibili vengono calcolati congiuntamente: non si applica un
limite al nominale e uno all'I lasciando che la loro somma vada al doppio della
velocità. Il nominale ha priorità nel budget di velocità; lo stato integrale viene
aggiornato al **trim effettivamente ottenibile** dopo tutti i limiti. Questo è
l'anti-windup, anche in caso di saturazione della velocità totale.

Il tempo è il clock effettivo del backend. Non si integra più volte per recuperare
tick saltati. Il trigger del controllo rimane quello del controller base, nominalmente
20 ms e potenzialmente irregolare sul PC reale; qui si normalizza l'integrazione
con dt, **non si cambia il rate né lo scheduler degli altri anelli**.
Un intervallo tra 100 e 300 ms (ctrl_dt arriva a 92–126 ms nelle run normali del 23/10)
non ferma la prova: si integra e si muove al massimo per 100 ms, `ffi_dt_clamped=1`.
Intervallo non positivo o oltre 300 ms causa Abort prima della modifica dello stato.

La presa iniziale usa un'I separata (1 mm/(N s)) per caricarsi entro il timeout.

## Scelta di K_I: regola del ritardo

**Impianto misurato** (21 run del 22–23/10, archivio `E:/TESI/NVIDIA_LOG_RUNS` + `docs/runs`).
Si confrontano la chiusura comandata (distanza fra le pose mandate) e quella misurata
(encoder, `mx*`), e la forza ATI:

- comando → flange misurate: ritardo **120–230 ms** (correlazione incrociata 0.3–4 Hz;
  120 e 160 ms nelle offline, senza anello di forza);
- flange misurate → forza: ritardo ~0 ms, rigidezza ~2500 N/m: il contatto è statico;
- età dei dati di posa ~5 ms: non è un ritardo del sensore.

Modello per la somma delle forze rispetto a `s` (per mano):

\[
G(s)=k_L e^{-\tau s},\qquad k_L = 2k_{eff}\approx 1360\ \text{N/m},\quad \tau\le 0.23\ \text{s}.
\]

**Anello** con I pura \(C=K_I/s\): \(L=k_LK_I e^{-\tau s}/s\). Il modulo vale 1 a
\(\omega_c=k_LK_I\), dove la fase è \(-90^\circ-\omega_c\tau\). Quindi

\[
\text{PM}=90^\circ-\frac{180}{\pi}\,k_LK_I\tau,\qquad
\text{stabile}\iff k_LK_I\tau<\frac{\pi}{2}.
\]

Scelta: \(k_LK_I\tau=0.5\) (PM ≈ 61°) con il caso peggiore τ = 0.23 s:
\(K_I = 0.5/(1360\cdot0.23)\approx 0.0016\) m/(N s). Con k_eff = 964 (rigidezza nominale,
la più alta vista) il prodotto sale a 0.71, PM ≈ 49°; l'instabilità richiede un errore
di modello di 2.2 volte. Costante di tempo in anello chiuso circa \(1/\omega_c\approx 0.45\) s.

**Verifica contro le run** (τ = 0.17 s, \(k_L\) = 1360; il vecchio integratore
`GAIN=1/(4·1500)` m/N ogni 20 ms vale 8.33 mm/(N s) a gain 1):

| Gain storico | \(k_LK_I\tau\) | PM prevista | Osservato |
|---|---:|---:|---|
| 1 | 1.93 | instabile (ciclo a ~1.47 Hz) | ciclo limite a 1.37 Hz |
| 0.5 | 0.96 | 35° | risonanza, RMS banda 0.27–0.38 N |
| 0.25–0.3 | 0.48–0.58 | 57–62° | calma, ma con l'I vecchia senza nominale cala nel finale |
| 0.1 | 0.19 | 79° | errore RMS 0.43 N, lenta |

Il nominale SOCP arriva in feedforward e non passa per questo limite: l'I corregge
solo i disturbi (k_eff sbagliato, carichi del finale). Banda massima ragionevole
dell'I ≈ 0.25–0.35 Hz: per andare più veloci va ridotto τ, non cambiato il controllore.
L'analisi è lineare: vale finché i limiti di velocità/corsa non saturano
(`ffi_rate_limited`, `ffi_i_limited` nel CSV). Con 2 mm/s il limite scatta per
errori oltre ~1.25 N sulla somma.

Test: `test_delay_rule_separates_measured_gains` simula \(G\) con ritardo puro:
il default si assesta con k_eff 680 e 964 e τ = 0.23 s; il vecchio gain 1 oscilla.

## Pianificazione e canali

`--offline`: stessa pianificazione nominale completa di `insert_open`, con tutti
i wrench e i coni con margine task-adaptive. Il solver non viene chiamato durante
l'esecuzione della route. Gli ATI restano in feedback: non è offline pura.
Senza `--offline`, il SOCP viene aggiornato online con lo stesso feedforward + I.

La legge nuova riguarda solo la normale. `--twist`/`--wrench-channels` selezionano
ancora i canali di feedback precedenti, senza aggiungere un feedforward vettoriale
basato su rigidezze del tripad non calibrate. `--no-bending` mantiene il suo significato.
`--squeeze-gain` non cambia né l'I normale né quella di pickup: scala solo i canali
tangenziali legacy `forces/contact`. La torsione conserva il suo gain separato.

## Registrazione e validazione

CSV `_ffi_`, colonne `ffi_seq`, `ffi_dt_ms`, `ffi_error_N`, `ffi_ff_want_mm`,
`ffi_ff_mm`, `ffi_i_mm`, `ffi_output_mm`, `ffi_ff_limited`, `ffi_i_limited`,
`ffi_rate_limited`, `ffi_position_limited`. Sono lo stato dell'ultimo aggiornamento;
selezionare gli aggiornamenti distinti con `ffi_seq`, e le fasi di controllo attivo.
Il log distingue il nominale richiesto, il nominale limitato e il trim applicato.
Le colonne `pi_*` presenti durante pickup appartengono al riuso dell'implementazione
I con P=0; non indicano l'attivazione di un PI durante la route.

Il meta registra tipo, parametri, riferimento di pickup, k_eff e profilo offline.
I campi del vecchio gain normale sono null. Gli altri dati (wrench completi,
pose, joints, timestamp) conservano la semantica del logger esistente.

`tests/test_squeeze_ffi.py` verifica forza sommata, doppio conteggio, tempo reale,
limiti accoppiati, anti-windup, input errati e compensazione di un disturbo su
una pianta statica artificiale. `analysis_20261023/validate_ffi.py` esegue rotte
Z/T/L con SOCP online/offline e canali scalari/contact, includendo mismatch tra
rigidezza del modello e del toy backend. I CSV sono marcati SYNTHETIC e salvati
in `tmp/ffi_validation`. Non simulano attrito, risonanze, ritardi, vicini o slip.

Nessuna prova software dimostra l'eliminazione dell'oscillazione reale. Il confronto
deve controllare sia l'onda sia il minimo di forza al contatto e la saturazione
del trim. Se il trim resta al limite con errore persistente, i limiti/modello non
permettono di raggiungere il target: non basta aumentare il gain.
