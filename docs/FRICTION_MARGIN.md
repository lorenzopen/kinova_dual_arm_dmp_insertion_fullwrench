# Coni a distanza firmata: formulazione attuale

Aggiornamento del 20 ottobre 2026 richiesto da Lorenzo sulla base di
`TESI_antiG/PAPERS/friction_cone.pdf`, *Keeping Contact: Task-Adaptive Friction
Margins for Bimanual Non-Prehensile Manipulation*, autori anonimi nel PDF.
Controllati testo e immagini delle pagine 2-4, equazioni 1-8, 16-27, e
l'appendice a pagina 8. Il precedente fattore `1-r_s=0.70` e l'offset lungo
la normale `cone_shift_N` sono rimossi dalla formulazione attiva.

## Vincolo di contatto: esattamente la distanza del PDF, estesa a 3D

Per ogni punto attivo, n e' la normale uscente dall'oggetto, f la forza sul
pezzo, `N=-n.T*f>=0` la compressione e `ft=(I-n*n.T)*f` la tangenziale:

```text
psi_mu(f) = (mu*N - norm(ft)) / sqrt(1+mu^2)       [N]
psi_mu(f) >= rho_i                              [N]
```

Nel solver si impone la SOC equivalente (eq. 7):

```text
norm(ft_i) <= mu*N_i - rho_i*sqrt(1+mu^2)
N_i >= 0
```

La norma della tangenziale usa entrambe le direzioni del piano di contatto.
L'apertura nominale resta `atan(mu)`: nessuna riduzione di mu. A tangenziale
nulla il minimo normale indotto e' `rho_i*sqrt(1+mu^2)/mu` (eq. 8).
Il valore rho e' una distanza nel piano/spazio delle forze, **non** lo
spostamento del vertice lungo la normale: con mu=1, rho=0.5 N induce un
minimo di 0.7071 N per punto, non 0.5 N.

Il controllo numerico ricalcola `psi_mu` dalla soluzione e verifica il margine
imposto per ogni punto. Le tolleranze restano 1e-4 N sulle forze e sul residuo
SOC, 1e-5 Nm sull'equilibrio dei momenti. `eta_max` continua a essere l'utilizzo
del cono nominale `norm(ft)/(mu*N)`, che puo' superare 0.70 senza violare il
nuovo metodo: e' la distanza assoluta, non un utilizzo percentuale fisso, a
definire la protezione.

## Margine adattivo: adattamento dichiarato all'inserimento 3D

Il PDF deriva la legge affine su un task planare di tilting con pivot noto e
una sola equazione di momento. Il nostro task bilancia invece un wrench 3D
con sei punti; non si inventa un pivot e non si copiano i pesi 0.56/0.83 della
sua geometria. Si usa la costruzione generale di correzione a norma minima
dell'appendice, con la seguente scelta esplicita dell'insieme di deviazioni.

Sia A la matrice di grasp G limitata alle colonne dei punti attivi. Se
`no_bending=true`, A include anche le equazioni di momento di flessione nullo
di ciascun tool, con target zero. Il carico b e' il wrench richiesto corrente:
peso e reazione prevista dalla route, con gli eventuali zeri aggiuntivi.

```text
F_task = A^+ b                      # soluzione a norma minima, prima dei coni
rho_task = margin_task_scale*norm(F_task) + margin_floor_N
P = A^+ A
beta_i = norm(P_i, 2)               # norma spettrale del blocco di 3 righe
rho_i = beta_i*rho_task
```

`F_task` ha unita' N. Per un task ammissibile, e' la soluzione esatta a norma
minima dell'uguaglianza; non e' una pseudoinversa usata per sacrificare
l'equilibrio. Si controlla il residuo e un task non bilanciabile viene rifiutato.
Il margine e' calcolato prima del solve: non cresce in funzione della stretta
che l'ottimizzatore sta scegliendo.

La scelta generale corrisponde all'insieme
`D={delta_w=A*u : norm(u)<=rho_task}`. Per ciascuna di queste deviazioni la
correzione a norma minima e' `-A^+delta_w=-P*u`; quindi il blocco di ciascun
punto soddisfa `norm(delta_f_i)<=norm(P_i,2)*rho_task=rho_i`.
La proprieta' Lipschitz della distanza del paper mantiene tali forze nel cono
nominale, per la geometria e mu assunti. Il massimo della norma della
correzione complessiva e' rho_task (per A di rango positivo).

**Nel caso scalare del PDF** questa allocazione si riduce esattamente a
`beta_i=norm(G_i)/norm(G)`, cioe' all'equazione 25. Per il nostro task
multidimensionale e' una scelta coerente con l'appendice, non un'equivalenza
fra l'inserimento e il tilting del paper. L'insieme D e' modellato tramite
perturbazioni di forza; non viene presentato come una misura certificata
degli errori reali di massa/CoM, attrito, geometria o tracking.

## Configurazione attiva

Massa reale aggiornata da Lorenzo il 20 ottobre: **50 g per tetromino intero**.
Il config attivo imposta `piece_mass: 0.050` kg. Il modello precedente usava
circa 0.14468 kg ricavati dalla route: i risultati iniziali sotto restano
documentati con quella massa. Confronto aggiornato sulla T a 50 g:
[esperimento 50 g](experiments/signed_friction_margin_50g_20261020/README.md).

```yaml
socp:
  margin_mode: task_adaptive
  margin_task_scale: 0.25
  margin_floor_N: 1.0
  margin_fixed_N: 0.5
  w_force: 0.0
  no_bending: false
```

Il coefficiente 0.25 riprende il parametro di progetto del PDF; il termine
fisso iniziale di 0.5 N e' stato alzato a **1.0 N** su richiesta di Lorenzo,
per aumentare la presa in aria sul pezzo da 50 g. La distribuzione e' adattata
al tri-pad. Sono punti iniziali per la verifica, non limiti di incertezza
calibrati sul robot. `margin_floor_N` e' una quantita' a livello del task,
non 1 N su ogni punto e non 1 N per mano. Il costruttore standalone del solver
mantiene il riferimento del paper a 0.5 N; il controllore e il config operativo
passano esplicitamente 1 N.

Confronto di taratura sulla T: [aumento del margine a 50 g](experiments/margin_floor_50g_20261020/README.md).

Modalita' disponibili:

- `task_adaptive`: costruzione sopra; **default attivo** nel codice e nel config.
- `fixed`: `rho_i=margin_fixed_N` per punto attivo, utile per confronto.
- `none`: rho_i=0, cono nominale di Coulomb, utile per l'ablation.

Ogni punto inattivo ha forza e margine zero. Il coefficiente mu resta quello
della route (1.0 nelle tre prove), non il valore 0.75 dell'hardware del paper.
Il costo a precisione umana sulle flange, i loop di feedback, la geometria
fisica e il clamp di 1 N sul target per mano non sono modificati.

I vecchi parametri `r_s` e `cone_shift_N`, se non nulli, causano un errore
esplicito: un vecchio profilo non riattiva silenziosamente il vecchio metodo.
Aggiornati anche gli esempi che generano profili e l'analizzatore offline.
Quest'ultimo usa il config fornito e avvisa che ricostruisce il solver attuale,
non la soluzione storica completa, che i vecchi CSV non salvavano.

## Verifiche e risultati

Dodici test passati: forma analitica con mu=1 e mu=0.75, precarico a tangenziale
nulla, torsione del triangolo, punti inattivi, verifica indipendente del vincolo,
parametri invalidi e rifiuto del vecchio restringimento, equivalenza con eq. 25
per il task scalare, correzioni multidimensionali e perturbazioni entro rho,
variazione con il carico e riuso DPP, equazioni no-bending nel margine adattivo,
invarianza al cambio di unita' nelle equazioni di equilibrio.

Confronto iniziale con massa di modello circa **0.14468 kg**, prima della misura
reale di 50 g, con lo stesso controllore e backend DryRun su I/T/Z. Forza mediana
richiesta per mano in N:

| Route | Cono nominale, rho=0 | Margine fisso rho=0.5 N/punto | Margine adattivo attivo |
|---|---:|---:|---:|
| I `F75_It1_s9001` | 1.961 | 3.860 | 4.013 |
| T `F75_Tt1_s9007` | 2.756 | 4.572 | 4.651 |
| Z `F75_Zt1_s9012` | 1.570 | 3.633 | 3.738 |

Tutte le 930 allocazioni delle 9 prove sono state accettate dopo i controlli
numerici. Tutte le sequenze arrivano all'appoggio nel backend semplificato.
Residui massimi sulle nove prove: `3.61e-9 N` sull'equilibrio delle forze,
`2.06e-10 Nm` sui momenti, `2.78e-8 N` sulla SOC. Il comando `main check`
standard, con il config attivo, e' verificato anche sulla T: 104/104 allocazioni,
target mediano 4.7 N/mano. I tempi mediani di allocazione adattiva sul PC di
questa prova sono circa 2.4-2.5 ms; non sono misure dei tempi sul PC del robot.
Il solver puo' emettere `optimal_inaccurate`; l'accettazione richiede comunque
il controllo dei residui. Il dry run non simula slittamento, perdita dei punti,
dinamica dei contatti o tracking dei momenti: non prova il successo sul robot.

Grafico: [confronto delle tre modalita'](experiments/signed_friction_margin_20261020/comparison.png).
CSV e metadati: [summary.csv](experiments/signed_friction_margin_20261020/summary.csv),
[summary.json](experiments/signed_friction_margin_20261020/summary.json).
I file `*_allocations.csv` riportano rho minimo/massimo, surplus di distanza,
residui e tempi; i file `*_ticks.csv` conservano target e risposta del controllore.

Riproduzione:

```powershell
python -m unittest discover -s tests -v
python -m kinova_dual_arm_dmp_insertion.compare_friction_margin
python -m kinova_dual_arm_dmp_insertion.main check F75_Tt1_s9007 --arm c100_five_cut800
```

Il vecchio esperimento di offset con cono al 70% resta archiviato con i suoi
dati nel commit `a9cceda` e nella directory `cone_shift_20261020`.

## Ricontrollo di coerenza con il PDF

La corrispondenza riguarda il **modello del margine di attrito**, non una copia
di tutto l'algoritmo sperimentale del paper:

| Elemento | Paper | Implementazione di questo progetto |
|---|---|---|
| Distanza dal cono | Eq. 3-8 | Stessa espressione, con norma tangenziale 3D |
| Parametro per ogni solve | rho fissato prima dell'ottimizzazione | Stesso criterio, Parametri CVXPY, struttura DPP |
| Task | Momento scalare attorno a pivot, 2 contatti planari | Wrench 3D al CoM assunto, 6 punti |
| Distribuzione adattiva | Eq. 25 per il task scalare | Estensione dichiarata tramite P=A^+A e D specificato sopra; coincide col caso scalare |
| Costo finale | Minimo della somma dei quadrati delle forze, eq. 27 | Prior umano sui wrench di flangia, mantenuto dal progetto |
| Termine fisso | 0.5 N nel loro setup | 1 N nel profilo attivo, scelto nella taratura richiesta da Lorenzo |

Non viene quindi affermato che il profilo implementato riproduca i loro
esperimenti, ne' che D sia un insieme identificato degli errori di massa/CoM
del robot reale. Il vincolo implementa la distanza del paper; la costruzione
3D e' un adattamento modellistico coerente con la sua appendice.

Audit numerico: **16 test passati**. Oltre ai dodici iniziali, verificati il
cambio rigido di frame, la normalizzazione coerente delle normali, il rifiuto
di forze su punti inattivi e il rifiuto di flessione interna quando
`no_bending=true` anche se l'equilibrio globale e i coni risultano soddisfatti.
Sono stati rafforzati questi ultimi controlli indipendenti del risultato:
prima non verificavano esplicitamente i residui di inattivita' e flessione.
Le correzioni non cambiano la formula dei coni o la taratura del margine.

Ripetuto il dry run I/T/Z con 50 g e termine fisso 1 N dopo il rafforzamento
dei controlli: risultati in
[audit attivo](experiments/margin_floor_50g_20261020/audit_active_floor100/summary.json).
La validazione numerica riguarda forze pianificate; il loop reale continua
a regolare principalmente la somma delle compressioni, non ogni punto.
