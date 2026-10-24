# Gen3 articolati, posizionamento nominale del laboratorio

**Steering successivo:** trovato il sorgente del controller nei Download.
`run_lab_impedance.py` usa le classi originali conservate in `lab_source/`,
con Pinocchio reale. Il backend `run_gen3.py` descritto qui sotto è il confronto
preliminare con impedenza approssimata; non rappresenta il nuovo controller
originale. Le sue conclusioni geometriche sono sensibili al passo numerico.
Risultato attuale: [LAB_IMPEDANCE_RESULTS.md](LAB_IMPEDANCE_RESULTS.md).

Esecuzione della legge originale (un nome di output nuovo per ogni prova):

```powershell
& 'C:/Users/Lorenzo/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' `
  docs/analysis_20261023/controller_night/pybullet_nominal_20261024/gen3/run_lab_impedance.py `
  --piece Z --tag NUOVO_NOME_UNIVOCO --dt .00025 --kalman-warm-seconds 2 --video
```

Usare le variabili BLAS a un thread riportate sotto. Il riferimento fornito in
chat è verificato in `user_reference_check.json`; il segno Kortex è verificato
in `estimated_wrench_sign_check.json`. `lab_verification.json` include gli abort
in aria: questi risultati non validano un inserimento con il controller originale.

Il setup usa due robot a sette giunti, masse e inerzie URDF, gravità,
collisioni, tri-pad elastici, pezzo e vicini dinamici. Le basi sono ricavate
dalle trasformazioni `shelf_in_base` archiviate, nel frame dello scaffale.
I primi angoli registrati sono solo il seed della posa iniziale; durante la
simulazione non si impongono i movimenti reali né le forze registrate.

Leggere [RESULTS.md](RESULTS.md) e `articulated_sensor_checks.json` per risultati,
verifiche e limiti. La geometria nominale è consistente con la cinematica dei
log; questo controllo non è una nuova misura della calibrazione del banco.
Tavoli, cavi e fissaggi esterni non sono stati ricostruiti.

## Modello e controllo

- `assets/gen3_lab_bullet.urdf`: copia derivata, originale del repository intatto.
- `mesh_provenance.json`: mesh ufficiali Kinova, commit, SHA-256 e Git blob.
- `urdf_provenance.json`: trasformazioni della copia per il caricamento Bullet.
- `run_gen3.py`: stessi FFI/SOCP e sequenza esterna delle run; servo interno
  simulato come impedenza cartesiana dichiarata, con coppie ai 14 giunti.
- Molla centrale assiale **2700 N/m**; tre pad **500 N/m ciascuno**; totale
  assiale **4200 N/m**. K/D bracci 1500/20 e 60/7, invariati fra le prove.
- Tool totale 150 g: supporto rigido 110 g, piastra/pad mobili 40 g.
- FT al polso da giunto fisso; gravità dei 40 g mobili rimossa. Il supporto
  rigido a monte del sensore contribuisce alla dinamica ma non alla misura FT.

Il firmware interno Kinova non è disponibile: la legge in coppia è una
realizzazione nominale, non una replica verificata del servo del laboratorio.
La realizzazione localmente implicita usa un predittore di accelerazione
con matrice `M + h Bq + h² Kq`, h = 1 ms, e reazione FT corrente. Il metodo
si ispira ai controllori PD stabili, ma questa formula locale non viene
presentata come implementazione identica dell'articolo o del firmware.
Riferimento: [Tan, Liu, Turk — Stable Proportional-Derivative Controllers](https://faculty.cc.gatech.edu/~turk/my_papers/stable_pd.pdf).

## Riproduzione

Dalla radice del repository, usando il runtime Python della campagna:

```powershell
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
$env:MKL_NUM_THREADS='1'
$env:NUMEXPR_NUM_THREADS='1'
$env:PYTHONDONTWRITEBYTECODE='1'
& 'C:/Users/Lorenzo/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' `
  docs/analysis_20261023/controller_night/pybullet_nominal_20261024/gen3/run_gen3.py `
  --piece Z --tag NUOVO_NOME_UNIVOCO --dt .0005 --servo implicit --video
```

Una sola simulazione alla volta. Un nome già esistente viene rifiutato;
non si sovrascrivono i risultati. Il runtime Bullet e le DLL sono isolati in
`../deps/conda`, con provenienza in `../runtime_provenance.json`.
Ogni run conserva `robot_used.urdf` e `runner_source.py`, dati NPZ/CSV, piano
e allocazioni SOCP, diagnostica delle trasformazioni, immagini e GIF se richiesta.
L'uscita `seat_found` è conservata accanto all'errore geometrico vero: non
sono la stessa misura di riuscita.
