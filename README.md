# kinova_dual_arm_dmp_insertion (copia full wrench)

**Per usarla sui robot: [GUIDA.md](GUIDA.md)**. È l'unica guida: setup, terminali, ATI, rotte, board e comandi.

Due Kinova Gen3 inseriscono UN tetramino in uno slot ristretto di uno scaffale, lungo una rotta DMP
addestrata in PyBullet. La rotta è 2D: piano x-z dello scaffale, con rotazione attorno all'asse dei pad.
La stretta è quella del SOCP: coni di attrito come vincoli, prior umano TP-GMM nel costo, retroazione sugli
ATI da polso. Con `--twist` si aggiunge il twist opposto attorno all'asse dei pad (full wrench).
Solo l'inserimento: il pezzo viene presentato a mano e la board si costruisce a mano.

- Metodo e differenze dal pacchetto originale: [FULL_WRENCH.md](FULL_WRENCH.md)
- Margini di attrito (coni): [docs/FRICTION_MARGIN.md](docs/FRICTION_MARGIN.md)
- Prove reali del 16/10/2026: [docs/runs/](docs/runs/README.md)

| file | cosa |
|---|---|
| `main.py` | `list`, `check` (dry-run senza robot), `insert [--twist]`; log CSV + `_meta.json` |
| `controller.py` | l'inserimento: frame oggetto, anello di stretta, SOCP, sentinella di jam, appoggio, rilascio |
| `socp.py`, `prior.py` | il programma convesso full wrench e il prior umano |
| `ros_backend.py` | l'unico file che conosce ROS: target, cinematica diretta, ATI (`ft`) o stima dei giunti |
| `ft_check.py` | ATI: unità, tare, rumore, orientamento e segno (`--calibrate`), lettura dal vivo |
| `follow_two_arms.py` | il percorso a due bracci a mani vuote (nessun SOCP) |
| `dryrun.py`, `geometry.py`, `route.py`, `pair.py` | backend senza robot, geometria del tool, lettura delle rotte |
| `routes/<set>/<mondo>.json` | le rotte esportate da PyBullet (dati) |
| `config/cell.yaml` | cella, topic, ATI, SOCP, sicurezza; `config/local.yaml` (fuori da git) = valori misurati sul PC robot |

Ancora da sapere prima di citare un risultato: ogni rotta è stata addestrata su una board e una presa nominali;
la compliance del tool reale e quella del controller non sono quelle simulate. Il piazzamento si misura a mano.
