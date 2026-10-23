# 23/10/2026 19:13 - S_s7073m3 (la S, `c100_narrow75`) con `insert_ffi --offline --k-eff 680`: ENTRATA (seconda)

Ripetizione identica della 19:04 (`../20261023_1904_S_s7073m3_ffi_riuscito/`). In mezzo, due tentativi fermati con
Ctrl-C e non archiviati: 19:10:02 (in `preparation`, CSV vuoto) e 19:10:27 (in `release`, dopo la route;
`logs/S_s7073m3_c100_narrow75_ffi_20261023_191027.csv` sul PC robot).

## Comando

```bash
# T1 (terminale normale, senza env.sh): launch di default
source /opt/ros/jazzy/setup.bash && source /home/nvidia/ros2_ws/install/setup.bash
ros2 launch /home/nvidia/lorenzo/kinova_dual_arm_dmp_insertion_fullwrench/launch/robot.launch.py

# T2: bag
source ~/lorenzo/env.sh && cd "$LORENZO/bags"
ros2 bag record -e "/(left_arm|right_arm)/.*|/rosout" -o sessione_$(date +%Y%m%d_%H%M%S)

# T3
source ~/lorenzo/env.sh && cd "$LORENZO/kinova_dual_arm_dmp_insertion_fullwrench"
export W=S_s7073m3 R=c100_narrow75
python -m kinova_dual_arm_dmp_insertion.insert_ffi insert "$W" --arm "$R" --slow 3 --offline --k-eff 680
```

## Parametri

| | valore |
|---|---|
| pezzo / route | `S_s7073m3` (S), braccio `c100_narrow75`, 50 g (`piece_mass` di cell.yaml), `--slow 3` |
| codice | commit `2361f01`, branch `codex/feedforward-i-20261023`, pulito (codice identico a `1c25bd7`: in mezzo solo docs); `local.yaml` copiato qui |
| bracci (T1) | cart_k 1500 1500 1500 20 20 20, cart_d 60 60 60 7 7 7, friction e kalman accesi, 1 kHz |
| k_eff | 680 N/m per mano (`--k-eff`), sia per il piano offline sia per il nominale |
| piano SOCP offline | 345/345 allocazioni ammissibili, `..._offline_profile_20261023_191316.csv` |
| presa iniziale | I sugli ATI: K_I 1 mm/(N s), P 0, 2 mm/s, max 15 mm; forza misurata alla presa 5,80 N per mano |
| nominale | (target − 5,80 N) / 680 per mano, relativo alla presa |
| trim I | `--i-ki` 0,0016 m/(N s) (default) sull'errore della somma; ±3 mm per mano |
| limiti | 2 mm/s per mano (nominale + trim), ±12 mm dalla presa, dt max 100 ms |
| twist / canali | no (`wrench_channels: none`) |
| pezzo | dato a mano alla posa di presa |

Bag: `/home/nvidia/lorenzo/bags/sessione_20261023_191351` sul PC robot.

## Esito

**Entrata:** `seat_found`, nessun JAM, drop 0, errore stimato **+2,9 mm**, forza laterale max **2,99 N**
(forza esterna massima 5,0 N, nello slow_seat), route completa (15,4 s), slow_seat, release e retreat.

**Stretta nella route** (somma delle due mani, mediana per sesto: chiesta / misurata):

| | 1° | 2° | 3° | 4° | 5° | 6° |
|---|---|---|---|---|---|---|
| **19:13 (questa)** | 8,1 / 8,1 | 8,3 / 8,4 | 8,3 / 8,4 | 8,0 / 7,7 | 9,1 / 8,6 | 10,5 / 10,3 |
| 19:04 | 8,2 / 8,0 | 8,3 / 8,4 | 8,3 / 8,3 | 8,0 / 7,9 | 9,1 / 8,2 | 10,5 / 10,2 |

**Le due S FFI a confronto** (e le S del 22/10, `main insert --twist`, gain 1, cart_k 1000, damping 5/1):

| | 22/10 12:45 / 13:48 | 23/10 19:04 | **23/10 19:13** |
|---|---|---|---|
| errore stimato in sede | 1,8 / −0,5 mm | 2,9 mm | **2,9 mm** |
| drop | 0 / 4,9 mm | 0 | 0 |
| forza laterale max | 4,67 / 4,45 N | 1,72 N | 2,99 N |
| forza alla presa (per mano) | – | 5,86 N | 5,80 N |
| errore stretta in route (senza ultimi 2 s): RMS / media / max | 1,42 / 1,59 N; max 7,1 / 8,6 | 1,17 / −0,31 / 4,6 N | 0,95 / −0,21 / 3,9 N |
| oscillazione 0,8–2,5 Hz (somma): RMS, picco | 0,85 / 0,65 N | 0,14 N, 0,10 a 0,84 Hz | **0,12 N**, 0,08 a 1,30 Hz |
| nominale (mm per mano) | – | −3,4 … +0,6 | −3,3 … +0,7 |
| trim I (mm per mano) | – | −0,2 … +1,0 | −0,6 … +0,5 |
| rate limit nella route | – | 18% | 19% |
| inseguimento max sx / dx | 10,4–11,5 / 12,3–12,6 mm | 6,7 / 8,5 mm | 6,7 / 8,6 mm |
| slow_seat: min stretta chiesta / misurata; max misurata per mano | 7,5–7,6 / 6,2–6,3 N | 7,1 / 6,9 N; 5,5 N | 7,1 / 6,7 N; 6,3 N |

Ripetibile: stessa stretta per sesto (entro 0,4 N), stessa oscillazione, stesso errore in sede. Come alla 19:04, a fine
route il picco del piano (~7,4 N per mano) arriva con ritardo per il limite di 2 mm/s (errore max 3,9 N). La forza
laterale è più alta della 19:04 (3,0 contro 1,7 N), sempre sotto i ~4,5 N del 22/10.

Grafico `forze_ffi.png` (`plot_ffi.py`, autonomo: legge questa cartella, la 19:04 e `../20261022_S_s7073m3_twist_ati/`).
