# 23/10/2026 18:37 - F75_Lt1_s9014 (la L) con `insert_ffi --offline --k-eff 680`: ENTRATA (seconda di fila)

Ripetizione identica della 18:27 (`../20261023_1827_F75_Lt1_s9014_ffi_riuscito/`).

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
export W=F75_Lt1_s9014 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.insert_ffi insert "$W" --arm "$R" --slow 3 --offline --k-eff 680
```

## Parametri

| | valore |
|---|---|
| pezzo / route | `F75_Lt1_s9014` (L), braccio `c100_five_cut800`, scala 0,833, 50 g, apertura 100 mm, `--slow 3` |
| codice | commit `a34d2ba`, branch `codex/feedforward-i-20261023`, pulito (codice identico a `1c25bd7`: in mezzo solo docs); `local.yaml` copiato qui |
| bracci (T1) | cart_k 1500 1500 1500 20 20 20, cart_d 60 60 60 7 7 7, friction e kalman accesi, 1 kHz |
| k_eff | 680 N/m per mano (`--k-eff`), sia per il piano offline sia per il nominale |
| piano SOCP offline | 263/263 allocazioni ammissibili, `..._offline_profile_20261023_183755.csv` |
| presa iniziale | I sugli ATI: K_I 1 mm/(N s), P 0, 2 mm/s, max 15 mm; forza misurata alla presa 5,89 N per mano |
| nominale | (target − 5,89 N) / 680 per mano, relativo alla presa |
| trim I | `--i-ki` 0,0016 m/(N s) (default) sull'errore della somma; ±3 mm per mano |
| limiti | 2 mm/s per mano (nominale + trim), ±12 mm dalla presa, dt max 100 ms |
| twist / canali | no (`wrench_channels: none`) |
| pezzo | dato a mano alla posa di presa |

Bag: `/home/nvidia/lorenzo/bags/sessione_20261023_183827` sul PC robot.

## Esito

**Entrata:** `seat_found`, nessun JAM, drop 0, errore stimato **+3,0 mm**, forza laterale max 1,27 N, route
completa (22,0 s), slow_seat, release e retreat.

**Stretta nella route** (somma delle due mani, mediana per sesto: chiesta / misurata):

| | 1° | 2° | 3° | 4° | 5° | 6° |
|---|---|---|---|---|---|---|
| **18:37 (questa)** | 11,0 / 10,9 | 9,9 / 9,8 | 9,3 / 9,2 | 8,5 / 8,7 | 10,0 / 9,8 | 9,2 / 8,8 |
| 18:27 | 11,0 / 11,1 | 9,9 / 9,9 | 9,3 / 9,2 | 8,5 / 8,7 | 10,0 / 9,8 | 9,2 / 9,0 |

**Le due L FFI a confronto** (e le L del 22/10, `main insert --twist`, cart_k 1000, damping 5/1):

| | 22/10 (3 prove) | 23/10 18:27 | **23/10 18:37** |
|---|---|---|---|
| errore stimato in sede | 5,5–6,5 mm | 2,5 mm | **3,0 mm** |
| forza laterale max | 0,83–1,42 N | 0,95 N | 1,27 N |
| forza alla presa (per mano) | – | 5,63 N | 5,89 N |
| errore stretta in route (senza ultimi 2 s): RMS / media / max | 0,70–0,78 N | 0,46 / −0,01 / 3,0 N | 0,51 / −0,01 / 3,4 N |
| oscillazione 0,8–2,5 Hz (somma): RMS, picco | 0,62–0,68 N, 0,36–0,44 a 1,3–1,55 Hz | 0,23 N, 0,17 a 0,96 Hz | 0,23 N, 0,24 a 0,91 Hz |
| nominale (mm per mano) | – | 0 … −3,2 | −0,4 … −3,6 |
| trim I (mm per mano) | – | −1,6 … +0,4 | −1,4 … +0,9 |
| rate limit nella route | – | 13% | 12% |
| inseguimento max sx / dx | 13,7–14,2 / 17,4–18,1 mm | 9,3 / 11,7 mm | 9,3 / 11,5 mm |
| slow_seat: min stretta chiesta / misurata; max misurata per mano | – | 7,4 / 7,4 N; 6,4 N | 7,4 / 7,1 N; 6,2 N |

Ripetibile come per la Z: stessa stretta per sesto (entro 0,2 N), stessa oscillazione, trim lontano dal limite.
Nello slow_seat il pezzo spinge di nuovo sui pad (misurata ~6,2 N per mano contro ~3,8 chiesti), come alla 18:27.
Il confronto con il 22/10 mescola controllore, impedenza e damping.

Grafico `forze_ffi.png` (`plot_ffi.py`, autonomo: legge questa cartella, la 18:27 e `../20261022_F75_Lt1_s9014_twist_ati/`).
