# 23/10/2026 17:51 - F75_Zt1_s9012 con `insert_ffi --offline --k-eff 680`: ENTRATA (seconda di fila)

Ripetizione identica della 17:43 (`../20261023_1743_F75_Zt1_s9012_ffi_riuscito/`).

## Comando

```bash
# T1 (terminale normale, senza env.sh): launch di default
source /opt/ros/jazzy/setup.bash && source /home/nvidia/ros2_ws/install/setup.bash
ros2 launch /home/nvidia/lorenzo/kinova_dual_arm_dmp_insertion_fullwrench/launch/robot.launch.py

# T3
source ~/lorenzo/env.sh && cd "$LORENZO/kinova_dual_arm_dmp_insertion_fullwrench"
export W=F75_Zt1_s9012 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.insert_ffi insert "$W" --arm "$R" --slow 3 --offline --k-eff 680
```

## Parametri

| | valore |
|---|---|
| codice | commit `6b1a36d`, branch `codex/feedforward-i-20261023`, pulito (codice identico a `1c25bd7` della 17:43: in mezzo solo docs); `local.yaml` copiato qui |
| bracci (T1) | cart_k 1500 1500 1500 20 20 20, cart_d 60 60 60 7 7 7, friction e kalman accesi, 1 kHz |
| k_eff | 680 N/m per mano (`--k-eff`), sia per il piano offline sia per il nominale |
| piano SOCP offline | 232/232 allocazioni ammissibili, `..._offline_profile_20261023_175114.csv` |
| presa iniziale | I sugli ATI: K_I 1 mm/(N s), P 0, 2 mm/s, max 15 mm; forza misurata alla presa 5,87 N per mano |
| nominale | (target − 5,87 N) / 680 per mano, relativo alla presa |
| trim I | `--i-ki` 0,0016 m/(N s) (default) sull'errore della somma; ±3 mm per mano |
| limiti | 2 mm/s per mano (nominale + trim), ±12 mm dalla presa, dt max 100 ms |
| twist / canali | no (`wrench_channels: none`) |
| pezzo | dato a mano |

Bag: `/home/nvidia/lorenzo/bags/sessione_20261023_175235` sul PC robot.

## Esito

**Entrata:** `seat_found`, nessun JAM, errore stimato −0,05 mm, forza laterale max 1,32 N,
route completa (19,1 s), slow_seat, release e retreat.

**Stretta nella route** (somma delle due mani, mediana per sesto: chiesta / misurata):

| | 1° | 2° | 3° | 4° | 5° | 6° |
|---|---|---|---|---|---|---|
| **17:51 (questa)** | 9,0 / 9,0 | 8,8 / 8,9 | 8,6 / 8,5 | 8,2 / 8,3 | 8,0 / 7,8 | 7,1 / 7,2 |
| 17:43 | 9,0 / 9,0 | 8,8 / 8,8 | 8,6 / 8,6 | 8,2 / 8,3 | 8,0 / 7,8 | 7,1 / 7,1 |

**Confronto con la 17:43:**

| | 17:43 | **17:51** |
|---|---|---|
| errore stimato in sede | −0,45 mm | **−0,05 mm** |
| forza laterale max | 0,88 N | 1,32 N |
| errore stretta in route (senza ultimi 2 s): RMS / media / max | 0,42 / 0,00 / 2,2 N | 0,54 / +0,02 / 2,4 N |
| oscillazione 0,8–2,5 Hz (somma) | 0,24 N, picco 0,17 a 1,20 Hz | 0,25 N, picco 0,15 a 1,10 Hz |
| nominale (mm per mano) | −1,9 … −3,5 | −2,0 … −3,6 |
| trim I (mm per mano) | −0,6 … +0,8 | −1,1 … +0,6 |
| rate limit nella route | 7% | 9% |
| inseguimento max in route sx / dx | 9,9 / 11,0 mm | 9,4 / 10,9 mm |
| slow_seat: minimo stretta chiesta / misurata | 7,2 / 7,0 N | 7,2 / 6,6 N |

Ripetibile: stesse forze, stessa oscillazione, trim sempre lontano dal limite di ±3 mm.

Grafico `forze_ffi.png` (`plot_ffi.py`, legge i CSV di confronto da `logs/` sul PC robot).
