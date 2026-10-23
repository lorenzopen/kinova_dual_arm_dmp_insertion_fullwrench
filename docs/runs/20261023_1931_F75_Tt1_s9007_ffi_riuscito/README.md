# 23/10/2026 19:31 - F75_Tt1_s9007 (la T) con `insert_ffi --offline --k-eff 680`: ENTRATA (terza)

Ripetizione identica della 19:21 e della 19:27 (`../20261023_1921_...`, `../20261023_1927_...`). Un tentativo alle
19:29 (`..._ffi_20261023_192957.csv`, vuoto, nessun meta) si è fermato prima di partire: non archiviato.

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
export W=F75_Tt1_s9007 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.insert_ffi insert "$W" --arm "$R" --slow 3 --offline --k-eff 680
```

## Parametri

| | valore |
|---|---|
| pezzo / route | `F75_Tt1_s9007` (T), braccio `c100_five_cut800`, 50 g (`piece_mass` di cell.yaml), `--slow 3` |
| codice | commit `9444de2`, branch `codex/feedforward-i-20261023`, pulito (codice identico a `1c25bd7`: in mezzo solo docs); `local.yaml` copiato qui |
| bracci (T1) | cart_k 1500 1500 1500 20 20 20, cart_d 60 60 60 7 7 7, friction e kalman accesi, 1 kHz |
| k_eff | 680 N/m per mano (`--k-eff`), sia per il piano offline sia per il nominale |
| piano SOCP offline | 235/235 allocazioni ammissibili, `..._offline_profile_20261023_193125.csv` |
| presa iniziale | I sugli ATI: K_I 1 mm/(N s), P 0, 2 mm/s, max 15 mm; forza misurata alla presa 5,73 N per mano |
| nominale | (target − 5,73 N) / 680 per mano, relativo alla presa |
| trim I | `--i-ki` 0,0016 m/(N s) (default) sull'errore della somma; ±3 mm per mano |
| limiti | 2 mm/s per mano (nominale + trim), ±12 mm dalla presa, dt max 100 ms |
| twist / canali | no (`wrench_channels: none`) |
| pezzo | dato a mano alla posa di presa |

Bag: `/home/nvidia/lorenzo/bags/sessione_20261023_193209` sul PC robot.

## Esito

**Entrata:** `seat_found`, nessun JAM, drop 0, errore stimato **+0,4 mm**, forza laterale max 1,45 N (forza esterna
massima 3,4 N, nello slow_seat), route completa (18,9 s), slow_seat, release e retreat.

**Stretta nella route** (somma delle due mani, mediana per sesto: chiesta / misurata):

| | 1° | 2° | 3° | 4° | 5° | 6° |
|---|---|---|---|---|---|---|
| **19:31 (questa)** | 9,7 / 9,4 | 9,4 / 9,4 | 9,9 / 9,9 | 9,2 / 9,1 | 8,5 / 8,4 | 8,4 / 8,1 |
| 19:27 | 9,7 / 9,3 | 9,4 / 9,4 | 9,9 / 10,0 | 9,2 / 9,1 | 8,5 / 8,3 | 8,4 / 8,2 |
| 19:21 | 9,7 / 9,4 | 9,4 / 9,4 | 9,9 / 9,9 | 9,2 / 9,1 | 8,5 / 8,3 | 8,4 / 8,2 |

**Le tre T FFI a confronto** (e le T del 21/10, `main insert --twist`, gain 1, cart_k 1000):

| | 21/10 (3 prove) | 19:21 | 19:27 | **19:31** |
|---|---|---|---|---|
| errore stimato in sede | −0,4 / 2,2 / 2,0 mm | 0,5 mm | 0,3 mm | **0,4 mm** |
| drop | 1,8 / 0 / 0 mm | 0 | 0 | 0 |
| forza laterale max | 0,80–1,42 N | 1,45 N | 1,63 N | 1,45 N |
| forza esterna max (slow_seat) | 1,1–3,6 N | 3,5 N | 3,7 N | 3,4 N |
| forza alla presa (per mano) | – | 5,81 N | 5,83 N | 5,73 N |
| errore stretta in route (senza ultimi 2 s): RMS / media / max | 0,53–0,61 N; max 2,4–2,8 | 0,57 / 0,00 / 2,9 N | 0,56 / −0,01 / 3,3 N | 0,58 / −0,02 / 3,0 N |
| oscillazione 0,8–2,5 Hz (somma): RMS, picco | 0,34–0,42 N | 0,22 N, 0,14 a 1,27 Hz | 0,19 N, 0,10 a 1,16 Hz | 0,21 N, 0,13 a 1,58 Hz |
| trim I (mm per mano) | – | −1,2 … +0,7 | −1,0 … +0,8 | −1,6 … +0,3 |
| rate limit nella route | – | 6% | 6% | 7% |
| inseguimento max sx / dx | 16,0–18,2 / 16,1–18,4 mm | 11,8 / 12,6 mm | 11,6 / 12,7 mm | 11,7 / 12,8 mm |
| slow_seat: min stretta chiesta / misurata; max misurata per mano | 7,6–8,0 / 7,0–7,2 N | 7,7 / 6,7 N; 4,4 N | 7,7 / 5,6 N; 4,4 N | 7,7 / 6,0 N; 4,6 N |

T con FFI **3 su 3**, errore in sede 0,3–0,5 mm, stretta in route identica (entro 0,1 N), oscillazione 0,19–0,22 N.
Nello slow_seat la stretta minima sta fra 5,6 e 6,7 N su 7,7 chiesti in tutte e tre.

Grafico `forze_ffi.png` (`plot_ffi.py`, autonomo: legge questa cartella, la 19:27, la 19:21 e
`../20261021_F75_Tt1_s9007_twist_ati/`).
