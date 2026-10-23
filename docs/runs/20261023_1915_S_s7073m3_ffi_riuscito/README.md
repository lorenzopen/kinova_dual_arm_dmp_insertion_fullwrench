# 23/10/2026 19:15 - S_s7073m3 (la S, `c100_narrow75`) con `insert_ffi --offline --k-eff 680`: ENTRATA (terza)

Ripetizione identica della 19:04 e della 19:13 (`../20261023_1904_...`, `../20261023_1913_...`).

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
| codice | commit `f761b60`, branch `codex/feedforward-i-20261023`, pulito (codice identico a `1c25bd7`: in mezzo solo docs); `local.yaml` copiato qui |
| bracci (T1) | cart_k 1500 1500 1500 20 20 20, cart_d 60 60 60 7 7 7, friction e kalman accesi, 1 kHz |
| k_eff | 680 N/m per mano (`--k-eff`), sia per il piano offline sia per il nominale |
| piano SOCP offline | 345/345 allocazioni ammissibili, `..._offline_profile_20261023_191604.csv` |
| presa iniziale | I sugli ATI: K_I 1 mm/(N s), P 0, 2 mm/s, max 15 mm; forza misurata alla presa 5,91 N per mano |
| nominale | (target − 5,91 N) / 680 per mano, relativo alla presa |
| trim I | `--i-ki` 0,0016 m/(N s) (default) sull'errore della somma; ±3 mm per mano |
| limiti | 2 mm/s per mano (nominale + trim), ±12 mm dalla presa, dt max 100 ms |
| twist / canali | no (`wrench_channels: none`) |
| pezzo | dato a mano alla posa di presa |

Bag: `/home/nvidia/lorenzo/bags/sessione_20261023_191651` sul PC robot.

## Esito

**Entrata:** `seat_found`, nessun JAM, drop 0, errore stimato **+2,5 mm**, forza laterale max **3,16 N**
(forza esterna massima 5,4 N, nello slow_seat), route completa (15,4 s), slow_seat, release e retreat.

**Stretta nella route** (somma delle due mani, mediana per sesto: chiesta / misurata):

| | 1° | 2° | 3° | 4° | 5° | 6° |
|---|---|---|---|---|---|---|
| **19:15 (questa)** | 8,1 / 8,3 | 8,3 / 8,4 | 8,3 / 8,4 | 8,0 / 7,8 | 9,1 / 8,6 | 10,5 / 10,2 |
| 19:13 | 8,1 / 8,1 | 8,3 / 8,4 | 8,3 / 8,4 | 8,0 / 7,7 | 9,1 / 8,6 | 10,5 / 10,3 |
| 19:04 | 8,2 / 8,0 | 8,3 / 8,4 | 8,3 / 8,3 | 8,0 / 7,9 | 9,1 / 8,2 | 10,5 / 10,2 |

**Le tre S FFI a confronto** (e le S del 22/10, `main insert --twist`, gain 1, cart_k 1000, damping 5/1):

| | 22/10 12:45 / 13:48 | 19:04 | 19:13 | **19:15** |
|---|---|---|---|---|
| errore stimato in sede | 1,8 / −0,5 mm | 2,9 mm | 2,9 mm | **2,5 mm** |
| drop | 0 / 4,9 mm | 0 | 0 | 0 |
| forza laterale max | 4,67 / 4,45 N | 1,72 N | 2,99 N | 3,16 N |
| forza esterna max (fase) | – | 4,2 N (slow_seat) | 5,0 N (slow_seat) | 5,4 N (slow_seat) |
| forza alla presa (per mano) | – | 5,86 N | 5,80 N | 5,91 N |
| errore stretta in route (senza ultimi 2 s): RMS / media / max | 1,42 / 1,59 N; max 7,1 / 8,6 | 1,17 / −0,31 / 4,6 N | 0,95 / −0,21 / 3,9 N | 1,12 / −0,27 / 4,5 N |
| oscillazione 0,8–2,5 Hz (somma): RMS, picco | 0,85 / 0,65 N | 0,14 N, 0,10 a 0,84 Hz | 0,12 N, 0,08 a 1,30 Hz | 0,15 N, 0,24 a 0,84 Hz |
| nominale (mm per mano) | – | −3,4 … +0,6 | −3,3 … +0,7 | −3,5 … +0,6 |
| trim I (mm per mano) | – | −0,2 … +1,0 | −0,6 … +0,5 | −1,1 … +0,5 |
| rate limit nella route | – | 18% | 19% | 19% |
| inseguimento max sx / dx | 10,4–11,5 / 12,3–12,6 mm | 6,7 / 8,5 mm | 6,7 / 8,6 mm | 6,7 / 8,3 mm |
| slow_seat: min stretta chiesta / misurata; max misurata per mano | 7,5–7,6 / 6,2–6,3 N | 7,1 / 6,9 N; 5,5 N | 7,1 / 6,7 N; 6,3 N | 7,1 / 5,6 N; 4,8 N |

S con FFI **3 su 3**, errore in sede 2,5–2,9 mm, stretta in route e oscillazione ripetibili; in tutte e tre il picco
del piano a fine route (~7,4 N per mano) arriva con ritardo per il limite di 2 mm/s. Da una prova all'altra crescono
la forza laterale (1,7 → 3,0 → 3,2 N) e la forza esterna nello slow_seat (4,2 → 5,0 → 5,4 N), e qui la stretta minima
nello slow_seat scende a 5,6 N su 7,1: forse la board si assesta dopo ogni inserimento (da verificare).

Grafico `forze_ffi.png` (`plot_ffi.py`, autonomo: legge questa cartella, la 19:13, la 19:04 e
`../20261022_S_s7073m3_twist_ati/`).
