# 23/10/2026 19:21 - F75_Tt1_s9007 (la T) con `insert_ffi --offline --k-eff 680`: ENTRATA

Stesso controllore e stessi parametri delle Z, L e S riuscite con l'FFI oggi; cambia solo il pezzo. Prima T con l'FFI.

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
python -m kinova_dual_arm_dmp_insertion.insert_ffi check  "$W" --arm "$R" --slow 3 --offline --k-eff 680   # dry run
python -m kinova_dual_arm_dmp_insertion.insert_ffi insert "$W" --arm "$R" --slow 3 --offline --k-eff 680
```

## Parametri

| | valore |
|---|---|
| pezzo / route | `F75_Tt1_s9007` (T), braccio `c100_five_cut800`, 50 g (`piece_mass` di cell.yaml), `--slow 3` |
| codice | commit `21fa4a8`, branch `codex/feedforward-i-20261023`, pulito (codice identico a `1c25bd7`: in mezzo solo docs); `local.yaml` copiato qui |
| bracci (T1) | cart_k 1500 1500 1500 20 20 20, cart_d 60 60 60 7 7 7, friction e kalman accesi, 1 kHz |
| k_eff | 680 N/m per mano (`--k-eff`), sia per il piano offline sia per il nominale |
| piano SOCP offline | 235/235 allocazioni ammissibili, `..._offline_profile_20261023_192142.csv` |
| presa iniziale | I sugli ATI: K_I 1 mm/(N s), P 0, 2 mm/s, max 15 mm; forza misurata alla presa 5,81 N per mano |
| nominale | (target − 5,81 N) / 680 per mano, relativo alla presa |
| trim I | `--i-ki` 0,0016 m/(N s) (default) sull'errore della somma; ±3 mm per mano |
| limiti | 2 mm/s per mano (nominale + trim), ±12 mm dalla presa, dt max 100 ms |
| twist / canali | no (`wrench_channels: none`) |
| pezzo | dato a mano alla posa di presa |

Bag: `/home/nvidia/lorenzo/bags/sessione_20261023_192218` sul PC robot.

## Esito

**Entrata:** `seat_found`, nessun JAM, drop 0, errore stimato **+0,5 mm**, forza laterale max 1,45 N (forza esterna
massima 3,5 N, nello slow_seat), route completa (18,9 s), slow_seat, release e retreat. Errore di inseguimento max
nella route 11,8 mm (sx) / 12,6 mm (dx).

**Stretta nella route** (somma delle due mani, mediana per sesto: chiesta / misurata):

| 1° | 2° | 3° | 4° | 5° | 6° |
|---|---|---|---|---|---|
| 9,7 / 9,4 | 9,4 / 9,4 | 9,9 / 9,9 | 9,2 / 9,1 | 8,5 / 8,3 | 8,4 / 8,2 |

- Errore della somma nella route (senza gli ultimi 2 s): RMS 0,57 N, media 0,00, max 2,9 N.
- Nominale da −1,2 a −3,4 mm, trim I tra −1,2 e +0,7 mm, mai al limite di posizione; rate limit nel 6% della route.
- Slow_seat: minimo della stretta 6,7 N su 7,7 chiesti; massimo misurato ~4,4 N per mano.

**Confronto con le T del 21/10** (`../20261021_F75_Tt1_s9007_twist_ati/`, `main insert --twist`, gain 1 di allora,
cart_k 1000; il piano di allora chiedeva di più a metà route, fino a ~14 N sulla somma, con i margini di allora):

| | 21/10 20:15 | 21/10 20:28 | 21/10 20:34 | **23/10 19:21 FFI** |
|---|---|---|---|---|
| esito | entrata (drop 1,8 mm) | inserita (vicini spostati) | inserita | **entrata, drop 0** |
| errore stimato in sede | −0,4 mm | 2,2 mm | 2,0 mm | **0,5 mm** |
| forza laterale max | 0,80 N | 1,42 N | 1,40 N | 1,45 N |
| forza esterna max (fase) | 1,1 N (route) | 3,6 N (release) | 3,2 N (slow_seat) | 3,5 N (slow_seat) |
| errore stretta in route (RMS / max) | 0,61 / 2,8 N | 0,60 / 2,6 N | 0,53 / 2,4 N | 0,57 / 2,9 N |
| oscillazione 0,8–2,5 Hz (somma): RMS, picco | 0,41 N, 0,24 a 1,85 Hz | 0,42 N, 0,22 a 1,00 Hz | 0,34 N, 0,22 a 1,95 Hz | **0,22 N**, 0,14 a 1,27 Hz |
| inseguimento max sx / dx | 17,5 / 18,4 mm | 18,2 / 16,4 mm | 16,0 / 16,1 mm | 11,8 / 12,6 mm |
| slow_seat: min stretta chiesta / misurata | 7,6 / 7,0 N | 7,7 / 7,2 N | 8,0 / 7,2 N | 7,7 / 6,7 N |

Il confronto mescola controllore, piano SOCP (margini cambiati dal 21/10), impedenza e twist.

Grafico `forze_ffi.png` (`plot_ffi.py`, autonomo: legge questa cartella e `../20261021_F75_Tt1_s9007_twist_ati/`).
