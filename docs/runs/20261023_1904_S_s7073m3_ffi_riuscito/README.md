# 23/10/2026 19:04 - S_s7073m3 (la S, `c100_narrow75`) con `insert_ffi --offline --k-eff 680`: ENTRATA

Stesso controllore e stessi parametri delle Z (17:43 / 17:51 / 17:56) e delle L (18:27 / 18:37 / 18:41) riuscite;
cambiano pezzo e braccio (`c100_narrow75`). Prima S con l'FFI. Un tentativo alle 19:03
(`..._ffi_20261023_190311.csv`, solo intestazione, nessun meta) si è fermato prima di partire: non archiviato.

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
python -m kinova_dual_arm_dmp_insertion.insert_ffi check  "$W" --arm "$R" --slow 3 --offline --k-eff 680   # dry run
python -m kinova_dual_arm_dmp_insertion.insert_ffi insert "$W" --arm "$R" --slow 3 --offline --k-eff 680
```

## Parametri

| | valore |
|---|---|
| pezzo / route | `S_s7073m3` (S), braccio `c100_narrow75`, 50 g (`piece_mass` di cell.yaml), `--slow 3` |
| codice | commit `c44821a`, branch `codex/feedforward-i-20261023`, pulito (codice identico a `1c25bd7`: in mezzo solo docs); `local.yaml` copiato qui |
| bracci (T1) | cart_k 1500 1500 1500 20 20 20, cart_d 60 60 60 7 7 7, friction e kalman accesi, 1 kHz |
| k_eff | 680 N/m per mano (`--k-eff`), sia per il piano offline sia per il nominale |
| piano SOCP offline | 345/345 allocazioni ammissibili, `..._offline_profile_20261023_190423.csv` |
| presa iniziale | I sugli ATI: K_I 1 mm/(N s), P 0, 2 mm/s, max 15 mm; forza misurata alla presa 5,86 N per mano |
| nominale | (target − 5,86 N) / 680 per mano, relativo alla presa |
| trim I | `--i-ki` 0,0016 m/(N s) (default) sull'errore della somma; ±3 mm per mano |
| limiti | 2 mm/s per mano (nominale + trim), ±12 mm dalla presa, dt max 100 ms |
| twist / canali | no (`wrench_channels: none`) |
| pezzo | dato a mano alla posa di presa |

Bag: `/home/nvidia/lorenzo/bags/sessione_20261023_190457` sul PC robot.

## Esito

**Entrata:** `seat_found`, nessun JAM, drop 0, errore stimato **+2,9 mm**, forza laterale max **1,72 N**, route
completa (15,4 s, dopo un to_mouth lungo ~17 s), slow_seat, release e retreat. Errore di inseguimento max nella route
6,7 mm (sx) / 8,5 mm (dx).

**Stretta nella route** (somma delle due mani, mediana per sesto: chiesta / misurata):

| 1° | 2° | 3° | 4° | 5° | 6° |
|---|---|---|---|---|---|
| 8,2 / 8,0 | 8,3 / 8,4 | 8,3 / 8,3 | 8,0 / 7,9 | 9,1 / 8,2 | 10,5 / 10,2 |

- Errore della somma nella route (senza gli ultimi 2 s): RMS 1,17 N, media −0,31, max 4,6 N. Più alto che su Z e L
  per un solo tratto: verso la fine della route il piano sale in fretta fino a ~7,4 N per mano; il nominale va al
  limite di 2 mm/s (rate limit nel 18% della route) e la misurata arriva con ~1 s di ritardo (errore fino a +4,6 N),
  poi segue. Fuori da quel tratto l'errore sta entro circa ±1 N.
- Trim I tra −0,2 e +1,0 mm nella route (fino a ~+2,2 mm nel to_mouth), mai al limite di posizione.
- Slow_seat: minimo della stretta 6,9 N su 7,1 chiesti; massimo misurato ~5,5 N per mano.

**Confronto con le S del 22/10** (`../20261022_S_s7073m3_twist_ati/`, `main insert --twist`, gain 1 di allora;
impedenza e damping di allora, 1000 e 5/1, dedotti dall'inseguimento ~10–12 mm e da GUIDA):

| | 22/10 12:45 | 22/10 13:48 | **23/10 19:04 FFI** |
|---|---|---|---|
| esito | entrata | entrata (sede al tatto, drop 4,9 mm) | **entrata, drop 0** |
| errore stimato in sede | 1,8 mm | −0,5 mm | 2,9 mm |
| forza laterale max | 4,67 N | 4,45 N | **1,72 N** |
| errore stretta in route (RMS / max) | 1,42 / 7,1 N | 1,59 / 8,6 N | **1,17 / 4,6 N** |
| oscillazione 0,8–2,5 Hz (somma): RMS, picco | 0,85 N, 1,20 a 0,84 Hz | 0,65 N, 0,83 a 0,97 Hz | **0,14 N**, 0,10 a 0,84 Hz |
| inseguimento max sx / dx | 10,4 / 12,6 mm | 11,5 / 12,3 mm | 6,7 / 8,5 mm |
| slow_seat: min stretta chiesta / misurata | 7,5 / 6,3 N | 7,6 / 6,2 N | 7,1 / 6,9 N |

Il confronto mescola controllore (FFI contro integratore a gain 1 con twist), impedenza e damping.

Grafico `forze_ffi.png` (`plot_ffi.py`, autonomo: legge questa cartella e `../20261022_S_s7073m3_twist_ati/`):
forza normale per mano chiesta e misurata, errore, termini nominale / trim / totale, banda 0,8–2,5 Hz e spettro
contro le due S del 22/10.
