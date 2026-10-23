# 23/10/2026 18:27 - F75_Lt1_s9014 (la L) con `insert_ffi --offline --k-eff 680`: ENTRATA

Stesso controllore e stessi parametri delle tre Z riuscite delle 17:43 / 17:51 / 17:56 (forza nominale dal
SOCP offline + trim I sugli ATI, `docs/FFI_CONTROLLER.md`); cambia solo il pezzo. Prima L con l'FFI.

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
python -m kinova_dual_arm_dmp_insertion.insert_ffi check  "$W" --arm "$R" --slow 3 --offline --k-eff 680   # dry run
python -m kinova_dual_arm_dmp_insertion.insert_ffi insert "$W" --arm "$R" --slow 3 --offline --k-eff 680
```

## Parametri

| | valore |
|---|---|
| pezzo / route | `F75_Lt1_s9014` (L), braccio `c100_five_cut800`, scala 0,833, 50 g, apertura 100 mm, `--slow 3` |
| codice | commit `c80dd78`, branch `codex/feedforward-i-20261023`, pulito (codice identico a `1c25bd7`); `local.yaml` copiato qui |
| bracci (T1) | cart_k 1500 1500 1500 20 20 20, cart_d 60 60 60 7 7 7, friction e kalman accesi, 1 kHz |
| k_eff | 680 N/m per mano (`--k-eff`), sia per il piano offline sia per il nominale |
| piano SOCP offline | 263/263 allocazioni ammissibili (route 3,5–5,6 N per mano), `..._offline_profile_20261023_182755.csv` |
| presa iniziale | I sugli ATI: K_I 1 mm/(N s), P 0, 2 mm/s, max 15 mm; forza misurata alla presa 5,63 N per mano |
| nominale | (target − 5,63 N) / 680 per mano, relativo alla presa |
| trim I | `--i-ki` 0,0016 m/(N s) (default) sull'errore della somma; ±3 mm per mano |
| limiti | 2 mm/s per mano (nominale + trim), ±12 mm dalla presa, dt max 100 ms |
| twist / canali | no (`wrench_channels: none`) |
| pezzo | dato a mano alla posa di presa |

Bag: `/home/nvidia/lorenzo/bags/sessione_20261023_182945` sul PC robot.
Prima della prova, alle 18:07 il link ethernet del PC robot (`enP8p1s0`) era caduto: ATI e bracci non
rispondevano e gli `ati_node` partivano senza calibrazione. Ricollegato il cavo e riavviato T1, la prova è regolare
(calibrazione ATI caricata, unità N/Nm nel meta).

## Esito

**Entrata:** `seat_found`, nessun JAM, drop 0, errore stimato **+2,5 mm**, forza laterale max 0,95 N, route
completa (22,0 s), slow_seat, release e retreat. Errore di inseguimento max nella route 9,3 mm (sx) / 11,7 mm (dx).

**Stretta nella route** (somma delle due mani, mediana per sesto: chiesta / misurata):

| 1° | 2° | 3° | 4° | 5° | 6° |
|---|---|---|---|---|---|
| 11,0 / 11,1 | 9,9 / 9,9 | 9,3 / 9,2 | 8,5 / 8,7 | 10,0 / 9,8 | 9,2 / 9,0 |

- Errore della somma nella route (senza gli ultimi 2 s): RMS 0,46 N, media −0,01, max 3,0 N.
- Il piano della L cambia di più della Z (risale a ~5,4 N per mano verso il 5° sesto): il nominale lo segue
  (da 0 a −3,2 mm), il trim I resta tra −1,6 e +0,4 mm, mai al limite di posizione. Rate limit nel 13% della route.
- Slow_seat: il target scende a ~3,8 N per mano ma la misurata sale fino a ~6,4 N per mano (errore fino a −5 N
  sulla somma, il trim apre fino a −1,9 mm): in sede il pezzo spinge sui pad. Minimo della stretta 7,4 N su 7,4 chiesti.

**Confronto con le L del 22/10** (`../20261022_F75_Lt1_s9014_twist_ati/`, `main insert --twist`, gain 1 di allora;
cart_k 1000 e damping 5/1 di allora, dedotti dall'inseguimento ~14–18 mm e da GUIDA):

| | 22/10 15:00 | 22/10 15:07 | 22/10 15:12 | **23/10 18:27 FFI** |
|---|---|---|---|---|
| errore stimato in sede | 6,4 mm | 6,5 mm | 5,5 mm | **2,5 mm** |
| forza laterale max | 1,33 N | 0,83 N | 1,42 N | 0,95 N |
| errore stretta in route (RMS) | 0,78 N | 0,70 N | 0,76 N | **0,46 N** |
| oscillazione 0,8–2,5 Hz (somma): RMS, picco | 0,64 N, 0,39 a 1,27 Hz | 0,62 N, 0,36 a 1,55 Hz | 0,68 N, 0,44 a 1,50 Hz | **0,23 N**, 0,17 a 0,96 Hz |
| inseguimento max sx / dx | 13,8 / 17,5 mm | 13,7 / 18,1 mm | 14,2 / 17,4 mm | 9,3 / 11,7 mm |

Il confronto mescola controllore (FFI contro integratore a gain 1, twist), impedenza (1500 contro 1000) e damping
(60/7 contro 5/1): non attribuisce il miglioramento al solo FFI.

Grafico `forze_ffi.png` (`plot_ffi.py`, autonomo: legge questa cartella e `../20261022_F75_Lt1_s9014_twist_ati/`):
forza normale per mano chiesta e misurata, errore, termini nominale / trim / totale, banda 0,8–2,5 Hz e spettro
contro le tre L del 22/10.
