# 23/10/2026 17:43 - F75_Zt1_s9012 con `insert_ffi --offline --k-eff 680`: ENTRATA

Forza nominale dal SOCP offline + piccola correzione I sugli ATI (`docs/FFI_CONTROLLER.md`).

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
| codice | commit `1c25bd7`, branch `codex/feedforward-i-20261023`, pulito; `local.yaml` copiato qui |
| bracci (T1) | cart_k 1500 1500 1500 20 20 20, cart_d 60 60 60 7 7 7, friction e kalman accesi, 1 kHz |
| k_eff | 680 N/m per mano (`--k-eff`), sia per il piano offline sia per il nominale |
| piano SOCP offline | 232/232 allocazioni ammissibili, `..._offline_profile_20261023_174157.csv` |
| presa iniziale | I sugli ATI: K_I 1 mm/(N s), P 0, 2 mm/s, max 15 mm; forza misurata alla presa 5,81 N per mano |
| nominale | (target − 5,81 N) / 680 per mano, relativo alla presa |
| trim I | `--i-ki` 0,0016 m/(N s) (default) sull'errore della somma; ±3 mm per mano |
| limiti | 2 mm/s per mano (nominale + trim), ±12 mm dalla presa, dt max 100 ms |
| twist / canali | no (`wrench_channels: none`) |
| pezzo | dato a mano |

Bag: `/home/nvidia/lorenzo/bags/sessione_20261023_174232` sul PC robot.

## Esito

**Entrata:** `seat_found`, nessun JAM, errore stimato −0,45 mm, forza laterale max 0,88 N,
route completa (19,1 s), slow_seat, release e retreat. Errore di inseguimento max nella route
9,9 mm (sinistro) / 11,0 mm (destro).

**Stretta nella route** (somma delle due mani, mediana per sesto: chiesta / misurata):

| 1° | 2° | 3° | 4° | 5° | 6° |
|---|---|---|---|---|---|
| 9,0 / 9,0 | 8,8 / 8,8 | 8,6 / 8,6 | 8,2 / 8,3 | 8,0 / 7,8 | 7,1 / 7,1 |

- Errore della somma nella route (senza gli ultimi 2 s): RMS 0,42 N, media 0,00, max 2,2 N.
- Nominale da −1,9 a −3,5 mm (apre: il target è sotto la forza di presa); trim I tra −0,6 e +0,8 mm,
  mai al limite di posizione. Rate limit nel 7% della route.
- Nello slow_seat il target sale a ~5,6 N per mano: la misurata segue con ~0,5 s di ritardo
  (errore fino a +4 N, poi rientra); minimo della stretta 7,0 N su 7,2 chiesti.

**Oscillazione** (stretta totale in route, detrend cubico, passa-banda 0,8–2,5 Hz):

| | RMS banda | picco |
|---|---|---|
| **FFI 17:43 (questa)** | 0,24 N | 0,17 N a 1,20 Hz |
| online gain 0,5 (12:18, entrata) | 0,32 N | 0,19 N a 1,36 Hz |
| PI (11:43) | 0,19 N | 0,14 N a 1,85 Hz |
| offline k_eff 680 (12:43, senza feedback) | 0,18 N | 0,08 N |

Grafico `forze_ffi.png` (`plot_ffi.py`, legge i CSV di confronto da `logs/` sul PC robot): forza normale
per mano chiesta e misurata, errore, termini nominale / trim / totale, banda e spettro contro le altre.
