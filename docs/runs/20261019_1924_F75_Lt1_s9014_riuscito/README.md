# 19/10/2026 19:24 - F75_Lt1_s9014, L inserita

```bash
W=F75_Lt1_s9014 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.main insert $W --arm $R --config config/run_F75_Lt1_s9014_k1000.yaml --slow 3
```

- Configurazione: `run_F75_Lt1_s9014_k1000.yaml` (la copia è qui). Rigidezza 1000/20/20,
  `gen3_bare.urdf`, `shelf_in_base` x 0.7475 (60 mm più vicino del 16/10).
- Tool `pad_x` 0.055 + `pad_r` 0.015. `jam_scale: 1.8`, `jam_min: 8.0`. Nuovo SOCP,
  `full_wrench: false`, `r_s` 0.30, **`f_floor: 5.0`** (target mai sotto 5.0 N, al minimo il 65% del tempo).
  Dopo questa prova `f_floor` è stato tolto dal profilo (→ 1.0 N): la copia qui è corretta a mano.
  Le L delle 19:33 e 19:42, con f_floor 1.0, si sono fermate in `route`.
- `wrench_baseline`: `baseline_F75_Lt1_s9014_k1000.csv`, la sua (la copia è qui).
- Nessun JAM; fasi fino a `slow_seat` → `seat_by_feel` → `release` → `retreat`, 113 s.
- Da `analyze_run` (mediane): stretta misurata 4.6-6.2 N (SOCP ne chiede 0.9-3.4);
  residuo jam max 5.6 N sulla route, 2.5 N nel seat (limite min 15.5 N); attrito misurato
  p95 ≤ 0.64 contro mu_eff 1.0, mai oltre.
- Placement misurato a mano: ___ mm (da scrivere).

Gli altri tentativi della sera (I alle 18:05-18:34, tutti in JAM; L alle 19:14, 19:20,
19:33, 19:42, fermi in `route`) sono solo in `logs/`.

File:
- `F75_Lt1_s9014_c100_five_cut800_20261019_192404.csv`: l'insert, un tick ogni 10 ms.
- `..._socp.png`: SOCP contro robot, da `analyze_run`.
- `baseline_F75_Lt1_s9014_k1000.csv`: la linea di base del wrench (giro a vuoto).
- `follow2_F75_Lt1_s9014_20261019_190933.csv`: il giro a vuoto, lato comandi.
- `run_F75_Lt1_s9014_k1000.yaml`: la configurazione usata.
- Bag: nessuna registrata.
