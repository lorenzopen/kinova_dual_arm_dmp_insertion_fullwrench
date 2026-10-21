# 20/10/2026 11:29 - T_s7039m4 (T normale, colonne 2-5), primo inserimento con twist

```bash
source config/session_varieta.env   # W=T_s7039m4 R=c100_narrow75
python -m kinova_dual_arm_dmp_insertion.main insert "$W" --arm "$R" --config "$CFG" --slow 3
```

- **Esito: inserita, ma non fluidissima** (Lorenzo). Da ripetere per migliorare.
- Configurazione: `run_varieta_T_s7039m4_k1000.yaml` (la copia è qui), creato da
  ESEMPI_VARIETA_COLONNE_2_5.md e poi corretto a mano: `full_wrench: true`,
  `allow_hand_wrench: true`, `full_wrench_channels: twist`, `jam_min: 8.0`.
  Rigidezza 1000/20/20, `socp` r_s 0.30, w_force 0, `f_floor` non impostato = 1.0 N (default del codice).
  Tool `pad_x` 0.055 + `pad_r` 0.015, `shelf_in_base` x 0.7475.
- `wrench_baseline`: `baseline_varieta_T_s7039m4_k1000_20261020_105315.csv`, la sua (copia qui).
- Momenti a vuoto prima della prova (`verifica_momenti_...csv`): std 0.01 Nm su tz,
  0.02 su tx/ty, deriva ≤ 0.002 Nm. Segno e guadagno del twist NON verificati.
- Nessun JAM, 172 s; to_mouth 16.9 s, route 16.9 s, slow_seat 2.8 s, seat_by_feel 0 s.
- Da `analyze_run` (mediane, to_mouth / route / slow_seat):
  - stretta: SOCP 1.7 / 2.1 / 1.7 N; misurata L 2.6 / 2.1 / 1.5, R 2.4 / 3.2 / 2.3 N
    (ieri con la L, senza twist: 4.6-6.2 N).
  - verticale: L porta 2.6-2.7 N, R spinge in giù (-0.6/-1.3 N): il pezzo pesa 1.42 N,
    le due mani si contrastano invece di dividersi il peso.
  - attrito: nel 68% dei tick la forza misurata richiede più di mu_eff 1.0
    (p95 fino a 3.7 a sinistra sulla route): stretta troppo bassa per quel carico
    verticale, probabile causa della poca fluidità / scivolamento.
  - jam: residuo max 7.1 N in to_mouth contro limite 8.0: margine stretto.
- Placement misurato a mano: ___ mm (da scrivere).

Il tentativo delle 11:22 (`logs/T_s7039m4_c100_narrow75_20261020_112253.csv`, 214 s) si è
fermato in `route`, senza JAM; è solo in `logs/`.

File:
- `T_s7039m4_c100_narrow75_20261020_112919.csv`: l'insert, un tick ogni 10 ms; `..._socp.png`.
- `baseline_varieta_...csv`: linea di base del wrench; `follow2_T_s7039m4_20261020_111027.csv`:
  il giro a vuoto della baseline, lato comandi.
- `verifica_momenti_T_s7039m4_20261020_110018.csv`: il test dei momenti a bracci fermi.
- Bag: la prova NON è coperta per intero. `~/lorenzo/bags/varieta_col2_5_20261020_112247`
  finisce alle 11:28:32 (prima dell'insert), `..._113126` parte alle 11:31:26 (solo l'ultimo
  minuto: seat, release, retreat).
