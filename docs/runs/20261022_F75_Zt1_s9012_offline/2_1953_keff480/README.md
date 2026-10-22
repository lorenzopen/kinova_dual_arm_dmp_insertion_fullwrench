# 22/10/2026 19:53 - F75_Zt1_s9012, offline con --k-eff 480 (seconda della serie)

```bash
W=F75_Zt1_s9012 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.insert_open insert $W --arm $R --slow 3 --offline --k-eff 480
```

Come la 1 (`../1_1932_keff730/`) ma `k_eff` 480 N/m invece di 730: la rigidezza del contatto misurata
(molla + spugna + gomma + polistirolo) è ≈ 950 N/m, non i 2700 della molla libera, e in serie al braccio
(1000) fa ≈ 490. Commit `464c030`, `cart_d` 60/7, `local.yaml` uguale. Pezzo dato a mano.

**Esito:** entrata. `RESULT`: route_followed True, jammed False, seat_found True, drop 0,0 mm, believed 2,3 mm,
side_force_max 1,7 N.

**Stretta nella route** (somma delle due mani, mediana per quinto: chiesta / misurata):

| | 1° | 2° | 3° | 4° | 5° | misurata/chiesta (mediana) |
|---|---|---|---|---|---|---|
| k_eff 480 (questa) | 9,0 / 9,0 | 9,0 / 9,2 | 8,5 / 7,6 | 8,0 / 6,4 | 7,2 / 5,2 | **0,88** |
| k_eff 730 (la 1) | 8,9 / 5,4 | 8,8 / 5,3 | 8,3 / 5,5 | 8,0 / 5,2 | 7,2 / 4,9 | 0,62 |

- Nella prima metà della route la forza è **giusta** (k_eff 480 corretto).
- Nella seconda metà cala fino al 72% della chiesta: senza feedback niente corregge la deriva (il pezzo ruota
  lungo la rotta, il braccio insegue con errori diversi, l'attrito). Per questo la std di Fz è 0,8 N contro 0,4.
- Nessuna sinusoide: picco 1–3 Hz 0,07 N per mano (anello chiuso gain 1: 0,24 N).

Grafico `forze_k480_vs_altre.png`: Fz misurata e chiesta per mano; spettro contro la 1 (k_eff 730) e le run in
anello chiuso delle 18:43 (gain 1) e 18:27 (gain 0,1). Bag (10 MB, parte da to_mouth):
`/home/nvidia/lorenzo/bags/sessione_20261022_195542`.
