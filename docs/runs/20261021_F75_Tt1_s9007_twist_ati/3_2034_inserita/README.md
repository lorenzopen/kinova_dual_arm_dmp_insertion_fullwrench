# 21/10/2026 20:34 - F75_Tt1_s9007, T inserita di nuovo (terza della serie)

```bash
W=F75_Tt1_s9007 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.main insert $W --arm $R --slow 3 --twist
```

Stessa cella e stesso codice della 2 (`../2_2028_inserita_vicini_spostati/`), commit `cb2a541`
(config completa nel `_meta.json`, calibrazione ATI in `local.yaml`).

**Esito (Lorenzo):** inserita di nuovo. Errore di piazzamento: da misurare.
`RESULT`: route_followed True, jammed False, seat_found True, drop 0,0 mm, believed 2,0 mm, SOCP ok 209/210,
side_force_max 1,4 N.

**Numeri dal log:**
- fasi: to_open_pose 16,5 s, approach 0,9 s, squeeze 0,7 s, to_mouth 1,8 s, route 18,9 s, slow_seat 2,5 s,
  release 2,0 s, retreat 5,3 s;
- loop a 100 Hz: tick mediano 10,3 ms durante route e slow_seat, p95 16,5 ms (max 109 ms);
- stretta: il SOCP chiede ~4,9 N per mano, somma misurata mediana 9,7 N (max 15,2); |F| per mano mediana 4,9 N, max 7,7 N;
- twist: offset di rotazione al massimo 1,6°; forza esterna sul pezzo (x, z) mediana 0,4 N, max 3,0 N.

Il bag (`sessione_20261021_203458`, 25 MB) è sul PC robot in `/home/nvidia/lorenzo/bags/sessione_20261021_203458`.
