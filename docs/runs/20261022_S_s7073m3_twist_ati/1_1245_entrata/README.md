# 22/10/2026 12:45 - S_s7073m3, S inserita con --twist e ATI (prima della serie)

```bash
W=S_s7073m3 R=c100_narrow75
python -m kinova_dual_arm_dmp_insertion.main insert $W --arm $R --slow 3 --twist
```

Pezzo dato a mano alla posa di presa (non `insert_pick`). Commit `592ea49`, albero pulito
(config completa nel `_meta.json`, calibrazione ATI in `local.yaml`).

**Esito (Lorenzo):** entrata. Errore di piazzamento: da misurare.
`RESULT`: route_followed True, jammed False, seat_found True, drop 0,0 mm, believed 1,8 mm, SOCP ok 307/308,
side_force_max 4,7 N.

**Numeri dal log:**
- fasi: to_open_pose 22,0 s, approach 0,9 s, squeeze 0,2 s, to_mouth 16,8 s, route 15,4 s, slow_seat 2,0 s,
  release 2,0 s, retreat 5,3 s;
- loop a 100 Hz: tick mediano 10,3 ms durante route e slow_seat, p95 16,6 ms (max 26 ms);
- stretta: il SOCP chiede ~3,9 N per mano, somma misurata mediana 7,9 N (max 15,6); |F| per mano mediana 4,0 N, max 8,7 N;
- twist: offset di rotazione al massimo 1,4°; forza esterna sul pezzo (x, z) mediana 0,2 N, max 6,2 N.

Il bag (`sessione_20261022_124620`, 26 MB) è sul PC robot in `/home/nvidia/lorenzo/bags/sessione_20261022_124620`.
