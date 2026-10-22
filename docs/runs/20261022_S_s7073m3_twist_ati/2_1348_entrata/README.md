# 22/10/2026 13:48 - S_s7073m3, S inserita di nuovo (seconda della serie)

```bash
W=S_s7073m3 R=c100_narrow75
python -m kinova_dual_arm_dmp_insertion.main insert $W --arm $R --slow 3 --twist
```

Stessa cella e stesso codice della 1 (`../1_1245_entrata/`), pezzo dato a mano. Commit `26708ce`, albero pulito
(config completa nel `_meta.json`, calibrazione ATI in `local.yaml`, uguale alla 1).

**Esito (Lorenzo):** entrata di nuovo. Errore di piazzamento: da misurare.
`RESULT`: route_followed True, jammed False, seat_found True, drop 4,9 mm, believed -0,5 mm, SOCP ok 309/309,
side_force_max 4,5 N. A differenza della 1 c'è la fase `seat_by_feel` (0,2 s): la sede trovata al tatto, con
4,9 mm di discesa finale.

**Numeri dal log:**
- fasi: to_open_pose 25,2 s, approach 0,9 s, squeeze 1,0 s, to_mouth 16,8 s, route 15,4 s, slow_seat 2,0 s,
  seat_by_feel 0,2 s, release 2,0 s, retreat 5,3 s;
- loop a 100 Hz: tick mediano 10,3 ms durante route e slow_seat, p95 16,6 ms (max 28 ms);
- stretta: il SOCP chiede ~3,9 N per mano, somma misurata mediana 8,0 N (max 15,6); |F| per mano mediana 4,1 N, max 8,0 N;
- twist: offset di rotazione al massimo 2,3°; forza esterna sul pezzo (x, z) mediana 0,2 N, max 5,6 N.

Il bag (`sessione_20261022_135045`, 26 MB) è sul PC robot in `/home/nvidia/lorenzo/bags/sessione_20261022_135045`.
