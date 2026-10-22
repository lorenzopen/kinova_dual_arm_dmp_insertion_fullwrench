# 22/10/2026 15:00 - F75_Lt1_s9014, L inserita con --twist e ATI

```bash
W=F75_Lt1_s9014 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.main insert $W --arm $R --slow 3 --twist
```

Pezzo dato a mano alla posa di presa (non `insert_pick`). Commit `edc8806`, albero pulito
(config completa nel `_meta.json`, calibrazione ATI in `local.yaml`, uguale alle prove della S dello stesso giorno).

**Esito (Lorenzo):** riuscita. Errore di piazzamento: da misurare.
`RESULT`: route_followed True, jammed False, seat_found True, drop 0,0 mm, believed 6,4 mm, SOCP ok 234/235,
side_force_max 1,3 N.

**Numeri dal log:**
- fasi: to_open_pose 25,3 s, approach 1,0 s, squeeze 0,2 s, to_mouth 2,0 s, route 22,0 s, slow_seat 2,1 s,
  release 2,0 s, retreat 5,3 s;
- loop a 100 Hz: tick mediano 10,3 ms durante route e slow_seat, p95 16,4 ms (max 27 ms);
- stretta: il SOCP chiede ~4,8 N per mano, somma misurata mediana 9,7 N (max 14,4); |F| per mano mediana 4,9 N, max 7,4 N;
- twist: offset di rotazione al massimo 2,5°; forza esterna sul pezzo (x, z) mediana 0,2 N, max 4,6 N.

Il bag (`sessione_20261022_150020`, 41 MB) è sul PC robot in `/home/nvidia/lorenzo/bags/sessione_20261022_150020`.
