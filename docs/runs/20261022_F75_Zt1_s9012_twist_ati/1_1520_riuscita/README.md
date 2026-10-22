# 22/10/2026 15:20 - F75_Zt1_s9012, Z inserita con --twist e ATI (prima della serie)

```bash
W=F75_Zt1_s9012 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.main insert $W --arm $R --slow 3 --twist
```

Pezzo dato a mano alla posa di presa (non `insert_pick`). Commit `64ae260` (stesso codice di `edc8806`,
cambiano solo i docs), albero pulito; `local.yaml` uguale alle prove della L dello stesso giorno.

**Esito (Lorenzo):** riuscita. Errore di piazzamento: da misurare.
`RESULT`: route_followed True, jammed False, seat_found True, drop 0,0 mm, believed 3,8 mm, SOCP ok 204/204,
side_force_max 2,4 N.

**Numeri dal log:**
- fasi: to_open_pose 25,3 s, approach 1,0 s, squeeze 0,2 s, to_mouth 1,9 s, route 19,1 s, slow_seat 2,0 s,
  release 2,0 s, retreat 5,3 s;
- loop a 100 Hz: tick mediano 10,3 ms durante route e slow_seat, p95 16,7 ms (max 26 ms);
- stretta: il SOCP chiede ~4,4 N per mano, somma misurata mediana 8,5 N (max 12,0); |F| per mano mediana 4,4 N, max 7,4 N;
- twist: offset di rotazione al massimo 2,4°; forza esterna sul pezzo (x, z) mediana 0,2 N, max 6,7 N.

Il bag (`sessione_20261022_152037`, 22 MB) è sul PC robot in `/home/nvidia/lorenzo/bags/sessione_20261022_152037`.
