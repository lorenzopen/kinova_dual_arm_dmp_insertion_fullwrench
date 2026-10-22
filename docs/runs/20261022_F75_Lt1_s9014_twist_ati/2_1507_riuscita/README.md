# 22/10/2026 15:07 - F75_Lt1_s9014, L inserita di nuovo (seconda della serie)

```bash
W=F75_Lt1_s9014 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.main insert $W --arm $R --slow 3 --twist
```

Stessa cella e stesso codice della 1 (`../1_1500_riuscita/`), pezzo dato a mano. Commit `5cd89c3`
(stesso codice di `edc8806`, cambiano solo i docs), albero pulito; `local.yaml` uguale alla 1.

**Esito (Lorenzo):** completata. Errore di piazzamento: da misurare.
`RESULT`: route_followed True, jammed False, seat_found True, drop 0,0 mm, believed 6,5 mm, SOCP ok 235/236,
side_force_max 0,8 N.

**Numeri dal log:**
- fasi: to_open_pose 25,2 s, approach 0,9 s, squeeze 0,3 s, to_mouth 1,9 s, route 22,0 s, slow_seat 2,1 s,
  release 2,0 s, retreat 5,3 s;
- loop a 100 Hz: tick mediano 10,3 ms durante route e slow_seat, p95 16,6 ms (max 31 ms);
- stretta: il SOCP chiede ~4,9 N per mano, somma misurata mediana 9,8 N (max 14,5); |F| per mano mediana 4,9 N, max 7,4 N;
- twist: offset di rotazione al massimo 2,8°; forza esterna sul pezzo (x, z) mediana 0,2 N, max 4,3 N.

Il bag (`sessione_20261022_150839`, 18 MB) è sul PC robot in `/home/nvidia/lorenzo/bags/sessione_20261022_150839`.
