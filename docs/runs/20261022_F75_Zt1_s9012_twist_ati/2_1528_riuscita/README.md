# 22/10/2026 15:28 - F75_Zt1_s9012, Z inserita di nuovo (seconda della serie)

```bash
W=F75_Zt1_s9012 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.main insert $W --arm $R --slow 3 --twist
```

Stessa cella e stesso codice della 1 (`../1_1520_riuscita/`), pezzo dato a mano. Commit `9d7a0f7`
(stesso codice di `edc8806`, cambiano solo i docs), albero pulito; `local.yaml` uguale alla 1.

**Esito (Lorenzo):** riuscita. Errore di piazzamento: da misurare.
`RESULT`: route_followed True, jammed False, seat_found True, drop 0,0 mm, believed 4,2 mm, SOCP ok 203/204,
side_force_max 2,4 N.

**Numeri dal log:**
- fasi: to_open_pose 2,0 s (bracci già aperti dal giro prima), attesa del pezzo ~1,5 min, approach 1,0 s,
  squeeze 0,3 s, to_mouth 1,9 s, route 19,1 s, slow_seat 2,0 s, release 2,0 s, retreat 5,3 s;
- loop a 100 Hz: tick mediano 10,3 ms durante route e slow_seat, p95 16,8 ms (max 28 ms);
- stretta: il SOCP chiede ~4,4 N per mano, somma misurata mediana 8,8 N (max 12,7); |F| per mano mediana 4,5 N, max 7,6 N;
- twist: offset di rotazione al massimo 3,6°; forza esterna sul pezzo (x, z) mediana 0,2 N, max 6,5 N.

Il bag (`sessione_20261022_153004`, 12 MB, parte poco prima della route) è sul PC robot in
`/home/nvidia/lorenzo/bags/sessione_20261022_153004`.

Prima di questa, alle 15:26 una prova si è fermata in to_mouth per l'ATI sinistro muto per 340 ms
(`/left_arm/ft_data_avg`); non salvata.
