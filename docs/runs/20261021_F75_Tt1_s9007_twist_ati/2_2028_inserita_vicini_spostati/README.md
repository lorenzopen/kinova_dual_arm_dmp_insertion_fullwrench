# 21/10/2026 20:28 - F75_Tt1_s9007, T inserita con twist e ATI ai polsi, vicini spostati

```bash
W=F75_Tt1_s9007 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.main insert $W --arm $R --slow 3 --twist
```

Stessa cella e stesso codice della prova delle 20:15 (`../1_2015_entrata_L_ferma/`):
ATI ai polsi come sorgente del wrench, `--twist`, margini d'attrito `task_adaptive`, commit `6ae471a`
(config completa nel `_meta.json`, calibrazione ATI in `local.yaml`).

**Esito (Lorenzo):** la T è entrata e questa volta **ha spostato tutti i vicini** (alle 20:15 la L era rimasta ferma).
Errore di piazzamento: da misurare.
`RESULT`: route_followed True, jammed False, seat_found True, drop 0,0 mm, believed 2,2 mm, SOCP ok 209/210,
side_force_max 1,4 N.

**Numeri dal log:**
- fasi: to_open_pose 16,8 s, approach 0,9 s, squeeze 0,7 s, to_mouth 1,8 s, route 18,9 s, slow_seat 2,5 s,
  release 2,0 s, retreat 5,3 s;
- loop a 100 Hz: tick mediano 10,3 ms durante route e slow_seat, p95 16 ms (max 92 ms);
- stretta: il SOCP chiede ~4,8 N per mano, somma misurata mediana 9,6 N (max 15,0); |F| per mano mediana 4,9 N, max 7,6 N;
- twist: offset di rotazione al massimo 1,8°; forza esterna sul pezzo (x, z) mediana 0,3 N, max 3,4 N
  (alle 20:15 il massimo era 1,1 N): la spinta sui vicini.

Il bag (`sessione_20261021_202912`, 19 MB, con `ft_data_avg` a 200 Hz) non è in git. È sul PC robot in
`/home/nvidia/lorenzo/bags/sessione_20261021_202912`.
