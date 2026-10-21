# 21/10/2026 20:15 - F75_Tt1_s9007, T entrata con twist e ATI ai polsi

```bash
W=F75_Tt1_s9007 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.main insert $W --arm $R --slow 3 --twist
```

Prima prova riuscita con gli **ATI ai polsi** come sorgente del wrench (`wrench_source: ft`), il **full wrench**
(`--twist`: stretta + twist opposto attorno all'asse dei pad) e i margini d'attrito `task_adaptive`.
Codice al commit `c63cc4a` (in `_meta.json`, con la config completa usata).

**Esito (Lorenzo):** la T è entrata nello slot; la L vicina non è stata spostata. Errore di piazzamento: da misurare.
`RESULT`: route_followed True, jammed False, seat_found True, drop 1,8 mm, SOCP ok 207/208.

**Cella:**
- bracci con il controller di `~/ros2_ws` tramite `launch/robot.launch.py`: home in impedenza, impedenza alla flangia,
  `cart_k` 1000/20, `cart_d` 5/1, `urdf/gen3_ati_tool.urdf` (tool da 150 g); ATI con `ati_node.py`
  (.132 sul sinistro .200, .131 sul destro .107), medie a 200 Hz su `ft_data_avg`;
- tool: 100 mm dalla flangia al fronte delle sfere (`pad_x` 0,085), molla libera k 2700 N/m;
- scaffale: angolo vicino a x 120 mm dalle basi (`shelf_in_base` x 0,7525);
- `local.yaml`: la calibrazione degli ATI di `ft_check --calibrate` usata in questa prova.

**Numeri dal log:**
- loop a 100 Hz regolare: tick mediano 10,0–10,4 ms in tutte le fasi, p95 della route 17 ms (max 101 ms);
- stretta: il SOCP chiede ~4,8 N per mano, somma misurata mediana 9,4 N (max 14,7); |F| per mano mediana 4,8 N, max 7,5 N;
- twist: offset di rotazione al massimo 1,4°; forza esterna sul pezzo (x, z) mediana 0,3 N, max 1,1 N.

Il bag (`sessione_20261021_201607`, 28 MB, con `ft_data_avg` a 200 Hz) non è in git. È sul PC robot in
`/home/nvidia/lorenzo/bags/sessione_20261021_201607`.
