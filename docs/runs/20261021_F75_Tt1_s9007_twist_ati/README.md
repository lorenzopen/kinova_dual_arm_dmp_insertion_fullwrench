# 21/10/2026 - F75_Tt1_s9007 (T), `--twist`, ATI ai polsi

Serie di prove della stessa run, la T `F75_Tt1_s9007` con le rotte `c100_five_cut800`:

```bash
W=F75_Tt1_s9007 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.main insert $W --arm $R --slow 3 --twist
```

Prima giornata con gli **ATI ai polsi** come sorgente del wrench (`wrench_source: ft`), il **full wrench** (`--twist`)
e i margini d'attrito `task_adaptive`. Cella: bracci con `launch/robot.launch.py` (controller di `~/ros2_ws`, home in
impedenza, `cart_k` 1000/20, `urdf/gen3_ati_tool.urdf` con il tool da 150 g), ATI con `ati_node.py` a 200 Hz
(.132 sul sinistro .200, .131 sul destro .107), tool da 100 mm (`pad_x` 0,085), molla libera k 2700 N/m,
scaffale con l'angolo vicino a x 120 mm (`shelf_in_base` x 0,7525). In ogni cartella: CSV del log, `_meta.json` (config
completa, commit, unità e seriali degli ATI, tare, esito), `local.yaml` (calibrazione ATI) e README.

| # | Ora | Esito (Lorenzo) | Forza esterna max | Stretta max per mano | Twist max | Bag sul PC robot |
|---|---|---|---|---|---|---|
| 1 | 20:15 | entrata, la L vicina non si è spostata | 1,1 N | 7,5 N | 1,4° | `sessione_20261021_201607` |
| 2 | 20:28 | inserita, tutti i vicini spostati | 3,4 N | 7,6 N | 1,8° | `sessione_20261021_202912` |
| 3 | 20:34 | inserita di nuovo | 3,0 N | 7,7 N | 1,6° | `sessione_20261021_203458` |

In tutte e tre: route_followed, seat_found, nessun jam, SOCP ok circa 208/209, loop a 100 Hz con tick mediano di circa 10,3 ms.
I tentativi interrotti a mano (Ctrl-C) fra una prova e l'altra sono solo in `logs/` sul PC robot.
