# 22/10/2026 20:13 - F75_Zt1_s9012, offline con K 1500 e --k-eff 580 (terza della serie)

```bash
# T1: robot.launch con cart_k:='1500 1500 1500 20 20 20' (cart_d come passato da Lorenzo; consigliato 73/7)
W=F75_Zt1_s9012 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.insert_open insert $W --arm $R --slow 3 --offline --k-eff 580
```

Impedenza traslazionale **1500 N/m** (come l'addestramento in PyBullet) invece di 1000. Il `_meta.json` non
registra `cart_k` (è del launch) e `cell.yaml` diceva ancora `trans: 1000` (con `--k-eff` esplicito non conta);
che il braccio fosse a 1500 lo confermano gli errori di inseguimento, scesi a ~0,55-0,6 (1000/1500 = 0,67).
Commit `077112f`, `local.yaml` uguale. `k_eff` 580 = 1/(1/1500 + 1/950), 950 la rigidezza del contatto stimata
con K 1000. Pezzo dato a mano.

**Esito:** fermata con Ctrl-C alla fine della route (17,0 s di route, contatto con i vicini da ~19 s del bag,
forza esterna fino a 4,7 N); dopo il Ctrl-C la stretta scende a ~1,5 N per mano. Non arrivata in sede.

**Inseguimento** (contro la 2, K 1000):

| | 2: K 1000, k_eff 480 | **3: K 1500, k_eff 580** |
|---|---|---|
| errore in aria a fine to_open_pose | 12,4 mm | **6,5 mm** |
| fine approach, separazione misurata − comandata | 27,2 mm | **16,6 mm** |
| errore nella route | 7 → 13 mm | **5 → 9 mm** |

**Stretta nella route** (somma delle due mani, mediana per sesto: chiesta / misurata):

| | 1° | 2° | 3° | 4° | 5° | 6° |
|---|---|---|---|---|---|---|
| **3: K 1500, k_eff 580** | 9,0 / 10,6 | 9,0 / 10,4 | 8,8 / 10,7 | 8,4 / 10,3 | 8,0 / 9,3 | 7,5 / 7,8 |
| 2: K 1000, k_eff 480 | 9,0 / 8,9 | 9,0 / 9,3 | 8,7 / 8,6 | 8,2 / 6,9 | 8,0 / 6,2 | 7,2 / 5,2 |

- Deriva molto più piccola: misurata/chiesta da 1,18 a 1,04 (con K 1000: da 0,99 a 0,72).
- Forza ~18% alta: il `k_eff` vero a K 1500 è ≈ 580 × 1,18 ≈ **680 N/m**. Prossima prova: `--k-eff 680`.
- Nessuna sinusoide: picco 1–3 Hz 0,04 N per mano.

Grafico `forze_K1500_keff580.png`: Fz misurata e chiesta per mano; spettro contro la 2 (k_eff 480) e l'anello
chiuso delle 18:43 (gain 1). Bag: `/home/nvidia/lorenzo/bags/sessione_20261022_201551` sul PC robot.
