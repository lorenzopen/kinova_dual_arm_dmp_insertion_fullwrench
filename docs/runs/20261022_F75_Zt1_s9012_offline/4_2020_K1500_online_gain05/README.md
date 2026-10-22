# 22/10/2026 20:20 - F75_Zt1_s9012, anello chiuso (`--online`) con K 1500 e gain 0,5 (quarta della serie)

```bash
# T1: robot.launch con cart_k:='1500 1500 1500 20 20 20'
W=F75_Zt1_s9012 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.insert_open insert $W --arm $R --slow 3 --online --squeeze-gain 0.5
```

Stessa impedenza della 3 (`cart_k` 1500; il `_meta.json` non lo registra), ma la stretta in **anello chiuso
sull'ATI** (integratore, gain 0,5) e il SOCP online ogni 100 ms. Senza twist, per confronto con le offline.
Commit `567450f`, `local.yaml` uguale. Pezzo dato a mano.

**Esito:** entrata. `RESULT`: route_followed True, jammed False, seat_found True, drop 0,0 mm, believed 0,4 mm,
SOCP ok 207/207, side_force_max 1,4 N. Fasi: to_open_pose 25,3 s, approach 0,9 s, squeeze 0,2 s, to_mouth 1,9 s,
route 19,1 s, slow_seat 2,0 s, release 2,0 s, retreat 5,3 s.

**Stretta nella route** (somma, mediana per sesto, chiesta / misurata; errore di inseguimento sinistro):
9,1 / 9,0 (4 mm), 9,0 / 9,2 (4), 8,9 / 8,9 (5), 8,8 / 8,8 (7), 8,6 / 8,5 (9), 7,8 / 8,0 (8); minima 6,6 N.

| run | misurata/chiesta | picco 1–3 Hz (per mano, bag) | std Fz nella route |
|---|---|---|---|
| **4: online, K 1500, gain 0,5** | **0,99–1,00** | 0,10 N a 1,2 Hz | **0,35 N** |
| 3: offline, K 1500, k_eff 580 | 1,18 → 1,04 | 0,04 N | 0,7 N |
| 18:43: `main insert`, K 1000, gain 1 | 0,99 | 0,24 N a 1,36 Hz | 0,42 N |

- La migliore finora: segue la forza chiesta lungo tutta la route (anche dove la offline derivava) e la
  sinusoide resta piccola (0,10 N per mano, meno della metà del gain 1).
- Ritardo del braccio lungo l'asse dei pad ancora ~165 ms (come a K 1000), quindi la frequenza resta ~1,2 Hz;
  l'inseguimento in aria è migliore (4–9 mm invece di 7–13).

Grafico `forze_K1500_online_gain05.png`: Fz misurata e chiesta per mano; spettro contro la 3 (offline K 1500) e
la 18:43 (K 1000, gain 1). Bag (12 MB): `/home/nvidia/lorenzo/bags/sessione_20261022_202153` sul PC robot.
