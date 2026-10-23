# 23/10/2026 12:32 - F75_Zt1_s9012 con `insert_open --offline` (SOCP prima della prova, stretta in feedforward)

```bash
# T1: robot.launch di default: cart_k 1500/20, cart_d 60/7, friction e kalman accesi
W=F75_Zt1_s9012 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.insert_open insert $W --arm $R --slow 3 --offline
```

Commit `cd53f2d` (branch `fix-ati-mappa`, pulito), `local.yaml` copiato. Senza `--k-eff`: il profilo è
pianificato con k_eff **964 N/m** = 1/(1/1500 + 1/2700), cioè l'impedenza in serie alla molla nominale.
Profilo SOCP: 232/232 allocazioni ammissibili. Pezzo dato a mano.
Bag: `/home/nvidia/lorenzo/bags/sessione_20261023_123307` sul PC robot.

**Esito: entrata.** `seat_found`, nessun JAM, errore stimato -0,43 mm, forza laterale max 1,1 N,
errore di inseguimento max nella route 9,1 mm (sinistro) / 10,6 mm (destro).

**Stretta nella route** (somma delle due mani, mediana per sesto: chiesta / misurata):

| | 1° | 2° | 3° | 4° | 5° | 6° |
|---|---|---|---|---|---|---|
| **offline 12:32, k_eff 964** | 9,0 / 4,7 | 8,8 / 4,6 | 8,6 / 4,8 | 8,2 / 4,9 | 8,0 / 3,9 | 7,1 / 2,9 |
| online gain 0,5, 12:18 (cart_d 80) | 9,1 / 9,0 | 9,1 / 9,2 | 8,8 / 8,8 | 8,8 / 8,8 | 8,5 / 8,5 | 7,8 / 7,8 |

- Misurata circa **metà** della chiesta (~2,3 N per mano): il k_eff 964 di default è troppo alto per K 1500
  (il 22/10 alle 20:13 con `--k-eff 580` era il 18% alta; il valore vero stimato allora era ~680).
  Per la prossima offline passare `--k-eff 680`.
- Le due mani sono simmetriche.

**Oscillazione** (stretta totale in route, detrend cubico, passa-banda 0,8-2,5 Hz, `plot_offline.py`):

| | RMS banda | picco spettro |
|---|---|---|
| offline 12:32 | 0,18 N | 0,08 N (nessun picco netto) |
| online gain 0,5, 12:18 | 0,32 N | 0,19 N a 1,36 Hz |

Il picco a 1,2-1,4 Hz c'è solo ad anello chiuso. Ma qui la stretta è circa metà, quindi il confronto
in valore relativo (circa 4% in entrambe) non è ancora pulito: va rifatta a forza giusta (`--k-eff 680`).
Prova a pezzo fermo dello stesso giorno (bag `fermo_20261023_122511`, nessun loop, 47 s): 0,03 N.

Grafico `forze_offline_vs_online.png`: forza normale per mano chiesta e misurata, banda 0,8-2,5 Hz e spettro
contro la `--online` delle 12:18 (CSV non copiato qui: `logs/..._online_20261023_121841.csv` sul PC robot).
