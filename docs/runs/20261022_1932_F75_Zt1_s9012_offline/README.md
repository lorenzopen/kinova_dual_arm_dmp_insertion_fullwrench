# 22/10/2026 19:32 - F75_Zt1_s9012, stretta in feedforward con SOCP offline (come il paper)

```bash
W=F75_Zt1_s9012 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.insert_open insert $W --arm $R --slow 3 --offline
```

Pezzo dato a mano. Commit `6b11e45`, `cart_d` 60/7 nel launch, `local.yaml` uguale alle run della sera.
Il SOCP è risolto **prima** della prova lungo tutta la sequenza (`..._offline_profile_20261022_193304.csv`,
1157 target: fase, tempo nella fase, forza per mano); durante la prova la stretta è `ds = f*/k_eff` con
`k_eff` 730 N/m e **gli ATI sono solo registrati**, non in feedback. Niente twist (è un loop sui momenti).

**Esito:** entrata. `RESULT`: route_followed True, jammed False, seat_found True, drop 0,0 mm, believed 0,9 mm,
side_force_max 3,4 N. Fasi: to_open_pose 25,2 s, approach 0,8 s, squeeze 1,0 s, to_mouth 2,0 s, route 19,1 s,
slow_seat 2,0 s, release 2,0 s, retreat 5,3 s.

**Confronto nella route** (Fz dell'ATI = asse di spinta, per mano; bag e log):

| run | stretta | picco 1–3 Hz | std Fz | misurata / chiesta |
|---|---|---|---|---|
| **19:32 offline** (feedforward, no ATI) | `ds = f*/730` | **0,06 N** (nessuna sinusoide) | 0,42 N | **0,62** |
| 18:43 anello chiuso, gain 1 | integratore sull'ATI | 0,24 N a 1,36 Hz | 0,42 N | 0,99 |
| 18:27 anello chiuso, gain 0,1 | integratore sull'ATI | 0,06 N | 0,28 N | 1,00 |

- **Nessuna sinusoide**: senza feedback non c'è il ciclo limite (conferma `docs/squeeze_loop_math.md`).
- **La forza è sbagliata del 38%**: misurata 5,4 N in tutto contro 8,4 N chiesti (mediane nella route).
  Il `k_eff` che la renderebbe giusta è **≈ 460 N/m**, non 730: la rigidezza vera braccio + molla + spugna
  + pezzo è più bassa del modello (e conta la larghezza vera del pezzo). Prossima prova: `--k-eff 460`.
- La variazione (std 0,42 N) non è oscillazione: è che la stretta segue il profilo e la geometria
  invece del target, con il gradino a inizio route (chiesta 3,6 → 4,5 N, misurata 3,7 → 2,7 N).

Grafico: `forze_offline_vs_anello_chiuso.png` (Fz misurata e chiesta per mano; spettro contro 18:43 e 18:27).
Bag (11 MB, parte da to_mouth): `/home/nvidia/lorenzo/bags/sessione_20261022_193455` sul PC robot.
