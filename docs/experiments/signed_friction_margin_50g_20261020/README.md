# T reale da 50 g: dry run aggiornato

Questa e' la prima prova a 50 g, con termine fisso del margine a 0.5 N.
Il profilo attivo e' stato poi alzato a 1.0 N per aumentare la presa in aria:
[confronto della taratura](../margin_floor_50g_20261020/README.md).

Massa comunicata da Lorenzo il 20 ottobre 2026: **0.050 kg per tetromino intero**,
non per singolo cubetto. `config/cell.yaml` ora usa questo valore esplicito,
invece dei circa 0.14468 kg derivati dalla route. Il peso passa da circa
1.419 N a **0.4905 N**. La geometria della traiettoria rimane identica.

Il caricatore delle route gia' esistente scala anche le forze e i momenti
previsti dalla route con il rapporto di massa: e' un'assunzione del modello,
non una misura delle reazioni dei vicini. Non e' stata cambiata questa legge.

Confronto sulla T `F75_Tt1_s9007`, `c100_five_cut800`, stesso margine e stesso
controllore. Forza mediana richiesta per mano:

| Metodo | Modello precedente, 144.68 g | Massa reale, 50 g |
|---|---:|---:|
| Cono nominale, nessun margine | 2.756 N | 1.000 N |
| Distanza fissa 0.5 N/punto | 4.572 N | 3.109 N |
| Distanza adattiva, profilo attivo | 4.651 N | 2.617 N |

Le tre prove a 50 g hanno **104/104 allocazioni accettate** ciascuna. Il minimo
di 1 N nel caso senza margine e' il clamp del target nel controllore, non la
forza teorica minima di Coulomb. Il dry run non modella lo slittamento.

La legge adattiva resta `rho_task=0.25*norm(F_task)+0.5 N`: diminuisce il
contributo del carico, ma **il termine fisso di 0.5 N non diminuisce con la
massa**. Pertanto la forza totale non cala nel rapporto 50/144.68. I parametri
del margine rimangono scelte iniziali riprese dal paper, non una calibrazione
per questo oggetto leggero.

La presa iniziale resta 6 N/mano, fissata dal controllore; non e' determinata
dalla massa o dal nuovo margine. Anche soglie derivate dal peso cambiano:
la rilevazione di appoggio al 60% del peso ora vale **0.2943 N**, inferiore agli
errori osservati nelle stime wrench dei log reali. Il dry run non valida
l'affidabilita' di quella rilevazione sul robot.

File separati dalle prove precedenti: `summary.csv`, `summary.json`,
`*_allocations.csv`, `*_ticks.csv`, `comparison.png`.

```powershell
python -m kinova_dual_arm_dmp_insertion.compare_friction_margin --worlds F75_Tt1_s9007 --output docs/experiments/signed_friction_margin_50g_20261020
```
