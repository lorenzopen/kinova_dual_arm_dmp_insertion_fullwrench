# Aumento del margine per la manipolazione in aria, 20 ottobre 2026

Su richiesta di Lorenzo, aumentato il termine fisso della distanza adattiva
nel profilo operativo: **`margin_floor_N: 0.5 -> 1.0` N**. Rimangono massa
50 g, `margin_task_scale: 0.25`, coefficiente nominale mu=1 e piena apertura
del cono. La legge attiva e' `rho_task=0.25*norm(F_task)+1.0 N`.

Il valore e' a livello del task: non e' una forza normale di 1 N per punto
o per mano. Rimane distribuito ai punti dalla geometria e il carico continua
a modificare il margine durante la traiettoria.

Il valore precedente di 2.62 N/mano era la mediana sull'intero trasporto della
T; **nel tratto iniziale in aria** (`to_mouth`, prima dell'ingresso) il target
era circa 1.94 N/mano. Per questo sono stati distinti i valori per fase.

| Termine fisso rho_task | Prima dell'ingresso, mediana/mano | Durante route, mediana/mano | Mediana complessiva/mano | Picco/mano |
|---|---:|---:|---:|---:|
| 0.50 N (precedente) | 1.94 N | 3.18 N | 2.617 N | 3.62 N |
| 0.75 N (prova intermedia) | 2.74 N | 3.91 N | 3.390 N | 4.31 N |
| **1.00 N (attivo)** | **3.49 N** | **4.63 N** | **4.177 N** | **5.06 N** |

Ogni prova T ha 104/104 allocazioni valide. Scelto 1 N per portare la presa
prevista in aria sopra 3 N/mano; il valore intermedio dava ancora 2.74 N/mano.
Il parametro resta una taratura iniziale da verificare sul sistema reale.
Il peso da solo (0.4905 N) non dimostra che 3 N/mano siano insufficienti:
l'aumento serve come riserva contro deviazioni delle forze e contatti imperfetti.

Verificato anche il profilo attivo, letto direttamente dal config, su tre route:

| Route | Mediana complessiva/mano | SOCP accettati |
|---|---:|---:|
| I | 3.722 N | 101/101 |
| T | 4.177 N | 104/104 |
| Z | 3.793 N | 105/105 |

Totale: **310/310**, controllo esplicito dei residui incluso; tutte le sequenze
trovano l'appoggio nel backend semplificato. Il dry run non modella lo slip e
non dimostra la stabilita' reale. Il pickup iniziale resta 6 N/mano. Non sono
stati cambiati guadagni, frequenze, geometria, feedback o soglie di appoggio.

Cartelle:

- `floor075`: prova intermedia sulla T.
- `floor100`: prima prova sulla T con termine fisso 1 N.
- `active_floor100`: verifica finale I/T/Z con il config operativo aggiornato.
- `sweep.csv`, `sweep.png`: confronto della T, includendo il precedente risultato
  a 0.5 N da `../signed_friction_margin_50g_20261020`.

Anche gli esempi che generano i profili passano ora 1 N. Il costruttore standalone
del solver conserva 0.5 N come riferimento del paper, mentre il controllore e
il config operativo usano 1 N. I risultati delle prove precedenti sono conservati.

```powershell
python -m kinova_dual_arm_dmp_insertion.compare_friction_margin --modes task_adaptive --output docs/experiments/margin_floor_50g_20261020/active_floor100
```
