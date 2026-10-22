# 22/10/2026 sera - F75_Zt1_s9012: la sinusoide nella stretta, `cart_d` e `--squeeze-gain`

```bash
W=F75_Zt1_s9012 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.main insert $W --arm $R --slow 3 --twist [--squeeze-gain G]
```

Pezzo dato a mano. Nelle run del pomeriggio la stretta misurata (somma delle due mani) oscillava a
~1,5 Hz, ±0,7 N attorno ai ~9 N chiesti, in fase sulle due mani. Il target del SOCP non oscilla; la
separazione tra le mani **comandata** sì, più di quella misurata: è un ciclo limite del loop della stretta
(`controller.py`, `_control`: integratore `ds += GAIN·(2·target − f)` ogni 20 ms, `GAIN = 1/(4·1500)` m/N)
con il braccio in impedenza, non un disturbo esterno.

| run | `cart_d` (robot.launch) | `--squeeze-gain` | esito | picco | ampiezza picco | rms stretta | stretta min |
|---|---|---|---|---|---|---|---|
| 15:28 (`../20261022_F75_Zt1_s9012_twist_ati/2_1528_riuscita/`) | 5/1 | 1 | entrata | 1,53 Hz | 0,44 N | 0,68 N | 6,2 N |
| 17:22 `..._172232` | 60/7 | 1 | Ctrl-C in route | 1,23 Hz | 0,63 N | 0,92 N | 3,5 N |
| 17:34 `..._173458` | 60/7 | 0,5 | **entrata** (drop 0, SOCP 207/207) | 1,24 Hz | **0,23 N** | **0,50 N** | **6,6 N** |
| 17:44 `..._174421` | 60/7 | 0,25 | Ctrl-C in route | 1,18 Hz | 0,18 N | 0,58 N | 3,6 N |

Fase route, primi 17,5 s; picco e ampiezza dallo spettro della stretta senza il trend (finestra di Hann);
`python plot_squeeze.py` rifà `squeeze_gain.png` e stampa questi numeri.

- `cart_d` 5/1 → 60/7 (a gain 1): la sinusoide non sparisce, rallenta (1,5 → 1,2 Hz) e cresce. Il braccio
  più smorzato è più lento a seguire il target e aggiunge ritardo al loop.
- gain 0,5: picco a un terzo, la separazione comandata non oscilla più, la stretta media resta quella chiesta;
  `squeeze` 0,6 s. Nell'ultimo tratto (il pezzo tra i vicini) la stretta cala e viene recuperata.
- gain 0,25: la più liscia nei primi 16 s, ma nell'ultimo tratto la stretta scende a 3,6 N e resta ~4 N:
  il loop recupera troppo piano.

Scelta: `--squeeze-gain` **0,5 di default** (commit `515f088`), `cart_d` 60/7 nel launch (commit `6844e03`).
Da provare: ripetibilità a 0,5; `cart_d:='20 20 20 3 3 3'` con gain 0,5.

`local.yaml` uguale alle run del pomeriggio. Il `_meta.json` non registra `cart_d` (è del launch): vale la tabella.
Bag sul PC robot in `/home/nvidia/lorenzo/bags/`: `sessione_20261022_172309` (17 MB), `sessione_20261022_173528`
(20 MB), `sessione_20261022_174538` (6,3 MB).
