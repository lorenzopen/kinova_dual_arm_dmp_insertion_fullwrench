# Run sul robot vero che hanno inserito il pezzo

- `20261016_1211_F75_Tt1_s9007_riuscito/`: la T inserita, nessun JAM. Con la linea di base del
  wrench usata (`wrench_20261016_112659.csv`), il giro a vuoto da cui viene e il cell.yaml.
- `20261016_1333_F75_Zt1_s9012_mezzo_successo/`: la Z entrata, pila mezza caduta. Forze NON
  corrette (base della T attiva, percorso lontano). Accanto, la base della Z registrata dopo
  (`wrench_20261016_134246_Z_senza_contatto.csv`) per correggerle offline.
- `20261019_1924_F75_Lt1_s9014_riuscito/`: la L inserita, stima delle coppie con baseline.
- `20261020_1129_T_s7039m4_twist_riuscito/`: la T barra in alto (colonne 2-5), primo
  inserimento con twist (stima delle coppie, prima degli ATI). I profili `--config` citati nei
  README sono copiati nella cartella.
- `20261021_F75_Tt1_s9007_twist_ati/`: la T con `--twist` e gli ATI ai polsi come sorgente del wrench, tre
  prove: 1 entrata con la L ferma, 2 e 3 inserite con i vicini spostati. Tabella nel README della cartella.
- `20261022_S_s7073m3_twist_ati/`: la S (`c100_narrow75`) con `--twist` e gli ATI, pezzo dato a mano, due
  prove entrambe entrate (12:45, 13:48). Tabella nel README della cartella.
- `20261022_F75_Lt1_s9014_twist_ati/`: la L con `--twist` e gli ATI, pezzo dato a mano, tre prove
  tutte riuscite (15:00, 15:07, 15:12). Tabella nel README della cartella.
- `20261022_F75_Zt1_s9012_twist_ati/`: la Z con `--twist` e gli ATI, pezzo dato a mano, due prove
  entrambe riuscite (15:20, 15:28). Tabella nel README della cartella.
- `20261022_F75_Zt1_s9012_squeeze_gain/`: la Z la sera, la sinusoide a ~1,5 Hz nella stretta: `cart_d` 60/7 e
  `--squeeze-gain` 1 / 0,5 / 0,25 (0,5 entrata, scelta come default). Grafico e `plot_squeeze.py` nella cartella.
  La matematica del loop della stretta e perché oscillava: `docs/squeeze_loop_math.md`.
- `20261022_F75_Zt1_s9012_offline/`: la Z con `insert_open --offline` (SOCP prima della prova, stretta in
  feedforward, ATI non in feedback), k_eff 730 e 480: entrambe entrate, nessuna sinusoide; con 480 la stretta è
  giusta nella prima metà della route e poi cala. Tabella nel README della cartella.

Le bag sono troppo grandi per git. Dal tuo PC:

```bash
rsync -av --progress nvidia@<ip-pc-robot>:/home/nvidia/lorenzo/home/lorenzo/bags/sessione_1016_1211 nvidia@<ip-pc-robot>:/home/nvidia/lorenzo/home/lorenzo/bags/sessione_1016_1335 ./bags_robot/
```
