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
- `20261021_2015_F75_Tt1_s9007_twist_ati_entrata/`: la T entrata con `--twist` e gli ATI ai polsi come
  sorgente del wrench (prima prova così); la L vicina non si è spostata.
- `20261021_2028_F75_Tt1_s9007_twist_ati_riuscito/`: la stessa, rifatta: la T inserita e tutti i vicini
  spostati.

Le bag sono troppo grandi per git. Dal tuo PC:

```bash
rsync -av --progress nvidia@<ip-pc-robot>:/home/nvidia/lorenzo/home/lorenzo/bags/sessione_1016_1211 nvidia@<ip-pc-robot>:/home/nvidia/lorenzo/home/lorenzo/bags/sessione_1016_1335 ./bags_robot/
```
