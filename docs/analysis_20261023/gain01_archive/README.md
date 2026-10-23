# Gain 0.1: verifica del CSV originale nell'archivio E:

Sorgente letta senza modifiche: `E:/TESI/NVIDIA_LOG_RUNS`.
Run cercata: `F75_Zt1_s9012_c100_five_cut800_20261022_182705.csv`.
Il meta conferma `main insert`, `--twist`, `--slow 3`, `--squeeze-gain 0.1`.
Risultato: **Ctrl-C in route**, dopo 16.77 s. Non è una prova completata.

| Misura | Gain 0.1 |
|---|---:|
| Mediana forza normale totale nella route | 9.00 N |
| Mediana rapporto misurata/richiesta | 1.010 |
| Errore RMS totale, intera parte di route registrata | 0.453 N |
| Minimo normale totale | 7.16 N |
| Ultimo secondo: misurata / richiesta | 7.82 / 8.20 N |
| Picco spettrale comune, somma delle mani | 0.125 N |
| RMS banda 0.8–2.5 Hz, finestra comune | 0.173 N |

Confronto: gain 1 delle 18:43 ha picco 0.462 N e RMS di banda 0.527 N sulla
**stessa finestra**. Quindi il picco è circa 3.7 volte più piccolo e l'RMS di
banda circa tre volte più piccolo a gain 0.1. La banda non è zero, ma non c'è il
picco dominante a 1.37 Hz visibile a gain 1. Questi valori non sono peak-to-peak.
La precedente stima 0.06 N **per mano** è coerente come ordine di grandezza.

Il gain 0.01 delle 18:15 completa la sequenza e oscilla meno, ma conserva una
forza mediamente superiore al target (rapporto mediano circa 1.20): abbassare il
gain senza guardare il tracking può lasciare il precarico iniziale troppo a lungo.

Il gain 0.1 è un candidato ragionevole come **baseline storica da ripetere completa**.
Non è ancora validato l'ultimo tratto: le altre due run registrano 19.08 s di route,
quindi mancano circa 2.3 s e le fasi successive rispetto alla sequenza nominale.
Ctrl-C non dimostra né un fallimento né un successo fisico dell'inserimento.
I parametri K/D reali non sono registrati in questi meta; non trasferire questo
risultato a un'impedenza diversa considerandolo una taratura verificata.
Il valore 0.1 è il gain del vecchio integratore, non `--i-ki` del nuovo FFI.

## Plot e metodo

- `gain01_sintesi.png`: normale di ciascuna mano e confronto spettrale 0.1/1.
- `gain01_forze.png` / `.pdf`: totale, per mano, errore, tangenziali, correzione ds e ultimo tratto.
- `confronto_gain.png` / `.pdf`: gain 0.01, 0.1 e 1.
- `metrics.json`: comandi, esiti, hash dei CSV e valori ricalcolati.

Non viene applicata una nuova media ai plot di tracking. Forze normali verificate
con la proiezione ATI sugli assi misurati delle flange: scarto massimo dalla somma
registrata circa 0.013 N (compatibile con l'arrotondamento del CSV).
Spettri: timestamp reali, ricampionamento a 100 Hz, finestra comune da 1.00 a
16.27 s di route, detrend cubico e Hann normalizzata in ampiezza. RMS di banda:
Butterworth ordine 2 avanti/indietro, bordi di 1.5 s esclusi. Il filtraggio di banda
è soltanto una metrica offline: non è stato usato dal robot e non altera i plot ATI.

Riproduzione dalla root del repository:

```powershell
python docs/analysis_20261023/plot_gain01_external.py --source E:/TESI/NVIDIA_LOG_RUNS
```

Nessuna modifica ai controller, ai dati sorgente o ai parametri hardware.
