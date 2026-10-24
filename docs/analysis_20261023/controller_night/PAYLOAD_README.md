# Conservazione del codice e delle simulazioni

La consegna locale nella directory `deliveries/` conserva sorgenti, disegni
congelati, metriche, figure, log dei fallimenti e **tutte le tracce NPZ della
campagna**, incluse quelle che i checkpoint del solo codice non duplicavano.
Comprende anche una copia byte-identica dello ZIP baseline protetto e i file
dell'analisi precedente citati nei report, senza modificare gli originali.

Il manifest sidecar elenca ogni file con dimensione e SHA-256 e lo SHA dell'intero
ZIP. La verifica rilegge tutti gli elementi compressi, verificando anche il CRC,
senza estrarli. `STATE.json`, `NIGHT_LOG.md`, `RESULTS.md` e la verifica di
preservazione nel pacchetto sono istantanee operative alla data del manifest;
possono ricevere aggiornamenti successivi nel workspace. Le evidenze congelate
si verificano anche contro i rispettivi manifest di consegna precedenti.

Per verificare il pacchetto nel workspace con il Python già documentato:

```powershell
& 'C:/Users/Lorenzo/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' `
  docs/analysis_20261023/controller_night/verify_campaign_payload.py --compare-workspace
```

Il verificatore usa solo la libreria standard e legge `CURRENT_PAYLOAD.json`.
Si può passare `--manifest` per indicare il sidecar esplicitamente. Non avvia
simulatori, ROS, robot, deploy, Git o rete, non scrive ed estrae alcun file.
Per una verifica spostata su un'altra macchina usare ZIP+manifest insieme e
omettere `--compare-workspace`.

Questo non è un ambiente di esecuzione portatile o una release robot. Le analisi
richiedono ancora dati originali, dipendenze Python e struttura del workspace
documentata nei protocolli. **NVIDIA_RUNS non viene copiato nel nuovo ZIP**:
i suoi CSV e bag rimangono intatti nella posizione originale, con il manifest
SHA-256 dell'archivio. Lo ZIP scientifico aggiuntivo non cambia i risultati né
risolve i limiti dei modelli. I precedenti checkpoint restano conservati nella
loro directory e non vengono eliminati, spostati o riscritti.

Per le conclusioni leggere [THESIS_SYNTHESIS.md](THESIS_SYNTHESIS.md) e
[READING_GUIDE.md](READING_GUIDE.md). Il confronto D-only con FFI invariata è
ancora un protocollo futuro: nessun valore di cart_d o nuovo PID viene promosso
da questo atto di conservazione.
