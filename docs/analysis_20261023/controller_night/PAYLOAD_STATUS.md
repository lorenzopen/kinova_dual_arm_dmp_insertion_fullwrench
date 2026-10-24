# Pacchetto locale del codice e delle simulazioni

Creato e verificato 2026-10-23T19:47:40.793451+00:00.

- [ZIP scientifico, 267.5 MB](<deliveries/controller_night_20261024_0344.zip>)
- [Manifest SHA-256 per file e ZIP](<deliveries/controller_night_20261024_0344_manifest.json>)
- [Esito della verifica completa](deliveries/controller_night_20261024_0344_verification.json)
- [Istruzioni di verifica senza esecuzioni](PAYLOAD_README.md)

Il pacchetto contiene 383 file, 316,904,324 byte non compressi:
61 tracce NPZ della campagna, 10 tabelle
CSV e 48 sorgenti Python, oltre a figure, disegni
congelati, metriche, report, log dei fallimenti e riferimenti precedenti citati.
Include una copia dello ZIP baseline protetto con i suoi 431
elementi originari, conservato anche nella posizione originale.

SHA-256 ZIP: `4a9f08debd4012a6775cbb3a165163ef69ec6825d4e3510f907a75f1492fda4f`.

Ogni elemento è stato riletto dal nuovo ZIP e verificato per contenuto, CRC,
dimensione e SHA-256; confrontati anche i file immutabili nel workspace.
STATE/NIGHT_LOG/RESULTS/preservation_verification inclusi sono istantanee
operative: il loro aggiornamento successivo non modifica l'archivio già creato.
Tutti gli altri artifact congelati conservano anche i manifest originari.

NVIDIA_RUNS non è stato copiato, cancellato, spostato o modificato. I765file
originali restano nei percorsi esistenti, con SHA verificati separatamente;
i CSV/bag e le dipendenze runtime sono ancora necessari per rieseguire gli studi.
Questo è un pacchetto scientifico locale, non una release o un controller robot.
I checkpoint precedenti e le vecchie analisi non sono stati riscritti.

Le conclusioni restano quelle di [THESIS_SYNTHESIS.md](THESIS_SYNTHESIS.md):
nessun nuovo controllore validato, effetto reale di cart_d non discriminato,
protocollo D-only con FFI invariata pronto e non eseguito. Non rilanciare il
builder esclusivo per una verifica: usare il verificatore in sola lettura.
