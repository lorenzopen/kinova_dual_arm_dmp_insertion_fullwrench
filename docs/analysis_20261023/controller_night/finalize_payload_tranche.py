"""Record the preserved scientific payload; do not rebuild or simulate."""
import sys
sys.dont_write_bytecode=True
import hashlib
import json
from pathlib import Path

OUT=Path(__file__).resolve().parent
def read(path):return json.loads(path.read_text(encoding='utf-8'))


def main():
    pointer=read(OUT/'CURRENT_PAYLOAD.json');manifest=read(OUT/pointer['manifest'])
    status=read((OUT/pointer['manifest']).with_name('controller_night_20261024_0344_verification.json'))
    assert status['zip_verified'] and manifest['counts']['campaign_npz']==61
    assert manifest['zip_sha256']==pointer['zip_sha256']
    preserved=read(OUT/'preservation_verification.json');assert preserved['archive_identical']
    count=manifest['counts'];size=(OUT/pointer['zip']).stat().st_size
    report=f'''# Pacchetto locale del codice e delle simulazioni

Creato e verificato {manifest['created_utc']}.

- [ZIP scientifico, {size/1e6:.1f} MB](<{pointer['zip']}>)
- [Manifest SHA-256 per file e ZIP](<{pointer['manifest']}>)
- [Esito della verifica completa](deliveries/controller_night_20261024_0344_verification.json)
- [Istruzioni di verifica senza esecuzioni](PAYLOAD_README.md)

Il pacchetto contiene {count['files']} file, {count['bytes']:,} byte non compressi:
{count['campaign_npz']} tracce NPZ della campagna, {count['campaign_csv']} tabelle
CSV e {count['campaign_python']} sorgenti Python, oltre a figure, disegni
congelati, metriche, report, log dei fallimenti e riferimenti precedenti citati.
Include una copia dello ZIP baseline protetto con i suoi {count['protected_baseline_entries']}
elementi originari, conservato anche nella posizione originale.

SHA-256 ZIP: `{manifest['zip_sha256']}`.

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
'''
    (OUT/'PAYLOAD_STATUS.md').write_text(report,encoding='utf-8')
    old=(OUT/'RESULTS.md').read_text(encoding='utf-8')
    intro='''## Codice e tracce numeriche conservati insieme

[Pacchetto scientifico locale e manifest](PAYLOAD_STATUS.md): ZIP verificato di
267,5MB con tutte61tracceNPZ della campagna, tabelle, sorgenti, disegni congelati,
grafici, log dei fallimenti, riferimenti citati e copia byte-identica dello
snapshot baseline.383file,316.904.324byte non compressi; ogni elemento riletto
e verificatoSHA/CRC. I checkpoint del solo codice restano conservati separatamente.

Nessun contenuto NVIDIA_RUNS spostato o riscritto; originali verificati a parte.
Il pacchetto non sostituisce CSV/bag e dipendenze per una riesecuzione, e non
è una release robot. Stato/registro/report operativi sono istantanee al momento
della confezione, mentre i risultati congelati restano confrontabili byte-per-byte.
Conclusioni scientifiche e prioritàD-only invariate, nessun nuovo tuning.

'''
    marker='## Verifica di coerenza e precisazione sul clock del limite software'
    assert marker in old and '## Codice e tracce numeriche conservati insieme' not in old
    (OUT/'RESULTS.md').write_text(old.replace(marker,intro+marker,1),encoding='utf-8')
    state=read(OUT/'STATE.json');state['status']='offline_research_payload_preserved_no_new_controller_validated'
    state['persistent_worker']=None
    state['payload']=pointer
    state['completed'] += [
        'Exclusive scientific payload controller_night_20261024_0344.zip saved:383files316904324uncompressedbytes,267468570ZIPbytes,all61campaign NPZ/10CSV/48Python plus figures/reports/failure logs and referenced old analysis. No NVIDIA content copied or modified.',
        'Payload contains byte-identical copy of protected baseline ZIP431entries; all payload entries independently read/CRC/SHA verified and immutable live files compared. SHA4a9f08debd4012a6775cbb3a165163ef69ec6825d4e3510f907a75f1492fda4f.',
        'Standard-library read-only verify_campaign_payload.py and PAYLOAD_README/STATUS published; operational STATE/NIGHT_LOG/RESULTS/preservation files distinguished as packaging-time snapshots. No simulations/extraction/ROS/network/deploy.',
        'Original preservation passed19:48:14UTC:765files4080005175bytes identical,431baselineZIP entries and332runtime sources unchanged. Previous five delivery manifests and14frozen sources verified before packaging.'
    ]
    state['next']=[
        'Offline research, synthesis, consistency checks and code/trace preservation are complete. Do not rebuild payload or repeat passed grids/audits without changed inputs or a concrete new issue.',
        'No additional tuning or heldout retiming. A new controller/cart_d recommendation still needs actual inner-servo semantics and matched fixed-FFI D-only data; protocol already prepared, no robot execution.',
        'Payload verification is read-only via verify_campaign_payload.py; all61NPZ are covered, older checkpoints remain separate. Changes to four operational snapshot files do not imply immutable science artifacts changed.',
        'At/after cutoff2026-10-24T01:00Z consolidate only, perform final preservation/delivery verification and final operational checkpoint, end continuation. Until then notify only meaningful new evidence/completion/user-required obstacle, not unchanged status.'
    ]
    state['latest_source_checkpoint']='checkpoints/tranche_20261024_0344.zip'
    (OUT/'STATE.json').write_text(json.dumps(state,indent=2)+'\n',encoding='utf-8')
    with (OUT/'NIGHT_LOG.md').open('a',encoding='utf-8') as f:
        f.write('''
## Heartbeat24/10 03:44Singapore — conservazione completa delle tracce numeriche

LettiAGENTS/PROTOCOL/STATE/NIGHT_LOG/RESULTS,steeringD/audit e logQA/marker.
Checkpoint0314 SHAbba11454bc455ca4a1f3ae0b18ae38564adffc1d51f1824e22576ea34b07f761
verificato,nessunPythonvivo;fallbackGet-Process per CIMgiànegato. Un solo worker,
BLAS/OMP/MKL/NUMEXPR1,nopycache. Nessun nuovo fit/simulazione/tuning/ROS/robot.

Verificata copertura conservazione:61NPZ root148.649.220byte,10CSV13.499.803byte;
i checkpoint precedenti del codice escludevano intenzionalmente NPZ. Creata
conservazione aggiuntiva esclusiva in deliveries,senza spostare/cancellare nulla.
preserve_campaign_payload.py usa libreria standard,verifica14sorgentifreeze,
5manifestconsegna precedenti,baselineSHA,poi copia soltanto artifact scientifici
campagna,riferimenti di archive_review e dipendenze sorgenti note. Nessun contenuto
NVIDIA_RUNS incluso; 4GBoriginali rimangono nei percorsi originali in sola lettura.

ZIP controller_night_20261024_0344.zip:383file316.904.324byte noncompressi,
267.468.570byteZIP,SHA4a9f08debd4012a6775cbb3a165163ef69ec6825d4e3510f907a75f1492fda4f.
Include61NPZ/10CSV/48Python della campagna e copia byte-identica snapshotbaseline
con431elementi,senza modificare il file protetto. Cache/checkpoint pregressi
nonduplicati,restano conservati. Loglive/pointermanifest esclusi per evitare
self-reference;4fileoperativi inclusi come snapshot datati con hashZIP esatto.

Verificatore indipendente verify_campaign_payload.py rilegge ogni elemento
ZIP(CRC+SHA+dimensioni),interoSHAZIP e tuttiimmobili workspace;nessuna estrazione
o scrittura da verificatore. Manifestsidecar,esito eCURRENT_PAYLOAD salvati
esclusivi;builder non va rilanciato sullo stesso target. PAYLOAD_README/STATUS
spiegano input/deps ancora necessari:non ambiente portatile o release robot.

Originali verificati19:48:14UTC:765file4080005175byte identici,ZIPbaseline431
elementi e332sorgenti runtime invariati. Stato aggiornato solo dopo confezione:
si verifica poi chegliimmutable rimangano uguali mentre snapshotoperativi
possono cambiare. Nessuna nuova conclusione sul damping o nuovo candidato.
STATE:sintesi/studi/preservazione completi,niente ripetizioni senza nuovo
problema oinput;alcutoff solochiusura/integrità. Termine09:00Singaporeinvariato.
''')
    print('Payload preservation recorded; immutable science and operational snapshots distinguished.')


if __name__=='__main__':main()
