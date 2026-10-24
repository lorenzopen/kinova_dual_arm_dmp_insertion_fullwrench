"""Preserve source/metadata audit and update the ongoing campaign record."""
import sys
sys.dont_write_bytecode=True
import hashlib
import json
from pathlib import Path
from datetime import datetime,timezone
from evaluate_locked_holdout import verify_freeze

OUT=Path(__file__).resolve().parent
def read(name):return json.loads((OUT/name).read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    _,freeze,verification=verify_freeze()
    prior_files=0
    for manifest in ['HOLDOUT_DELIVERY_MANIFEST.json','LINEAR_ROBUSTNESS_DELIVERY_MANIFEST.json']:
        for item in read(manifest)['files']:
            assert sha(OUT/item['path'])==item['sha256'],item['path'];prior_files+=1
    audit=read('SIGNAL_PIPELINE_AUDIT.json');assert audit['freeze_sha256']==freeze
    assert len(audit['runs'])==12 and audit['source_checks_summary']['all_selected_anchors_match']==72
    for source in audit['sources']:assert sha(OUT.parents[2]/source['path'])==source['sha256']
    for run in audit['runs']:assert sha(OUT.parents[2]/run['metadata'])==run['metadata_sha256']
    preserved=read('preservation_verification.json');assert preserved['archive_identical']
    names=['signal_pipeline_audit.py','SIGNAL_PIPELINE_AUDIT.json','SIGNAL_PIPELINE_REPORT.md',
        '18_filtri_esistenti_e_ritardo.png','signal_pipeline_process.log','finalize_signal_tranche.py']
    payload=dict(created_utc=datetime.now(timezone.utc).isoformat(),freeze_sha256=freeze,verification=verification,
        previous_delivery_files_verified=prior_files,preservation=preserved,
        files=[dict(path=n,bytes=(OUT/n).stat().st_size,sha256=sha(OUT/n)) for n in names],
        scope='Source/metadata and ideal FIR algebra only. No force CSV load, tuning, robot or inner-servo identification.')
    with (OUT/'SIGNAL_PIPELINE_DELIVERY_MANIFEST.json').open('x',encoding='utf-8') as f:json.dump(payload,f,indent=2)
    report=OUT/'RESULTS.md';text=report.read_text(encoding='utf-8')
    intro='''## Aggiornamento: la misura ATI della FFI è già mediata

Nuovo [audit della pipeline](SIGNAL_PIPELINE_REPORT.md) con figura dei filtri.
I 12 metadata originali dichiarano `ft_data_avg` a 200 Hz e stream a 100 Hz;
il backend usa già la **media degli ultimi due messaggi**, indipendentemente
dal dt reale della FFI. Il nodo/launch prevede anche blocchi da cinque campioni,
ma il default non dimostra il valore runtime né il rate hardware. Verificate
22 definizioni contro tutte le 12 revisioni registrate: 72 confronti file,
264 definizioni semanticamente coincidenti. Nessun CSV forza rianalizzato.

Se la catena nominale è raw 1 kHz→blocchi da 5→ultimi 2, equivale a 10 campioni:
riduzione d'ampiezza circa **0,016% a 1 Hz**, 0,102% a 2,5 Hz. Non è abbastanza
per attenuare sensibilmente le onde lente. Una media aggiuntiva più lunga
modifica anche la fase; il grafico calcola solo filtri ideali, senza simulare
o promuovere un nuovo controller. Nei report «raw/grezzo» indica i valori
originali senza ulteriore filtro di analisi, non il singolo campione hardware.

Documentato l'ordine pubblicazione→wait→lettura→update FFI→log: il nuovo ds può
appartenere ai comandi successivi, mentre i target della riga sono già inviati.
Stamp e age esistono, ma identificano messaggi recenti/ricezione locale, non
l'intera cronologia dei campioni mediati e del consumo del servo. La legge
cartesiana interna rimane mancante: sul damping serve ancora il D-only.
Gli artifact precedenti e gli originali restano integri, verificati con SHA-256.

'''
    marker='## Aggiornamento: stabilità locale, onde e dati minimi necessari'
    assert marker in text and '## Aggiornamento: la misura ATI' not in text
    report.write_text(text.replace(marker,intro+marker,1),encoding='utf-8')
    state=read('STATE.json');state['status']='source_signal_pipeline_audited_synthesis_pending'
    state['persistent_worker']=None
    state['completed'] += [
        'Source/metadata pipeline audit:12 original FFI metadata,24 arm configs, all ft_data_avg200Hz/stream100Hz; backend averages last2 messages independently of real FFI dt. No CSV numerical data loaded.',
        '22 relevant source definitions per recorded revision semantically checked:72 file/revision comparisons and264 AST definitions match. Exact metadata/source hashes saved; git revision match is not proof of deployed uncommitted contents.',
        'Ideal nominal1kHz/5-block/last2 FIR equals10 consecutive raw samples: attenuation1Hz0.0163%,2.5Hz0.1017%,filter group delay4.5ms excluding queue/consume-age/servo. Defaults/hardware rate/runtime every explicitly distinguished; cascade identity verified.',
        'Documented command->wait->read->FFIupdate->log sequencing, source latest stamps versus local receive age, missing per-sample provenance and actual servo application. No retiming/tuning of frozen holdout.',
        'SIGNAL_PIPELINE_REPORT.md,provenance JSON and figure18 saved/visually inspected. Previous heldout/local delivery artifacts and all14 frozen fingerprints verified.765 NVIDIA files,431 preserved ZIP entries and332 runtime sources intact at18:19:06UTC.'
    ]
    state['next']=[
        'No further controller tuning or heldout numerical grid; no supported delay compensator or actual cart_d response model. Priority remains fixed-FFI D-only with inner-controller semantics.',
        'Prepare a concise thesis interpretation grounded in measured outcomes and the already saved metrics: distinguish insertion success, tracking, physical oscillations and model limitations; state which claims can be defended without declaring an unvalidated controller adequate.',
        'Synthesize known transition/nominal-rate limits and source timing evidence without converting conditional alternatives or filter-only plots into robot predictions. Preserve original objectives and experimental data.',
        'When no discriminating source evidence remains, consolidate rather than launch more hypothetical sweeps. Cutoff2026-10-24T01:00Z: consolidate only afterward, verify manifests and end continuation.'
    ]
    state['latest_source_checkpoint']='checkpoints/tranche_20261024_0214.zip'
    (OUT/'STATE.json').write_text(json.dumps(state,indent=2)+'\n',encoding='utf-8')
    with (OUT/'NIGHT_LOG.md').open('a',encoding='utf-8') as f:
        f.write('''
## Heartbeat 24/10 02:14 Singapore — audit sorgenti della pipeline

Letti AGENTS/PROTOCOL/STATE/NIGHT_LOG/RESULTS, steering damping/audit e process
log precedente; nessun Python vivo. Checkpoint0144 SHA79ffd358ffca909e1c3738b07e56335225b9039747dacff3f91cb50fb32ef000
verificato. CIM già negato su questo host, fallbackGet-Process e marker PIDnull.
Un solo worker; BLAS/OMP/MKL/NUMEXPR1, no pycache, cache Matplotlib nella campagna.
Alcune ricerche iniziali con glob in argomento path non supportato da rg su
Windows/nonexistent logger.py/cell.yaml corrette usando file trovati e percorsi
espliciti; nessuna modifica o esecuzione di sorgenti ROS. Tutti i read utili riusciti.

signal_pipeline_audit.py legge soltanto sei sorgenti e12metadata originali del
freeze; nessun CSV numerico, replay o tuning. Backendmedia n_avg=round(period
config100Hz x ft_rate200Hz)=2 messaggi su entrambe le mani di tutte12run.
Il topic è ft_data_avg, non il singolo campione raw. Nodo AtiNode acc/n mean
ogni every(default5), launchlo passa conconfigure_sensorfalse; default non
prova runtimeevery o frequenza hardware. raw ft CSV è media in sensoraxes
prima di bias/gravità/frame. Insertion proietta su normali misurate e somma
clippata, FFI aggiunge I senza ulteriore filtro misura. Cart_d è servo diverso.

Gitshow su12revisioni x6file=72confronti; tutti disponibili;22anchor per
revisione=264definizioni AST identiche al corrente. Sorgente fisso/snapshot e
metadata dichiarato non dimostrano eventuali patch non committate sul processo;
limite esplicito. SHAmetadata/sorgenti/anchor/righe salvati nel JSONprovenienza.

Algebra FIR ideale:1kHz raw,5campioni perbloccononoverlap,ultimi2blocchi equivale
a10consecutivi; controlloindipendentecascata entro2e-14 eguadagnoDC1. A1Hz
riduzione0,016284%,a2,5Hz0,101750%,a10Hz1,62065%;ritardofiltro4,5ms relativo
all'ultimo raw incluso, senza doppioconteggio blocchi né coda/consumeage/servo.
Figura18illustra anchefinestreideali100/200ms, NON nuovi controllerottimizzati
o simulazione run. Figura ispezionata, report rigenerato dal soloauditJSON per
chiarire chei4,5ms includono già finestra blocchi; niente riesecuzioneGit/CSV.

Sorgente dimostra sequenza command->wait->read->FFIupdate->log;dsnuovo e target
cartesianogiàpubblicato non sono stessoistante. ft_stampèultimomessaggio,
noncentro/tuttiicampionidellamedia; ft_age/joint_age sono da callback locale,
non acquisizione hardware. ROSstamp pubblicazione nonackconsumo servo.
Nessuna correzione ad hoc ai risultati congelati; limiti timing dichiarati.
Dentrokinova_control mancail sorgente effettivo/semantica, no mappingD->tau/ζ.

SIGNAL_PIPELINE_REPORT, auditJSON,figura18 e manifestconsegna esclusivo salvati;
nuove definizioni/metadata verificati nuovamente prima consegna. Consegne
holdout/analitica precedenti e14fingerprint intatti. Originali SHAverificati
18:19:06UTC:765file4080005175byte identici,ZIPbaseline431elementi,332sorgenti
runtime invariati. STATE:prossima sintesi difendibilepertesi senza nuovegriglie.
Campagna attiva entrocutoff24/10 09:00Singapore, nessun ROS/robot/deploy/push.
''')
    print('Source/metadata audit consolidated; prior delivery artifacts verified:',prior_files)


if __name__=='__main__':main()
