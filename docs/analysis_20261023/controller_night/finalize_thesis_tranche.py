"""Consolidate the thesis interpretation without changing frozen experiments."""
import sys
sys.dont_write_bytecode=True
import hashlib
import json
import re
from pathlib import Path
from datetime import datetime,timezone
from evaluate_locked_holdout import verify_freeze

OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2]
def read(name):return json.loads((OUT/name).read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    _,freeze,verification=verify_freeze();prior_files=0
    for name in ['HOLDOUT_DELIVERY_MANIFEST.json','LINEAR_ROBUSTNESS_DELIVERY_MANIFEST.json','SIGNAL_PIPELINE_DELIVERY_MANIFEST.json']:
        for item in read(name)['files']:
            assert sha(OUT/item['path'])==item['sha256'],item['path'];prior_files+=1
    evidence=read('THESIS_EVIDENCE.json');assert evidence['freeze_sha256']==freeze
    assert len(evidence['cohort'])==12 and len(evidence['claims'])==10
    for item in evidence['input_files']:assert sha(ROOT/item['path'])==item['sha256']
    links=[]
    for link in re.findall(r'\]\(([^)]+)\)',(OUT/'THESIS_SYNTHESIS.md').read_text(encoding='utf-8')):
        if not link.startswith(('http:','https:')):
            assert (OUT/link).is_file(),link;links.append(link)
    preserved=read('preservation_verification.json');assert preserved['archive_identical']
    names=['thesis_synthesis.py','THESIS_SYNTHESIS.md','THESIS_EVIDENCE.json','thesis_observed_runs.csv',
           '19_risultati_sperimentali_per_tesi.png','thesis_synthesis_process.log','finalize_thesis_tranche.py']
    manifest=dict(created_utc=datetime.now(timezone.utc).isoformat(),freeze_sha256=freeze,verification=verification,
        prior_delivery_files_verified=prior_files,local_report_links_checked=len(links),preservation=preserved,
        files=[dict(path=n,bytes=(OUT/n).stat().st_size,sha256=sha(OUT/n)) for n in names],
        scope='Synthesis from saved observations and frozen studies only. No tuning, causal improvement claim, robot recommendation or universal thesis acceptance decision.')
    with (OUT/'THESIS_DELIVERY_MANIFEST.json').open('x',encoding='utf-8') as f:json.dump(manifest,f,indent=2)
    old=(OUT/'RESULTS.md').read_text(encoding='utf-8')
    intro='''## Sintesi disponibile per la tesi

[Sintesi ragionata con testo da adattare](THESIS_SYNTHESIS.md), tabella delle
12 run, figura19 e registro di dieci affermazioni con evidenza e limiti.
Il risultato sostenuto è una dimostrazione sperimentale nelle condizioni
archiviate, con oscillazioni e transitori quantificati. Non si dispone di una
soglia universale di accettabilità per la tesi, di un nuovo controllore validato
o di una misura indipendente dell'accuratezza finale del pezzo.

Il confronto descrittivo con la Z delle20:20 chiarisce il compromesso: banda
0,156N storica contro0,111–0,123N nelle tre FFI; RMSE route0,256N storico contro
0,304–0,339N FFI. L/S/T hanno altri riferimenti storici espliciti nel report;
nessuno di questi confronti isola causalmente l'effetto del solo SOCP offline.
Anche architettura normale, profilo e condizioni differiscono.

Le12 run selezionate sono archiviate come riuscite con seat_found/noJAM;
non sono12/12 tentativi.24 metadata FFI includono13 flag seat_found e altre
condizioni/stop: nemmeno13/24 è una stima pulita della configurazione finale.
La sintesi conserva transitoriL, difficoltàS e minimi/asimmetrie, senza confondere
believed_mm con metrologia esterna o forza media positiva con margine di presa.

Damping prioritario: ipotesi ancora aperta, protocollo D-only a FFI fissa già
preparato. Compensazione del ritardo e nuovo PD/PID non promossi da modelli che
non superano le verifiche predittive. Tutto il tuning è concluso e congelato;
questa tranche ha soltanto aggregato risultati salvati e verificato le fonti.

'''
    marker='## Aggiornamento: la misura ATI della FFI è già mediata'
    assert marker in old and '## Sintesi disponibile per la tesi' not in old
    (OUT/'RESULTS.md').write_text(old.replace(marker,intro+marker,1),encoding='utf-8')
    state=read('STATE.json');state['status']='thesis_synthesis_complete_offline_findings_consolidated'
    state['persistent_worker']=None
    state['completed'] += [
        'THESIS_SYNTHESIS.md and figure19 synthesize saved12-run observations, historical comparisons, original target metrics, transition limitations, damping priority and model validity. No raw CSV re-analysis or tuning.',
        'Historical Z20:20 has band0.15558N/RMSE0.25614N versus currentFFI band0.11107-0.12306N/RMSE0.30429-0.33929N; descriptive tradeoff only. Nine exact historical reference IDs/argv/available runtime metadata saved.',
        'Selected12seat_found/noJAM is not12/12 success rate:24FFI metadata include13seat_found flags and heterogeneous conditions/stops. Final believed_mm explicitly not independent metrology; no universal thesis acceptance threshold asserted.',
        'Ten-claim evidence ledger, input SHA provenance and12-row table saved; no causal claim that offline SOCP alone improved control. Report local links and prior delivery artifacts verified; figure inspected.',
        'Archive preservation passed18:49:44UTC:765files4080005175bytes identical,431baselineZIP entries and332runtime sources unchanged. All14 frozen source fingerprints still match; thesis delivery manifest saved.'
    ]
    state['next']=[
        'Offline comparisons and synthesis are complete; no additional tuning, heldout re-evaluation or repetitive hypothetical sweeps. Candidate gains remain frozen; no controller has passed plant predictive and robot validation.',
        'Only pursue remaining bounded work if it adds discriminating evidence or resolves a concrete documentation/model-validity issue. Keep measured outcomes, conditional replay, autonomous hypotheses and source-only/filter algebra distinct.',
        'Any new internal servo source or matched D-only data can change the conclusion; without it retain D-only protocol/measurement requirements rather than invent a damping recommendation. No robot/ROS/deploy/push.',
        'At/after cutoff2026-10-24T01:00Z consolidate only, verify all preservation/delivery manifests and final checkpoint, finish the overnight continuation. Until then preserve state and avoid status notifications without meaningful change.'
    ]
    state['latest_source_checkpoint']='checkpoints/tranche_20261024_0244.zip'
    (OUT/'STATE.json').write_text(json.dumps(state,indent=2)+'\n',encoding='utf-8')
    with (OUT/'NIGHT_LOG.md').open('a',encoding='utf-8') as f:
        f.write('''
## Heartbeat 24/10 02:44 Singapore — interpretazione per la tesi

Letti AGENTS/PROTOCOL/STATE/NIGHT_LOG/RESULTS, steeringD/audit, log pipeline e
marker holdout complete/PIDnull. Nessun Python vivo; checkpoint0214 SHA
16330da496cd70bea9b16e536b112f0da10314388735235f5d4372eba5c104b1 verificato.
Un solo worker con BLAS/OMP/MKL/NUMEXPR1, no pycache, cacheMatplotlib campagna.

thesis_synthesis.py aggrega soltanto metriche già salvate nell'archivio e negli
studi congelati: nessun CSV forza ricaricato, modello rifittato, replay o tuning.
12identità dal freeze,8development/4holdout conservati. Nuova tabella e figura19
distinguono RMS banda/errore totale; riferimento targetCSV arrotondato/metodo
storico dichiarato diverso da metriche precise/ricampionamento del holdout.
Tre ripetizioni per pezzo, no stima inferenziale o curva media che nasconde casi.

RMS banda12FFI0,059-0,123N;routeRMSE0,270-0,554N;LslowseatRMSE1,396-1,521N.
PerZ20:20metrica storica banda0,155582N,RMSE0,256139N versus FFI Zbanda
0,111068-0,123064N eRMSE0,304285-0,339291N. Meno banda non migliora tutto.
Riferimenti storici9 espliciti(Z1,L3,S2,T3), condizioni non matched; metadati
argv/git/esecuzione/runtime disponibili salvati.20:20insert_open --online
gain0.5, non confronto soloSOCPoffline. D runtime20:20 non confermato.

Verificati12flagseat_found senzaJAM nella selezione; catalogo24FFI con13flag
seat_found. Non12/12tentativi né13/24probabilità configurazionefinale. Valori
believed_mm non metrologia esterna epositivitàmedianonmarginefullwrench.
Sorgenti/misure/modelli/counterfactual distinti; no universalità accettazione
tesi. Dieci affermazioni con evidence/limiti registrate, testo tesi adattabile.
Damping ipotesi aperta, primaD-onlyconFFI invariata; nessun nuovoPID/compensatore.

THESIS_SYNTHESIS.md,THESIS_EVIDENCE.json,tabellaCSV,figura19 e codice salvati;
figura ispezionata. Seconda generazione della sola sintesi da metriche salvate
per aggiungere argv/riferimenti e spaziatura; nessuna griglia numerica ripetuta.
Riferimenti locali verificati, tutte fonti hash inledger, consegne precedenti
e14fingerprint congelati invariati. Manifestthesis esclusivo prodotto.

Originali verificati18:49:44UTC:765file4080005175byte identici,ZIPbaseline431
elementi e332sorgenti runtimeinvariati. Git tracciato invariato, nessun worker
persistente. STATE:studi e sintesi offline completi, soltanto ulteriore lavoro
che aggiunga evidenza discriminante o risolva un problema concreto; niente
nuovi sweep senza modello. Termine24/10 09:00Singapore sempre invariato.
''')
    print('Thesis synthesis consolidated; prior artifacts verified',prior_files,'; local links',len(links))


if __name__=='__main__':main()
