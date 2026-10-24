"""Save QA/timing correction evidence without altering frozen comparisons."""
import sys
sys.dont_write_bytecode=True
import json
import hashlib
import re
from pathlib import Path
from datetime import datetime,timezone
from evaluate_locked_holdout import verify_freeze

OUT=Path(__file__).resolve().parent
def read(name):return json.loads((OUT/name).read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    _,freeze,verification=verify_freeze();audit=read('SAVED_EVIDENCE_AUDIT.json')
    assert audit['freeze_sha256']==freeze and audit['consistency_passed']
    assert audit['saved_cases_checked']==448 and audit['same_metric_reductions_checked']==4032
    assert audit['script_sha256']==sha(OUT/'audit_saved_evidence.py')
    clocks=read('SAVED_CLOCK_DIAGNOSTIC.json')
    for item in clocks['runs']:
        assert sha(OUT/item['clock_file'])==item['clock_sha256']
        assert item['max_increment_per_FFI_used_dt_mm_s']<=2+1e-9
    for item in audit['failures_preserved']:
        assert sha(OUT/item['path'])==item['sha256']
        assert sha(OUT/item['threshold_source'])==item['threshold_source_sha256']
    for name in ['SAVED_EVIDENCE_AUDIT.md','READING_GUIDE.md']:
        for target in re.findall(r'\]\(([^)]+)\)',(OUT/name).read_text(encoding='utf-8')):
            assert (OUT/target).is_file(),(name,target)
    preserved=read('preservation_verification.json');assert preserved['archive_identical']
    names=['audit_saved_evidence.py','audit_saved_evidence_attempt1.py','audit_saved_evidence_attempt2.py',
        'SAVED_EVIDENCE_AUDIT.json','SAVED_EVIDENCE_AUDIT_ATTEMPT2.json','SAVED_EVIDENCE_AUDIT.md',
        'SAVED_EVIDENCE_ATTEMPT1_FAILURE.json','READING_GUIDE.md','diagnose_saved_tick_timing.py',
        'SAVED_CLOCK_DIAGNOSTIC.json','evidence_audit_process.log','evidence_audit_process_attempt2.log',
        'evidence_audit_process_attempt3.log','saved_clock_diagnostic_process.log','finalize_evidence_audit.py']
    names += [x['clock_file'] for x in clocks['runs']]
    output=dict(created_utc=datetime.now(timezone.utc).isoformat(),freeze_sha256=freeze,verification=verification,
        preservation=preserved,files=[dict(path=n,bytes=(OUT/n).stat().st_size,sha256=sha(OUT/n)) for n in names],
        scope='Consistency of saved outputs plus timing-only diagnostic; no altered comparison/parameters/weights or claimed robot speed certificate.',
        first_audit_failure_retained=True,threshold_transcription_correction_retained=True)
    with (OUT/'EVIDENCE_AUDIT_DELIVERY_MANIFEST.json').open('x',encoding='utf-8') as f:json.dump(output,f,indent=2)
    intro='''## Verifica di coerenza e precisazione sul clock del limite software

[Guida alla lettura](READING_GUIDE.md) e [audit degli output](SAVED_EVIDENCE_AUDIT.md).
I448 casi salvati, CSV/JSON, score e4.032 riduzioni metriche dalle tracce sono
coerenti; nessun replay o tuning rilanciato. Verificati manifest precedenti,
otto checkpoint e link locali. I tre fallimenti di raffinamento restano tali.

Il primo audit usava erroneamente gli intervalli del log come dt della FFI.
Conservati codice/log falliti, verificati solo i campi temporali dei quattro CSV
e salvati clock ausiliari separati. Il limite **2mm/s della correzione normale
software** è rispettato usando ffi_dt_ms, come nel kernel congelato. La divisione
per intervalli t_wall del log produce massimi circa2,05–2,14mm/s: è un'altra
quantità, non la velocità misurata dei tool o l'istante di consumo del servo.
Lo scheduling usa tempi di log e il limitatore usa dt FFI/clamped; l'incertezza
temporale del replay rimane dichiarata e non è stata ottimizzata dopo l'holdout.

Corretto anche un valore trascritto nella bozza dell'audit: i sorgenti delle
verifiche dichiarano0,03N per RMSE,0,01N per banda,0,05N per picco. Gli esiti
falliti non cambiano e nessuna tolleranza originaria è stata modificata.
Originali e tutti14sorgenti congelati invariati, nuovamente verificati SHA-256.

'''
    report=OUT/'RESULTS.md';old=report.read_text(encoding='utf-8')
    marker='## Sintesi disponibile per la tesi'
    assert marker in old and '## Verifica di coerenza e precisazione' not in old
    report.write_text(old.replace(marker,intro+marker,1),encoding='utf-8')
    state=read('STATE.json');state['status']='saved_evidence_consistent_FFI_clock_scope_clarified'
    state['persistent_worker']=None
    state['completed'] += [
        'Saved-evidence QA:all448 case identities,4032 scalar reductions exactly match stored NPZ; CSV/JSON, frozen score formula/aggregation,0joint-goal passes and prior delivery artifacts consistent. No new simulation/tuning.',
        'First QA assertion failed from treating logger event_time as FFI dt; source/log retained. Timing-only ffi_seq/t_wall/ffi_dt_ms read from4 reserved CSVs, hashes verified; separate clock NPZ saved, no force reanalysis.',
        'Correct FFI-used-dt limit check passes2mm/s for all448; log-interval ratios maxZ2.13852,L2.12735,S2.04525,T2.07439mm/s are different diagnostics, not actual tool speed. Timing/clamp differences explicitly retained as model limitation.',
        'Original refinement thresholds extracted from AST:RMSE0.03N,band0.01N,peak0.05N; an audit draft0.02N transcription corrected with draft/log retained. Three original development numerical failures remain failures; no original threshold changed.',
        'READING_GUIDE.md resolves historical next-tranche notes without rewriting frozen reports. Saved-evidence report,diagnostic provenance and exclusive delivery manifest saved;8checkpoint hashes and local links verified.',
        'Preservation passed19:20:48UTC:765archivefiles4080005175bytes,431baselineZIP entries,332runtime sources and14freeze fingerprints intact. No production edits or worker remains.'
    ]
    state['latest_source_checkpoint']='checkpoints/tranche_20261024_0314.zip'
    state['next']=[
        'No additional tuning, heldout re-evaluation or repetitive hypothetical sweeps. Frozen controller comparison and thesis synthesis complete; no predictive/hardware-validated new controller selected.',
        'When discussing limits specify2mm/s normal correction per FFI-used/clamped dt, not observed whole-tool velocity or logger-clock ratio. Recorded event timing is approximate; no post-holdout retiming to favor results.',
        'Further work only for new discriminating source/data or a concrete unresolved validity/documentation issue. Damping priority unchanged: actual servo semantics and matched fixed-FFI D-only still missing; protocol ready.',
        'At/after cutoff2026-10-24T01:00Z consolidate only, verify all delivery/preservation manifests and final checkpoint, finish overnight continuation. Avoid unchanged status notifications.'
    ]
    (OUT/'STATE.json').write_text(json.dumps(state,indent=2)+'\n',encoding='utf-8')
    with (OUT/'NIGHT_LOG.md').open('a',encoding='utf-8') as f:
        f.write('''
## Heartbeat24/10 03:14Singapore — coerenza output e distinzione dei clock

LettiAGENTS/PROTOCOL/STATE/NIGHT_LOG/RESULTS,steeringD/audit,log/markerholdout.
Checkpoint0244 SHAe61d6dfd75a3f2639db5815e81dac81a28a13e6ce364c543466889fc22c2d23f
verificato,nessunPythonvivo;CIMgiànegato,fallbackGet-Process. Unsolo worker,
BLAS/OMP/MKL/NUMEXPR1,nopycache. Nessun ROS/robot/deploy/push o codice produzione.

audit_saved_evidence.py controlla solo artifact salvati,4manifestconsegna,
8SHAcheckpoint,link11report. DaNPZ448casi ricalcola leSTESSE9riduzioni metriche
senza invocare replay/fit/controller:4032confronti con differenza esattamente0.
Tutti campiCSV/JSON, combinazioni7x16x4, formula score/penalità/aggregazione
e0/448obiettivi congiunti coerenti. Tracce/originalirisultati non modificati.

Primo audit FALLITO:dividevaΔcomandi per event_time=logt_wall e lo chiamava
actualtick. Conservati audit_saved_evidence_attempt1.py,log originale e
SAVED_EVIDENCE_ATTEMPT1_FAILURE.json. Diagnosi concreta richiede lettura dei
SOLIffi_seq/t_wall/ffi_dt_ms dai4CSVriservati (hash verificati), non forze,
target,nuovi punteggi o tuning. Sorgente distingue p.te da p.clock=cumsumFFIdt.
Nuovi clockausiliari separatiNPZ+SAVED_CLOCK_DIAGNOSTIC,originaliNPZimmutati.

ConFFIdt effettivamenteusato, maxΔds/dt2,00000000000003mm/s su tutti448casi.
Conintervallilog:maxZ2,13852,L2,12735,S2,04525,T2,07439mm/s. Maxdifferenza
intervalli3,438/11,300/2,888/1,876ms:includeclockposizione/clamp,nonlatenza
isolataidentificata. Event_time log resta schedulingreplay,dtFFI limiter;
incertezza dichiarata,nessuna riallocazione tempi per cambiare graduatoria.
2mm/s èlimitesoftwarecorrezione normale,non velocità reale interatool/route.

Seconda bozza audit passava ma riportava erroneamente0,02N tolleranzaRMSE:
conservati bozzaJSON,codiceattempt2,log. Estratte ora soglie direttamente
dall'ASTadvanced_refinement.py/planned_reference_validation.py:0,03N RMSE,
0,01banda,0,05picco. Nessuna sogliaoriginariaallargata; treverifiche sviluppo
restanoFALLITE:advanced128casi 5->2,5ms e5->1,25ms,planned608picco0,1839N.
Terzo audit finalePASSAstessa448/4032,nonnuova simulazione. Fonte/sha criteri
inJSON; logfallimenti eiterazioni QA conservati. Unread iniziale filename
refinement nonesistente risolto con file presente,nessun impatto sull'analisi.

READING_GUIDE risolve prospettiva storica dei vecchi«prossimo holdout» senza
editarli; auditreport chiarisce softwarevsvalidazione fisica. Linknuovi verificati,
esclusivo EVIDENCE_AUDIT_DELIVERY_MANIFEST includefonti/bozze/clocks/fallimenti.
Originali nuovamenteverificati19:20:48UTC:765file4080005175byte identici,
ZIPbaseline431elementi,332runtime e14fingerprintfreeze intatti. Gittracciato
invariato,nessun worker persistente. Stati/limiti/conclusioneD invariati;
termine24/10 09:00Singapore. Nessun nuovocandidato promosso dalla QA.
''')
    print('Evidence QA consolidated with retained failures and correct FFI-clock limit scope.')


if __name__=='__main__':main()
