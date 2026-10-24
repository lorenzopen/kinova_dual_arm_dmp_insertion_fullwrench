"""Document fixed-parameter local analysis and preserve prior deliveries."""
import sys
sys.dont_write_bytecode=True
import hashlib
import json
from pathlib import Path
from datetime import datetime,timezone
from evaluate_locked_holdout import verify_freeze

OUT=Path(__file__).resolve().parent


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(name):return json.loads((OUT/name).read_text())


def main():
    _,freeze,verification=verify_freeze()
    holdout=read('HOLDOUT_DELIVERY_MANIFEST.json')
    for entry in holdout['files']:assert sha(OUT/entry['path'])==entry['sha256'],entry['path']
    design=read('LINEAR_ROBUSTNESS_DESIGN.json');checks=read('linear_robustness_validation.json')
    assert design['freeze_sha256']==freeze
    assert design['script_sha256']==sha(OUT/'linear_robustness.py')
    assert checks['all_passed'] and checks['all_cases']==336
    preserved=read('preservation_verification.json');assert preserved['archive_identical']
    names=['linear_robustness.py','LINEAR_ROBUSTNESS_DESIGN.json','linear_robustness_validation.json',
        'linear_robustness_metrics.json','linear_robustness_metrics.csv','linear_robustness_summary.json',
        'LINEAR_ROBUSTNESS_REPORT.md','17_robustezza_locale_sensibilita.png','MINIMUM_IDENTIFICATION_PROTOCOL.md',
        'finalize_linear_tranche.py','linear_robustness_process.log']
    delivery=dict(created_utc=datetime.now(timezone.utc).isoformat(),freeze_sha256=freeze,
        verification=verification,previous_holdout_delivery_files_verified=len(holdout['files']),
        preservation=preserved,files=[dict(path=n,bytes=(OUT/n).stat().st_size,sha256=sha(OUT/n)) for n in names],
        conclusion='Fixed local unsaturated LTI hypotheses only; no plant validation or new selected controller. No data from CSVs or new holdout tuning.')
    with (OUT/'LINEAR_ROBUSTNESS_DELIVERY_MANIFEST.json').open('x',encoding='utf-8') as f:json.dump(delivery,f,indent=2)
    report=OUT/'RESULTS.md';old=report.read_text(encoding='utf-8')
    intro='''## Aggiornamento: stabilità locale, onde e dati minimi necessari

Nuovo [report analitico](LINEAR_ROBUSTNESS_REPORT.md) con una figura.
Le sette leggi restano fisse: **336 anelli discreti locali**, 16 modelli e tick
20/25/30 ms, senza leggere i CSV o rifare l'holdout. Tutti gli anelli ipotizzati
sono localmente stabili a limiti inattivi, ma alcune frequenze del disturbo
possono essere amplificate: stabilità e assenza di onde non sono equivalenti.
Ritardo e integrale possono produrre questa amplificazione anche senza rumore.
È un risultato dei modelli, non l'identificazione della causa reale delle onde.

Il massimo di sensibilità in banda 0,8–2,5 Hz della FFI ipotizzata arriva a4,20
nel caso peggiore della griglia; non è una previsione di moltiplicazione della
RMS reale. P/PD moderati lasciano invece circa il94% di un disturbo DC nel caso
centrale, mentre I lo annulla solo nell'ideale stabile e non limitato.
Poli verificati anche da una matrice di stato indipendente su tutti336 casi:
scarto massimo5,5e-13. Nessuna nuova selezione, compensazione del ritardo o
equivalenza fra cart_d e i parametri dei modelli.

Il [pacchetto dati e protocollo minimo](MINIMUM_IDENTIFICATION_PROTOCOL.md)
specifica timestamp di acquisizione/ricezione/consumo, comando realmente
applicato, pose/twist e wrench per mano. Priorità al D-only con FFI invariata;
prima servono semantica/versione del servo. L'effetto reale di aumentare D
resta non discriminato. Controlli SHA-256 degli originali e della consegna
holdout precedente passati; nessun dato/controller di produzione modificato.

'''
    marker='## Aggiornamento: confronto bloccato sulle quattro run riservate completato'
    assert marker in old and '## Aggiornamento: stabilità locale, onde' not in old
    report.write_text(old.replace(marker,intro+marker,1),encoding='utf-8')
    state=read('STATE.json')
    state['status']='fixed_local_robustness_complete_no_robot_controller_validated'
    state['persistent_worker']=None
    state['completed'] += [
        '336 fixed local sampled LTI loops:7 frozen laws x16 models x20/25/30ms ticks, exact held-input lag/resonance fractional-delay discretization and filtered-D/backward-Euler control. No CSV or holdout replay/tuning.',
        'All336 hypothetical unsaturated loops locally stable; stable-loop disturbance sensitivity can exceed1 (FFI worst in0.8-2.5Hz4.204). This is not robot stability or measured-wave amplification evidence. P/PD central DC sensitivity0.936, I0 under stable unlimited idealization.',
        'Independent state-matrix vs polynomial radius on336 loops agrees to5.49e-13; pure-I analytic delay thresholds and known lag transfer verified. LINEAR_ROBUSTNESS_REPORT.md/figure17 saved and visually inspected.',
        'MINIMUM_IDENTIFICATION_PROTOCOL.md consolidates future D-only priority, synchronized acquisition/consumption timestamps, actual applied target, per-hand raw wrench, full-wrench distinction and free-simulation model validation. No hardware actions.',
        'Prior immutable HOLDOUT_DELIVERY_MANIFEST artifacts and all14 source fingerprints reverified; original archive765files, baseline431entries and332runtime sources unchanged at17:49:13UTC. New local-analysis delivery manifest saved.'
    ]
    state['next']=[
        'No further tuning, repeated holdout grid or unsupported delay compensator. Frozen controller comparisons are complete and no controller passed predictive/hardware validation.',
        'A bounded source-only audit of current signal/time pipeline may add useful evidence: distinguish ATI acquisition/filtering/projection/FFI consumption and command publication vs servo application. Do not invent missing inner kinova_control semantics or rerun reserved force numerics.',
        'If no discriminating source evidence remains, consolidate known limitations and experiment/data protocol rather than launch more hypothetical parameter sweeps. Preserve D-only priority and unchanged FFI.',
        'Cutoff2026-10-24T01:00Z: only consolidate after cutoff, verify preservation manifests and end continuation. No ROS/robot/deploy/push.'
    ]
    state['latest_source_checkpoint']='checkpoints/tranche_20261024_0144.zip'
    (OUT/'STATE.json').write_text(json.dumps(state,indent=2)+'\n',encoding='utf-8')
    with (OUT/'NIGHT_LOG.md').open('a',encoding='utf-8') as f:
        f.write('''
## Heartbeat24/10 01:44Singapore — robustezza locale, nessun nuovo tuning

LettiAGENTS,PROTOCOL,STATE,NIGHT_LOG,RESULTS,steeringD/audit e processo holdout.
Nessun Python vivo; checkpoint0114 SHA corrisponde f38aee5d3c9105153ad1bc336a398c0c351cfee902fd82d434a7dd92fdb9a44c.
Fallback Get-Process già autorizzato perché CIM negato; nessun processo terminato.
Un solo worker BLAS/OMP/MKL/NUMEXPR1. Codice/nuove uscite solo in campagna.

LINEAR_ROBUSTNESS_DESIGN creato esclusivo prima del calcolo:7leggi già congelate
x16impianti ipotizzati x3tick20/25/30ms=336anelli locali, nominale/target fermi,
limiti inattivi. Non legge CSV né ripete replay/tuning holdout. Plant ZOH esatto
con ritardo frazionario per lag/risonanza, puro gain+delay al campionamento;
C(q) deriva dall'I backward Euler, D sulla misura tau50ms e rolloff già congelati.
Nessuna conversione cart_d->ritardo/ζ. Nessun nuovo candidato o compensatore.

Tutti336 anelli locali hannoρ<1. Verifica radici polinomiali versus matrice
di stato costruita separatamente su tutti336:Δρmax5,493e-13; soglia I+ritardo
nota e transfer lag ZOH verificati, errore frontiera6,66e-16. Non certifica
robot/saturazioni/jitter/contatto, che i modelli non descrivono adeguatamente.
FFI minPMattraversamento24,6° nella griglia; massimo|S|banda0,8-2,5Hz4,204,
PI filtrata1,962,PIsemplice1,559,PID1,426,P1,176,PD1,411,FF1. Questi rapporti
di ampiezza sono ipotesi di disturbo imposto, non previsioni RMS delle run.
P/PD centrali S(0)=0,936; I S(0)=0 solo nell'ideale stabile non limitato.
Conseguenza: onde non implicano instabilità locale; banda e tracking possono
peggiorare/migliorare separatamente. Nessuna attribuzione causale ai log.

LINEAR_ROBUSTNESS_REPORT,JSON/CSV/verifiche e figura17 salvati; figura ispezionata.
MINIMUM_IDENTIFICATION_PROTOCOL aggiunge pacchetto minimo di timestamp nativi,
ricezione/consumo, target applicato, stato, wrench raw per mano e FFI, in futuro
D-only. Ripetizioni e finestre fissate, nessun cherry picking o nuova soglia
di sicurezza. Margine fullwrench distinto dalla richiesta scalare comune.

Originali verificati17:49:13UTC:765file/4.080.005.175byte identici, baselineZIP
e431elementi,332sorgenti runtime invariati. Verificati inoltre tutti14fingerprint
e ciascun artifact del manifestconsegna holdout precedente; nuova consegna
locale esclusiva. STATE aggiornato: eventuale audit sorgenti della pipeline
segnali/clock, niente altro tuning; cutoff24/10 09:00Singapore invariato.
''')
    print('Consolidated local robustness and minimum data protocol. Prior deliveries intact.')


if __name__=='__main__':main()
