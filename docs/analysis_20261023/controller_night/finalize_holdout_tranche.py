"""Consolidate locked held-out comparison; no new simulation or tuning."""
import sys
sys.dont_write_bytecode=True
import json
import hashlib
from datetime import datetime,timezone
from pathlib import Path
from evaluate_locked_holdout import verify_freeze

OUT=Path(__file__).resolve().parent
TAG='tranche_20261024_0114'


def main():
    design,sha,verified=verify_freeze()
    summary=json.loads((OUT/'holdout_summary.json').read_text())
    done=json.loads((OUT/'HOLDOUT_EVALUATION_COMPLETED.json').read_text())
    progress=json.loads((OUT/'HOLDOUT_PROGRESS.json').read_text())
    preserved=json.loads((OUT/'preservation_verification.json').read_text())
    assert summary['freeze_sha256']==sha==done['freeze_sha256']
    assert done['total_cases']==448 and progress['status']=='complete' and progress['pid'] is None
    assert preserved['archive_identical'] and preserved['baseline_zip_sha256_verified']
    artifacts=[OUT/item[key] for item in done['results'] for key in ['result_file','trace_file']]
    artifacts += [OUT/name for name in ['evaluate_locked_holdout.py','holdout_report.py','finalize_holdout_tranche.py',
        'HOLDOUT_REPORT.md','holdout_metrics.csv','holdout_summary.json','HOLDOUT_EVALUATION_STARTED.json',
        'HOLDOUT_EVALUATION_COMPLETED.json','14_holdout_forze_osservate.png',
        '15_holdout_confronto_condizionato.png','16_holdout_score_compromessi.png']]
    manifest=dict(created_utc=datetime.now(timezone.utc).isoformat(),freeze_sha256=sha,verification=verified,
        files=[dict(path=p.name,bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in artifacts],
        preservation_at_delivery=preserved,
        note='Completed retrospective holdout comparison, no tuning after frozen parameters. Results/NPZ hashes are also in the exclusive completion marker. No hardware candidate selected.')
    with (OUT/'HOLDOUT_DELIVERY_MANIFEST.json').open('x',encoding='utf-8') as f:json.dump(manifest,f,indent=2)
    state=json.loads((OUT/'STATE.json').read_text())
    state['status']='locked_retrospective_holdout_complete_no_validated_controller'
    state['persistent_worker']=None
    state['completed'] += [
        'ONE locked retrospective holdout batch completed:7 fixed controllers x16 models x4 reserved runs=448 cases at2.5ms. Exclusive started/completed/progress markers, immutable per-run results/NPZ,14 source hashes and CSV hashes verified. No retuning.',
        'Holdout robust score:FFI11.0070,filtered PI10.8293(-1.61%),simple PI10.8616(-1.32%),PID10.8989(-0.98%). P/PD weaker;0/448 cases pass all exploratory goals. No hardware controller selected.',
        'Observed per-hand normals, multiband force/error metrics, mean-force counterfactuals,200ms trailing-mean transient and limiting fractions saved. Baseline error3.11e-15N,2.17e-15mm; all2mm/s bounds verified.',
        'HOLDOUT_REPORT.md and figures14/15/16 generated solely from saved outputs and visually inspected. Development FF-only score penalty10^9 in two nonphysical scalar-force cases explicitly retained and annotated, no scale distortion.',
        'Delivery SHA manifest saved;765 NVIDIA files4080005175bytes,431 baseline ZIP entries and332 runtime sources unchanged. All final freeze/source supplement fingerprints still match.'
    ]
    state['next']=[
        'No further controller tuning or repeated holdout grid. Use frozen outcomes only for synthesis. Analytical robustness may be assessed with explicitly hypothetical plants, distinguishing it from robot stability evidence.',
        'Consolidate minimal synchronized-command/pose/wrench acquisition and future D-only repeated experiment protocol, prioritizing the actual inner Cartesian controller source/semantics. No robot/ROS execution.',
        'Explicit delay compensation remains unsupported: empirical and URDF-only model predictive gates failed; do not select a compensator by the small conditional holdout score gap.',
        'Before cutoff2026-10-24T01:00Z continue bounded analytical work only if it adds discriminating evidence; after cutoff consolidate/verify preserved manifests and end continuation. No promise of zero oscillation.'
    ]
    state['latest_source_checkpoint']='checkpoints/'+TAG+'.zip'
    (OUT/'STATE.json').write_text(json.dumps(state,indent=2)+'\n',encoding='utf-8')
    report=OUT/'RESULTS.md';old=report.read_text(encoding='utf-8')
    intro='''## Aggiornamento: confronto bloccato sulle quattro run riservate completato

Nuovo [report con tre grafici](HOLDOUT_REPORT.md). Eseguito **un solo batch di
448 casi** a 2,5 ms, con sette configurazioni congelate prima dell'analisi e
nessun tuning successivo. La PI filtrata cambia lo score robusto di **−1,61%**
rispetto alla FFI; PI semplice −1,32%, PID −0,98%. Nessuno dei 448 casi raggiunge
insieme gli obiettivi esplorativi. Nei modelli centrali si attenua parte della
banda ma restano tracking e picco L importanti: FFI 6,61 N/mano, PI filtrata
ipotizzata 6,33 N/mano, contro richiesta finale circa 3,79 N/mano.

Questi replay impongono il residuo delle run originali; i modelli non hanno
superato le verifiche predittive. Lo scarto di score non valida un controllore.
L'holdout è retrospettivo per il tuning di questa notte, già visto in precedenti
analisi descrittive. Grafici osservati SX/DX, alternative scalari separate,
bande dell'errore, minimi, limitazioni e transiente medio su 200 ms salvati.
Nella run S le mani si separano durante slow_seat: il modello scalare non
ricostruisce questo fenomeno e non dimostra conservazione dei margini di presa.

Il damping interno resta invariato e non identificato dai confronti disponibili:
la verifica prioritaria resta il [protocollo D-only](DAMPING_AB_PROTOCOL.md).
Conservati anche i fallimenti: FF solo nello sviluppo ha due scenari con
media simulata ≤0, esclusi con penalità 10⁹; il report non li nasconde né
interpreta la forza negativa lineare come misura reale. Tutti i limiti e le
baseline software sono verificati. Verifica SHA-256 completa degli originali
e dei sorgenti protetti passata dopo questa tranche.

'''
    marker='## Aggiornamento: piani limitati, tempi esatti e confronto finale congelato'
    assert marker in old and '## Aggiornamento: confronto bloccato sulle quattro' not in old
    old=old.replace(marker,intro+marker,1)
    old=old.replace('La prossima tranche eseguirà il batch bloccato senza ritoccare i parametri.',
                    'Il batch bloccato è stato poi completato: vedere l’aggiornamento in apertura.')
    report.write_text(old,encoding='utf-8')
    with (OUT/'NIGHT_LOG.md').open('a',encoding='utf-8') as f:
        f.write('''
## Heartbeat 24/10 01:14 Singapore — holdout congelato completato

Letti AGENTS/PROTOCOL/STATE/NIGHT_LOG/RESULTS, steering damping e audit.
Checkpoint0044 verificato prima del batch, nessun Python vivo; fallback
Get-Process a causa dei precedenti limiti CIM, nessuna interferenza con altri
processi. Un solo worker sincrono, OPENBLAS/OMP/MKL/NUMEXPR=1, niente pycache.

evaluate_locked_holdout.py: prima14 fingerprint sorgenti+versioni runtime,
manifest freeze/supplemento e SHA dei4 CSV; marker esclusivo START alle
17:18:55UTC, completamento17:18:59UTC.7candidati x16modelli x4run=448 casi,
dt2,5ms. Un solo batch senza modificare i parametri. Salvataggi immutabili
per run, NPZ completo, hash e marker completed, progresso complete/PIDnull.
No ROS/robot/push; holdout retrospettivo dichiarato. Identità baseline max
3,11e-15N/2,17e-15mm, limiti2mm/s verificati.

Score peggiore modello per run poi media4: FFI11,00702; PI filtrata10,82928
(-1,6148%); PI semplice10,86160(-1,3211%); PID10,89892(-0,9821%). FFsolo14,80371,
P16,15831,PD15,95527.0/448 supera tutti gli obiettivi esplorativi. Nessun nuovo
controller selezionato per il robot, nessuna nuova identificazione dei tempi
o garanzia di stabilità. Le ipotesi fissano residuo/contatto/geometria/fasi.

Report da soli risultati salvati, senza rieseguire replay. Osservati route
RMSE Z/L/S/T0,322/0,275/0,530/0,277N; banda0,8-2,5Hz0,111/0,104/0,077/0,104N.
Minimi mani attivi Z3,131/3,118,L3,308/3,291,S2,055/3,380,T2,546/3,280N.
Asimmetria slow_seat S/T visibile; delta scalare non predice singoli contatti.
Richiesta comune non è margine full wrench, timestamp non sincroni; ricostruzione
fn_N/wrench differisce fino0,0144N somma. PiccoLbaseline6,610N; PI filtrata
centrale ipotizzata6,329N, ancora2,543N sopra richiesta. Eccesso200ms2,646/2,365N.
Forza grezza/errori separati in3bande, limitazioni di tutti i tick documentate.

Durante QA figurascale individuata penalità sviluppo FFsolo1,25e8: due casi S
del modello k1500 con media≤0, minimo-0,025N, già nel kernel congelato. Non è
nuovo errore o enorme forza misurata. Report aggiunge spiegazione, barra annotata
fuori scala; score/JSON originali conservati.3figure14/15/16 ispezionate,
report/CSV/JSON e manifestconsegna salvati. Nessuna modifica ai14 sorgenti congelati.

Priorità damping ribadita: FFIsempreD60/7, niente esperimentoD-only disponibile,
ζ/ritardo dei replay non sono cart_d. Protocollo futuro già pronto, nessun
nuovo compensatore ritardo ammesso senza modello predittivo adeguato.

Conservazione verificata17:22:08UTC:765file NVIDIA_RUNS identici,
4.080.005.175byte; ZIPbaseline+431contenuti e332sorgenti invariati.
Gitdiff/stato tracciati vuoti, nessun Python persistente. STATE aggiornato:
prossime tranche solo robustezza analitica ipotetica/sintesi/protocollo dati,
nessun ulteriore tuning su holdout. Termine24/10 09:00Singapore invariato.
''')
    print('Consolidated locked holdout, immutable delivery manifest, state and log. Create checkpoint',TAG)


if __name__=='__main__':main()
