"""Storage/report consistency only: NEVER run replay, retune or open source CSVs."""
import sys
sys.dont_write_bytecode=True
import csv
import json
import re
import hashlib
from pathlib import Path
from datetime import datetime,timezone
import conditional_controllers as c
import numpy as np
from evaluate_locked_holdout import verify_freeze

OUT=c.OUT;ROOT=c.ROOT
DELIVERIES=['HOLDOUT_DELIVERY_MANIFEST.json','LINEAR_ROBUSTNESS_DELIVERY_MANIFEST.json',
            'SIGNAL_PIPELINE_DELIVERY_MANIFEST.json','THESIS_DELIVERY_MANIFEST.json']
REPORTS=['HOLDOUT_REPORT.md','LINEAR_ROBUSTNESS_REPORT.md','SIGNAL_PIPELINE_REPORT.md','THESIS_SYNTHESIS.md',
         'DAMPING_REPORT.md','DAMPING_AB_PROTOCOL.md','MINIMUM_IDENTIFICATION_PROTOCOL.md',
         'EMPIRICAL_CONTROL_REPORT.md','ADVANCED_CONTROL_REPORT.md','PROFILE_PREVIEW_REPORT.md','PLANNED_REFERENCE_REPORT.md']


def load(name):return json.loads((OUT/name).read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def main():
    design,freeze_sha,verification=verify_freeze()
    result=dict(created_utc=datetime.now(timezone.utc).isoformat(),freeze_sha256=freeze_sha,verification=verification,
        scope='Saved-output consistency, not new holdout evaluation, plant validation or controller selection.',
        script_sha256=sha(Path(__file__)),deliveries=[],checkpoints=[],links=[],external_links_not_revalidated=[],
        trace_checks=[],failures_preserved=[])
    for name in DELIVERIES:
        metadata=load(name)
        for item in metadata['files']:
            assert sha(OUT/item['path'])==item['sha256'],(name,item['path'])
        result['deliveries'].append(dict(manifest=name,manifest_sha256=sha(OUT/name),artifacts_verified=len(metadata['files'])))
    for path in sorted((OUT/'checkpoints').glob('*_manifest.json')):
        m=json.loads(path.read_text());z=path.with_name(m['tag']+'.zip')
        assert sha(z)==m['zip_sha256'],z
        result['checkpoints'].append(dict(path=z.name,sha256=m['zip_sha256'],entries=len(m['files'])))
    for name in REPORTS:
        for target in re.findall(r'\]\(([^)]+)\)',(OUT/name).read_text(encoding='utf-8')):
            if target.startswith(('http://','https://')):
                result['external_links_not_revalidated'].append(dict(report=name,target=target));continue
            raw=target.strip('<>');raw=re.sub(r':\d+(?:-\d+)?$','',raw)
            p=Path(raw) if re.match(r'^[A-Za-z]:/',raw) else OUT/raw
            assert p.is_file(),(name,target)
            result['links'].append(dict(report=name,target=target))
    for entry in load('THESIS_EVIDENCE.json')['input_files']:
        assert sha(ROOT/entry['path'])==entry['sha256']
    csv_rows=list(csv.DictReader((OUT/'holdout_metrics.csv').open(newline='',encoding='utf-8')))
    csv_map={(r['id'],r['candidate'],r['model']):r for r in csv_rows}
    assert len(csv_map)==len(csv_rows)==448
    ids=[x['id'] for x in design['candidates']];models=[x['id'] for x in design['models']]
    summary=load('holdout_summary.json');all_rows=[];maximum_difference=0.
    done=load('HOLDOUT_EVALUATION_COMPLETED.json')
    start=load('HOLDOUT_EVALUATION_STARTED.json')
    diagnostic=load('SAVED_CLOCK_DIAGNOSTIC.json')
    used_clocks={r['id']:r for r in diagnostic['runs']}
    result['first_failed_attempt']=load('SAVED_EVIDENCE_ATTEMPT1_FAILURE.json')
    failed=result['first_failed_attempt']
    assert sha(OUT/failed['attempt_source'])==failed['attempt_source_sha256']
    assert sha(OUT/failed['log'])==failed['log_sha256']
    result['clock_diagnostic']=diagnostic
    assert design['created_utc']<start['created_utc']<done['completed_utc']
    for item in done['results']:
        p=load(item['result_file']);rr=p['metrics'];assert len(rr)==112
        assert {(r['candidate'],r['model']) for r in rr}=={(i,m) for i in ids for m in models}
        assert sha(OUT/item['result_file'])==item['result_sha256']
        assert sha(OUT/item['trace_file'])==item['trace_sha256']
        with np.load(OUT/item['trace_file'],allow_pickle=False) as z:
            t=z['t'];y=z['force'];target=z['target'];err=y-target[:,None,None]
            assert list(z['candidate_ids'])==ids and list(z['model_ids'])==models
            assert y.shape==(len(t),7,16) and np.isfinite(y).all()
            assert np.allclose(np.diff(t),.0025,rtol=0,atol=1e-12)
            route=(t>=float(z['route0']))&(t<=float(z['route1']))
            seat=(t>=float(z['route1']))&(t<=float(z['route1'])+1.6)
            # Recalculate the *same* frozen scalar reductions from saved arrays,
            # independently of replay: detect serialization/units/window mismatches.
            reductions=dict(route_rmse_N=np.sqrt(np.mean(err[route]**2,axis=0)),
                route_p95_abs_error_N=np.percentile(abs(err[route]),95,axis=0),
                route_max_positive_error_N=np.max(err[route],axis=0),
                route_max_negative_error_N=np.max(-err[route],axis=0),
                first_seat_peak_N=np.max(y[seat],axis=0),
                first_seat_max_positive_error_N=np.maximum(0,np.max(err[seat],axis=0)),
                first_seat_max_negative_error_N=np.maximum(0,np.max(-err[seat],axis=0)),
                minimum_mean_hand_force_N=np.min(y,axis=0),
                max_command_abs_mm=np.max(abs(z['commands']),axis=0))
            baseline_difference=float(abs(y[:,0,:]-z['baseline_force'][:,None]).max())
            assert baseline_difference<1e-10
            max_log_slew=float(np.max(abs(np.diff(z['commands'],axis=0))/np.diff(z['event_time'])[:,None,None]))
            clock_meta=used_clocks[p['id']]
            assert sha(OUT/clock_meta['clock_file'])==clock_meta['clock_sha256']
            with np.load(OUT/clock_meta['clock_file'],allow_pickle=False) as times:
                assert np.array_equal(times['event_time'],z['event_time'])
                max_slew=float(np.max(abs(np.diff(z['commands'],axis=0))/np.diff(times['ffi_integration_clock'])[:,None,None]))
            assert max_slew<=2+1e-9
            maximum_this=0.
            for row in rr:
                i=ids.index(row['candidate']);j=models.index(row['model'])
                for key,val in reductions.items():
                    difference=abs(row[key]-float(val[i,j]));maximum_this=max(maximum_this,difference)
                    assert difference<1e-10,(row['id'],row['candidate'],key,difference)
                score=row['route_rmse_N']/.2+row['route_band_rms_N']/.05+row['first_seat_max_positive_error_N']/.25+row['route_p95_abs_error_N']/.5
                if row['minimum_mean_hand_force_N']<=0:score=1e9
                assert abs(row['score']-score)<1e-10
                serialized=csv_map[(row['id'],row['candidate'],row['model'])]
                for key,value in row.items():
                    if isinstance(value,(int,float)):assert abs(float(serialized[key])-value)<1e-12
                    else:assert serialized[key]==value
                assert row['max_command_speed_mm_s']<=2+1e-9 and row['max_command_abs_mm']<=12+1e-9
            maximum_difference=max(maximum_difference,maximum_this)
            result['trace_checks'].append(dict(id=p['id'],saved_cases=len(rr),reduction_fields=len(reductions),
                max_saved_vs_trace_difference=maximum_this,baseline_max_error_N=baseline_difference,
                FFI_used_dt_command_max_slew_mm_s=max_slew,
                logger_dt_command_max_slew_mm_s=max_log_slew))
        all_rows.extend(rr)
    for saved in summary['controllers']:
        rr=[r for r in all_rows if r['candidate']==saved['id']]
        score=float(np.mean([max(r['score'] for r in rr if r['piece']==piece) for piece in 'ZLST']))
        assert abs(score-saved['holdout_mean_run_worst_model_score'])<1e-12
    goals=sum(r['route_rmse_N']<=.2 and r['route_band_rms_N']<=.05 and r['first_seat_max_positive_error_N']<=.25 for r in all_rows)
    assert goals==summary['joint_goal_pass_cases']==0
    for name in ['advanced_numerical_refinement.json','advanced_numerical_refinement_1p25ms.json','planned_reference_refinement.json']:
        m=load(name);maxima=m['maximum_absolute_difference_N']
        # Inspect retained original limits rather than silently relaxing them.
        failed=(maxima['route_rmse_N']>.02 or maxima['route_band_rms_N']>.01 or maxima['first_seat_peak_N']>.05)
        assert failed
        result['failures_preserved'].append(dict(path=name,sha256=sha(OUT/name),scenarios=m['scenarios'],
            maximum_absolute_difference_N=maxima,passed=m.get('passed'),provenance='Retained development-study numerical failure, not a new holdout run.'))
    result.update(saved_cases_checked=len(all_rows),same_metric_reductions_checked=len(all_rows)*9,
        max_saved_vs_trace_difference=maximum_difference,joint_provisional_goals_met_cases=goals,
        consistency_passed=True,physical_model_validated=False,new_controller_selected=False,
        statement='Software/storage checks passed; all previously documented model and numerical limitations remain. No new simulated trajectories, score weights or parameter choices.')
    (OUT/'SAVED_EVIDENCE_AUDIT.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    report(result)
    print('Saved evidence consistency passed:',len(all_rows),'cases,',result['same_metric_reductions_checked'],'same-metric reductions,',len(result['checkpoints']),'checkpoint hashes.')
    print('Maximum scalar reduction difference:',maximum_difference,'; retained failed refinement files:',len(result['failures_preserved']))


def report(r):
    failures='\n'.join(f"| {x['path']} | {x['scenarios']} | {x['maximum_absolute_difference_N']['route_rmse_N']:.4f} | {x['maximum_absolute_difference_N']['route_band_rms_N']:.4f} | {x['maximum_absolute_difference_N']['first_seat_peak_N']:.4f} |" for x in r['failures_preserved'])
    text=f'''# Coerenza degli output salvati

Verifica {r['created_utc']}. **Nessuna nuova traiettoria simulata, analisi delle
forze originali, selezione o tuning.** Il diagnostico del clock ha letto soltanto
ffi_seq/t_wall/ffi_dt_ms nei quattro CSV riservati. I controlli riguardano coerenza software
e conservazione dei risultati, non validazione fisica del Kinova.

Verificati i quattro manifest di consegna e i relativi artifact,14 fingerprint
congelati, tutti gli input hash del registro tesi, {len(r['checkpoints'])} SHA dei
checkpoint e {len(r['links'])} collegamenti locali negli11 report principali.
I link web già citati non sono stati ricontrollati; non si aggiungono nuove
affermazioni bibliografiche. Nessun report immutabile precedente modificato.

## Corrispondenza tra JSON, CSV e tracce

Tutti448 casi hanno identità univoca e corrispondono alle7leggi ×16modelli
×4run del freeze. La data del freeze precede START e COMPLETED. Dal solo NPZ
salvato sono ricalcolate le **stesse9 riduzioni già pubblicate** (RMSE/p95/errori
route, picchi/eccessi/difetti seat, minimo medio, corsa comando), senza invocare
il simulatore. {r['same_metric_reductions_checked']} confronti scalari,
massima differenza {r['max_saved_vs_trace_difference']:.2e} nelle unità dichiarate.
Confrontati tutti i campi JSON↔CSV, formula dello score con penalità originale,
aggregazione peggiore modello per run e obiettivi provvisori.0/448 con tutti
gli obiettivi rimane l'esito già consegnato. Limiti corsa/velocità e baseline
salvate sono coerenti. Non è una seconda valutazione delle alternative sul robot.

## Un controllo fallito: clock del log scambiato per dt FFI

Il primo audit è fallito perché divideva gli incrementi della correzione
normale per gli intervalli `event_time` salvati (t_wall della scrittura CSV),
chiamandoli impropriamente «actual tick». I sorgenti usano invece ffi_dt_ms,
che è misurato all'update e può essere clamped. Codice e log del controllo
fallito sono conservati in audit_saved_evidence_attempt1.py e
SAVED_EVIDENCE_ATTEMPT1_FAILURE.json; il criterio non è stato allargato.

Riletti soltanto i campi temporali, verificati gli hash originali e salvati
clock ausiliari separati: nessuna modifica alle tracce congelate. Gli incrementi
divisi per il **dt effettivamente usato dalla legge FFI** rispettano2mm/s in
tutti448casi, in accordo con le metriche originali. Dividendo invece per il
clock del log i massimi sono circa2,139/2,127/2,045/2,074mm/s (Z/L/S/T).
Le differenze fra intervalli raggiungono3,44/11,30/2,89/1,88ms; includono
diversa posizione della timestamp e possibile clamp, non una misura isolata
di latenza o jitter. Il diagnostico non identifica il consumo del target nel
servo o la velocità reale dei tool. Il limite riguarda la correzione normale
software, non tutta la velocità cartesiana dovuta anche alla route.

È una precisazione del dominio della verifica, non una nuova superiorità di
un controllore o la dimostrazione di una violazione hardware. Il replay usa
il clock dei log per collocare gli eventi e ffi_dt_ms per gli incrementi,
come dichiarato nel sorgente congelato: l'incertezza temporale rimane. Non
sono stati spostati eventi o rifatti i confronti per migliorare lo score.

## I fallimenti restano fallimenti

| Verifica di sviluppo conservata | Casi | ΔRMSE max [N] | Δbanda max [N] | Δpicco max [N] |
| --- | --- | --- | --- | --- |
{failures}

Le tolleranze originali dello studio avanzato sono0,02N RMSE/0,01N banda/0,05N
picco; la verifica dei piani dichiara anch'essa fallimento sul picco. Non sono
state allargate. Il completamento di un batch o l'identità della baseline non
risolvono tali fallimenti. Le successive verifiche a eventi esatti coprono solo
una parte dei modelli, come dichiarato nei relativi report.

## Come leggere documenti scritti in tranche diverse

Le sezioni «prossima tranche», «holdout non ancora analizzato» o «verifiche da
fare» nei vecchi report descrivono il momento di quella tranche. Non annullano
i risultati successivi. L'ordine e l'esito attuale sono in [READING_GUIDE.md](READING_GUIDE.md)
e STATE.json. I report congelati non vengono riscritti retroattivamente per
nascondere cosa fosse noto prima del holdout o di un fallimento numerico.

Rimangono tre limiti decisivi: modelli comando→moto→forza insufficientemente
predittivi, nessun confronto D-only matched con FFI invariata, nessuna garanzia
sui contatti individuali da un replay scalare. La [sintesi tesi](THESIS_SYNTHESIS.md)
mantiene distinte osservazione, replay condizionato e simulazione autonoma.
'''
    (OUT/'SAVED_EVIDENCE_AUDIT.md').write_text(text,encoding='utf-8')
    guide='''# Guida ai risultati della campagna

La campagna è ancora entro il termine del24/10 ore09:00Singapore. I confronti
offline e la sintesi sono completati; nessun nuovo controllore è validato sul
robot. Questo indice distingue la conclusione attuale dalle note storiche
dei report progressivi, senza modificare artifact congelati.

| Documento | Contenuto e ruolo |
| --- | --- |
| [THESIS_SYNTHESIS.md](THESIS_SYNTHESIS.md) | Punto di ingresso:12run osservate, confronto storico descrittivo, testo adattabile e limiti |
| [THESIS_EVIDENCE.json](THESIS_EVIDENCE.json) | Dieci affermazioni, livello di evidenza, fonti hash e limiti |
| [HOLDOUT_REPORT.md](HOLDOUT_REPORT.md) | Confronto finale congelato448casi, nessun tuning successivo; holdout retrospettivo |
| [DAMPING_REPORT.md](DAMPING_REPORT.md) | Priorità damping: modello fisico dichiarato, verifica predittiva fallita, nessun nuovo D scelto |
| [DAMPING_AB_PROTOCOL.md](DAMPING_AB_PROTOCOL.md) | Futuro D-only a FFI fissa; preparato e non eseguito |
| [MINIMUM_IDENTIFICATION_PROTOCOL.md](MINIMUM_IDENTIFICATION_PROTOCOL.md) | Dati sincronizzati e prove minime necessarie |
| [SIGNAL_PIPELINE_REPORT.md](SIGNAL_PIPELINE_REPORT.md) | Media ATI già presente, timestamp e limiti del log; filtri ideali separati dal robot |
| [EMPIRICAL_CONTROL_REPORT.md](EMPIRICAL_CONTROL_REPORT.md) | Free simulation vs one-step e primo studio controllori su sviluppo |
| [ADVANCED_CONTROL_REPORT.md](ADVANCED_CONTROL_REPORT.md) | PD/PID/allocazione e stress autonomi; fallimenti di raffinamento conservati |
| [PROFILE_PREVIEW_REPORT.md](PROFILE_PREVIEW_REPORT.md) | Preview disponibile e fattibilità del solo comando nominale |
| [PLANNED_REFERENCE_REPORT.md](PLANNED_REFERENCE_REPORT.md) | Piani/riferimenti, raffinamento fallito, verifica parziale a eventi esatti |
| [LINEAR_ROBUSTNESS_REPORT.md](LINEAR_ROBUSTNESS_REPORT.md) | Poli e sensibilità solo del modello locale non limitato; non certifica il robot |
| [SAVED_EVIDENCE_AUDIT.md](SAVED_EVIDENCE_AUDIT.md) | Coerenza tracce/metriche/CSV/hash e distinzione delle tranche storiche |
| [RESULTS.md](RESULTS.md) | Registro narrativo progressivo, aggiornamenti recenti in apertura |

Per stato operativo usare [STATE.json](STATE.json); per storia delle esecuzioni
e verifiche [NIGHT_LOG.md](NIGHT_LOG.md). La configurazione delle sette leggi,
split e score non vanno ricavati da una vecchia shortlist: la fonte finale è
[FINAL_COMPARISON_FREEZE.json](FINAL_COMPARISON_FREEZE.json) con il suo manifest
e supplemento sorgenti. Le righe storiche «prossimo batch holdout» sono state
superate dal marker [HOLDOUT_EVALUATION_COMPLETED.json](HOLDOUT_EVALUATION_COMPLETED.json),
non sono ordini di rilanciarlo. Non scegliere nuovi parametri su quelle run.

I controlli software e SHA possono passare mentre il modello fisico non è
adeguato: non sommare queste due nozioni in un unico «tutto verificato».
Le soglie0,05/0,20/0,25N sono esplorative, non criteri di sicurezza/tesi.
Le12run selezionate non sono un tasso di successo12/12; `believed_mm` non è
metrologia esterna; la positività della media non certifica la presa bilaterale.

NVIDIA_RUNS, snapshot protetto, sorgenti di produzione e analisi precedenti
rimangono in sola lettura. Fonti e risultati della campagna sono conservati;
nessuna azione robot/ROS/deploy/push è stata eseguita dalla ricerca notturna.
'''
    (OUT/'READING_GUIDE.md').write_text(guide,encoding='utf-8')


if __name__=='__main__':main()
