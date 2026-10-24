"""Freeze final retrospective evaluation BEFORE opening reserved run CSVs.

This freezes comparison hypotheses, NOT a robot controller recommendation.
Selection uses only already-saved development rankings. Exclusive output.
"""
import sys
sys.dont_write_bytecode=True
import advanced_controllers as a
import hashlib
import json
from datetime import datetime,timezone
from pathlib import Path

OUT=a.OUT


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    dest=OUT/'FINAL_COMPARISON_FREEZE.json'
    if dest.exists():
        saved=json.loads(dest.read_text());print('Existing final comparison freeze retained:',saved['created_utc']);return
    design=a.freeze();ranking=json.loads((OUT/'advanced_controller_ranking.json').read_text())
    ids=[design['baseline_id'],'FF_only_P0_I0_D0']
    reasons={design['baseline_id']:'Actual recorded FFI baseline, fixed.',
        'FF_only_P0_I0_D0':'Nominal-only diagnostic with same entry state and fixed entry trim bias.'}
    for family in ['P','PI','PD','PID']:
        best=next(x for x in ranking if x['family']==family and x['allocation']=='ff_priority')
        ids.append(best['id']);reasons[best['id']]='Best development worst-model score in family '+family+' under original FF priority; no holdout inspected.'
    extra='PI_P0.05_I0.8_D0';ids.append(extra)
    reasons[extra]='Simpler unfiltered small-P PI nearly tied with baseline in initial development screen; compare to filtered-P PI without selecting on holdout.'
    ids=list(dict.fromkeys(ids))
    configs=[next(x for x in design['candidates'] if x['id']==rid) for rid in ids]
    frozen=json.loads((OUT/'frozen_design.json').read_text())
    files=[OUT/'conditional_controllers.py',OUT/'advanced_controllers.py',OUT/'frozen_design.json',
        OUT/'conditional_controller_design.json',OUT/'advanced_controller_design.json',
        OUT.parent/'moving_average_sim/simulate.py',OUT.parent/'archive_review/analyze.py',
        Path(a.c.old_replay.a.previous.__file__),a.c.ROOT/'kinova_dual_arm_dmp_insertion/insert_ffi.py']
    fingerprints=[dict(path=str(path.resolve()),sha256=sha(path)) for path in files]
    result=dict(created_utc=datetime.now(timezone.utc).isoformat(),purpose='Final fixed-parameter retrospective comparison; no deployment recommendation.',
        candidates=configs,selection_reasons=reasons,models=design['models'],baseline_id=design['baseline_id'],
        data_split=frozen['split'],
        holdout_limit='Third chronological success per piece, already seen in earlier descriptive and moving-average work. Reserved only against THIS night tuning, not prospective independent validation.',
        time_grid_s=.0025,source_fingerprints=fingerprints,
        primary_evaluation='One locked batch,7 candidates x16 hypothetical models x4 reserved runs=448 cases. Do not retune parameters or choose a favorable model after reading holdout. Failure/negative results retained.',
        scheduling='Original nominal and original recorded target, original FF priority, fixed phase/event schedule from logs, no preview or replanning in the primary comparison.',
        limits='FF/trim/sum speeds2mm/s; FF+-9mm trim+-3mm total+-12mm. Same first recorded physical FF/trim/output for all; P/PD and nominal-only use fixed entry bias, no hidden I.',
        score=design['score'],
        provisional_thresholds=dict(route_rmse_N_hand=.2,route_band_rms_N_hand=.05,first_seat_positive_error_N_hand=.25,
            note='Exploratory goals, not agreed acceptance or safety limits.'),
        secondary_diagnostics=['Original-target RMSE,p95,positive/negative errors','Raw force and target-subtracted error bands0.2-0.8,0.8-2.5,2.5-10Hz',
            'Observed left/right force minima and deviation from common scalar target; not actual friction margin',
            'Alternative force mean-only minima; no inference of bilateral contact','Rate/trim limiting fractions and command bounds',
            'Maximum200ms trailing mean force/positive error in first-seat window as an additional less sample-sensitive transient feature, without replacing the frozen score'],
        numerical_limit='Moderate candidates passed representative development refinement, but point maxima can remain grid-sensitive with ZOH residual. Report raw maxima and this uncertainty; do not turn small score gaps into a robot recommendation.',
        governance='Create exclusive started/progress marker before first reserved CSV analysis; save each completed run, resume only missing runs after interruption. Reports can regenerate from saved results, not by retuning. Verify source fingerprints, original CSV hashes and protection manifest.',
        excluded=dict(planned_references='No full-wrench margin validation and no robust improvement. Minimax lowers original normal; planners remain diagnostic only.',
            stronger_PD='Unfavorable robust tradeoffs and flagged numerical sensitivity.',
            explicit_delay_compensation='No identified plant passed predictive gate; no supported compensator candidate.',
            changed_cart_d='No controlled FFI D-only logs and actual internal servo source missing; future protocol only.'))
    assert len(configs)==7
    with dest.open('x',encoding='utf-8') as handle:json.dump(result,handle,indent=2)
    manifest=dict(file=dest.name,sha256=sha(dest),created_utc=result['created_utc'],holdout_CSVs_read_to_create_freeze=False)
    with (OUT/'FINAL_COMPARISON_FREEZE_manifest.json').open('x',encoding='utf-8') as handle:json.dump(manifest,handle,indent=2)
    print('Frozen final comparison before holdout:',len(configs),'candidates; SHA256',manifest['sha256'],flush=True)
    print('IDs',ids,flush=True)


if __name__=='__main__':main()
