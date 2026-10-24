"""Numerical convergence check for the newly selected development hypotheses."""
import advanced_controllers as a
import json
import sys


def main():
    c=a.c;design=a.freeze()
    chosen=json.loads((a.OUT/'advanced_shortlist_development_only.json').read_text())['shortlist']
    ids={r['id'] for r in chosen}
    # Include a strong PD even when it loses the aggregate selection.
    ids.add('PD_P0.3_I0_D0.1_R0.1')
    compact=dict(design,candidates=[r for r in design['candidates'] if r['id'] in ids],
        models=[m for m in design['models'] if m['id'] in ['delay_k480_d0.12','delay_k680_d0.18',
            'delay_k1500_d0.23','resonant680_d18_z0.3']])
    original=json.loads((a.OUT/'advanced_controller_metrics.json').read_text())
    lookup={(r['id'],r['candidate'],r['model']):r for r in original}
    frozen=json.loads((a.OUT/'frozen_design.json').read_text())
    catalogue=json.loads((a.OUT.parent/'archive_review/run_catalog.json').read_text())
    dt=float(sys.argv[1]) if len(sys.argv)>1 else .0025
    assert dt in [.0025,.00125]
    c.DT=dt;c.old_replay.DT=dt;diffs=[]
    for piece in 'ZLST':
        rid=frozen['split'][piece]['development'][0]['id'];rec=next(r for r in catalogue if r['id']==rid)
        p=c.old_replay.prepare(rec);rows,validation,_=a.run(p,compact)
        for row in rows:
            ref=lookup[(row['id'],row['candidate'],row['model'])]
            diffs.append(dict(id=rid,candidate=row['candidate'],model=row['model'],difference={
                key:row[key]-ref[key] for key in ['route_rmse_N','route_band_rms_N','route_high_band_rms_N',
                    'first_seat_peak_N','minimum_mean_hand_force_N']}))
        print('Advanced refined',piece,len(rows),flush=True)
    maximum={key:max(abs(r['difference'][key]) for r in diffs) for key in diffs[0]['difference']}
    name='advanced_numerical_refinement.json' if dt==.0025 else 'advanced_numerical_refinement_1p25ms.json'
    passed=maximum['route_rmse_N']<.03 and maximum['route_band_rms_N']<.01 and maximum['first_seat_peak_N']<.05
    result=dict(scenarios=len(diffs),dt_s=dt,reference_dt_s=.005,
        maximum_absolute_difference_N=maximum,cases=diffs,original_tolerance_passed=passed)
    if dt==.00125:
        half=json.loads((a.OUT/'advanced_numerical_refinement.json').read_text())
        old={(r['id'],r['candidate'],r['model']):r['difference'] for r in half['cases']}
        result['max_difference_vs_2p5ms_N']={key:max(abs(r['difference'][key]-old[(r['id'],r['candidate'],r['model'])][key]) for r in diffs) for key in maximum}
    c.dp.dump(name,result)
    # Convergence tolerance predates evaluation; errors retained if it fails.
    assert maximum['route_rmse_N']<.03 and maximum['route_band_rms_N']<.01
    assert maximum['first_seat_peak_N']<.05
    print('Advanced refinement',maximum,flush=True)


if __name__=='__main__':main()
