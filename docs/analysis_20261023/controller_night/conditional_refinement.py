"""Half-grid check on preselected representative conditional replay scenarios."""
import conditional_controllers as c
import numpy as np
import json


def main():
    design=c.freeze();short=json.loads((c.OUT/'conditional_shortlist_development_only.json').read_text())['shortlist']
    selected={design['baseline_id']}|{r['id'] for r in short}
    original=json.loads((c.OUT/'conditional_controller_metrics.json').read_text())
    reference={(r['id'],r['candidate'],r['model']):r for r in original}
    compact=dict(design,candidates=[r for r in design['candidates'] if r['id'] in selected],
                 models=[m for m in design['models'] if m['id'] in ['delay_k480_d0.12','delay_k680_d0.18','delay_k1500_d0.23','resonant680_d18_z0.3']])
    frozen=json.loads((c.OUT/'frozen_design.json').read_text());catalogue=json.loads((c.OUT.parent/'archive_review/run_catalog.json').read_text())
    c.DT=.0025;c.old_replay.DT=.0025
    diffs=[]
    for piece in 'ZLST':
        rid=frozen['split'][piece]['development'][0]['id'];rec=next(r for r in catalogue if r['id']==rid)
        p=c.old_replay.prepare(rec);rows,validation,_=c.replay(p,compact,dt=c.DT)
        for row in rows:
            base=reference[(row['id'],row['candidate'],row['model'])]
            difference={key:row[key]-base[key] for key in ['route_rmse_N','route_band_rms_N','route_high_band_rms_N',
                         'first_seat_peak_N','first_seat_max_positive_error_N','minimum_mean_hand_force_N']}
            diffs.append(dict(id=rid,candidate=row['candidate'],model=row['model'],difference_2p5_minus_5ms=difference))
        print('Refined',piece,len(rows),flush=True)
    maximum={key:max(abs(r['difference_2p5_minus_5ms'][key]) for r in diffs) for key in diffs[0]['difference_2p5_minus_5ms']}
    result=dict(scenarios=len(diffs),maximum_absolute_metric_difference_N=maximum,checks=diffs)
    c.dp.dump('conditional_numerical_refinement.json',result)
    assert maximum['route_rmse_N']<.03,maximum
    assert maximum['route_band_rms_N']<.01,maximum
    assert maximum['first_seat_peak_N']<.05,maximum
    print('Refinement differences',maximum,flush=True)


if __name__=='__main__':main()
