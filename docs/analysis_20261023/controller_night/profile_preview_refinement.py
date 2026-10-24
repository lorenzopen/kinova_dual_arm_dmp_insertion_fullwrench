"""Half-grid check of frozen preview hypotheses, plus logged-input causality."""
import profile_preview as p
import json
import numpy as np


def main():
    design=p.freeze();groups=json.loads((p.OUT/'profile_preview_grouped_ranking.json').read_text())
    chosen=[]
    for base in ['I_P0_I1.6_D0','PI_P0.1_I0.8_D0_R0.1']:
        for mode in ['shift','rise_envelope']:
            chosen.append(next(r for r in groups if r['base_controller']==base and r['preview']==mode))
    ids={rid for group in chosen for rid in group['candidate_ids']}
    ids|={design['baseline_id'],'PI_P0.1_I0.8_D0_R0.1','I_P0_I1.6_D0__shift_H1.2_O0'}
    compact=dict(design,candidates=[x for x in design['candidates'] if x['id'] in ids],
        models=[m for m in design['models'] if m['id'] in ['delay_k480_d0.12','delay_k680_d0.18',
            'delay_k1500_d0.23','resonant680_d18_z0.3']])
    p.c.dp.dump('profile_preview_refinement_selection.json',dict(groups=chosen,candidate_ids=sorted(ids),
        note='Selected after development ranking, before refinement; all clock offsets retained. Not a final holdout shortlist.'))
    original=json.loads((p.OUT/'profile_preview_metrics.json').read_text())
    lookup={(r['id'],r['candidate'],r['model']):r for r in original}
    frozen=json.loads((p.OUT/'frozen_design.json').read_text())
    catalogue=json.loads((p.OUT.parent/'archive_review/run_catalog.json').read_text())
    p.c.DT=.0025;p.c.old_replay.DT=.0025;diffs=[];causality=[]
    for piece in 'ZLST':
        rid=frozen['split'][piece]['development'][0]['id'];rec=next(r for r in catalogue if r['id']==rid)
        prepared=p.c.old_replay.prepare(rec);profile,_=p.read_profile(rec);nominal,_=p.wants(prepared,compact,profile)
        cut=len(prepared['ix'])//2;row_cut=int(prepared['ix'][cut]);changed=dict(prepared)
        changed['d']=dict(prepared['d']);changed['phase']=prepared['phase'].copy()
        changed['d']['phase']=prepared['d']['phase'].copy();changed['d']['t_wall']=prepared['d']['t_wall'].copy()
        changed['d']['ffi_ff_want_mm']=prepared['d']['ffi_ff_want_mm'].copy()
        changed['d']['phase'][row_cut:]='slow_seat';changed['phase'][cut:]='slow_seat'
        changed['d']['t_wall'][row_cut:]+=10.;changed['d']['ffi_ff_want_mm'][row_cut:]+=100.
        # Measured force does not participate in nominal preview construction.
        changed['y0']=np.full_like(prepared['y0'],12345.)
        changed_wants,_=p.wants(changed,compact,profile)
        prefix_error=float(abs(nominal[:cut]-changed_wants[:cut]).max())
        assert prefix_error==0
        causality.append(dict(id=rid,maximum_prefix_change_m=prefix_error,
            changed_inputs='Future logged phases/time/nominal and all measured-force values. Offline profile unchanged.'))
        rows,_,_=p.c.replay(prepared,compact,dt=p.c.DT,nominal_wants=nominal)
        for row in rows:
            ref=lookup[(row['id'],row['candidate'],row['model'])]
            diffs.append(dict(id=rid,candidate=row['candidate'],model=row['model'],difference={key:row[key]-ref[key] for key in
                ['route_rmse_N','route_band_rms_N','route_high_band_rms_N','first_seat_peak_N','minimum_mean_hand_force_N']}))
        print('Refined preview',piece,len(rows),flush=True)
    maximum={key:max(abs(r['difference'][key]) for r in diffs) for key in diffs[0]['difference']}
    passed=maximum['route_rmse_N']<.03 and maximum['route_band_rms_N']<.01 and maximum['first_seat_peak_N']<.05
    p.c.dp.dump('profile_preview_refinement.json',dict(scenarios=len(diffs),maximum_absolute_difference_N=maximum,
        passed=passed,causal_prefix_checks=causality,cases=diffs))
    assert passed,maximum
    print('Preview refinement',maximum,flush=True)


if __name__=='__main__':main()
