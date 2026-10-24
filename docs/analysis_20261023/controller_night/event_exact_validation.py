"""Independent real-FFI update check and saved critical peak traces."""
import planned_reference as r
import json
import numpy as np


def main():
    OUT=r.OUT;c=r.c;design=r.freeze();frozen=json.loads((OUT/'frozen_design.json').read_text())
    catalogue=json.loads((OUT.parent/'archive_review/run_catalog.json').read_text())
    largest=0.;updates=0
    for piece in 'ZLST':
        rid=frozen['split'][piece]['development'][0]['id'];rec=next(x for x in catalogue if x['id']==rid)
        c.old_replay.DT=.005;prepared=c.old_replay.prepare(rec)
        with np.load(OUT/(rid+'_event_exact_delay.npz')) as archive:trace={key:archive[key] for key in archive.files}
        profile,_=r.p.read_profile(rec);_,feedback,*_=r.references(prepared,design,r.plan_profiles(profile))
        for i,cfg in enumerate(design['candidates']):
            if cfg['base_controller']!='I_P0_I1.6_D0':continue
            if cfg['plan']!='logged' and cfg['feedback_reference']!='planned':continue
            for m in range(len(trace['model_ids'])):
                ctrl=prepared['make']()
                for k in range(1,len(prepared['te'])):
                    result=ctrl.update(float(feedback[k,i]),float(2*trace['control_force_N'][k,i,m]),
                        float(prepared['clock'][k]),ctrl.output)*1000
                    largest=max(largest,abs(result-trace['event_commands_mm'][k,i,m]));updates+=1
    assert largest<1e-9
    c.dp.dump('event_exact_software_validation.json',dict(actual_production_I_updates=updates,
        maximum_command_difference_mm=largest,passed=True,
        scope='Pure I with planned nominal and planned feedback, plus logged baseline; all3 delay models and4 development shapes. Production classes AST-extracted, no ROS.'))
    rid=frozen['split']['Z']['development'][0]['id'];rec=next(x for x in catalogue if x['id']==rid)
    critical='PI_P0.1_I0.8_D0_R0.1__ABS_majorant_FBplanned_O0.04'
    compact=dict(design,candidates=[x for x in design['candidates'] if x['id'] in [design['baseline_id'],critical]],
        models=[x for x in design['models'] if x['id']=='delay_k1500_d0.23'])
    saved={}
    for dt,name in [(.005,'grid5'),(.0025,'grid2p5')]:
        c.DT=dt;c.old_replay.DT=dt;prepared=c.old_replay.prepare(rec)
        profile,_=r.p.read_profile(rec);nominal,feedback,*_=r.references(prepared,compact,r.plan_profiles(profile))
        rows,_,trace=c.replay(prepared,compact,dt=dt,nominal_wants=nominal,feedback_targets=feedback)
        idx=next(i for i,x in enumerate(compact['candidates']) if x['id']==critical)
        saved[name+'_time_s']=trace['t'];saved[name+'_force_N']=trace['force'][:,idx,0]
        saved[name+'_peak_N']=next(x['first_seat_peak_N'] for x in rows if x['candidate']==critical)
    np.savez_compressed(OUT/'critical_peak_grid_traces.npz',**saved)
    print('Event-exact actual FFI validation',updates,'updates, max mm',largest,flush=True)


if __name__=='__main__':main()
