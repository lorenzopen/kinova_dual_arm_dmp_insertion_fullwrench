"""Bounded development-only extension: stronger filtered PD and slew allocation.

This is still fixed-residual conditional replay, not an identified robot.
The v1 kernel is preserved in checkpoints/tranche_20261023_2314.zip.
"""
import sys
sys.dont_write_bytecode=True
import conditional_controllers as c
import itertools
import json
from datetime import datetime,timezone
import numpy as np
import matplotlib.pyplot as plt

OUT=c.OUT


def freeze():
    path=OUT/'advanced_controller_design.json'
    if path.exists():return json.loads(path.read_text())
    original=c.freeze()
    candidates=[dict(x,allocation='ff_priority',pd_rolloff_tau_s=0.) for x in original['candidates']]
    for kp,ki,kd,rolloff in itertools.product([.0001,.0003,.0008],[0.,.0004,.0008],
                                              [0.,.00003,.0001],[.03,.1]):
        family='P' if ki==kd==0 else 'PD' if ki==0 else 'PI' if kd==0 else 'PID'
        candidates.append(dict(id=f'{family}_P{kp*1000:g}_I{ki*1000:g}_D{kd*1000:g}_R{rolloff:g}',
            family=family,kp_m_N=kp,ki_m_N_s=ki,kd_m_s_N=kd,derivative_tau_s=.05,
            pd_rolloff_tau_s=rolloff,allocation='ff_priority'))
    # Predeclared allocation experiment: fixed controller set, not chosen on new results.
    allocation_ids=['I_P0_I1.6_D0','PI_P0.05_I0.8_D0','PID_P0.05_I0.8_D0.005',
                    'PD_P0.2_I0_D0.015','PID_P0.3_I0.8_D0.1_R0.1','PD_P0.3_I0_D0.03_R0.03']
    for rid in allocation_ids:
        cfg=next(x for x in candidates if x['id']==rid)
        for policy in ['feedback_priority','combined_total']:
            candidates.append(dict(cfg,id=rid+'__'+policy,allocation=policy))
    design=dict(created_utc=datetime.now(timezone.utc).isoformat(),candidates=candidates,
        models=original['models'],baseline_id=original['baseline_id'],dt_s=c.DT,
        score=original['score'],data_split='Only fixed 8 development runs, no holdout',
        rolloff='Causal backward-Euler low-pass on the combined P and filtered measurement-D term; I is not low-pass filtered. Initial PD state equals entry P. Tracking antiwindup uses achieved trim minus filtered PD and entry bias.',
        allocations=dict(ff_priority='Original FFI allocation.',feedback_priority='First project desired trim into feasible trim interval, then allocate remaining feasible sum rate to nominal FF.',
            combined_total='First project desired sum FF+trim into feasible total interval, then choose the closest equal-weight split to the two desired component positions subject to component and sum bounds.'),
        limits='All policies: component FF and trim speed2mm/s, sum speed2mm/s, FF+-9mm, trim+-3mm, sum+-12mm.',
        caveat=original['model_caveat']+' '+original['limitation'],
        no_claim='Neither stronger derivative nor allocation changes internal Cartesian cart_d. No robot recommendation. Development shortlist only.')
    with path.open('x',encoding='utf-8') as f:json.dump(design,f,indent=2)
    return design


def allocator(policies):
    policies=np.array(policies)[:,None]
    def update(ff,trim,current,want,trial,dt):
        step=c.st.SPEED*dt
        flo=np.maximum(-.009,ff-step);fhi=np.minimum(.009,ff+step)
        tlo=np.maximum(-.003,trim-step);thi=np.minimum(.003,trim+step)
        olo=np.maximum(-.012,current-step);ohi=np.minimum(.012,current+step)
        oldff,oldtrim,oldout,oldlimited=c.st.bounded_update(ff,trim,current,want,trial,dt)
        ft=np.clip(trial,np.maximum(tlo,olo-fhi),np.minimum(thi,ohi-flo))
        fff=np.clip(want,np.maximum(flo,olo-ft),np.minimum(fhi,ohi-ft))
        desired_sum=np.clip(want+trial,np.maximum(olo,flo+tlo),np.minimum(ohi,fhi+thi))
        cf=np.clip((desired_sum+want-trial)/2,np.maximum(flo,desired_sum-thi),np.minimum(fhi,desired_sum-tlo))
        ct=desired_sum-cf
        newff=np.where(policies=='feedback_priority',fff,np.where(policies=='combined_total',cf,oldff))
        newtrim=np.where(policies=='feedback_priority',ft,np.where(policies=='combined_total',ct,oldtrim))
        out=newff+newtrim
        assert np.max(abs(newff-ff))<=step+1e-11
        assert np.max(abs(newtrim-trim))<=step+1e-11
        assert np.max(abs(out-current))<=step+1e-11
        assert np.max(abs(newff))<=.009+1e-11 and np.max(abs(newtrim))<=.003+1e-11
        assert np.max(abs(out))<=.012+1e-11
        return newff,newtrim,out,abs(newtrim-trial)>1e-12
    return update


def run(p,design):
    return c.replay(p,design,dt=c.DT,update_fn=allocator([x['allocation'] for x in design['candidates']]))


def rank(rows,design):
    ranking=[]
    for cfg in design['candidates']:
        rr=[r for r in rows if r['candidate']==cfg['id']]
        runs=sorted({r['id'] for r in rr})
        scores=[max(r['score'] for r in rr if r['id']==rid) for rid in runs]
        ranking.append(dict(cfg,mean_run_worst_model_score=float(np.mean(scores)),worst_score=float(max(scores)),
            minimum_mean_hand_force_N=min(r['minimum_mean_hand_force_N'] for r in rr)))
    ranking.sort(key=lambda r:r['mean_run_worst_model_score'])
    return ranking


def software_checks(design):
    rng=np.random.default_rng(23442026);checks={}
    # Reachable states, varied dt and extreme desired positions; all allocator assertions active.
    for policy in ['ff_priority','feedback_priority','combined_total']:
        fn=allocator([policy]*100);ff=np.zeros((100,4));trim=ff.copy();out=ff.copy()
        for tick in range(200):
            dt=rng.uniform(.005,.1);want=rng.uniform(-.02,.02,(100,4));trial=rng.uniform(-.01,.01,(100,4))
            ref=c.st.bounded_update(ff,trim,out,want,trial,dt)
            result=fn(ff,trim,out,want,trial,dt)
            if policy=='ff_priority':
                for a,b in zip(result,ref):assert np.array_equal(a,b)
            ff,trim,out,_=result
        checks[policy]=dict(checked_updates=80000,all_component_and_total_bounds=True)
    return checks


def main():
    design=freeze();checks=software_checks(design)
    frozen=json.loads((OUT/'frozen_design.json').read_text())
    catalogue=json.loads((OUT.parent/'archive_review/run_catalog.json').read_text())
    reference={(r['id'],r['candidate'],r['model']):r for r in json.loads((OUT/'conditional_controller_metrics.json').read_text())}
    rows=[];validations=[];regression=0.;run_ids=[]
    # Save only compact central-model traces for every candidate, not full model cubes.
    central=next(i for i,m in enumerate(design['models']) if m['id']=='delay_k680_d0.18')
    for piece in 'ZLST':
        for rr in frozen['split'][piece]['development']:
            rec=next(r for r in catalogue if r['id']==rr['id']);p=c.old_replay.prepare(rec)
            metrics,val,tr=run(p,design);rows.extend(metrics);validations.append(val);run_ids.append(rec['id'])
            for row in metrics:
                key=(row['id'],row['candidate'],row['model'])
                if key in reference:
                    for field in ['route_rmse_N','route_band_rms_N','route_high_band_rms_N','first_seat_peak_N']:
                        regression=max(regression,abs(row[field]-reference[key][field]))
            np.savez_compressed(OUT/(rec['id']+'_advanced_central.npz'),t=tr['t'],target=tr['target'],
                force=tr['force'][:,:,central],commands=tr['commands'][:,:,central],event_time=tr['event_time'],
                route0=p['route0'],route1=p['route1'],candidate_ids=np.array([x['id'] for x in design['candidates']]))
            print('Advanced',piece,rec['id'],len(metrics),'scenarios; regression max',regression,flush=True)
            del tr,p
    assert regression<1e-10,regression
    ranking=rank(rows,design)
    c.dp.dump('advanced_controller_metrics.json',rows)
    with (OUT/'advanced_controller_metrics.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=c.dp.csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
    c.dp.dump('advanced_controller_ranking.json',ranking)
    c.dp.dump('advanced_software_validation.json',dict(allocators=checks,
        all_original_5888_scenarios_max_metric_difference_N=regression,replay=validations,
        scenario_count=len(rows),candidate_count=len(design['candidates'])))
    selected=[next(x for x in ranking if x['id']==design['baseline_id'])]
    for family in ['P','PI','PD','PID']:
        selected.append(next(x for x in ranking if x['family']==family and x['allocation']=='ff_priority'))
    for policy in ['feedback_priority','combined_total']:
        selected.append(next(x for x in ranking if x['allocation']==policy))
    c.dp.dump('advanced_shortlist_development_only.json',dict(shortlist=selected,
        caveat='Still conditional replay; holdout untouched, no validated robot controller.'))
    plot(design,selected,frozen)
    print('BEST',json.dumps(ranking[:8]),flush=True)


def plot(design,selected,frozen):
    fig,axes=plt.subplots(2,2,figsize=(14,9));fig.subplots_adjust(left=.08,right=.98,top=.77,bottom=.16,hspace=.44,wspace=.23)
    fig.suptitle('PD più forte e priorità del comando: confronto condizionato',x=.08,ha='left',y=.98,fontsize=17,weight='bold')
    fig.text(.08,.925,'Solo sviluppo · residuo registrato imposto · modello centrale 680 N/m/mano, ritardo 180 ms',fontsize=10)
    colors=['#253747','#dd941f','#1675bb','#40a69b','#9564b2','#cf4e59','#875032']
    for ax,piece in zip(axes.flat,'ZLST'):
        rid=frozen['split'][piece]['development'][0]['id']
        tr=np.load(OUT/(rid+'_advanced_central.npz'));t=tr['t']-tr['route0'];end=float(tr['route1']-tr['route0'])
        keep=(t>=end-2)&(t<=end+1.6)
        for cfg,color in zip(selected,colors):
            idx=list(tr['candidate_ids']).index(cfg['id'])
            ax.plot(t[keep],tr['force'][keep,idx],color=color,lw=1,label=cfg['id'])
        ax.plot(t[keep],tr['target'][keep],'k--',lw=1,label='Target originale/mano')
        ax.axvline(end,color='gray',lw=.8,alpha=.6)
        ax.set(title=piece+' · passaggio route→slow_seat',xlabel='Tempo da inizio route [s]',ylabel='Forza media mani [N]')
        ax.grid(alpha=.2)
    handles,labels=axes.flat[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper left',bbox_to_anchor=(.075,.90),ncol=2,fontsize=7,frameon=False)
    fig.text(.08,.035,'Alternative simulate con limiti identici: totale 2 mm/s, ±12 mm; trim 2 mm/s, ±3 mm.\n'
        'La derivata esterna sulla forza non è il damping cartesiano interno. Nessuna previsione di slip o riuscita.\n'
        'Le quattro run riservate restano escluse: queste curve servono a confrontare ipotesi, non a validare il robot.',fontsize=10,linespacing=1.5)
    fig.savefig(OUT/'08_PD_forte_e_priorita.png',dpi=150);plt.close(fig)


if __name__=='__main__':main()
