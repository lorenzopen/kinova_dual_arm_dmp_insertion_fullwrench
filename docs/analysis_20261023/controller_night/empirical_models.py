"""Cross-run empirical model checks; closed-loop data are not causal ID.

Eight development runs only. All preprocessing is causal and uses ZOH sampled
logged data. Free response is computed without feeding measured output back to
the model. One-step ARX prediction is reported separately as a diagnostic.
"""
import hashlib
import itertools
import json
from datetime import datetime, timezone
from pathlib import Path
import damping_physics as dp
import numpy as np
from scipy.signal import butter,sosfilt,lfilter,cont2discrete
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

OUT=dp.OUT;ROOT=dp.ROOT;a=dp.a;DT=.01


def freeze():
    path=OUT/'empirical_design.json'
    if path.exists():return json.loads(path.read_text())
    fixed=json.loads((OUT/'frozen_design.json').read_text())
    models=[]
    for delay,tau in itertools.product(np.arange(0,.351,.01),[0,.03,.06,.12,.2,.35]):
        if tau==0:b,den=[1.],[1.];family='delay'
        else:
            pole=np.exp(-DT/tau);b,den=[0.,float(1-pole)],[1.,float(-pole)];family='first_order'
        models.append(dict(id=f'lag_d{round(delay*1000)}_t{round(tau*1000)}',family=family,
                           delay_samples=round(delay/DT),tau_s=tau,b=b,a=den))
    for delay,hz,zeta in itertools.product([0,.06,.12,.18,.24],[1.,1.5,2.5,4.,6.],[.2,.5,.8,1.2]):
        omega=2*np.pi*hz;b,den,_=cont2discrete(([omega**2],[1,2*zeta*omega,omega**2]),DT)
        b=b[0];b[abs(b)<1e-13]=0
        models.append(dict(id=f'second_d{round(delay*1000)}_f{hz:g}_z{zeta:g}',family='second_order',
                           delay_samples=round(delay/DT),frequency_Hz=hz,damping_ratio=zeta,
                           b=b.tolist(),a=den.tolist()))
    design=dict(created_utc=datetime.now(timezone.utc).isoformat(),dt_s=DT,
        development=[entry for piece in 'ZLST' for entry in fixed['split'][piece]['development']],
        preprocessing=dict(resample='causal ZOH, never linear interpolation of future samples',
                           closing_axis='opposing normal from the FIRST observed route pose, fixed thereafter',
                           filter='causal Butterworth bandpass order 2, 0.3-8 Hz; signal minus first sample; zero filter state',
                           score_window='route from 3 seconds through last sample minus 0.5 seconds'),
        models=models,arx=dict(order=2,input_taps=2,delays_samples=list(range(0,36,2)),
                              ridge=[0.,.001,.1],require_pole_modulus_below=1.),
        fitting='equal normalized error weight per run; positive pooled gain for LTI; stable ARX selected by FREE simulation training error, not one-step error',
        validation='8 leave-one-run-out folds; fit on 7 development runs then simulate eighth without observed outputs; four original holdout runs unused',
        provisional_model_gate=dict(median_crossrun_force_r2_min=.7,worst_crossrun_force_r2_min=.5),
        caveat='Retrospective development cross-validation of recorded command sequences under the same FFI; no independent excitation, no change-of-controller causal guarantee. DC/absolute force is not identified by the band-limited model.')
    with path.open('x',encoding='utf-8') as f:json.dump(design,f,indent=2)
    return design


def prepare(design):
    records=[];sos=butter(2,[.3,8],btype='bandpass',fs=1/DT,output='sos')
    for rec in design['development']:
        raw=(ROOT/rec['source']).read_bytes();assert hashlib.sha256(raw).hexdigest()==rec['csv_sha256']
        d=a.previous.load(ROOT/rec['source']);ix=np.flatnonzero(d['phase']=='route')
        rel=d['t_wall'][ix]-d['t_wall'][ix[0]];t=np.arange(0,rel[-1],DT)
        pick=np.searchsorted(rel,t,side='right')-1
        normals=[R.from_quat([d['mq'+c+side][ix[0]] for c in 'xyzw']).apply([1.,0,0]) for side in 'LR']
        n=normals[0]-normals[1];n/=np.linalg.norm(n)
        cmd=[np.column_stack([d[c+side][ix] for c in 'xyz']) for side in 'LR']
        meas=[np.column_stack([d['m'+c+side][ix] for c in 'xyz']) for side in 'LR']
        variables=[.5*1000*(cmd[0]-cmd[1])@n,.5*1000*(meas[0]-meas[1])@n,d['fn_N'][ix]/2]
        values=[sosfilt(sos,v[pick]-v[pick][0]) for v in variables]
        keep=(t>=3)&(t<=t[-1]-.5)
        records.append(dict(**rec,piece=rec['id'][4] if rec['id'].startswith('F75_') else 'S',
                            t=t,u=values[0],y=values[1],force=values[2],keep=keep,
                            motion_variance=float(np.mean(values[1][keep]**2)),
                            force_variance=float(np.mean(values[2][keep]**2))))
    assert len(records)==8
    return records


def shifted(u,delay):
    return np.r_[np.zeros(delay),u[:-delay]] if delay else u.copy()


def shape(model,u):
    return lfilter(model['b'],model['a'],shifted(u,model['delay_samples']))


def assess(y,pred,keep):
    observed=y[keep];prediction=pred[keep];residual=observed-prediction
    mse=float(np.mean(residual**2));variance=float(np.var(observed))
    return dict(r2=1-mse/variance,rmse=float(np.sqrt(mse)),
                normalized_mse=mse/float(np.mean(observed**2)))


def fit_force_gain(records,train):
    numerator=denominator=0.
    for i in train:
        r=records[i];k=r['keep'];weight=1/(k.sum()*r['force_variance'])
        numerator+=weight*(r['y'][k]@r['force'][k]);denominator+=weight*(r['y'][k]@r['y'][k])
    return max(0,float(numerator/denominator))


def fit_lti(records,train,models,cache):
    best={}
    for model in models:
        numerator=denominator=0
        for i in train:
            r=records[i];keep=r['keep'];weight=1/(keep.sum()*r['motion_variance'])
            x=cache[model['id']][i][keep];y=r['y'][keep]
            numerator+=weight*(x@y);denominator+=weight*(x@x)
        gain=float(numerator/denominator) if denominator>1e-20 else -1
        if gain<=0:continue
        score=float(np.mean([assess(records[i]['y'],gain*cache[model['id']][i],records[i]['keep'])['normalized_mse'] for i in train]))
        family=model['family']
        if family not in best or score<best[family]['training_score']:
            best[family]=dict(**model,gain=gain,training_score=score)
    return best


def arx_features(r,delay):
    y=r['y'];u=shifted(r['u'],delay)
    # All columns at sample k use only y before k and u at/before k.
    z=np.column_stack([np.r_[0,y[:-1]],np.r_[0,0,y[:-2]],u,np.r_[0,u[:-1]]])
    return z


def arx_free(model,u):
    return lfilter([model['coefficients'][2],model['coefficients'][3]],
                   [1.,-model['coefficients'][0],-model['coefficients'][1]],shifted(u,model['delay_samples']))


def fit_arx(records,train,design,features):
    best=None;rejections=0
    for delay,ridge in itertools.product(design['arx']['delays_samples'],design['arx']['ridge']):
        zz=np.zeros((4,4));zy=np.zeros(4)
        for i in train:
            r=records[i];k=r['keep'];z=features[delay][i][k];y=r['y'][k]
            weight=1/(k.sum()*r['motion_variance'])
            zz+=weight*z.T@z;zy+=weight*z.T@y
        reg=ridge*np.trace(zz)/4
        try:coef=np.linalg.solve(zz+reg*np.eye(4),zy)
        except np.linalg.LinAlgError:rejections+=1;continue
        poles=np.roots([1,-coef[0],-coef[1]])
        if np.max(abs(poles))>=1:rejections+=1;continue
        model=dict(family='ARX2',id=f'arx_d{delay}_ridge{ridge:g}',delay_samples=delay,
                   ridge=ridge,coefficients=coef.tolist(),max_pole_modulus=float(np.max(abs(poles))))
        free=[assess(records[i]['y'],arx_free(model,records[i]['u']),records[i]['keep'])['normalized_mse'] for i in train]
        model['training_score']=float(np.mean(free))
        if best is None or model['training_score']<best['training_score']:best=model
    return best,rejections


def run(design,records):
    models=design['models'];cache={m['id']:[shape(m,r['u']) for r in records] for m in models}
    features={delay:[arx_features(r,delay) for r in records] for delay in design['arx']['delays_samples']}
    rows=[];predictions={};selections=[]
    for test in range(8):
        train=[i for i in range(8) if i!=test];force_gain=fit_force_gain(records,train)
        selected=fit_lti(records,train,models,cache)
        arx,rejections=fit_arx(records,train,design,features)
        if arx:selected['ARX2']=arx
        for family,model in selected.items():
            r=records[test]
            pred=(arx_free(model,r['u']) if family=='ARX2' else model['gain']*cache[model['id']][test])
            motion=assess(r['y'],pred,r['keep']);force=assess(r['force'],force_gain*pred,r['keep'])
            constitutive=assess(r['force'],force_gain*r['y'],r['keep'])
            row=dict(id=r['id'],piece=r['piece'],family=family,trained_on=7,
                     selected_model=model,force_gain_N_per_mm_projected_closure=force_gain,
                     motion_free=motion,force_free=force,force_given_measured_motion=constitutive)
            if family=='ARX2':
                one_step=features[model['delay_samples']][test]@np.array(model['coefficients'])
                row['motion_one_step']=assess(r['y'],one_step,r['keep'])
            rows.append(row);predictions[(test,family)]=pred
        selections.append(dict(test_id=records[test]['id'],ARX_unstable_or_singular_candidates_rejected=rejections))
        print('CV completed',test+1,records[test]['piece'],flush=True)
    dp.dump('empirical_crossrun_metrics.json',rows);dp.dump('empirical_crossrun_selection.json',selections)
    alltrain=list(range(8));final=fit_lti(records,alltrain,models,cache);arx,rejections=fit_arx(records,alltrain,design,features)
    if arx:final['ARX2']=arx
    dp.dump('empirical_models_fit_development.json',dict(models=final,
        force_gain_N_per_mm_projected_closure=fit_force_gain(records,alltrain),
        caveat='Fit on all eight development runs after LOO study. No holdout consumed; band-limited under same control regime; not a causal robot model.'))
    summary=[]
    for family in ['delay','first_order','second_order','ARX2']:
        rr=[r for r in rows if r['family']==family]
        if not rr:continue
        item=dict(family=family)
        for label in ['motion_free','force_free','force_given_measured_motion']:
            for metric in ['r2','rmse']:
                item[label+'_'+metric+'_min_median_max']=np.percentile([r[label][metric] for r in rr],[0,50,100]).tolist()
        item['passes_provisional_gate']=(item['force_free_r2_min_median_max'][0]>=.5 and item['force_free_r2_min_median_max'][1]>=.7)
        if family=='ARX2':item['motion_one_step_r2_min_median_max']=np.percentile([r['motion_one_step']['r2'] for r in rr],[0,50,100]).tolist()
        summary.append(item)
    dp.dump('empirical_crossrun_summary.json',summary)
    return rows,predictions,summary,final


def plots(records,rows,predictions):
    fig,axes=plt.subplots(4,2,figsize=(14,12));fig.subplots_adjust(left=.08,right=.98,top=.89,bottom=.12,hspace=.55,wspace=.25)
    fig.suptitle('Modelli empirici: predire una run con parametri ricavati dalle altre',x=.08,ha='left',y=.975,fontsize=18,weight='bold')
    fig.text(.08,.935,'8 verifiche tra run di sviluppo · preprocessing causale 0,3–8 Hz · nessuna uscita misurata riusata nella simulazione libera',fontsize=10)
    colors=dict(delay='#dd941f',first_order='#40a69b',second_order='#9564b2',ARX2='#cf4e59')
    for row,piece in enumerate('ZLST'):
        i=next(i for i,r in enumerate(records) if r['piece']==piece);r=records[i];k=(r['t']>=5)&(r['t']<=10)
        for col,kind in enumerate(['motion','force']):
            ax=axes[row,col];truth=r['y'] if col==0 else r['force']
            ax.plot(r['t'][k],truth[k],color='#263746',lw=1.3,label='Misurato, componente dinamica')
            for family in colors:
                rr=next((x for x in rows if x['id']==r['id'] and x['family']==family),None)
                if rr is None:continue
                pred=predictions[(i,family)]*(1 if col==0 else rr['force_gain_N_per_mm_projected_closure'])
                ax.plot(r['t'][k],pred[k],color=colors[family],lw=.9,label=family)
            ax.set(title=piece+' · run '+r['id'][-6:-2],xlabel='Tempo da inizio route [s]',
                   ylabel='Chiusura proiettata [mm/mano]' if col==0 else 'Forza media mani [N]')
            ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=7,ncol=2)
    fig.text(.08,.025,'Ogni curva usa parametri selezionati sulle altre 7 run, senza scalare o riallineare il segnale di test.\n'
             'Sono verifiche retrospettive in anello chiuso: generalizzare tra queste run non dimostra la risposta a un nuovo controllore.\n'
             'Quattro run riservate escluse; la coordinata proiettata non è una misura diretta della compressione dei pad.',fontsize=10,linespacing=1.5)
    fig.savefig(OUT/'05_modelli_empirici_crossrun.png',dpi=155);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(13,5.8));fig.subplots_adjust(left=.08,right=.98,top=.80,bottom=.23,wspace=.25)
    fig.suptitle('Previsione a un passo e simulazione libera danno risposte diverse',x=.08,ha='left',y=.96,fontsize=17,weight='bold')
    fig.text(.08,.87,'ARX a due poli · il primo metodo usa il moto misurato passato a ogni campione; il secondo usa soltanto i comandi',fontsize=10)
    rr=[r for r in rows if r['family']=='ARX2'];xx=np.arange(len(rr))
    axes[0].bar(xx-.18,[r['motion_one_step']['r2'] for r in rr],width=.35,label='Un passo: stato misurato passato',color='#1675bb')
    axes[0].bar(xx+.18,[r['motion_free']['r2'] for r in rr],width=.35,label='Libera: solo comandi',color='#cf4e59')
    axes[0].set(title='Moto proiettato: R² tra run',ylabel='R²');axes[0].legend(fontsize=7)
    for family,color in colors.items():
        vals=[r['force_free']['r2'] for r in rows if r['family']==family]
        axes[1].plot(np.arange(len(vals)),vals,'o-',color=color,label=family)
    axes[1].set(title='Forza prevista da comandi + modello del moto',ylabel='R² simulazione libera');axes[1].legend(fontsize=7,ncol=2)
    labels=[r['piece']+' '+r['id'][-6:-2] for r in rr]
    for ax in axes:
        ax.set_xticks(xx,labels,rotation=45,ha='right',fontsize=8);ax.axhline(0,color='grey',lw=.7);ax.grid(axis='y',alpha=.2)
    fig.text(.08,.035,'Un R² elevato a un passo può dipendere dall’autocorrelazione del segnale e non da una buona dinamica comando→moto.\n'
             'Nessuno di questi punteggi equivale a una validazione causale del robot o autorizza guadagni hardware.',fontsize=10,linespacing=1.5)
    fig.savefig(OUT/'06_predizione_vs_simulazione.png',dpi=155);plt.close(fig)
    arrays={}
    for i,r in enumerate(records):
        arrays[f'r{i}_t']=r['t'];arrays[f'r{i}_motion_mm']=r['y'];arrays[f'r{i}_force_N']=r['force']
        for family in colors:
            if (i,family) in predictions:arrays[f'r{i}_{family}_motion_mm']=predictions[(i,family)]
    np.savez_compressed(OUT/'empirical_crossrun_traces.npz',**arrays)


def main():
    design=freeze();records=prepare(design)
    rows,predictions,summary,final=run(design,records);plots(records,rows,predictions)
    for r in summary:print(json.dumps(r),flush=True)
    print('Final fit',json.dumps(final),flush=True)


if __name__=='__main__':main()
