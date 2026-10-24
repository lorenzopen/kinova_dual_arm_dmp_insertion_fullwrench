"""Descriptive original-vs-Bullet comparison, after model/controller are fixed."""
from analyze_bullet import *
from scipy.spatial.transform import Rotation as Rot
from scipy.signal import sosfiltfilt,butter
TIMES={'Z':'20261023_174143','L':'20261023_182743','S':'20261023_190407','T':'20261023_192128'}
def real(piece):
    paths=list((ROOT/'docs/runs').rglob('*'+TIMES[piece]+'.csv'));assert len(paths)==1,paths
    with paths[0].open() as f:rows=list(csv.DictReader(f))
    col=lambda k:np.array([float(r[k]) for r in rows])
    d=dict(t=col('t_wall'),phase=np.array([r['phase'] for r in rows]),target=col('target_N'))
    for s in 'LR':
        F=np.column_stack([col('F'+a+s) for a in 'xyz'])
        R=Rot.from_quat(np.column_stack([col('mq'+a+s) for a in 'xyzw']))
        d[s+'_normal']=np.sum(F*R.apply(np.tile([1.,0,0],(len(F),1))),axis=1)
    d['mean_normal']=.5*(d['L_normal']+d['R_normal'])
    return d,paths[0]
def metric(d):
    mask=d['phase']=='route';t=d['t'][mask];grid=np.arange(t[0],t[-1],.01)
    force=np.interp(grid,t,d['mean_normal'][mask]);target=np.interp(grid,t,d['target'][mask]);e=force-target
    active=np.isin(d['phase'],['to_mouth','route','slow_seat','seat_by_feel'])
    seat=np.isin(d['phase'],['slow_seat','seat_by_feel'])
    return dict(route_rmse_N=float(np.sqrt(np.mean(e*e))),route_force_band_08_25_N=band(force,.8,2.5),
        route_error_band_08_25_N=band(e,.8,2.5),route_force_band_25_10_N=band(force,2.5,10),
        active_min_L_N=float(d['L_normal'][active].min()),active_min_R_N=float(d['R_normal'][active].min()),
        seat_positive_error_N=float((d['mean_normal']-d['target'])[seat].max()) if seat.any() else None)
def main():
    fig,axes=plt.subplots(4,2,figsize=(15,13),layout='constrained');comparison=[]
    for j,piece in enumerate(TIMES):
        run=OUT/'runs'/f'{piece}_central2700_D60_dt05'
        if not (run/'COMPLETED.json').exists():
            for ax in axes[j]:ax.set_visible(False)
            continue
        sim,result=load(run);observed,path=real(piece)
        if result['error']:continue
        comparison.append(dict(piece=piece,real_source=str(path.relative_to(ROOT)),real=metric(observed),
                               simulation=metric(sim),simulation_final_worst_cell_mm=result['truth']['final_worst_cell_mm']))
        for ax,d,title in zip(axes[j],[observed,sim],['Reale — prima run di sviluppo','PyBullet — stesso FFI, centrale 2700 N/m']):
            start=np.flatnonzero(d['phase']=='to_mouth')[0];t=d['t']-d['t'][start]
            active=np.isin(d['phase'],['to_mouth','route','slow_seat','seat_by_feel'])
            for s,col in [('L','#2563eb'),('R','#e87924')]:ax.plot(t[active],d[s+'_normal'][active],color=col,lw=.8,label='Mano '+s)
            ax.plot(t[active],d['target'][active],'k--',lw=1.2,label='SOCP / mano')
            ax.grid(alpha=.2);ax.set_ylabel(piece+' — normale [N]');ax.set_title(title,fontsize=11)
            for i in np.flatnonzero(np.r_[True,d['phase'][1:]!=d['phase'][:-1]]):
                if active[i]:ax.axvline(t[i],c='.6',ls=':',lw=.6)
            ax.legend(loc='upper right',fontsize=8,ncol=3)
            ax.set_ylim(bottom=0.)
        for ax in axes[-1]:ax.set_xlabel('Tempo dalla partenza, proprio clock [s]')
    fig.suptitle('Confronto descrittivo: dati reali e simulazione autonoma\n'
        'Nessuna forza reale alimenta PyBullet. Geometria rotta e piano SOCP nominali; nessun fitting.\n'
        'Wrist proxy cartesiano + tri-pad originale: differenze non attribuibili al solo hardware.',fontsize=13)
    fig.savefig(OUT/'real_vs_bullet_central2700.png');plt.close(fig)
    (OUT/'real_vs_bullet_central2700_metrics.json').write_text(json.dumps(comparison,indent=2)+'\n')
    print(json.dumps(comparison,indent=2))
if __name__=='__main__':main()
