"""Immediate plots of available real/autonomous traces, no simulation/model changes."""
from analyze_bullet import *
from compare_real import real,metric
from datetime import datetime,timezone
import shutil,hashlib,zipfile
DEST=OUT/'grafici_20261024';DEST.mkdir(exist_ok=True)
CASES={'Z':'Gen3_Z_central2700_implicit_dt025','L':'Gen3_L_central2700_implicit_dt05'}
COL={'L':'#1971c2','R':'#e67700'}
ACTIVE=['to_mouth','route','slow_seat','seat_by_feel']
def clock(d):return d['t']-d['t'][np.flatnonzero(d['phase']=='to_mouth')[0]]
def force(ax,d,title,zoom=None):
    t=clock(d);m=np.isin(d['phase'],ACTIVE)
    for s in 'LR':ax.plot(t[m],d[s+'_normal'][m],c=COL[s],lw=.9,label='Mano '+s)
    ax.plot(t[m],d['target'][m],c='#222',ls='--',lw=1.2,label='SOCP / mano')
    ax.set_title(title,fontsize=11);ax.set_ylabel('Forza normale [N]');ax.set_xlabel('Tempo dalla partenza [s]')
    ax.grid(alpha=.22);ax.legend(ncol=3,fontsize=8,loc='upper right')
    if zoom:ax.set_xlim(*zoom)
def extra(d):
    x=metric(d);m=d['phase']=='route';e=d['mean_normal']-d['target']
    x.update(route_max_positive_error_N=float(np.max(e[m])),route_max_negative_error_N=float(np.min(e[m])))
    for s in 'LR':
        x[s+'_route_force_band_08_25_N']=band(d[s+'_normal'][m],.8,2.5)
        x[s+'_route_force_band_25_10_N']=band(d[s+'_normal'][m],2.5,10)
    return x
def main():
    results=[];loaded={}
    fig,axes=plt.subplots(2,2,figsize=(14,8),layout='constrained')
    for i,(piece,name) in enumerate(CASES.items()):
        sim,r=load(OUT/'runs'/name);observed,path=real(piece);loaded[piece]=(observed,sim,r)
        force(axes[i,0],observed,piece+' — run reale')
        force(axes[i,1],sim,piece+' — inserimento PyBullet completo, impedenza approssimata')
        limits=np.r_[axes[i,0].get_ylim(),axes[i,1].get_ylim()];ylim=(max(0,float(limits.min())),float(limits.max()))
        for ax in axes[i]:ax.set_ylim(*ylim)
        results.append(dict(piece=piece,run=name,real_source=str(path.relative_to(ROOT)),
            real=extra(observed),simulation=extra(sim),final_worst_cell_mm=r['truth']['final_worst_cell_mm'],
            realization='Original FFI/SOCP; approximate locally implicit inner Cartesian impedance, not the verbatim lab loop',dt=r['dt']))
    fig.suptitle('Forze durante l’inserimento: dati reali e simulazione autonoma\n'
        'Due Gen3, FFI/SOCP originali. Centrale 2700 + tre pad da 500 = 4200 N/m.\n'
        'Le curve complete usano l’impedenza interna approssimata; la prova del codice originale è nel grafico separato.',fontsize=12)
    fig.savefig(DEST/'01_forze_reale_simulazione.png');plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(13,7),layout='constrained')
    for i,(piece,(observed,sim,r)) in enumerate(loaded.items()):
        for j,(d,label) in enumerate([(observed,'Reale'),(sim,'PyBullet: impedenza approssimata')]):
            ax=axes[i,j];t=clock(d);m=np.isin(d['phase'],ACTIVE);error=d['mean_normal']-d['target']
            ax.plot(t[m],error[m],c='#5f3dc4',lw=.9);ax.axhline(0,c='.3',lw=.8)
            ax.set_title(piece+' — '+label,fontsize=11);ax.set_ylabel('Errore forza media [N]');ax.set_xlabel('Tempo dalla partenza [s]');ax.grid(alpha=.2)
        lo=min(ax.get_ylim()[0] for ax in axes[i]);hi=max(ax.get_ylim()[1] for ax in axes[i])
        for ax in axes[i]:ax.set_ylim(lo,hi)
    fig.suptitle('Errore rispetto al target SOCP originale\nMedia delle mani; i cambi di riferimento restano visibili, senza smoothing aggiuntivo.',fontsize=12)
    fig.savefig(DEST/'02_errori_target.png');plt.close(fig)
    observed,sim,r=loaded['Z']
    fig,axes=plt.subplots(1,2,figsize=(13,4),layout='constrained')
    for ax,d,label in zip(axes,[observed,sim],['Z reale','Z simulato: impedenza approssimata']):force(ax,d,label,(12,17))
    vals=[]
    for d in [observed,sim]:
        t=clock(d);m=(t>=12)&(t<=17)
        for s in 'LR':vals.extend(d[s+'_normal'][m])
        vals.extend(d['target'][m])
    limits=(min(vals)-.08,max(vals)+.08)
    for ax in axes:ax.set_ylim(*limits)
    fig.suptitle('Zoom Z: stesse scale per vedere le oscillazioni\nForze effettivamente usate dai controller, senza filtro grafico aggiuntivo.',fontsize=12)
    fig.savefig(DEST/'03_zoom_oscillazioni_Z.png');plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(14,8),layout='constrained')
    for i,(piece,name) in enumerate(CASES.items()):
        d,r=load(OUT/'runs'/name);observed,path=real(piece)
        with path.open() as f:rows=list(csv.DictReader(f))
        for j,(data,label) in enumerate([(observed,'Reale'),(d,'PyBullet: impedenza approssimata')]):
            t=clock(data);m=np.isin(data['phase'],ACTIVE)
            for s in 'LR':
                torque=(np.array([[float(x['T'+a+s]) for a in 'xyz'] for x in rows]) if j==0 else d[s+'_measured'][:,3:])
                axes[i,j].plot(t[m],np.linalg.norm(torque[m],axis=1),c=COL[s],lw=.9,label='Polso '+s)
            axes[i,j].set_title(piece+' — '+label,fontsize=11);axes[i,j].set_ylabel('Modulo momento al polso [Nm]')
            axes[i,j].set_xlabel('Tempo dalla partenza [s]');axes[i,j].grid(alpha=.2);axes[i,j].legend(fontsize=8)
        lim=(0,max(ax.get_ylim()[1] for ax in axes[i]))
        for ax in axes[i]:ax.set_ylim(*lim)
    fig.suptitle('Momenti misurati ai polsi durante l’inserimento\nFFI regola la normale; questi momenti non sono inseguiti da un nuovo anello di coppia.',fontsize=12)
    fig.savefig(DEST/'04_coppie_polsi.png');plt.close(fig)
    folder=OUT/'runs/Gen3Lab_Z_central2700_warm_dt025';d,r=load(folder)
    raw=d['raw_wrist_1kHz'];fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True,layout='constrained')
    for side,start in [('L',13),('R',19)]:
        axes[0].plot(raw[:,0]*1000,np.linalg.norm(raw[:,start:start+3],axis=1),c=COL[side],label='Polso '+side)
        axes[1].plot(raw[:,0]*1000,np.linalg.norm(raw[:,start+3:start+6],axis=1),c=COL[side],label='Polso '+side)
    for ax in axes:
        for e in r['laboratory_events']:ax.axvline(e['t']*1000,c=COL[e['side']],ls=':',lw=1)
        ax.grid(alpha=.2);ax.legend(ncol=2)
    axes[0].set_ylabel('Modulo forza [N]');axes[1].set_ylabel('Modulo momento [Nm]');axes[1].set_xlabel('Tempo simulato [ms]')
    axes[0].text(.02,.95,'Linee verticali: watchdog. La prova si interrompe prima del pickup.',transform=axes[0].transAxes,va='top',fontsize=9)
    fig.suptitle('Codice originale del laboratorio: forze e momenti ottenuti nel modello URDF\n'
        'Reazioni meccaniche a 1 kHz, gravità rimossa. SOCP non ancora attivo: nessuna curva di inserimento dopo lo stop.',fontsize=12)
    fig.savefig(DEST/'05_impedenza_originale_forze_coppie.png');plt.close(fig)
    for piece,name in CASES.items():
        for stem in ['wrist_wrench','socp_wrench']:
            shutil.copyfile(OUT/'runs'/name/(stem+'.png'),DEST/(piece+'_'+stem+'.png'))
    fig,axes=plt.subplots(3,2,figsize=(12,8),sharex=True,layout='constrained')
    for side,start in [('L',13),('R',19)]:
        for k,label in enumerate('xyz'):
            axes[k,0].plot(raw[:,0]*1000,raw[:,start+k],c=COL[side],lw=.9,label=side)
            axes[k,1].plot(raw[:,0]*1000,raw[:,start+3+k],c=COL[side],lw=.9,label=side)
            axes[k,0].set_ylabel('F'+label+' [N]');axes[k,1].set_ylabel('T'+label+' [Nm]')
    for ax in axes.flat:ax.grid(alpha=.2);ax.legend(ncol=2,fontsize=8)
    for ax in axes[-1]:ax.set_xlabel('Tempo simulato [ms]')
    fig.suptitle('Impedenza originale: sei componenti ai polsi fino allo stop\nAssi scaffale; reazioni meccaniche compensate per gravità, prima del pickup.',fontsize=12)
    fig.savefig(DEST/'originale_sei_componenti.png');plt.close(fig)
    with (DEST/'metriche.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['pezzo','origine','RMSE_route_N','RMS_forza_08_25_Hz_N','RMS_errore_08_25_Hz_N','massimo_eccesso_route_N','massimo_difetto_route_N','eccesso_seduta_N'])
        for x in results:
            for origin in ['real','simulation']:
                m=x[origin];w.writerow([x['piece'],origin,m['route_rmse_N'],m['route_force_band_08_25_N'],m['route_error_band_08_25_N'],m['route_max_positive_error_N'],m['route_max_negative_error_N'],m['seat_positive_error_N']])
    (DEST/'metriche.json').write_text(json.dumps(dict(created_utc=datetime.now(timezone.utc).isoformat(),comparisons=results,
        exact_source_trial=dict(run=folder.name,end_seconds=r['truth']['seconds'],error=r['error'],events=r['laboratory_events']),
        interpretation='Full insertion curves are available only for approximate inner impedance. Verbatim source trial stops before pickup. No force extrapolation or refitting.'),indent=2)+'\n')
    text=['# Grafici pronti','',
        'Le curve complete Z/L usano FFI/SOCP originali e due Gen3 con impedenza interna approssimata. '
        'Z usa il passo più fine disponibile (0.25ms); L0.5ms. Centrale2700 + tre pad500N/m. '
        'La prova con il codice originale del laboratorio è mostrata separatamente fino allo stop reale nel simulatore.','',
        '![Forze](01_forze_reale_simulazione.png)','', '![Errori](02_errori_target.png)','',
        '![Zoom](03_zoom_oscillazioni_Z.png)','', '![Coppie](04_coppie_polsi.png)','',
        '![Originale](05_impedenza_originale_forze_coppie.png)','',
        '## Numeri principali','', '| Pezzo | RMSE reale N | RMSE sim N | RMS banda0.8–2.5 reale N | sim N | Errore geometrico sim mm |',
        '|---|---:|---:|---:|---:|---:|']
    for x in results:text.append(f"| {x['piece']} | {x['real']['route_rmse_N']:.3f} | {x['simulation']['route_rmse_N']:.3f} | {x['real']['route_force_band_08_25_N']:.3f} | {x['simulation']['route_force_band_08_25_N']:.3f} | {x['final_worst_cell_mm']:.2f} |")
    text+=['','Banda e RMSE della media delle mani. RMS di banda non significa picco-picco. '
        'Metriche per mano e transitori in metriche.json. Nessuna nuova simulazione, modifica di modello o tuning per produrre questi grafici.','',
        'Sei componenti e SOCP completo: ../runs/Gen3_Z_central2700_implicit_dt025/wrist_wrench.png '
        'e socp_wrench.png; analoghi file per L. Prove floating-plate su quattro pezzi conservate '
        'in ../real_vs_bullet_central2700.png.','']
    (DEST/'GRAFICI.md').write_text('\n'.join(text),encoding='utf-8')
    entries=[dict(path=p.name,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sorted(DEST.iterdir()) if p.is_file() and p.name!='SHA256.json']
    (DEST/'SHA256.json').write_text(json.dumps(dict(files=entries,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        inputs=[dict(run=name,trace_sha256=hashlib.sha256((OUT/'runs'/name/'trace.npz').read_bytes()).hexdigest()) for name in [*CASES.values(),folder.name]]),indent=2)+'\n')
    with zipfile.ZipFile(OUT/'grafici_20261024.zip','w',compression=zipfile.ZIP_DEFLATED) as archive:
        for p in sorted(DEST.iterdir()):
            if p.is_file():archive.write(p,p.name)
    print(json.dumps(results,indent=2))
if __name__=='__main__':main()
