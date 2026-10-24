"""Compare two archived near-zero-I Z trials with three successful FFI Z trials.

Interrupted trials are diagnostic references, never counted as successful runs.
All metrics use the same elapsed window before filtering and excluding edges.
"""
from bags import *


def main():
    records=catalog();bags=bag_inventory()
    stamps=['165531','165902','174143','175102','175651']
    runs=[]
    for stamp_ in stamps:
        rec=next(r for r in records if '_ffi_20261023_'+stamp_ in r['id'])
        run=analyse_run(rec);b=match_bag(run,bags)
        if b:
            check=analyse_bag(run);run['bag_check']=check
        runs.append(run)
    end=min(r['metrics']['duration_s'] for r in runs)
    uniform=np.arange(0,end,.01)
    results=[]
    for r in runs:
        d,t,k=r['d'],r['t'],r['route'];m=r['metrics'];meta=r['rec']['meta']
        y=np.interp(uniform,t[k],d['fn_N'][k]/2)
        target=np.interp(uniform,t[k],d['target_N'][k])
        u,b,keep=previous.bandpass(uniform,y,[.8,2.5])
        pos=[np.column_stack([d[c+s][k] for c in 'xyz']) for s in 'LR']
        measured=[np.column_stack([d['m'+c+s][k] for c in 'xyz']) for s in 'LR']
        # Flange/tool separation magnitude; 1/2 converts total opening to per-hand closure.
        command=-500*np.linalg.norm(pos[1]-pos[0],axis=1)
        actual=-500*np.linalg.norm(measured[1]-measured[0],axis=1)
        closure_bands=[]
        for signal in [command,actual]:
            yy=np.interp(uniform,t[k],signal)
            _,bb,_=previous.bandpass(uniform,yy,[.8,2.5]);closure_bands.append(bb)
        trim=np.interp(uniform,t[k],d['ffi_i_mm'][k])
        _,ib,_=previous.bandpass(uniform,trim,[.8,2.5])
        _,tb,_=previous.bandpass(uniform,target,[.8,2.5])
        row={'id':m['id'],'label':m['label'],'status':m['status'],
             'window_start_s':0.,'window_end_s':float(end),'filter_edge_excluded_s':1.5,
             'ki_m_per_N_s':meta['execution']['ffi']['ki_m_per_N_s'],
             'k_eff_N_per_m':meta['execution']['ffi']['k_eff_N_per_m'],
             'git':meta.get('git'),'controller_parameters':meta.get('controller_parameters'),
             'mean_force_mean_hand_N':float(y.mean()),'mean_target_hand_N':float(target.mean()),
             'band_rms_mean_hand_N':float(np.std(b[keep])),
             'target_band_rms_N':float(np.std(tb[keep])),
             'band_percent_mean':float(100*np.std(b[keep])/y.mean()),
             'trim_total_range_mm':float(np.ptp(trim)), 'trim_band_rms_mm':float(np.std(ib[keep])),
             'command_half_gap_band_rms_mm':float(np.std(closure_bands[0][keep])),
             'measured_half_gap_band_rms_mm':float(np.std(closure_bands[1][keep])),
             'normal_force_vs_measured_closure_band_correlation':float(np.corrcoef(b[keep],closure_bands[1][keep])[0,1]),
             'bag_check':r.get('bag_check')}
        results.append(row);r['probe_band']=(u,b,keep)
        print({k:v for k,v in row.items() if k not in ['bag_check','controller_parameters']},flush=True)
    dump('integrator_probe.json',results)
    fig,axes=plt.subplots(3,1,figsize=(12,11))
    fig.subplots_adjust(left=.09,right=.98,top=.86,bottom=.14,hspace=.52)
    fig.suptitle('L’onda rimane anche con I quasi spenta',x=.09,ha='left',y=.975,fontsize=22,weight='bold',color='#172b40')
    fig.text(.09,.93,'Due prove interrotte con K_I = 0,001 mm/(N s), contro 1,6 mm/(N s): stesso codice, k_eff e impedenza.',fontsize=10)
    fig.text(.09,.9,'Sono riferimenti diagnostici: la forza media e la presa cambiano, quindi non è un confronto controllato del solo gain.',fontsize=10)
    colors=['#8a9199','#239081','#1675bb','#df711c','#8556a3']
    for r,c in zip([runs[0],runs[1],runs[2]],[colors[0],colors[1],colors[2]]):
        m,d,t,k=r['metrics'],r['d'],r['t'],r['route']
        z=k&(t>=5)&(t<=10)
        ki=r['rec']['meta']['execution']['ffi']['ki_m_per_N_s']*1000
        label=m['label'].split()[-1]+f' · K_I {ki:g} mm/(N s)'
        axes[0].plot(t[z],d['fn_N'][z]/2,color=c,lw=1,label=label)
        u,b,keep=r['probe_band'];axes[1].plot(u[keep],b[keep],color=c,lw=.9,label=label)
    axes[0].set(title='Zoom 5–10 s: forza registrata, nessun filtro aggiunto',ylabel='Forza media mani [N]',xlim=(5,10))
    axes[1].set(title='Stessa finestra di confronto: componente 0,8–2,5 Hz',ylabel='Variazione [N]',xlim=(0,end))
    for a in axes[:2]:a.legend(fontsize=8,ncol=3);a.grid(alpha=.2);a.set_xlabel('Tempo da inizio route [s]')
    a=axes[2];x=np.arange(len(results));v=[r['band_rms_mean_hand_N'] for r in results]
    a.bar(x,v,color=colors,width=.65);a.set_xticks(x,[r['label'].split()[-1] for r in results])
    for i,y in enumerate(v):a.text(i,y+.004,f'{y:.3f} N'.replace('.',','),ha='center',fontsize=10)
    a.set(title='RMS banda sulla stessa finestra · primi due: I quasi spenta',ylabel='RMS della media mani [N]',ylim=(0,max(v)*1.3))
    a.grid(axis='y',alpha=.2)
    fig.text(.09,.035,'Filtraggio comune: detrend cubico + Butterworth ordine 2 avanti/indietro, esclusi 1,5 s ai bordi.\n'
             'Questo mostra che l’I normale non è l’unica sorgente possibile dell’onda durante il movimento; non identifica\n'
             'separatamente servo, attrito, filtri, contatti e flessibilità, e non dimostra che l’I non possa amplificarla.',
             fontsize=10,color='#526173',linespacing=1.5)
    fig.savefig(OUT/'06_I_quasi_spenta.png',dpi=155);plt.close(fig)


if __name__=='__main__':main()
