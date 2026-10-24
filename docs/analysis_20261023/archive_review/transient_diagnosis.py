"""Check force/command/measured-opening chronology at the L seating transition."""
from analyze import *


def main():
    recs=catalog()
    chosen=[r for r in recs if r['piece']=='L' and r['status']=='curated_seat_found' and '_ffi_' in r['id']]
    fig,axes=plt.subplots(4,3,figsize=(15,12))
    fig.subplots_adjust(left=.075,right=.985,top=.88,bottom=.13,hspace=.55,wspace=.25)
    fig.suptitle('L: cosa succede mentre la forza supera il target',x=.075,ha='left',y=.975,
                 fontsize=21,weight='bold',color='#172b40')
    fig.text(.075,.933,'Posizioni comandate già inviate, posizioni misurate e forza ATI · nessun filtro temporale aggiunto',fontsize=11)
    results=[]
    for col,rec in enumerate(chosen):
        run=analyse_run(rec);d,t,k=run['d'],run['t'],run['route'];m=run['metrics']
        end=t[k][-1];x=t-end
        window=np.isin(d['phase'],['route','slow_seat'])&(x>=-.8)&(x<=1.8)
        force=d['fn_N']/2;target=d['target_N']
        pos=[np.column_stack([d[c+s] for c in 'xyz']) for s in 'LR']
        meas=[np.column_stack([d['m'+c+s] for c in 'xyz']) for s in 'LR']
        command=-500*np.linalg.norm(pos[1]-pos[0],axis=1)
        actual=-500*np.linalg.norm(meas[1]-meas[0],axis=1)
        candidates=np.flatnonzero(window&(x>=-.3)&(x<=.5))
        jump=candidates[np.argmin(np.diff(target,prepend=target[0])[candidates])]
        after=np.flatnonzero(window&(x>=x[jump])&(x<=1.5))
        peak=after[np.argmax(force[after])]
        row={'id':m['id'],'label':m['label'],
             'target_drop_route_end_offset_s':float(x[jump]),
             'target_before_drop_N_per_hand':float(target[jump-1]),
             'target_after_drop_N_per_hand':float(target[jump]),
             'peak_route_end_offset_s':float(x[peak]),
             'peak_delay_after_target_drop_s':float(x[peak]-x[jump]),
             'force_at_drop_N_mean_hand':float(force[jump]),
             'force_peak_N_mean_hand':float(force[peak]),
             'target_at_peak_N_per_hand':float(target[peak]),
             'commanded_closure_drop_to_peak_mm_per_hand':float(command[peak]-command[jump]),
             'measured_closure_drop_to_peak_mm_per_hand':float(actual[peak]-actual[jump]),
             'ff_state_delta_drop_to_peak_mm':float(d['ffi_ff_mm'][peak]-d['ffi_ff_mm'][jump]),
             'trim_state_delta_drop_to_peak_mm':float(d['ffi_i_mm'][peak]-d['ffi_i_mm'][jump]),
             'total_state_delta_drop_to_peak_mm':float(d['ffi_output_mm'][peak]-d['ffi_output_mm'][jump]),
             'trim_at_peak_mm':float(d['ffi_i_mm'][peak]),
             'rate_limited_at_peak':int(d['ffi_rate_limited'][peak]),
             'joint_data_age_at_peak_ms_L_R':[float(d['joint_age_'+s+'_ms'][peak]) for s in 'LR'],
             'ATI_data_age_at_peak_ms_L_R':[float(d['ft_age_'+s+'_ms'][peak]) for s in 'LR'],
             'ext_z_at_drop_N':float(d['ext_z'][jump]),'ext_z_at_peak_N':float(d['ext_z'][peak]),
             'closure_force_correlation_drop_to_1p5s':float(np.corrcoef(actual[after],force[after])[0,1])}
        results.append(row);print(json.dumps(row),flush=True)
        a=axes[0,col];a.plot(x[window],force[window],color='#1675bb',label='Misurata, media mani')
        a.plot(x[window],target[window],'k--',label='Target/mano')
        a.plot(x[peak],force[peak],'o',color='#c43c43',ms=5)
        a.set(title=m['label'],ylabel='Forza per mano [N]',ylim=(2.5,7))
        if col==0:a.legend(fontsize=8)
        a=axes[1,col]
        for key,c,lab in [('ffi_ff_want_mm','#8a9199','Nominale richiesto'),
                           ('ffi_ff_mm','#1675bb','Nominale limitato'),
                           ('ffi_i_mm','#df711c','Trim I'),('ffi_output_mm','#263746','Totale')]:
            a.plot(x[window],d[key][window],color=c,lw=1,label=lab)
        a.set(ylabel='Stato per mano [mm]')
        if col==0:a.legend(fontsize=7,ncol=2)
        a=axes[2,col]
        # Separate baselines at the target drop: compare changes, not absolute gap.
        a.plot(x[window],command[window]-command[jump],color='#df711c',label='Comando già inviato')
        a.plot(x[window],actual[window]-actual[jump],color='#1675bb',label='Tool misurati')
        a.axhline(0,color='grey',lw=.7)
        a.set(ylabel='Chiusura equivalente/tool [mm]')
        if col==0:a.legend(fontsize=8)
        a=axes[3,col]
        a.plot(x[window],d['ext_z'][window],color='#8556a3',label='Forza esterna verticale')
        a.set(ylabel='Forza esterna z [N]')
        if col==0:a.legend(fontsize=8)
        for a in axes[:,col]:
            a.axvline(x[jump],color='grey',ls=':',lw=1)
            a.axvline(x[peak],color='#c43c43',ls=':',lw=.8)
            a.set_xlim(-.8,1.8);a.set_xlabel('Tempo da ultimo campione route [s]');a.grid(alpha=.2)
    fig.text(.075,.03,'Terza riga: positivo = i tool si avvicinano; negativo = si allontanano. Chiusura = − metà della distanza tra tool.\n'
             'Linea grigia: maggiore calo del target alla transizione; rossa: picco della forza successivo. Gli stati FFI sono\n'
             'aggiornati dopo la lettura, mentre le pose comandate sono già state inviate: non sono lo stesso istante del comando.',
             fontsize=10,color='#526173',linespacing=1.5)
    dump('L_transient_chronology.json',results)
    fig.savefig(OUT/'07_L_cronologia_sovraelongazione.png',dpi=155);plt.close(fig)


if __name__=='__main__':main()
