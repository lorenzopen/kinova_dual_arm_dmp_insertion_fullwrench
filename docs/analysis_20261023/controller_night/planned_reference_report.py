"""Summarize planning/reference tradeoffs and retained numerical failure."""
import planned_reference as r
import json
import numpy as np
import matplotlib.pyplot as plt

OUT=r.OUT;c=r.c


def load(name):return json.loads((OUT/name).read_text())


def main():
    design=r.freeze();frozen=load('frozen_design.json');groups=load('planned_reference_grouped_ranking.json')
    rows=load('planned_reference_metrics.json');auto=load('planned_reference_autonomous_metrics.json')
    refinement=load('planned_reference_refinement.json');auto_ref=load('planned_reference_autonomous_refinement.json')
    software=load('planned_reference_software_validation.json');exact_check=load('event_exact_software_validation.json')
    paired=[]
    for group in groups:
        original=next(x for x in groups if x['base_controller']==group['base_controller'] and x['plan']=='original'
            and x['feedback_reference']==group['feedback_reference'])
        rr=[x for x in rows if x['candidate'] in group['candidate_ids']]
        paired.append(dict(group,score_change_vs_matched_clock_control_percent=100*(group['mean_run_worst_model_and_clock_score']/original['mean_run_worst_model_and_clock_score']-1),
            max_plan_below_original_same_clock_N=max(x['max_planned_below_original_same_clock_N'] for x in rr)))
    auto_summary=[]
    for cfg in load('planned_reference_autonomous_design.json')['candidates']:
        rr=[x for x in auto if x['candidate']==cfg['id']]
        score=float(np.mean([max(x['score'] for x in rr if x['piece']==piece) for piece in 'ZLST']))
        auto_summary.append(dict(id=cfg['id'],score=score,
            maximum_route_rmse_N=max(x['route_rmse_N'] for x in rr),
            maximum_seat_positive_error_N=max(x['first_seat_max_positive_error_N'] for x in rr),
            maximum_low_band_N=max(x['low_band_rms_N'] for x in rr),
            maximum_route_band_N=max(x['route_band_rms_N'] for x in rr),
            minimum_mean_hand_force_N=min(x['minimum_mean_hand_force_N'] for x in rr)))
    auto_summary.sort(key=lambda x:x['score'])
    # Target-subtracted band diagnostics, kept apart from the predeclared score.
    error_bands=[]
    for piece in 'ZLST':
        with np.load(OUT/(piece+'_planned_reference_autonomous.npz')) as archive:
            t=archive['t'];target=archive['target'];force=archive['force'];end=float(archive['route_end'])
            candidates=archive['candidate_ids'];models=archive['model_ids'];variants=archive['variant_ids']
        route=(t>=2)&(t<end);c.DT=.0025;error=force[route]-target[route,None,None]
        low=c.band_rms(t[route],error,[.2,.8]);mid=c.band_rms(t[route],error,[.8,2.5]);high=c.band_rms(t[route],error,[2.5,10])
        for i,candidate in enumerate(candidates):
            for m,model in enumerate(models):
                error_bands.append(dict(piece=piece,candidate=str(candidate),model=str(model),variant=str(variants[m]),
                    error_low_band_N=float(low[i,m]),error_mid_band_N=float(mid[i,m]),error_high_band_N=float(high[i,m])))
    c.dp.dump('planned_reference_autonomous_error_bands.json',error_bands)
    comparison=load('event_exact_delay_grid_comparison.json')
    critical_id='PI_P0.1_I0.8_D0_R0.1__ABS_majorant_FBplanned_O0.04'
    critical=next(x for x in comparison if '_Zt1_' in x['id'] and x['candidate']==critical_id and x['model']=='delay_k1500_d0.23')
    c.dp.dump('planned_reference_summary.json',dict(conditional_comparisons=paired,autonomous_summary=auto_summary,
        failed_conditional_peak_refinement=True,event_exact_diagnostic_scenarios=456,critical_case=critical,
        no_holdout_evaluated=True,no_cart_d_change=True,
        caveat='No validated controller emerges. Full-wrench margins unvalidated; original-target errors retained; minimax can lower required normal. Numerical coarse-grid point extrema are unreliable in flagged cases.'))
    text=['# Pianificazione del nominale e riferimento del feedback',
        '', '**Nessun nuovo controllore validato. I piani a velocità limitata cambiano il compromesso, ma non risolvono insieme tracking, picchi e minimi nei confronti provati.**',
        '', '## Cosa è stato confrontato',
        '', 'Disegno congelato:38 configurazioni ×16 modelli ×8 development = **4.864 replay condizionati**. FFI originale o PI P0,1/I0,8 con filtro P100ms. Tre piani nella fase corrente: originale, inviluppo superiore a1,36N/s, e approssimazione minimax alla stessa pendenza. Feedback sul target originale oppure sul piano. Tre offset di clock−40/0/+40ms valutati nel caso peggiore.',
        '', 'Questa volta si campiona il **piano assoluto**: nominale=(piano−pickup)/680, senza l’ancoraggio del precedente studio delta-preview. Sono incluse sia le baseline registrate esatte sia il lookup del piano originale come controllo della ricostruzione del clock. Non bisogna attribuire al planner una differenza dovuta al solo clock non registrato.',
        '', 'I piani maggiorante/minimax rispettano2mm/s nel nominale dentro ogni fase; gli estremi sono liberi e le transizioni possono ancora saturare. Il controllore applica sempre gli stessi limiti componenti/somma e la stessa priorità FF. Non sono usate misure future o futuri orari delle fasi dei log.',
        '', 'L’inviluppo superiore non abbassa il target scalare del profilo allo stesso clock. Questo non garantisce la forza reale di ciascuna mano o i margini del full wrench. Il minimax può richiedere meno normale dell’originale: è un confronto diagnostico, non un piano accettabile con i vecchi margini dichiarati invariati.',
        '', '## Replay condizionato: risultato e controllo del clock',
        '', 'Le forze alternative mantengono il residuo registrato fisso. Nessun modello empirico aveva superato il gate di previsione libera; quindi questi numeri non prevedono una nuova run reale. Lo score resta quello congelato, calcolato sul target originale. Le differenze nei picchi del calcolo5ms sono inoltre soggette al fallimento numerico documentato sotto.',
        '', '| Legge | Piano | Feedback | Score robusto5ms ↓ | Δ rispetto al lookup originale con stesso feedback/clock | Massimo abbassamento del piano rispetto al profilo |',
        '|---|---|---|---:|---:|---:|']
    for x in paired:text.append(f"| `{x['base_controller']}` | {x['plan']} | {x['feedback_reference']} | {x['mean_run_worst_model_and_clock_score']:.3f} | {x['score_change_vs_matched_clock_control_percent']:+.2f}% | {x['max_plan_below_original_same_clock_N']:.3f} N |")
    text+=['', 'Nessun piano modificato migliora questo score aggregato della stessa legge rispetto al controllo del clock nella griglia provata. Usare il piano maggiorante anche nel feedback evita che l’I annulli parte del precarico anticipato, ma può aumentare molto la forza rispetto al target originale. Una forza più regolare non equivale automaticamente a tracking migliore.',
        '', f"Le baseline registrate restano identiche. La classe reale FFI, senza ROS, coincide con i casi I a riferimento pianificato su **{software['actual_production_I_class_updates']:,} aggiornamenti**, errore comando massimo{software['maximum_production_command_difference_mm']:.2e}mm. Verificate causalità dei prefissi, pendenza nominale≤2mm/s dentro la fase e maggiorazione scalare esatta.",
        '', '## Stress autonomi con forme di profilo reali',
        '', '**1.792 scenari autonomi più1.792 raffinamenti**:14 configurazioni a clock0 ×16 impianti ×2 condizioni ×4 forme di profilo. Le configurazioni includono alias della baseline, utili come controllo di identità e non esperimenti indipendenti. Precarico iniziale uguale al primo target route,2s di attesa, poi route e slow_seat su un calendario sintetico noto,4s finali. Questo calendario non è preso dalle future transizioni delle run reali.',
        '', 'Una condizione è senza rumore, tick25ms; l’altra ha rumore sulla misura0,013N media mani, jitter15–35ms e disturbo fisico sinusoidale0,15N/mano a1,2Hz da6s. Nessun residuo di forza registrata imposto. Ogni controller vede solo misura presente/passata e piano della fase corrente. Impianti ipotetici, due mani simmetriche: nessuna previsione di slip, riuscita o eventi di contatto.',
        '', '| Configurazione | Score robusto autonomo ↓ | Peggior RMSE route originale | Peggior errore positivo iniziale seat | Peggior banda0,2–0,8 | Peggior banda0,8–2,5 | Minimo medio mani |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for x in auto_summary:text.append(f"| `{x['id']}` | {x['score']:.3f} | {x['maximum_route_rmse_N']:.3f} N | {x['maximum_seat_positive_error_N']:.3f} N | {x['maximum_low_band_N']:.3f} N | {x['maximum_route_band_N']:.3f} N | {x['minimum_mean_hand_force_N']:.3f} N |")
    text+=['', 'Score: media per pezzo del caso peggiore fra impianti/disturbi. Gli estremi delle altre colonne possono appartenere a casi diversi. Le bande della forza includono anche variazioni volute del target: salvate separatamente le bande dell’errore target-sottratto0,2–0,8/0,8–2,5/2,5–10Hz. Non si è cambiato lo score dopo avere visto questi risultati. La banda bassa è una diagnostica su finestre brevi e detrend cubico, non una prova di stabilità.',
        '', f"Raffinamento autonomo2,5→1,25ms con identici tick e rumore: ΔRMSE massimo{auto_ref['max_absolute_difference_N']['route_rmse_N']:.6f}N, Δbanda{auto_ref['max_absolute_difference_N']['route_band_rms_N']:.6f}N, Δpicco{auto_ref['max_absolute_difference_N']['first_seat_peak_N']:.6f}N. Criteri originali rispettati. Gli alias del piano originale riproducono esattamente le baseline nello stesso benchmark.",
        '', '## Fallimento del replay5ms conservato e verifica a eventi esatti',
        '', f"Su608 raffinamenti5→2,5ms, ΔRMSE massimo{refinement['maximum_absolute_difference_N']['route_rmse_N']:.4f}N e Δbanda{refinement['maximum_absolute_difference_N']['route_band_rms_N']:.4f}N, ma **Δpicco{refinement['maximum_absolute_difference_N']['first_seat_peak_N']:.4f}N**: il criterio<0,05N FALLISCE. Non è stata cambiata la tolleranza.",
        '', 'Creato un integratore senza arrotondamento dei tempi del controllo per i soli modelli a ritardo puro:456 casi,38 varianti ×3 guadagni/ritardi ×prima development per pezzo. Gli aggiornamenti usano gli istanti effettivi; tra essi la forza ipotizzata cambia all’unione degli eventi della misura registrata e dei comandi ritardati. RMSE è integrato con i pesi temporali esatti, estremi sulle durate degli intervalli; le sole bande sono campionate su una griglia2,5ms.',
        '', f"Caso critico Z, PI + maggiorante nel feedback, clock+40ms, modello ipotetico1500N/m e230ms: picco5ms **{critical['grid5ms_seat_peak_N']:.4f}N**,2,5ms **{critical['grid2p5ms_seat_peak_N']:.4f}N**, eventi esatti **{critical['event_exact_seat_peak_N']:.4f}N**. La griglia5ms perdeva parte del massimo. Sono forze simulate nello studio condizionato, non picchi misurati con quel controllore.",
        '', f"Verificato anche il kernel a eventi con la classe FFI reale: **{exact_check['actual_production_I_updates']:,} aggiornamenti**, errore comando{exact_check['maximum_command_difference_mm']:.2e}mm. Questo controlla il software, non la validità dell’impianto o del residuo imposto.",
        '', 'La forza fra misure è ancora una ricostruzione ZOH ipotizzata; i log FFI non stabiliscono la vera forma di picchi molto brevi. Questa verifica non converte tutti i4.864 casi in simulazioni validate, né estende il risultato ai modelli risonanti/primo ordine. La tabella5ms resta un’esplorazione con i casi di picco segnalati, non una precisione da usare per piccoli vantaggi di tuning.',
        '', '## Decisione e prossima tranche',
        '', 'Nessuna variante è proposta per il robot. I piani più morbidi devono conservare margini e condizioni iniziali; il vincolo2mm/s impedisce di realizzare subito i salti del nominale, mentre preload e anticipo possono aumentare gli errori prima del salto. Il riferimento del feedback è quindi una scelta di obiettivo, non solo un filtro da aggiungere.',
        '', 'Il damping interno resta distinto e invariato. Il confronto D-only a FFI fissa rimane il prossimo esperimento fisico minimo, già descritto in DAMPING_AB_PROTOCOL.md; sorgente del servo e dati controllati mancano. Una compensazione esplicita del ritardo non è sostenuta dai modelli identificati finora. FINAL_COMPARISON_FREEZE.json congela sette configurazioni e criteri prima di leggere le quattro run riservate: prossimo batch448 casi, senza tuning successivo su quelle run. Sono ipotesi di confronto, non candidati validati per il robot.',
        '', '## Figure e riproduzione',
        '', '![Piani e riferimenti](12_piani_e_riferimenti.png)',
        '', '![Verifica del picco](13_picco_tempi_esatti.png)',
        '', 'Sorgenti: planned_reference.py, planned_reference_autonomous.py, planned_reference_validation.py, event_exact_delay.py, event_exact_validation.py, planned_reference_report.py. Disegni congelati: planned_reference_design.json e planned_reference_autonomous_design.json. Metriche, tracce e log della verifica fallita conservati nella campagna. Il kernel condiviso riceve feedback_targets opzionale; il precedente è conservato nel checkpoint0014. Originali e controller di produzione invariati.']
    (OUT/'PLANNED_REFERENCE_REPORT.md').write_text('\n'.join(text)+'\n',encoding='utf-8')
    plot_comparison(frozen);plot_critical(frozen,critical_id)
    print('Saved planned reference report; autonomous best',json.dumps(auto_summary[:3]),flush=True)


def plot_comparison(frozen):
    selected=[('I_P0_I1.6_D0','FFI originale'),
        ('I_P0_I1.6_D0__ABS_majorant_FBoriginal_O0','Inviluppo alto · feedback originale'),
        ('I_P0_I1.6_D0__ABS_majorant_FBplanned_O0','Inviluppo alto · feedback sul piano'),
        ('I_P0_I1.6_D0__ABS_minimax_FBplanned_O0','Minimax · feedback sul piano')]
    colors=['#263746','#1675bb','#dd941f','#40a69b']
    fig,axes=plt.subplots(4,2,figsize=(14,12));fig.subplots_adjust(left=.08,right=.98,top=.82,bottom=.13,hspace=.6,wspace=.25)
    fig.suptitle('Piano del nominale e riferimento del feedback',x=.08,ha='left',y=.98,fontsize=18,weight='bold')
    fig.text(.08,.94,'Sinistra: replay condizionato sui log · destra: impianto autonomo ipotetico senza rumore ·680N/m/mano,180ms',fontsize=10)
    for row,piece in enumerate('ZLST'):
        rid=frozen['split'][piece]['development'][0]['id']
        with np.load(OUT/(rid+'_planned_reference_central.npz')) as archive:cond={key:archive[key] for key in archive.files}
        with np.load(OUT/(piece+'_planned_reference_autonomous.npz')) as archive:auto={key:archive[key] for key in archive.files}
        m=next(i for i,(model,variant) in enumerate(zip(auto['model_ids'],auto['variant_ids'])) if model=='delay_k680_d0.18' and variant=='clean_fixed')
        for col in [0,1]:
            ax=axes[row,col]
            if col==0:
                x=cond['t']-cond['route0'];end=float(cond['route1']-cond['route0']);keep=(x>=end-.8)&(x<=end+1.6)
                target=cond['target'];force=cond['force'];ids=cond['candidate_ids'];title='replay · route→slow_seat'
            else:
                x=auto['t']-2;keep=(x>=0)&(x<=float(auto['route_end'])-.0)
                target=auto['target'];force=auto['force'][:,:,m];ids=auto['candidate_ids'];title='autonomo · route e scarico'
            for (candidate,label),color in zip(selected,colors):
                idx=list(ids).index(candidate);ax.plot(x[keep],force[keep,idx],color=color,lw=1,label=label)
            ax.plot(x[keep],target[keep],'k--',lw=.9,label='Target originale/mano')
            ax.set(title=piece+' · '+title,xlabel='Tempo da inizio route [s]',ylabel='Forza media mani [N]');ax.grid(alpha=.2)
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper left',bbox_to_anchor=(.075,.91),ncol=2,fontsize=9,frameon=False)
    fig.text(.08,.045,'Stessi limiti, stesso integrale FFI: cambia il piano e/o il riferimento. Il minimax può abbassare la normale richiesta dal SOCP.\n'
        'I pannelli di destra usano un calendario sintetico e un precarico diverso dalle run; non sono una previsione del robot.\n'
        'La maggiore regolarità non prova migliori margini o tracking; numeri completi e fallimento del picco5ms nel report.',fontsize=10,linespacing=1.4)
    fig.savefig(OUT/'12_piani_e_riferimenti.png',dpi=150);plt.close(fig)


def plot_critical(frozen,candidate):
    rid=frozen['split']['Z']['development'][0]['id']
    with np.load(OUT/(rid+'_event_exact_delay.npz')) as archive:exact={key:archive[key] for key in archive.files}
    with np.load(OUT/'critical_peak_grid_traces.npz') as archive:grid={key:archive[key] for key in archive.files}
    i=list(exact['candidate_ids']).index(candidate);m=list(exact['model_ids']).index('delay_k1500_d0.23')
    t=exact[f'm{m}_time_s'];y=exact[f'm{m}_force_N'][:,i]
    catalog=load('run_catalog.json') if (OUT/'run_catalog.json').exists() else load('../archive_review/run_catalog.json')
    rec=next(x for x in catalog if x['id']==rid);c.old_replay.DT=.005;prepared=c.old_replay.prepare(rec)
    seat=(t>=prepared['route1'])&(t<=prepared['route1']+1.6);imax=np.flatnonzero(seat)[np.argmax(y[seat])];center=t[imax]
    fig,axes=plt.subplots(1,2,figsize=(13,5.7));fig.subplots_adjust(left=.08,right=.98,top=.76,bottom=.24,wspace=.3)
    fig.suptitle('Un massimo breve può sfuggire alla griglia del replay',x=.08,ha='left',y=.97,fontsize=17,weight='bold')
    fig.text(.08,.88,'Caso ipotetico Z · PI + inviluppo alto nel feedback · offset+40ms ·1500N/m/mano, ritardo230ms',fontsize=10)
    ax=axes[0];keep=(t>=center-.08)&(t<=center+.08)
    ax.step((t[keep]-center)*1000,y[keep],where='post',color='#263746',lw=1.3,label='Eventi esatti, residuo ZOH')
    for name,label,color in [('grid5','Griglia5ms','#cf4e59'),('grid2p5','Griglia2,5ms','#1675bb')]:
        tt=grid[name+'_time_s'];yy=grid[name+'_force_N'];mask=(tt>=center-.08)&(tt<=center+.08)
        ax.step((tt[mask]-center)*1000,yy[mask],where='post',lw=1,color=color,label=label)
    ax.set(xlabel='Tempo dal massimo a eventi [ms]',ylabel='Forza media mani ipotizzata [N]');ax.legend(fontsize=8);ax.grid(alpha=.2)
    values=[float(grid['grid5_peak_N']),float(grid['grid2p5_peak_N']),float(y[seat].max())]
    axes[1].plot(range(3),values,'o-',color='#263746');axes[1].set_xticks(range(3),['5ms','2,5ms','Eventi esatti'])
    for j,value in enumerate(values):axes[1].annotate(f'{value:.4f}N',(j,value),xytext=(0,9),textcoords='offset points',ha='center',fontsize=10)
    axes[1].set(ylabel='Picco ipotizzato media mani [N]',ylim=(min(values)-.07,max(values)+.09),xlim=(-.35,2.35));axes[1].grid(alpha=.2)
    fig.text(.08,.075,'Sono risultati del modello condizionato, non picchi reali misurati con quel controllore.\n'
        'Il residuo tra misure è assunto a gradino: la verifica temporale non dimostra la forma di un picco fisico molto breve.\n'
        'Il test5→2,5ms fallito è conservato; non è stata allargata la tolleranza.',fontsize=10,linespacing=1.4)
    fig.savefig(OUT/'13_picco_tempi_esatti.png',dpi=150);plt.close(fig)


if __name__=='__main__':main()
