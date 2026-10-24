"""Render documented results without tuning or reading the reserved CSVs."""
import advanced_controllers as a
import json
import numpy as np
import matplotlib.pyplot as plt

OUT=a.OUT


def load(name):return json.loads((OUT/name).read_text())


def main():
    design=a.freeze();rows=load('advanced_controller_metrics.json');ranking=load('advanced_controller_ranking.json')
    shortlist=load('advanced_shortlist_development_only.json')['shortlist'];baseline=next(r for r in ranking if r['id']==design['baseline_id'])
    ref=load('advanced_numerical_refinement.json');fine=load('advanced_numerical_refinement_1p25ms.json')
    strong='PD_P0.3_I0_D0.1_R0.1'
    moderate={key:max(abs(r['difference'][key]) for r in ref['cases'] if r['candidate']!=strong) for key in ref['maximum_absolute_difference_N']}
    central='delay_k680_d0.18';summaries=[]
    for cfg in shortlist:
        rr=[r for r in rows if r['candidate']==cfg['id'] and r['model']==central]
        summaries.append(dict(candidate=cfg['id'],score=cfg['mean_run_worst_model_score'],
            score_change_percent=100*(cfg['mean_run_worst_model_score']/baseline['mean_run_worst_model_score']-1),
            mean_route_rmse_N=float(np.mean([r['route_rmse_N'] for r in rr])),
            mean_route_band_N=float(np.mean([r['route_band_rms_N'] for r in rr])),
            mean_route_high_band_N=float(np.mean([r['route_high_band_rms_N'] for r in rr])),
            L_mean_seat_peak_N=float(np.mean([r['first_seat_peak_N'] for r in rr if r['piece']=='L'])),
            minimum_over_models_mean_hand_N=cfg['minimum_mean_hand_force_N']))
    # Same six parameter sets for all allocation comparisons, including unfavorable cases.
    allocations=[]
    for cfg in ranking:
        if cfg['allocation']=='ff_priority':continue
        base_id=cfg['id'].split('__')[0];matched=next(r for r in ranking if r['id']==base_id)
        allocations.append(dict(candidate=base_id,policy=cfg['allocation'],score=cfg['mean_run_worst_model_score'],
            score_change_percent=100*(cfg['mean_run_worst_model_score']/matched['mean_run_worst_model_score']-1),
            minimum_force_N=cfg['minimum_mean_hand_force_N']))
    newpd=next(r for r in ranking if r['family']=='PD' and r['kd_m_s_N']>=.00003 and r['allocation']=='ff_priority')
    auto=load('advanced_autonomous_metrics.json');auto_ref=load('advanced_autonomous_refinement.json')
    auto_summary=[]
    for cfg in load('advanced_autonomous_design.json')['candidates']:
        for variant in ['clean','noise_jitter','physical_wave_noise_jitter']:
            rr=[r for r in auto if r['candidate']==cfg['id'] and r['variant']==variant]
            auto_summary.append(dict(candidate=cfg['id'],variant=variant,
                worst_up_overshoot_N=max(r['up_overshoot_N_hand'] for r in rr),
                worst_down_undershoot_N=max(r['down_undershoot_N_hand'] for r in rr),
                worst_final_band_N=max(r['final_band_rms_N_hand'] for r in rr),
                worst_steady_error_N=max(r['steady_abs_error_N_hand'] for r in rr),
                minimum_force_N=min(r['minimum_force_N_hand'] for r in rr),
                worst_rmse_N=max(r['rmse_N_hand'] for r in rr)))
    passes=[]
    for cfg in design['candidates']:
        rr=[r for r in rows if r['candidate']==cfg['id']]
        if all(r['route_rmse_N']<=.2 and r['route_band_rms_N']<=.05 and r['first_seat_max_positive_error_N']<=.25 for r in rr):passes.append(cfg['id'])
    summary=dict(conditional_scenarios=len(rows),candidates=len(design['candidates']),central_model=summaries,
        allocation_pairs=allocations,best_new_stronger_PD=newpd,all_threshold_passing_candidates=passes,
        failed_refinement_retained=True,moderate_shortlist_5_to_2p5ms_max_difference_N=moderate,
        autonomous_summary=auto_summary,autonomous_refinement=auto_ref)
    a.c.dp.dump('advanced_summary.json',summary)
    lines=['# PD più forte, filtraggio e priorità di velocità',
        '', '**Esito: nessun nuovo controllore sostenuto dai dati. Nessun cambio al robot.**',
        '',f"Confrontati {len(design['candidates'])} candidati ×16 modelli ×8 run di sviluppo = **{len(rows):,} replay condizionati**. La griglia comprende P fino a 0,8 mm/N, D fino a 0,1 mm s/N, I 0/0,4/0,8 mm/(N s), filtro D 50ms e un ulteriore roll-off P+D 30/100ms. Le leggi iniziali restano nel confronto. Il kernel esteso riproduce tutte le 5.888 combinazioni precedenti: quattro metriche confrontate, differenze numeriche esattamente zero.",
        '', 'Il filtro P+D è causale; I separata con antiwindup sul trim effettivamente ottenuto. Le famiglie P/PD hanno solo un bias fisso di ingresso e nessuna I nascosta. Non si filtra o sostituisce il target originale per calcolare gli errori. Il residuo delle forze reali resta imposto: nessuno dei modelli empirici aveva superato il gate di simulazione libera. Questi risultati non prevedono la forza reale dopo un cambio di controllo.',
        '', '## Confronto principale: stessi limiti e priorità FFI',
        '', 'Il miglior punteggio aggregato è del PI P0,1/I0,8, roll-off P100ms: '+f"{ranking[0]['mean_run_worst_model_score']:.4f} contro {baseline['mean_run_worst_model_score']:.4f} della FFI, miglioramento **{100*(1-ranking[0]['mean_run_worst_model_score']/baseline['mean_run_worst_model_score']):.2f}%**. È inferiore all'incertezza del modello, non una prova di miglioramento del robot. Lo score combina RMSE, banda, p95 e picco finale, non è una soglia fisica o di sicurezza.",
        '', '| Candidato | Score robusto ↓ | Δ score vs FFI | RMSE route centrale | Banda0,8–2,5 centrale | Banda2,5–10 centrale | Picco L centrale | Minimo medio mani su tutti i modelli |',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for r in summaries:
        lines.append(f"| `{r['candidate']}` | {r['score']:.3f} | {r['score_change_percent']:+.2f}% | {r['mean_route_rmse_N']:.3f} N | {r['mean_route_band_N']:.3f} N | {r['mean_route_high_band_N']:.3f} N | {r['L_mean_seat_peak_N']:.3f} N | {r['minimum_over_models_mean_hand_N']:.3f} N |")
    lines+=['', 'Colonne centrali: medie delle8 run (picco L sulle due L). Score: media per run del peggiore tra16 modelli. La colonna minimi può provenire da altri scenari. Tutte le forze sono medie fra mani; non garantiscono entrambi i contatti.',
        '', f"Il migliore fra i PD nuovi con D≥0,03 mm s/N è `{newpd['id']}`, score{newpd['mean_run_worst_model_score']:.3f}. L'estensione del guadagno e il roll-off non hanno prodotto un vincitore robusto nella griglia. **Nessuno** dei112 candidati rispetta tutte le soglie provvisorie su tutti i casi.",
        '', '## Priorità nominale contro feedback: esperimento separato',
        '', 'Sei leggi predefinite sono confrontate con3 allocazioni: originale nominale-prima, feedback-prima, e proiezione della somma desiderata seguita da ripartizione a pesi uguali. In tutte: velocità FF e trim≤2mm/s, somma≤2mm/s; FF±9mm, trim±3mm, somma±12mm. Verificate240.000 proiezioni casuali raggiungibili, oltre a tutte le traiettorie simulate.',
        '', '| Legge fissata | Allocazione | Δ score rispetto alla stessa legge FFI | Minimo medio mani |',
        '|---|---|---:|---:|']
    for r in allocations:lines.append(f"| `{r['candidate']}` | {r['policy']} | {r['score_change_percent']:+.2f}% | {r['minimum_force_N']:.3f} N |")
    lines+=['', 'Dare priorità al feedback può ridurre un picco locale e peggiorare offset/minimi altrove. Non basta scegliere il tracciato più gradevole sulla L. Questi sono compromessi sotto residuo fisso, non causalità osservata.',
        '', '## Raffinamento: fallimento conservato',
        '', '128 casi su4 pezzi/4 modelli, compreso un PD forte aggiuntivo, confrontati5→2,5ms e5→1,25ms. Il criterio iniziale era ΔRMSE<0,03N, Δbanda<0,01N e Δpicco<0,05N. **La verifica complessiva fallisce**: non è stata allargata la tolleranza.',
        '', f"5→2,5ms: massimo ΔRMSE{ref['maximum_absolute_difference_N']['route_rmse_N']:.4f}N, Δbanda{ref['maximum_absolute_difference_N']['route_band_rms_N']:.4f}N, Δpicco{ref['maximum_absolute_difference_N']['first_seat_peak_N']:.4f}N. 5→1,25ms: ΔRMSE{fine['maximum_absolute_difference_N']['route_rmse_N']:.4f}N, Δbanda{fine['maximum_absolute_difference_N']['route_band_rms_N']:.4f}N, Δpicco{fine['maximum_absolute_difference_N']['first_seat_peak_N']:.4f}N.",
        '', f"Il caso critico è `{strong}` con modello1500N/m e230ms. La shortlist moderata senza questo caso ha5→2,5ms: ΔRMSE≤{moderate['route_rmse_N']:.4f}N, Δbanda≤{moderate['route_band_rms_N']:.4f}N, Δpicco≤{moderate['first_seat_peak_N']:.4f}N. Sono verifiche di un sottoinsieme, non di tutta la griglia.",
        '', 'Nel replay vengono arrotondati alla griglia i timestamp FFI registrati; il passo modifica anche il posizionamento degli aggiornamenti e degli scalini del residuo. La sensibilità del PD forte non dimostra da sola instabilità continua o divergenza del simulatore. I valori5ms di quel candidato non sono convergenti entro le tolleranze dichiarate. Conservati JSON dei fallimenti e script con assert originali.',
        '', '## Stress autonomi: senza forza registrata imposta',
        '', 'Congelate8 ipotesi prima dello stress, senza ulteriore tuning:16 impianti ×3 varianti =384 casi, più384 raffinamenti. Target4→6→4N/mano; rumore sintetico0,013N/mano e tick15–35ms; terza variante con disturbo fisico sinusoidale0,15N/mano a1,2Hz da12s e offset+0,5N/mano16–22s. Il rumore è uguale fra candidati e modelli; il controllo vede solo la misura presente/passata. La forza alternativa deriva interamente da impianto, comandi e disturbo sintetici.',
        '', '| Ipotesi | Variante | Peggior overshoot salita | Peggior undershoot discesa | Peggior banda finale | Peggior errore a regime | Minimo medio mani |',
        '|---|---|---:|---:|---:|---:|---:|']
    for r in auto_summary:
        lines.append(f"| `{r['candidate']}` | {r['variant']} | {r['worst_up_overshoot_N']:.3f} N | {r['worst_down_undershoot_N']:.3f} N | {r['worst_final_band_N']:.3f} N | {r['worst_steady_error_N']:.3f} N | {r['minimum_force_N']:.3f} N |")
    lines+=['', 'Gli estremi possono appartenere a modelli diversi. Banda finale calcolata24–32s con taglio bordi1,5s; errore a regime nelle finestre7–9 e11–12s. Overshoot3–9s, undershoot9–12s. Nei JSON sono inclusi i tempi10–90%, i valori per ogni modello, la saturazione e il secondo intervallo di banda.',
        '', f"Raffinamento2,5→1,25ms con **identici eventi di controllo e identico rumore**: ΔRMSE massimo{auto_ref['max_difference_N']['rmse_N_hand']:.6f}N, Δovershoot{auto_ref['max_difference_N']['up_overshoot_N_hand']:.6f}N, Δbanda{auto_ref['max_difference_N']['final_band_rms_N_hand']:.6f}N. Qui l'effetto del passo è contenuto; ciò non valida gli impianti ipotizzati.",
        '', 'Verifica indipendente della baseline autonoma con la vera classe numerica NominalIntegral estratta dal sorgente: **61.648 aggiornamenti su 48 scenari**, errore comando esattamente zero. Nessuna importazione ROS.',
        '', 'Nel caso PD forte alcuni impianti ipotizzati mantengono una banda di circa 0,28N RMS anche senza rumore o disturbo esterno. Questo mostra un rischio del guadagno nel modello, non identifica la causa delle onde del robot. Una maggiore reazione al rumore è visibile nel confronto a pari scala delle figure.',
        '', '## Damping e prossima decisione',
        '', 'D esterno sulla forza e cart_d interno sulla velocità restano due interventi diversi. Nessuno di questi test risolve la mancanza del sorgente del servo interno o di un confronto D-only a FFI fissa. Rimane prioritario il protocollo DAMPING_AB_PROTOCOL.md; non sono stati cambiati K, D o sorgenti di produzione.',
        '', 'Le4 run riservate restano escluse. Il prossimo studio è una preview del vero profilo offline o una pianificazione fattibile in velocità, con target originale nelle metriche e senza usare il futuro della misura o gli orari futuri registrati delle fasi. Solo dopo congelare i finalisti e valutare una volta il holdout retrospettivo.',
        '', '## Figure e riproduzione',
        '', '![Replay condizionato](08_PD_forte_e_priorita.png)',
        '', '![Stress autonomi](09_stress_autonomi_filtri.png)',
        '', 'Sorgenti: advanced_controllers.py, advanced_refinement.py, advanced_autonomous_stress.py e advanced_report.py; disegni congelati advanced_controller_design.json e advanced_autonomous_design.json. Kernel conditional_controllers.py esteso con roll-off opzionale e callback di allocazione; default identico ai risultati precedenti. Il kernel precedente è conservato nel checkpoint2314. Tutti gli output restano nella directory di campagna; gli originali sono solo letti.']
    (OUT/'ADVANCED_CONTROL_REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    a.plot(design,shortlist,load('frozen_design.json'));plot_autonomous()
    print('Saved report; moderate refinement',moderate,flush=True)
    print('Allocation pairs',json.dumps(allocations),flush=True)


def plot_autonomous():
    tr=np.load(OUT/'advanced_autonomous_traces.npz');t=tr['t']
    selected=[('I_P0_I1.6_D0','FFI'),('PI_P0.1_I0.8_D0_R0.1','PI filtrato P0,1 / I0,8'),
        ('PD_P0.3_I0_D0.1_R0.1','PD forte P0,3 / D0,1'),
        ('PID_P0.05_I0.8_D0.005__feedback_priority','PID · priorità feedback'),
        ('PI_P0.05_I0.8_D0__combined_total','PI · somma desiderata')]
    colors=['#253747','#1675bb','#40a69b','#cf4e59','#875032']
    fig,axes=plt.subplots(3,2,figsize=(14,11));fig.subplots_adjust(left=.08,right=.98,top=.80,bottom=.15,hspace=.5,wspace=.22)
    fig.suptitle('Stress autonomi: step, rumore, jitter e disturbo periodico',x=.08,ha='left',y=.98,fontsize=17,weight='bold')
    fig.text(.08,.94,'Impianto ipotetico centrale: 680 N/m/mano e ritardo 180 ms · uguali limiti · nessun residuo dei log',fontsize=10)
    variants=['clean','noise_jitter','physical_wave_noise_jitter']
    titles=['Senza rumore','Rumore sensore + jitter','Disturbo fisico 1,2 Hz + rumore + jitter']
    for r,(variant,title) in enumerate(zip(variants,titles)):
        m=next(i for i,(mid,v) in enumerate(zip(tr['model_ids'],tr['variant_ids'])) if mid=='delay_k680_d0.18' and v==variant)
        for col,(lo,hi) in enumerate([(2,12),(24,28)]):
            ax=axes[r,col];keep=(t>=lo)&(t<=hi)
            for (rid,label),color in zip(selected,colors):
                idx=list(tr['candidate_ids']).index(rid)
                ax.plot(t[keep],tr['force'][keep,idx,m],color=color,lw=1,label=label)
            ax.plot(t[keep],tr['target'][keep],'k--',lw=1,label='Target originale/mano')
            ax.set(title=title+(' · step' if col==0 else ' · regime'),xlabel='Tempo [s]',ylabel='Forza fisica simulata/mano [N]',xlim=(lo,hi))
            if col==1:
                ax.set_ylim(3.74,4.26)
                ax.ticklabel_format(axis='y',style='plain',useOffset=False)
            ax.grid(alpha=.2)
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper left',bbox_to_anchor=(.075,.91),ncol=3,fontsize=9,frameon=False)
    fig.text(.08,.025,'La curva mostra la forza fisica del modello; il rumore è aggiunto solo alla misura del controller. Stessa scala a destra.\n'
        '384 scenari e 384 raffinamenti: i grafici mostrano un solo impianto, le metriche includono tutti i 16.\n'
        'Nessun modello validato del robot, nessuna previsione di contatto bilaterale/slip/riuscita.',fontsize=9,linespacing=1.3)
    fig.savefig(OUT/'09_stress_autonomi_filtri.png',dpi=150);plt.close(fig)


if __name__=='__main__':main()
