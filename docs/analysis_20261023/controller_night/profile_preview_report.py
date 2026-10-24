"""Figures/report for actual-profile preview and nominal-command feasibility."""
import profile_preview as p
import json
import numpy as np
import matplotlib.pyplot as plt

OUT=p.OUT


def load(name):return json.loads((OUT/name).read_text())


def main():
    design=p.freeze();frozen=load('frozen_design.json');groups=load('profile_preview_grouped_ranking.json')
    rows=load('profile_preview_metrics.json');audit=load('profile_preview_audit.json');refine=load('profile_preview_refinement.json')
    rank=load('profile_preview_ranking.json');feas=load('profile_nominal_feasibility.json')
    baselines={rid:next(r for r in rank if r['id']==rid) for rid in ['I_P0_I1.6_D0','PI_P0.1_I0.8_D0_R0.1']}
    comparison=[]
    for group in groups:
        score0=baselines[group['base_controller']]['mean_run_worst_model_score']
        comparison.append(dict(group,score_change_percent=100*(group['mean_run_worst_model_and_clock_score']/score0-1)))
    threshold_pass=[]
    for cfg in design['candidates']:
        rr=[r for r in rows if r['candidate']==cfg['id']]
        if all(r['route_rmse_N']<=.2 and r['route_band_rms_N']<=.05 and r['first_seat_max_positive_error_N']<=.25 for r in rr):threshold_pass.append(cfg['id'])
    p.c.dp.dump('profile_preview_summary.json',dict(grouped_comparisons=comparison,
        threshold_passing_candidates=threshold_pass,profile_hashes_all_raw_match=all(r['hash_matches_raw'] for r in audit),
        distinct_development_profile_hashes=len({r['file_sha256'] for r in audit}),
        original_holdout_evaluated=False,conclusion='No within-phase delta-preview in the tested grid improves the robust original-target score relative to its matched controller. Nominal command itself is not rate-feasible at sharp profile changes; this is not a real-force tracking impossibility bound.'))
    text=['# Preview del profilo reale e fattibilità del nominale',
        '', '**Esito: l’anticipo semplice nella fase corrente non migliora il compromesso robusto provato. Il nominale richiesto contiene tratti non realizzabili con i suoi limiti di velocità.**',
        '', '## Provenienza e informazione disponibile',
        '', f"Letti esclusivamente i profili offline delle otto run di sviluppo. Tutti e otto corrispondono esattamente al SHA-256 registrato nei metadati, senza normalizzare le righe. Contenuti distinti: {len({r['file_sha256'] for r in audit})}; le due run per pezzo usano lo stesso piano.",
        '', 'Il sorgente `insert_open.Profile.sample` interpola nella fase corrente e mantiene il primo/ultimo valore agli estremi. `controller.follow` e `move` hanno un tempo di fase ma possono terminare per eventi. La preview qui usa soltanto il profilo della fase già iniziata. Non consulta gli orari futuri delle fasi osservate né la forza futura. Un salto route→slow_seat non è anticipato con l’orario registrato della transizione.',
        '', 'La fase viene riconosciuta quando compare nel log; il tempo trascorso è calcolato causalmente dalla prima riga di quella fase. Il vero `phase_t0` non è salvato. Tre scenari di offset −40/0/+40ms valutano la sensibilità; non sono limiti di errore identificati. Non è stato scelto l’offset che fa apparire il risultato migliore.',
        '', '| Run sviluppo | RMSE target ricostruito dal clock | Massimo scarto target |',
        '|---|---:|---:|']
    for r in audit:text.append(f"| `{r['id']}` | {r['clock_reconstruction_target_rmse_N']:.4f} N | {r['clock_reconstruction_target_max_error_N']:.4f} N |")
    text+=['', 'Per conservare l’identità della baseline si applica un **delta di preview**: nominale originale esatto più la differenza tra profilo futuro e presente campionati con lo stesso clock. Quindi l’offset di ricostruzione non altera il caso senza preview. Questa scelta è dichiaratamente uno studio condizionato; non ricostruisce esattamente una nuova esecuzione che usi direttamente il profilo assoluto.',
        '', '## Confronto: stessi limiti e target originale nelle metriche',
        '', 'Cinquanta candidati ×16 modelli ×8 run = **6.400 replay condizionati**. Leggi fissate: FFI originale e PI P0,1/I0,8 con filtro P100ms. Preview 150/300/600/1200ms; tre offset di clock. Il feedback continua a inseguire il target originale. Priorità FF e tutti i limiti di corsa/velocità restano uguali.',
        '', 'Due modalità: `shift` campiona il target pianificato a t+h e può anticipare anche una riduzione; `rise_envelope` considera max[r(t+s)−1,36s], 0≤s≤h, e aumenta solo il nominale rispetto alla baseline. Il cono usa la calibrazione nominale680N/m e2mm/s. Una preview finita di questo tipo non è una pianificazione globale con vincolo di velocità.',
        '', 'Il residuo della forza registrata rimane imposto come nei confronti precedenti. Il modello comando→forza non è validato: nessuna previsione causale delle nuove forze del robot o del contatto bilaterale.',
        '', '| Legge fissata | Preview | Anticipo | Score robusto su modello e clock ↓ | Δ rispetto alla stessa legge senza preview | Minimo medio mani |',
        '|---|---|---:|---:|---:|---:|']
    for r in comparison:
        text.append(f"| `{r['base_controller']}` | {r['preview']} | {r['preview_s']*1000:.0f}ms | {r['mean_run_worst_model_and_clock_score']:.3f} | {r['score_change_percent']:+.2f}% | {r['minimum_force_mean_hand_N']:.3f} N |")
    best=comparison[0]
    text+=['', f"La migliore preview della griglia peggiora il punteggio della legge corrispondente del **{best['score_change_percent']:.2f}%**: PI + rialzi anticipati150ms. Il confronto include sempre il caso senza preview. Nessun candidato supera tutte le soglie provvisorie. Questo non esclude una pianificazione diversa o anticipi minori; non sostiene l’idea che basti traslare avanti il profilo.",
        '', f"Verifiche: baseline I e PI riprodotte esattamente su quattro metriche in tutti i modelli; prefissi invariati cambiando futuri orari/fasi/nominali dei log e tutta la forza misurata. Raffinamento su {refine['scenarios']} casi5→2,5ms: ΔRMSE massimo{refine['maximum_absolute_difference_N']['route_rmse_N']:.4f}N, Δbanda{refine['maximum_absolute_difference_N']['route_band_rms_N']:.4f}N, Δpicco{refine['maximum_absolute_difference_N']['first_seat_peak_N']:.4f}N; tolleranze originali rispettate.",
        '', '## Minimi individuali effettivamente osservati',
        '', 'Normali compensate proiettate sull’asse di ciascuna mano, con clip a zero come nel controllo; minimo nelle fasi route e slow_seat. Sono misure delle run originali. Le alternative simulate producono solo una media fra mani e non possono garantire questi minimi individuali.',
        '', '| Run sviluppo | Minimo sinistra | Minimo destra |',
        '|---|---:|---:|']
    for r in load('profile_preview_validation.json')['replay_checks']:
        left,right=r['observed_min_individual_normal_N_L_R']
        text.append(f"| `{r['id']}` | {left:.3f} N | {right:.3f} N |")
    text+=['', '## Il vincolo del nominale, misurato sul vero piano',
        '', 'Il controller vuole `ff=(target−pickup)/680`. Per seguire esattamente questo nominale a2mm/s, il profilo deve avere |dtarget/dt|≤1,36N/s. Nei profili archiviati alcuni cambiamenti sono molto più rapidi. Questo confronto usa solo sorgente, piano e limiti configurati; non richiede un modello del servo.',
        '', '| Pezzo | Picco velocità nominale richiesta | Tempo route con richiesta >2mm/s | Minimo possibile del massimo errore nominale con preview illimitata |',
        '|---|---:|---:|---:|']
    for piece in 'ZLST':
        r=next(r for r in feas['phase_metrics'] if r['piece']==piece and r['phase']=='route')
        text.append(f"| {piece} | {r['maximum_desired_nominal_rate_mm_s']:.1f} mm/s | {100*r['fraction_phase_time_target_rate_exceeds_nominal_limit']:.2f}% | {r['best_possible_max_nominal_position_error_mm']:.3f} mm/mano |")
    text+=['', '**Sono scarti di posizione del nominale, non limiti inferiori sull’errore della forza reale.** Il moto della geometria, la board e il contatto possono cambiare la forza molto più rapidamente della sola chiusura normale. L’I può modificare il comando totale e il residuo può aiutare o peggiorare: non si deduce da questa tabella che il robot debba avere quell’errore di forza.',
        '', 'La migliore approssimazione matematica usa tutta la fase già pianificata, estremi liberi e nessun ritardo: è un limite ottimistico sul solo comando. Per valori r_i con velocità equivalente v, ogni soluzione deve avere errore massimo almeno ½max(|r_i−r_j|−v|t_i−t_j|)₊. Il punto medio dell’inviluppo superiore e inferiore a pendenza v raggiunge questo limite. Verificati il caso analitico di due punti e50 casi casuali contro tutti i vincoli a coppie.',
        '', 'Il punto medio può richiedere meno normale del SOCP originario e **non è una proposta di target accettabile per i margini**. È salvato anche l’inviluppo superiore: non abbassa il target scalare e rispetta la pendenza nella singola fase, ma aumenta il precarico. Non verifica da solo equilibrio, attrito, momenti, continuità tra fasi o possibilità di raggiungere la condizione iniziale.',
        '', '## Campionamento del piano e transizioni',
        '', 'Il sorgente del planner conserva la cadenza SOCP di100ms, mentre la traccia salvata contiene il target corrente a ogni controllo20ms. La lettura del CSV interpola poi fra queste righe: un nuovo risultato SOCP può quindi diventare una rampa di soli20ms dopo un tratto costante. Inoltre il primo punto di una fase può ancora contenere il target della soluzione precedente. Il solo salto fra ultima/prima riga non descrive tutto il cambiamento attorno alla transizione.',
        '', '| Pezzo | Ultima→prima riga route/slow_seat | Ultima route→minimo primi150ms slow_seat | Tempo minimo del nominale per questa riduzione |',
        '|---|---:|---:|---:|']
    for piece in 'ZLST':
        b=next(r for r in feas['boundaries'] if r['piece']==piece and r['from_phase']=='route')
        text.append(f"| {piece} | {b['planned_endpoint_target_jump_N']:+.3f} N/mano | {b['delta_from_previous_endpoint_to_first150ms_min_N']:+.3f} N/mano | {b['minimum_nominal_unloading_motion_time_s']:.3f}s |")
    text+=['', 'Questi tempi riguardano la riduzione del nominale richiesta nei primi150ms della fase successiva, assumendo partenza esatta dall’ultimo target route e senza ritardo. Non sono il tempo di assestamento della forza reale. Il salto L richiede già oltre un secondo di variazione del nominale: aumentare D non cambia il limite software di2mm/s. Può comunque cambiare lo smorzamento della risposta reale e resta un esperimento distinto. Il campionamento del piano è verificato nel sorgente; non dimostra che sia la causa delle oscillazioni misurate.',
        '', '## Prossimo confronto sostenibile',
        '', 'Separare una pianificazione scalare con velocità esplicita dal tuning del feedback: inviluppo superiore o piano ottimizzato, distinguendo riferimento nominale e riferimento del feedback. Valutare sempre anche l’errore rispetto al target originale e gli undershoot; non conservare per dichiarazione i margini del vecchio SOCP. Le transizioni richiedono un orologio del piano realmente disponibile e la gestione delle interruzioni, senza usare il futuro dei log.',
        '', 'Il damping interno rimane prioritario come prova D-only: tutti i dati FFI sono D60/7, le due D80 appartengono a un controllo diverso e il sorgente del servo interno manca. Nessun nuovo D è stato impostato. Le quattro run riservate restano escluse; nessun candidato è pronto al robot.',
        '', '## Figure e sorgenti',
        '', '![Preview condizionata](10_preview_profilo_reale.png)',
        '', '![Fattibilità del nominale](11_fattibilita_nominale.png)',
        '', 'Sorgenti: profile_preview.py, profile_preview_refinement.py, profile_feasibility.py, profile_preview_report.py. Disegno congelato: profile_preview_design.json. Metriche complete CSV/JSON e tracce NPZ nella campagna. Kernel conditional_controllers.py esteso con nominal_wants opzionale; default conservato e verificato contro i risultati precedenti. Sorgente precedente nel checkpoint2344. NVIDIA_RUNS e baseline restano in sola lettura.']
    (OUT/'PROFILE_PREVIEW_REPORT.md').write_text('\n'.join(text)+'\n',encoding='utf-8')
    plot_preview(frozen);plot_feasibility(frozen)
    print('Preview report saved; best matched score change',best['score_change_percent'],flush=True)


def plot_preview(frozen):
    selected=[('I_P0_I1.6_D0','FFI originale'),('PI_P0.1_I0.8_D0_R0.1','PI senza preview'),
        ('I_P0_I1.6_D0__rise_envelope_H0.15_O0','FFI · rialzi 150 ms'),
        ('PI_P0.1_I0.8_D0_R0.1__rise_envelope_H0.15_O0','PI · rialzi 150 ms'),
        ('PI_P0.1_I0.8_D0_R0.1__shift_H0.6_O0','PI · anticipo semplice 600 ms')]
    colors=['#263746','#1675bb','#dd941f','#40a69b','#cf4e59']
    fig,axes=plt.subplots(4,2,figsize=(14,12));fig.subplots_adjust(left=.08,right=.98,top=.82,bottom=.12,hspace=.6,wspace=.25)
    fig.suptitle('Preview del vero profilo: replay condizionato',x=.08,ha='left',y=.98,fontsize=18,weight='bold')
    fig.text(.08,.94,'Stessa fase, nessuna forza futura · stessi limiti · modello centrale 680 N/m/mano, ritardo 180 ms · offset clock 0',fontsize=10)
    for row,piece in enumerate('ZLST'):
        rid=frozen['split'][piece]['development'][0]['id']
        with np.load(OUT/(rid+'_preview_central.npz')) as archive:tr={key:archive[key] for key in archive.files}
        x=tr['t']-tr['route0'];end=float(tr['route1']-tr['route0'])
        for col,(lo,hi) in enumerate([(5,10),(end-.8,end+1.6)]):
            ax=axes[row,col];keep=(x>=lo)&(x<=hi)
            for (candidate,label),color in zip(selected,colors):
                idx=list(tr['candidate_ids']).index(candidate)
                ax.plot(x[keep],tr['force'][keep,idx],color=color,lw=1,label=label)
            ax.plot(x[keep],tr['target'][keep],'k--',lw=.9,label='Target originale/mano')
            ax.set(title=piece+(' · route' if col==0 else ' · route→slow_seat'),xlabel='Tempo da inizio route [s]',ylabel='Forza media mani [N]',xlim=(lo,hi))
            ax.grid(alpha=.2)
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper left',bbox_to_anchor=(.075,.91),ncol=3,fontsize=9,frameon=False)
    fig.text(.08,.045,'Forze alternative ipotizzate con residuo registrato fisso: nessuna nuova misura e nessun modello validato del robot.\n'
        'Il target nelle metriche resta quello originale; una minore onda può accompagnarsi a più errore o più picco.\n'
        'Clock di fase ricostruito; il confronto robusto include anche offset±40ms e altri15 impianti ipotizzati.',fontsize=10,linespacing=1.4)
    fig.savefig(OUT/'10_preview_profilo_reale.png',dpi=150);plt.close(fig)


def plot_feasibility(frozen):
    fig,axes=plt.subplots(4,2,figsize=(14,12));fig.subplots_adjust(left=.08,right=.98,top=.82,bottom=.13,hspace=.62,wspace=.25)
    fig.suptitle('Il nominale richiesto contiene variazioni oltre 2 mm/s',x=.08,ha='left',y=.98,fontsize=18,weight='bold')
    fig.text(.08,.94,'Solo posizione nominale per mano · vero profilo offline · k_ff 680 N/m · preview ideale illimitata nella fase',fontsize=10)
    for row,piece in enumerate('ZLST'):
        rid=frozen['split'][piece]['development'][0]['id']
        with np.load(OUT/(rid+'_nominal_feasibility.npz')) as archive:tr={key:archive[key] for key in archive.files}
        t=tr['t'];desired=tr['desired_ff_mm'];best=tr['ideal_best_ff_mm'];majorant=tr['majorant_ff_mm']
        j=int(np.argmax(abs(best-desired)));lo=max(t[0],t[j]-.8);hi=min(t[-1],t[j]+.8)
        for col,(xmin,xmax) in enumerate([(t[0],t[-1]),(lo,hi)]):
            ax=axes[row,col];keep=(t>=xmin)&(t<=xmax)
            ax.plot(t[keep],desired[keep],color='#263746',lw=1,label='Nominale richiesto originale')
            ax.plot(t[keep],best[keep],color='#1675bb',lw=1.2,label='Miglior approssimazione a 2 mm/s')
            ax.plot(t[keep],majorant[keep],color='#dd941f',ls='--',lw=1,label='Inviluppo superiore a 2 mm/s')
            ax.set(title=piece+(' · intera route' if col==0 else ' · massimo scarto minimo'),xlabel='Tempo del profilo nella route [s]',ylabel='Chiusura nominale/mano [mm]',xlim=(xmin,xmax))
            ax.grid(alpha=.2)
            if col==1:ax.text(.03,.95,f"Miglior scarto massimo: {abs(best-desired).max():.3f} mm",transform=ax.transAxes,va='top',fontsize=9,
                bbox=dict(facecolor='white',edgecolor='none',alpha=.8))
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper left',bbox_to_anchor=(.075,.91),ncol=2,fontsize=9,frameon=False)
    fig.text(.08,.045,'Questo è un limite del comando nominale, NON un limite inferiore dell’errore di forza reale.\n'
        'Il moto e la board possono cambiare la forza più rapidamente della sola chiusura normale.\n'
        'Estremi liberi, nessun ritardo: curve matematiche ottimistiche. Nessuna nuova verifica dei margini SOCP o della continuità tra fasi.',fontsize=10,linespacing=1.4)
    fig.savefig(OUT/'11_fattibilita_nominale.png',dpi=150);plt.close(fig)


if __name__=='__main__':main()
