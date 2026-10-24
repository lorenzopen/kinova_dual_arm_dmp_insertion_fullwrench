"""Describe immutable held-out outputs; never load source CSVs or rerun replay."""
import sys
sys.dont_write_bytecode=True
import conditional_controllers as c
import csv
import hashlib
import json
from datetime import datetime,timezone
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from evaluate_locked_holdout import verify_freeze

OUT=c.OUT
LABELS=['FFI attuale','FF solo','FF + P','FF + PI filtrata','FF + PD','FF + PID','FF + PI semplice']
SHORT=['FFI','FF','P','PI filtrata','PD','PID','PI semplice']
COLORS=['#202630','#b18b38','#667e34','#087e8b','#b34d95','#d26227','#4169b0']
CENTRAL='delay_k680_d0.18'


def load(name):return json.loads((OUT/name).read_text(encoding='utf-8'))
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+
                     ['| '+' | '.join(map(str,row))+' |' for row in rows])
def dump(name,value):(OUT/name).write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
def fmt(x):return f'{x:.3f}'


def main():
    design,freeze_sha,verification=verify_freeze()
    done=load('HOLDOUT_EVALUATION_COMPLETED.json')
    assert done['freeze_sha256']==freeze_sha and done['total_cases']==448
    payload=[];traces=[];rows=[]
    for item in done['results']:
        assert digest(OUT/item['result_file'])==item['result_sha256']
        assert digest(OUT/item['trace_file'])==item['trace_sha256']
        data=load(item['result_file']);payload.append(data);rows.extend(data['metrics'])
        traces.append(np.load(OUT/item['trace_file'],allow_pickle=False))
    ids=[x['id'] for x in design['candidates']]
    dev={r['id']:r for r in load('advanced_controller_ranking.json')}
    dev_rows=load('advanced_controller_metrics.json')
    summary=[]
    for i,cid in enumerate(ids):
        rr=[r for r in rows if r['candidate']==cid];assert len(rr)==64
        central=[r for r in rr if r['model']==CENTRAL];assert len(central)==4
        score=float(np.mean([max(r['score'] for r in rr if r['piece']==p) for p in 'ZLST']))
        s=dict(id=cid,label=LABELS[i],development_mean_run_worst_model_score=dev[cid]['mean_run_worst_model_score'],
            development_penalty_cases=sum(r['candidate']==cid and r['minimum_mean_hand_force_N']<=0 for r in dev_rows),
            holdout_mean_run_worst_model_score=score,
            holdout_per_piece_worst_score={p:max(r['score'] for r in rr if r['piece']==p) for p in 'ZLST'},
            exploratory_joint_pass_cases=sum(r['route_rmse_N']<=.2 and r['route_band_rms_N']<=.05 and
                 r['first_seat_max_positive_error_N']<=.25 for r in rr),
            central_mean={k:float(np.mean([r[k] for r in central])) for k in [
                'route_rmse_N','route_force_low_band_rms_N','route_error_low_band_rms_N',
                'route_force_mid_band_rms_N','route_error_mid_band_rms_N','route_force_high_band_rms_N',
                'route_error_high_band_rms_N','rate_limited_tick_fraction','trim_limited_tick_fraction']},
            central_L={k:next(r[k] for r in central if r['piece']=='L') for k in [
                'first_seat_peak_N','first_seat_max_positive_error_N','first_seat_max_negative_error_N',
                'first_seat_max_200ms_trailing_mean_force_N','first_seat_max_200ms_trailing_mean_positive_error_N']},
            full_model_minimum_mean_hand_force_N=min(r['minimum_mean_hand_force_N'] for r in rr),
            full_model_max_route_undershoot_N=max(r['route_max_negative_error_N'] for r in rr),
            full_model_max_seat_overshoot_N=max(r['first_seat_max_positive_error_N'] for r in rr),
            max_command_speed_mm_s=max(r['max_command_speed_mm_s'] for r in rr))
        summary.append(s)
    for s in summary:
        s['holdout_score_change_percent_vs_baseline']=100*(s['holdout_mean_run_worst_model_score']/summary[0]['holdout_mean_run_worst_model_score']-1)
    assert max(x['max_command_speed_mm_s'] for x in summary)<2+1e-10
    with (OUT/'holdout_metrics.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    results=dict(generated_utc=datetime.now(timezone.utc).isoformat(),freeze_sha256=freeze_sha,
        completed_utc=done['completed_utc'],cases=len(rows),verification=verification,
        metric_note='Retrospective conditional replay, mean-hand N. No retuning. Primary score unchanged; worst model per run then mean over four reserved runs.',
        joint_goal_pass_cases=sum(x['exploratory_joint_pass_cases'] for x in summary),controllers=summary,
        observed=[x['observed'] for x in payload])
    dump('holdout_summary.json',results)
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,
                         'axes.grid':True,'grid.alpha':.18,'savefig.dpi':160})
    fig,axs=plt.subplots(4,2,figsize=(15,13),layout='constrained')
    for k,(p,z) in enumerate(zip(payload,traces)):
        t=z['observed_time_from_route_s'];ph=z['observed_phase'];end=float(t[np.flatnonzero(ph=='route')[-1]])
        for j,ax in enumerate(axs[k]):
            ax.plot(t,z['observed_normal_L_R_N'][:,0],color='#117eab',lw=.9,label='Sinistra')
            ax.plot(t,z['observed_normal_L_R_N'][:,1],color='#d36c2d',lw=.9,alpha=.8,label='Destra')
            ax.step(t,z['observed_common_request_N'],where='post',color='black',ls='--',lw=1.2,label='Richiesta comune / mano')
            ax.axvline(end,color='#777',ls=':',lw=1)
            ax.set_xlim((-.05,end+1.6) if j==0 else (end-1.2,end+1.6))
            ax.set_title(f"{p['piece']} — "+('route e inizio slow_seat' if j==0 else 'dettaglio cambio fase'))
            ax.set_ylabel('Forza normale [N / mano]')
            if k==3:ax.set_xlabel('Tempo dall’inizio route [s]')
    axs[0,0].legend(loc='upper left',ncol=1,fontsize=8)
    fig.suptitle('Quattro run riservate: forze OSSERVATE, sinistra e destra\nProiezione dei campioni CSV; nessun filtro di controllo aggiunto. Linea verticale: fine route.',fontsize=15)
    fig.savefig(OUT/'14_holdout_forze_osservate.png');plt.close(fig)
    fig,axs=plt.subplots(2,2,figsize=(14,9),layout='constrained')
    show=[0,1,3,4,5,6]
    for ax,p,z in zip(axs.flat,payload,traces):
        t=z['t']-float(z['route0']);end=float(z['route1']-z['route0'])
        mi=list(z['model_ids']).index(CENTRAL)
        for i in show:
            ax.plot(t,z['force'][:,i,mi],color=COLORS[i],lw=1.6 if i==0 else 1.1,
                    alpha=1 if i==0 else .85,label=LABELS[i])
        ax.step(t,z['target'],where='post',color='black',ls='--',lw=1.2,label='Target originale')
        ax.axvline(end,color='#777',ls=':',lw=1)
        ax.set(xlim=(end-1.1,end+1.6),title=f"{p['piece']} — passaggio a slow_seat",
               xlabel='Tempo dall’inizio route [s]',ylabel='Media delle mani [N]')
    handles,labels=axs[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='outside lower center',ncol=4,fontsize=9)
    fig.suptitle('Alternative IPOTIZZATE, parametri congelati — modello centrale k=680 N/m/mano, ritardo=180 ms\nReplay con residuo registrato imposto; le curve alternative non sono misure o previsioni validate.',fontsize=13)
    fig.savefig(OUT/'15_holdout_confronto_condizionato.png');plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(14,5.8),layout='constrained')
    y=np.arange(7)
    dev_plot=[np.nan if x['development_penalty_cases'] else x['development_mean_run_worst_model_score'] for x in summary]
    axs[0].barh(y-.18,dev_plot,height=.35,color='#a7bccf',label='Sviluppo (8 run, griglia 5 ms)')
    axs[0].barh(y+.18,[x['holdout_mean_run_worst_model_score'] for x in summary],height=.35,color='#37728f',label='Riservate (4 run, griglia 2,5 ms)')
    axs[0].set(yticks=y,yticklabels=SHORT,xlabel='Media del peggiore score di modello per run',title='Stesso score congelato: più basso è meglio')
    axs[0].invert_yaxis();axs[0].legend(fontsize=8)
    for i,s in enumerate(summary):
        if s['development_penalty_cases']:
            axs[0].text(.3,i-.18,'Escluso nello sviluppo: min ≤0; penalità 10⁹',fontsize=8,va='center',color='#8a4336')
    for i,s in enumerate(summary):
        a=s['central_mean'];axs[1].scatter(a['route_rmse_N'],a['route_force_mid_band_rms_N'],color=COLORS[i],s=65)
        axs[1].annotate(SHORT[i],(a['route_rmse_N'],a['route_force_mid_band_rms_N']),xytext=(5,5 if i%2==0 else -12),textcoords='offset points',fontsize=8)
    axs[1].axhline(.05,color='#777',ls='--',lw=1);axs[1].axvline(.2,color='#777',ls='--',lw=1)
    axs[1].set(xlabel='RMSE sul target originale [N / mano]',ylabel='RMS forza 0,8–2,5 Hz [N / mano]',title='Caso centrale: media delle quattro run')
    fig.suptitle('Il compromesso resta: meno banda non assicura meno errore\nReplay condizionato; soglie tratteggiate solo esplorative, nessun criterio di sicurezza.',fontsize=14)
    fig.savefig(OUT/'16_holdout_score_compromessi.png');plt.close(fig)
    comp=table(['Configurazione','Score sviluppo','Score riservate','Δ vs FFI riservate','Casi con tutti gli obiettivi /64'],[
        [s['label'],('escluso (1,25×10⁸)' if s['development_penalty_cases'] else fmt(s['development_mean_run_worst_model_score'])),fmt(s['holdout_mean_run_worst_model_score']),f"{s['holdout_score_change_percent_vs_baseline']:+.2f}%",s['exploratory_joint_pass_cases']] for s in summary])
    real=table(['Pezzo','RMSE media [N]','RMS 0,8–2,5 Hz [N]','Minimo SX / DX [N]','Tempo SX / DX sotto richiesta di >0,25 N'],[
        [r['piece'],fmt(r['route_rmse_mean_hand_N']),fmt(r['force_mid_band_rms_N']), ' / '.join(map(fmt,r['active_min_hand_L_R_N'])),
         ' / '.join(f'{100*x:.1f}%' for x in r['fraction_route_time_hand_below_common_request_by025N_L_R'])] for r in results['observed']])
    metric=table(['Configurazione','RMSE [N]','Banda grezza / errore [N]','Picco L [N]','Eccesso L [N]','Eccesso L medio 200 ms [N]','Minimo tutti modelli [N]'],[
        [s['label'],fmt(s['central_mean']['route_rmse_N']),fmt(s['central_mean']['route_force_mid_band_rms_N'])+' / '+fmt(s['central_mean']['route_error_mid_band_rms_N']),
        fmt(s['central_L']['first_seat_peak_N']),fmt(s['central_L']['first_seat_max_positive_error_N']),fmt(s['central_L']['first_seat_max_200ms_trailing_mean_positive_error_N']),fmt(s['full_model_minimum_mean_hand_force_N'])] for s in summary])
    bands=table(['Configurazione','0,2–0,8 Hz forza / errore','2,5–10 Hz forza / errore','Tick velocità / trim limitati'],[
        [s['label'],fmt(s['central_mean']['route_force_low_band_rms_N'])+' / '+fmt(s['central_mean']['route_error_low_band_rms_N']),
         fmt(s['central_mean']['route_force_high_band_rms_N'])+' / '+fmt(s['central_mean']['route_error_high_band_rms_N']),
         f"{100*s['central_mean']['rate_limited_tick_fraction']:.1f}% / {100*s['central_mean']['trim_limited_tick_fraction']:.1f}%"] for s in summary])
    params=table(['Configurazione','P [mm/N]','I [mm/(N s)]','D [mm s/N]','Filtro aggiuntivo P+D [ms]'],[
        [LABELS[i],f"{d['kp_m_N']*1000:g}",f"{d['ki_m_N_s']*1000:g}",f"{d['kd_m_s_N']*1000:g}",f"{d.get('pd_rolloff_tau_s',0)*1000:g}"] for i,d in enumerate(design['candidates'])])
    main_pi=summary[3];simple=summary[6]
    text=f'''# Confronto congelato sulle quattro run riservate

**Nessun nuovo controllore validato sul robot.** Il batch fisso di **448 casi**
(7 configurazioni ×16 modelli ×4 run) è completato senza tuning dopo l'holdout.
La PI filtrata scelta sullo sviluppo cambia lo score rispetto alla FFI di
**{main_pi['holdout_score_change_percent_vs_baseline']:+.2f}%** sulle riservate;
la PI semplice di **{simple['holdout_score_change_percent_vs_baseline']:+.2f}%**.
Sono scarti piccoli in replay con impianti non validati. Nessuno dei {results['cases']}
casi soddisfa insieme i tre obiettivi esplorativi ({results['joint_goal_pass_cases']} casi).
Non emerge una soluzione che elimini onde e grandi sovraelongazioni mantenendo
il tracking. Questa conclusione non equivale a dichiarare inutili gli esperimenti.

## Cosa è stato congelato e cosa significano le curve

Freeze {design['created_utc']}; batch completato {done['completed_utc']}.
Manifest SHA-256 `{freeze_sha}`. Quattordici sorgenti, runtime e tutti i CSV
riservati verificati prima del batch. Risultati per run e tracce sono immutabili;
questo report legge solo i salvataggi. Baseline riprodotta con errore massimo
{done['baseline_force_max_error_N']:.2e} N e {done['baseline_command_max_error_mm']:.2e} mm:
è una verifica software del replay, non una validazione dell'impianto.

Le prime due run cronologiche riuscite per pezzo sono sviluppo, la terza è
riservata: Z 17:56:51, L 18:41:39, S 19:15:48, T 19:31:14 del 23/10 (timestamp
nel nome delle run). È un **holdout retrospettivo per questo tuning**: queste
run erano già state viste nelle vecchie analisi descrittive e moving average.
Non è una prova prospettica indipendente. Nessuna regolazione dopo questi esiti.

Le alternative sono replay condizionati: forza media originale + risposta
ipotizzata alla differenza di comando; residuo, fasi, moto geometrico e fine run
restano quelli osservati. Modelli: k=480/680/964/1500 N/m/mano, ritardi
120/180/230 ms; due lag 30/120 ms e due risonanti 1,5 Hz con ζ=0,3/0,7.
1500 è stress di guadagno, non rigidezza identificata. La risposta alternativa
non predice i due contatti separatamente né il successo dell'inserzione.
I precedenti modelli empirici e URDF non superano le verifiche predittive.

{params}

La derivata è sulla misura, filtrata a 50 ms. FF/P/PD mantengono il bias del trim
all'ingresso, senza I nascosta. Stesso nominale, target originale e priorità FF;
2 mm/s per componente e totale, FF ±9 mm, trim ±3 mm, totale ±12 mm.
Tutti i comandi rispettano 2 mm/s (tolleranza numerica 1e-10).

## Misure reali delle quattro run

{real}

RMSE pesato sul tempo, route. Minimi su route+slow_seat, proiezione del wrench
sulla normale misurata, tagliata a zero come nel controllo. Il confronto con
la richiesta scalare comune **non è un margine full wrench/frizione** e non
stabilisce la perdita di presa. Target e sensori non hanno timestamp simultanei.
La ricostruzione di fn_N dai campioni wrench/quat differisce fino a
{max(r['logged_sum_normal_reconstruction_max_discrepancy_N'] for r in results['observed']):.3f} N sulla somma:
le due rappresentazioni osservate non sono intercambiabili al millinewton.

![Forze osservate](14_holdout_forze_osservate.png)

## Confronto a parametri fissi

{comp}

Lo score congelato è RMSE/0,20 + RMS forza 0,8–2,5 Hz/0,05 + massimo eccesso
nei primi 1,6 s di slow_seat/0,25 + p95 errore assoluto route/0,50.
Il criterio congelato assegna 10⁹ al caso con forza media simulata ≤0:
FF solo lo incontra nello sviluppo in due casi S, con minimo −0,025 N.
È un'esclusione/limite fisico del modello lineare oltre il contatto, non uno
score di errore di centinaia di milioni né una forza negativa realmente misurata.
La figura annota l'esclusione senza deformare la scala degli altri confronti;
la media penalizzata esatta resta nel JSON. Nessun caso riservato ha forza media ≤0.
Per ciascuna run si prende il peggiore dei 16 modelli, poi la media delle
quattro run. La dicitura «8 run» ereditata nella descrizione della formula
del freeze indica il precedente sviluppo: l'operatore è invariato, qui sono
quattro run. Lo sviluppo è a 5 ms, l'holdout a 2,5 ms; non si interpreta il
livello assoluto tra i due insiemi come miglioramento o peggioramento temporale.
Gli obiettivi congiunti sono RMSE≤0,20 N, banda grezza≤0,05 N ed eccesso
slow_seat≤0,25 N, solo esplorativi: non soglie di sicurezza o accettazione tesi.

![Score e compromessi](16_holdout_score_compromessi.png)

Nel caso centrale (k=680, ritardo 180 ms), medie route sulle quattro run:

{metric}

Il minimo dell'ultima colonna è invece il peggiore su tutte le run e tutti i
modelli; è sempre una **media delle mani simulata**, non il minimo di un contatto.
I picchi/eccessi L sono nei primi 1,6 s di slow_seat. La media mobile causale
di 200 ms è solo una diagnostica a posteriori, non alimenta il controller e
non modifica score o criteri. Un picco più basso a scapito di tracking/minimi
non giustifica da solo la sostituzione della FFI.

{bands}

Tutte le bande sono in N/mano. La forza grezza contiene anche variazioni volute
del target; la banda dell'errore sottrae il target originale. Detrend cubico,
bandpass di analisi zero-phase, esclusi 1,5 s ai bordi: non un filtro causale
in anello. Le tabelle complete per run/modello sono in
[holdout_metrics.csv](holdout_metrics.csv) e [holdout_summary.json](holdout_summary.json).
Le frazioni di limitazione usano tutti i tick FFI del replay, non soltanto route:
«trim limitato» include velocità, ampiezza e allocazione disponibile, quindi
non è la sola permanenza sul finecorsa di ±3 mm.

![Replay di confronto](15_holdout_confronto_condizionato.png)

Figura per il solo modello centrale; P è tabulata ma omessa dal grafico per
leggibilità. Le curve a gradini seguono l'ipotesi ZOH del residuo. Già conservato
un fallimento di convergenza dei massimi puntuali nei piani modificati: fino a
0,184 N tra 5 e 2,5 ms. Una verifica a eventi esatti solo per ritardo puro ha
chiarito un massimo mancato; non valida tutti i modelli né la forza reale tra
campioni. Piccoli scarti sui picchi non sono una prova di superiorità.

## Damping, rapidità e decisione possibile adesso

Tutte le 12 FFI hanno D60/7. Le due online D80/7 differiscono per altre condizioni;
questo batch non cambia il damping interno e non risponde causalmente a
«bastava alzare D». Nel modello fisico ideale D60→80 attenua la risposta forzata
di appena 0,32–0,76% a 1 Hz e 1,37–3,4% a 2 Hz; il modello non ricostruisce
bene il moto reale. Non sono previsioni delle onde misurate e ζ dei modelli
di replay non è cart_d. Il [protocollo D-only](DAMPING_AB_PROTOCOL.md) resta
la verifica prioritaria, mantenendo FFI e limiti invariati, senza eseguire il robot.

Nel nominale quasi statico k_eff×2 mm/s=1,36 N/s/mano: una variazione di 2 N
richiede almeno 1,47 s di quel comando, più ritardo. Non è un limite universale
della forza reale, che cambia anche per moto del pezzo/contatto. Le rampe del
profilo possono chiedere 59–143 mm/s al nominale: D maggiore non rimuove
quel limite software. Le prove autonome già salvate studiano rumore/jitter e
step in entrambi i versi; il replay di queste run non aggiunge una nuova
identificazione dei tempi di salita del robot.

Non viene promosso un PID o un compensatore del ritardo dai risultati riservati.
Per discriminare servono sorgente/versione della legge cartesiana interna e
un confronto D-only ripetuto con segnali comando–posa–wrench sincronizzati.
L'identificazione con piccola eccitazione nota e margini full wrench ricalcolati
rimane necessaria prima di attribuire causalità o promettere una risposta.
Originali, snapshot, codice di produzione e vecchie analisi restano intatti;
la verifica completa SHA-256 viene aggiornata al termine della tranche.
'''
    (OUT/'HOLDOUT_REPORT.md').write_text(text,encoding='utf-8')
    for s in summary:print(s['label'], 'holdout',round(s['holdout_mean_run_worst_model_score'],6),
                            'delta%',round(s['holdout_score_change_percent_vs_baseline'],4),
                            'passes',s['exploratory_joint_pass_cases'])
    print('Observed:',json.dumps(results['observed'],indent=2))
    print('Reports/3 figures generated solely from immutable outputs; no replay rerun.')


if __name__=='__main__':main()
