"""Synthesize saved observational and frozen-model evidence; no new tuning/data fitting."""
import sys
sys.dont_write_bytecode=True
import csv
import json
import hashlib
from pathlib import Path
from datetime import datetime,timezone
import conditional_controllers as c
from evaluate_locked_holdout import verify_freeze
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=c.OUT;ROOT=c.ROOT
ARCHIVE=OUT.parent/'archive_review'
HISTORICAL={
 'Z':['F75_Zt1_s9012_c100_five_cut800_online_20261022_202036'],
 'L':[f'F75_Lt1_s9014_c100_five_cut800_20261022_{s}' for s in ['150027','150751','151230']],
 'S':[f'S_s7073m3_c100_narrow75_20261022_{s}' for s in ['124524','134830']],
 'T':[f'F75_Tt1_s9007_c100_five_cut800_20261021_{s}' for s in ['201546','202847','203436']]}


def read(path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def span(values,fmt='.3f'):
    a,b=min(values),max(values)
    return format(a,fmt) if a==b else format(a,fmt)+'–'+format(b,fmt)
def table(head,rows):return '\n'.join(['| '+' | '.join(head)+' |','| '+' | '.join(['---']*len(head))+' |']+['| '+' | '.join(map(str,r))+' |' for r in rows])


def main():
    frozen,freeze_sha,verification=verify_freeze()
    inputs=[ARCHIVE/'successful_and_reference_metrics.json',ARCHIVE/'run_catalog.json',
        ARCHIVE/'L_transient_chronology.json',ARCHIVE/'REPORT.md',
        OUT/'holdout_summary.json',OUT/'SIGNAL_PIPELINE_AUDIT.json',OUT/'profile_nominal_feasibility.json',
        OUT/'EMPIRICAL_CONTROL_REPORT.md',OUT/'DAMPING_REPORT.md']
    original=read(inputs[0]);all_metrics={x['id']:x for x in original}
    catalog=read(inputs[1]);byid={x['id']:x for x in catalog}
    cohort=[];flat=[]
    for piece in 'ZLST':
        entries=frozen['data_split'][piece]
        for group in ['development','holdout']:
            for entry in entries[group]:
                m=all_metrics[entry['id']];cohort.append(m)
                assert m['status']=='curated_seat_found' and m['result']['seat_found'] and not m['result']['jammed']
                flat.append(dict(id=m['id'],piece=piece,split=group,
                    band_force_0p8_2p5_N_mean_hand=m['bands_mean_hand']['0.8_2.5']['rms_N'],
                    band_percent_of_mean_force=100*m['bands_mean_hand']['0.8_2.5']['rms_N']/m['mean_force_per_hand_N'],
                    route_RMSE_logged_target_N_mean_hand=m['tracking_rmse_mean_hand_N'],
                    slow_seat_RMSE_logged_target_N_mean_hand=m['phases']['slow_seat']['rmse_mean_hand_N'],
                    route_rate_limited_percent=100*m['ffi']['route']['rate_limited_fraction'],
                    route_trim_max_abs_mm=m['ffi']['route']['trim_abs_max_mm'],
                    believed_final_mm=m['result']['believed_mm'],source_saved_metrics=inputs[0].relative_to(ROOT).as_posix()))
    assert len(cohort)==12 and len({m['id'] for m in cohort})==12
    ffi=[x for x in catalog if '_ffi_' in x['id']]
    counts=dict(metadata_FFI_records=len(ffi),seat_found_flags=sum(bool((x['meta'].get('result') or {}).get('seat_found')) for x in ffi),
                selected_curated_successes=len(cohort))
    assert counts['metadata_FFI_records']==24 and counts['seat_found_flags']==13
    with (OUT/'thesis_observed_runs.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=list(flat[0]));w.writeheader();w.writerows(flat)
    compare=[]
    for piece in 'ZLST':
        rr=[x for x in cohort if x['piece']==piece];old=[all_metrics[cid] for cid in HISTORICAL[piece]]
        compare.append(dict(piece=piece,historical_ids=HISTORICAL[piece],FFI_ids=[x['id'] for x in rr],
            historical_metadata=[dict(id=cid,git=byid[cid]['meta'].get('git'),argv=byid[cid]['meta'].get('argv'),
                execution=byid[cid]['meta'].get('execution'),controller_parameters=byid[cid]['meta'].get('controller_parameters')) for cid in HISTORICAL[piece]],
            historical_band_minmax_N=[min(x['bands_mean_hand']['0.8_2.5']['rms_N'] for x in old),max(x['bands_mean_hand']['0.8_2.5']['rms_N'] for x in old)],
            FFI_band_minmax_N=[min(x['bands_mean_hand']['0.8_2.5']['rms_N'] for x in rr),max(x['bands_mean_hand']['0.8_2.5']['rms_N'] for x in rr)],
            historical_RMSE_minmax_N=[min(x['tracking_rmse_mean_hand_N'] for x in old),max(x['tracking_rmse_mean_hand_N'] for x in old)],
            FFI_RMSE_minmax_N=[min(x['tracking_rmse_mean_hand_N'] for x in rr),max(x['tracking_rmse_mean_hand_N'] for x in rr)]))
    sources=dict(observed='thesis_observed_runs.csv',archive='../archive_review/REPORT.md',
        models='EMPIRICAL_CONTROL_REPORT.md',holdout='HOLDOUT_REPORT.md',damping='DAMPING_REPORT.md',
        D_only='DAMPING_AB_PROTOCOL.md',pipeline='SIGNAL_PIPELINE_REPORT.md',feasibility='PROFILE_PREVIEW_REPORT.md')
    claims=[
        dict(claim='Twelve selected archived FFI successes on four trajectories, three repeats per piece, seat_found and no JAM flags.',kind='observed_selected_records',evidence=[sources['observed'],sources['archive']],limit='Not12/12 attempts or a population success probability; insertion outcome is not independent final-pose metrology.'),
        dict(claim='Residual route band RMS and tracking error are separate measured properties; no universal thesis acceptance limit was established.',kind='observed_metrics_and_scope',evidence=[sources['observed']],limit='RMS band is not peak-to-peak amplitude or standard deviation of an independent white-noise sensor.'),
        dict(claim='Specific historical references have greater selected-band RMS; Z20:20 has smaller full-route RMSE than these FFI Z repeats.',kind='descriptive_unmatched_comparison',evidence=['THESIS_EVIDENCE.json'],limit='Controller, target/planning/preload and conditions differ; no causal isolation of SOCP offline or FFI contribution.'),
        dict(claim='All12 confirmed FFI records use D60/7; D80 online records are unmatched.',kind='runtime_metadata',evidence=['damping_metadata_audit.json'],limit='Cannot identify sufficient or optimal cart_d; actual inner-servo source missing.'),
        dict(claim='L overshoot accompanies continued measured tool closure while published targets open and integral trim remains fixed.',kind='observed_chronology_and_source',evidence=['../archive_review/L_TRANSIENT.md'],limit='Does not uniquely identify pure delay, windup, board impact or servo mechanics.'),
        dict(claim='At k_eff680 and nominal speed2mm/s the nominal model permits1.36N/s; plans contain sharper changes.',kind='declared_nominal_model_and_profile',evidence=[sources['feasibility']],limit='Not a lower bound on real force error or a universal contact force slew limit.'),
        dict(claim='PI variants change frozen conditional score only slightly;0/448 cases satisfy all provisional goals.',kind='conditional_replay',evidence=[sources['holdout']],limit='Recorded residual/contact/phases fixed; model predictive gates failed; no validated robot recommendation.'),
        dict(claim='Autonomous stress and local pole tests verify behavior only under stated hypothetical plants.',kind='synthetic_and_analytical',evidence=['ADVANCED_CONTROL_REPORT.md','LINEAR_ROBUSTNESS_REPORT.md'],limit='Neither exact baseline replay nor software refinement identifies the robot or establishes full-wrench margins.'),
        dict(claim='ATI source is already temporally averaged before FFI; further long averaging affects phase as well as attenuation.',kind='source_metadata_and_filter_algebra',evidence=[sources['pipeline']],limit='Default block size/hardware rate not runtime evidence; filter-only graph is not a new controller prediction.'),
        dict(claim='Next discriminating comparison is fixed-FFI repeated D-only, after confirming actual servo law, with synchronized signal acquisition/consumption.',kind='future_protocol_not_executed',evidence=[sources['D_only'],'MINIMUM_IDENTIFICATION_PROTOCOL.md'],limit='No robot experiment, new gains, deployment or acceptance threshold authorized/executed by this research.')]
    ledger=dict(created_utc=datetime.now(timezone.utc).isoformat(),freeze_sha256=freeze_sha,verification=verification,
        scope='Aggregation and writing from saved metrics only; no new source CSV loading, simulation or tuning.',
        input_files=[dict(path=p.relative_to(ROOT).as_posix(),sha256=sha(p)) for p in inputs],
        counts=counts,cohort=flat,descriptive_historical_comparison=compare,claims=claims)
    (OUT/'THESIS_EVIDENCE.json').write_text(json.dumps(ledger,indent=2)+'\n',encoding='utf-8')
    plot(flat)
    write_report(cohort,flat,compare,counts,read(OUT/'holdout_summary.json'),sources)
    print('Synthesis from saved metrics:',counts,';10 explicitly qualified claims; no simulation/tuning.')
    print('Z20:20 versus current FFI descriptive:',compare[0])


def plot(flat):
    plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.18})
    fig,axs=plt.subplots(1,2,figsize=(13,5.6),layout='constrained')
    colors=['#2274a5','#d16a32','#2e8a6c','#8664af']
    for i,(piece,col) in enumerate(zip('ZLST',colors)):
        rr=[x for x in flat if x['piece']==piece]
        for j,r in enumerate(rr):
            for ax,key in zip(axs,['band_force_0p8_2p5_N_mean_hand','route_RMSE_logged_target_N_mean_hand']):
                ax.scatter(i+(j-1)*.16,r[key],s=75,color=col,facecolor='white' if r['split']=='holdout' else col,
                           edgecolor=col,linewidth=1.8,zorder=3)
        for ax in axs:ax.set_xticks(range(4),list('ZLST'));ax.set_xlim(-.5,3.5)
    axs[0].axhline(.05,color='#666',ls='--',lw=1,label='Obiettivo esplorativo della ricerca')
    axs[1].axhline(.2,color='#666',ls='--',lw=1,label='Obiettivo esplorativo della ricerca')
    axs[0].set(title='Oscillazione nella banda scelta',ylabel='RMS forza 0,8–2,5 Hz [N / media mani]',ylim=(0,.145))
    axs[1].set(title='Tracking dell’intera route',ylabel='RMSE sul target registrato [N / media mani]',ylim=(0,.65))
    axs[0].legend(fontsize=8,loc='lower left')
    fig.suptitle('Dodici esecuzioni FFI archiviate come riuscite: prestazioni OSSERVATE\nTre punti per pezzo; vuoto = terza run riservata al tuning notturno. Linee: obiettivi provvisori, non criteri di accettazione tesi.',fontsize=12)
    fig.savefig(OUT/'19_risultati_sperimentali_per_tesi.png',dpi=160);plt.close(fig)


def write_report(cohort,flat,compare,counts,holdout,sources):
    observed=table(['Pezzo','RMS banda [N]','RMS banda / media [%]','RMSE route [N]','RMSE slow_seat [N]','Rate limited route [%]'],[
        [p,span([r['band_force_0p8_2p5_N_mean_hand'] for r in flat if r['piece']==p]),
         span([r['band_percent_of_mean_force'] for r in flat if r['piece']==p],'.2f'),
         span([r['route_RMSE_logged_target_N_mean_hand'] for r in flat if r['piece']==p]),
         span([r['slow_seat_RMSE_logged_target_N_mean_hand'] for r in flat if r['piece']==p]),
         span([r['route_rate_limited_percent'] for r in flat if r['piece']==p],'.1f')] for p in 'ZLST'])
    historical=table(['Pezzo e riferimenti','Banda storica [N]','Banda FFI [N]','RMSE storico [N]','RMSE FFI [N]'],[
        [x['piece']+(' (Z 22/10 20:20)' if x['piece']=='Z' else f" ({len(x['historical_ids'])} run)"),
         span(x['historical_band_minmax_N']),span(x['FFI_band_minmax_N']),span(x['historical_RMSE_minmax_N']),span(x['FFI_RMSE_minmax_N'])] for x in compare])
    estimates=table(['Pezzo','believed_mm finale, range fra tre run'],[[p,span([r['believed_final_mm'] for r in flat if r['piece']==p],'.2f')] for p in 'ZLST'])
    text=f'''# Interpretazione dei risultati per la tesi

**La documentazione sostiene una dimostrazione sperimentale con limiti
quantificati. Non sostiene tracking preciso in ogni fase o oscillazioni eliminate.**
Non esiste nei requisiti forniti una soglia universale che renda un'oscillazione
«accettabile per una tesi». Serve dichiarare il contributo, le metriche e i limiti.
Questa sintesi non cambia i dati o i parametri per far raggiungere le soglie.

## Sul damping, prima di scegliere un controllo più complesso

L'ipotesi «magari bastava alzare il damping» è ancora **aperta**. Tutte le 12 FFI
hanno runtime D60/7. Due online D80/7 non sono confronti a FFI invariata. Il
modello URDF è fisicamente dichiarato ma non prevede bene il moto registrato;
lo sweep D60/80/100/120 è diagnostico e non sceglie un valore robot. Il parametro
ζ del replay non equivale a cart_d. Non si conclude né «D80 risolve» né «D80
non può aiutare» dai risultati attuali. Il [D-only ripetuto](DAMPING_AB_PROTOCOL.md)
rimane il prossimo confronto prioritario, previo controllo della legge interna.

## Che cosa è stato osservato

Il gruppo selezionato contiene 12 esecuzioni, tre per Z/L/S/T, archiviate come
riuscite con flag `seat_found` e senza flag JAM. **Non sono 12 tentativi totali.**
Il catalogo contiene {counts['metadata_FFI_records']} record metadata FFI e
{counts['seat_found_flags']} flag seat_found; configurazioni, fasi e ragioni
di stop differiscono. Né 12/12 né 13/24 rappresentano una stima pulita della
probabilità di successo della configurazione finale. La classificazione
dell'archivio/operatore e l'esito algoritmico vanno tenuti distinti.

{observed}

Tutti i valori in N riguardano la **media delle mani**, non la somma o la singola
mano. Le statistiche sono quelle già salvate dall'analisi uniforme dell'archivio:
forza `fn_N/2`, target arrotondato del CSV, RMS sui campioni delle rispettive
fasi; bande con detrend/ricampionamento/filtro offline e bordi come nel sorgente
`archive_review/analyze.py` e nel suo helper. La banda è una metrica della
forza, non dell'errore; comprende anche eventuali variazioni volute del target.
Non confrontarla come identica alla banda dell'errore o alle metriche del nuovo
holdout, che usano anche target preciso e altri campionamenti dichiarati.
Nessuna nuova simulazione o lettura numerica dei CSV è fatta da questa sintesi.

![Prestazioni osservate](19_risultati_sperimentali_per_tesi.png)

Le linee 0,05 N di banda e 0,20 N di RMSE sono obiettivi esplorativi dichiarati
dalla ricerca notturna, **non criteri di sicurezza o accettazione tesi**.
Una RMS di 0,1 N non vuol dire oscillazione sempre ±0,1 N o 0,2 N picco-picco.
Solo per una sinusoide pura A=√2·RMS; queste tracce hanno transitori e più bande,
quindi non si converte automaticamente la RMS in ampiezza di ogni ciclo.

Le onde sono presenti anche nelle bag ATI 200 Hz. La registrazione a posa quasi
ferma dà circa 0,013 N di deviazione standard e 0,0011 N RMS in 0,8–2,5 Hz sulla media
mani, con condizioni diverse dalle run. Questo contrasta con l'idea che tutto
sia inevitabile rumore costante del sensore; non identifica il contributo di
rumore, servo, elasticità e contatto durante il moto. Due Z con I quasi spenta
mantengono onde ma differiscono per precarico/condizioni: non isolano solo I.
Riferimento: [analisi delle bag](../archive_review/REPORT.md).

## È meglio del feedback e SOCP online?

Per questi riferimenti storici specifici, la FFI mostra meno RMS nella banda;
il tracking completo non migliora uniformemente. È un confronto descrittivo:

{historical}

Le identità esatte dei nove riferimenti sono in [THESIS_EVIDENCE.json](THESIS_EVIDENCE.json).
La Z delle 20:20 ha banda 0,156 N contro circa 0,11–0,12 N della FFI, ma RMSE
route 0,256 N contro circa 0,30–0,34 N: **meno onda non significa migliore in tutto**.
Le altre righe non sono un esperimento randomizzato o abbinato; target, precarico,
guadagni, esecuzione e condizioni possono differire. Niente percentuali causali
di miglioramento del nuovo controllore o del solo SOCP offline.

Non cambia soltanto «SOCP online/offline». La Z delle 20:20 è registrata come
`insert_open.py --online --squeeze-gain 0.5`; la famiglia corrispondente integra
l'errore ATI con la precedente legge. FFI usa nominale riferito al pickup più
trim I con dt/velocità/corsa espliciti. La posizione del calcolo SOCP è un
fattore distinto dall'architettura normale e dall'eventuale feedback wrench.
Per isolare l'offline occorrono stessa legge e condizioni, variando solo quello.
Il runtime D della 20:20 non è confermato da un metadata del servo: non usarla
come confronto D60/80.

## La L è un limite ripetibile da discutere, non da nascondere

Nelle tre L il target scende 5,36→3,79 N/mano; la forza raggiunge 6,23–6,61 N dopo
circa 0,34–0,45 s. I target pubblicati aprono mentre i tool misurati continuano
temporaneamente ad avvicinarsi. Il trim I rimane fisso; il nominale occupa il
budget di apertura 2 mm/s e il trim è lontano dal finecorsa ±3 mm. Non è evidenza
sufficiente per dichiarare windup o un urto con la board come causa unica.
[Cronologia completa](../archive_review/L_TRANSIENT.md).

La variazione nominale 1,57/680≈2,31 mm per mano richiede circa 1,15 s a 2 mm/s,
senza ritardo. È il tempo del **nominale nel modello dichiarato**, non il tempo
di assestamento reale o un limite assoluto alla velocità di variazione della
forza. D maggiore non cambia il limite software, ma può cambiare la dinamica.

La S ha la banda minore ma fatica a seguire i picchi del target; ciò appare
nel maggiore RMSE e nella limitazione di velocità. Nella posa, S/T mostrano
anche asimmetria delle normali: il replay scalare non garantisce i contatti
individuali. Positività della media o `seat_found` non dimostrano margine della
presa. La verifica full wrench e la metrologia finale restano separate.

{estimates}

`believed_mm` è una stima cinematica interna, non una misura esterna della
quota finale del pezzo. Questi dati non sostengono accuratezza uniforme entro
0,5 mm. Per dichiarare precisione di posa servono criterio e misura indipendenti.

## Che valore hanno le simulazioni della notte

I confronti FF/P/PI/PD/PID sono stati completati con stessi limiti principali,
più stress di rumore/jitter, rigidezza e ritardo. Verifiche software, causalità
e raffinamenti sono salvati, inclusi i fallimenti dei picchi sensibili alla
griglia. Non si è continuato il tuning dopo le quattro run riservate.

Nell'unico batch congelato di 448 casi, la PI filtrata cambia lo score robusto
di {holdout['controllers'][3]['holdout_score_change_percent_vs_baseline']:+.2f}%
rispetto alla FFI; nessun caso raggiunge tutti gli obiettivi esplorativi.
Il piccolo scarto non supera l'incertezza del modello. Non è stata validata
una nuova legge sul robot né una compensazione del ritardo. Le run riservate
sono holdout **retrospettivo per questo tuning**, già viste in analisi precedenti.

Tre livelli da mantenere distinti:

| Livello | Cosa fornisce | Cosa non dimostra |
| --- | --- | --- |
| Dati osservati | Prestazioni e transitori del sistema eseguito | Causa unica delle onde o risposta di un controllore mai eseguito |
| Replay condizionato | Sensibilità a leggi/impianti ipotizzati con residuo e fasi registrati fissi | Evoluzione autonoma del contatto o successo dell'inserzione alternativa |
| Simulazione autonoma e poli locali | Comportamento della matematica dichiarata e verifica implementativa | Validazione del Kinova o stabilità della presa reale |

I modelli liberi della forza dai comandi non superano il gate predittivo
esplorativo (R² mediano circa 0,36–0,40); usare moto già misurato dà un'associazione
più forte, non una previsione del nuovo moto. Fonte: [verifica empirica](EMPIRICAL_CONTROL_REPORT.md).
La I ha una ragione per compensare offset nel modello, ma non è dimostrata
causa unica delle oscillazioni; PD non è automaticamente risolutivo.

## Formulazione pronta da adattare alla tesi

> Sono state analizzate dodici esecuzioni archiviate come riuscite, distribuite
> su quattro traiettorie con tre ripetizioni per traiettoria. La configurazione
> comprende un nominale di forza derivato dal piano SOCP offline e una correzione
> integrale della normale misurata. Nella route l'RMS della forza media delle
> mani nella banda 0,8–2,5 Hz è compresa fra circa 0,06 e 0,12 N, pari a circa 1,35–2,97%
> della forza media. L'errore RMS sul riferimento registrato varia circa 0,27–0,55 N
> ed evidenzia un compromesso distinto dall'oscillazione in banda. Persistono
> transitori dipendenti dal pezzo, in particolare un eccesso di forza ripetuto
> sulla L durante la posa e difficoltà di inseguimento dei picchi sulla S.
> I confronti storici sono descrittivi e non isolano l'effetto del solo SOCP
> offline. Gli studi di controllori alternativi, pur verificati numericamente,
> non hanno fornito un modello sufficientemente predittivo o una nuova legge
> validata sul robot. I risultati documentano il funzionamento nelle condizioni
> archiviate e ne quantificano i limiti; non stabiliscono un tasso di successo
> generale, accuratezza di posa indipendentemente misurata o assenza di onde.

Questa formulazione va adattata agli obiettivi effettivi concordati per la tesi.
Non basta cambiare il nome «rumore» in «oscillazione»: occorre mostrare metriche,
finestre, condizioni e transitori, mantenendo le forze originali nei grafici.

## Prossimo esperimento minimo e materiale disponibile

Prima: confermare sorgente/versione/semantica di cart_d; poi confrontare D-only
con FFI fissa, ripetizioni bilanciate e log acquisizione–consumo–applicazione.
Se l'effetto non discrimina, identificazione separata comando→moto→forza con
ingresso noto e verifica libera su nuove run. Il [protocollo dati](MINIMUM_IDENTIFICATION_PROTOCOL.md)
è preparato; nessun robot è stato eseguito dalla campagna.

La misura ATI è già mediata: il [source audit](SIGNAL_PIPELINE_REPORT.md) spiega
le due medie e i timestamp. Filtri più lunghi o target raccordati richiedono
verificare fase, rapidità, minimi e margini originali, non solo una curva liscia.

Per riprodurre questa sintesi: `thesis_synthesis.py`, tabella delle 12 run
[thesis_observed_runs.csv](thesis_observed_runs.csv), provenance/ledger delle
affermazioni [THESIS_EVIDENCE.json](THESIS_EVIDENCE.json). Le immagini originali,
metriche e sorgenti dei singoli studi restano conservati. Nessun dato NVIDIA_RUNS,
snapshot, sorgente di produzione o vecchia analisi è stato modificato.
'''
    (OUT/'THESIS_SYNTHESIS.md').write_text(text,encoding='utf-8')


if __name__=='__main__':main()
