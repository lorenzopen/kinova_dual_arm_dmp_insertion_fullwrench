"""Read-only source/metadata audit. No ROS imports, CSV analysis or controller tuning."""
import sys
sys.dont_write_bytecode=True
import ast
import hashlib
import json
import subprocess
from pathlib import Path
from datetime import datetime,timezone
import conditional_controllers as c
from evaluate_locked_holdout import verify_freeze
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=c.OUT;ROOT=c.ROOT
SPECS={
 'kinova_dual_arm_dmp_insertion/ati_node.py':['AtiNode.__init__','AtiNode._msg','AtiNode._rx'],
 'kinova_dual_arm_dmp_insertion/ros_backend.py':['_Arm._ft','_Arm.ft_last','RosBackend.read','RosBackend.command','RosBackend.now','RosBackend.wait','RosBackend.ft_on_tool'],
 'kinova_dual_arm_dmp_insertion/controller.py':['CONTROL_PERIOD','SOCP_PERIOD','Insertion.tick','Insertion.normal_force','Insertion.drive'],
 'kinova_dual_arm_dmp_insertion/insert_ffi.py':['FFISettings','NominalIntegral','FFIMixin._squeeze_update'],
 'kinova_dual_arm_dmp_insertion/main.py':['Log','write_meta'],
 'launch/robot.launch.py':['_setup','generate_launch_description']}


def digest(b):return hashlib.sha256(b).hexdigest()
def definitions(source):
    out={}
    def visit(body,prefix=''):
        for node in body:
            if isinstance(node,(ast.ClassDef,ast.FunctionDef,ast.AsyncFunctionDef)):
                name=prefix+node.name;out[name]=node
                if isinstance(node,ast.ClassDef):visit(node.body,name+'.')
            elif isinstance(node,ast.Assign):
                for target in node.targets:
                    if isinstance(target,ast.Name):out[prefix+target.id]=node
    visit(ast.parse(source).body)
    return out
def semantic(node):return digest(ast.dump(node,include_attributes=False).encode())
def source_ref(path,anchor,node,sha):
    return dict(path=path,anchor=anchor,line=node.lineno,end_line=node.end_lineno,source_sha256=sha,semantic_sha256=semantic(node))
def fir(f,n,fs):
    f=np.asarray(f,dtype=float)
    # sinc avoids the removable singularity at DC.
    return abs(np.sinc(n*f/fs)/np.sinc(f/fs))


def main():
    frozen,freeze_sha,verification=verify_freeze()
    entries=[x for p in 'ZLST' for group in ['development','holdout'] for x in frozen['data_split'][p][group]]
    assert len(entries)==12
    audit=dict(created_utc=datetime.now(timezone.utc).isoformat(),freeze_sha256=freeze_sha,verification=verification,
        scope='Source and original metadata only. No CSV numerical loading, no replay or tuning. Source revision congruence is not proof of deployed uncommitted file contents.',
        sources=[],runs=[],historical_source_checks=[])
    current={}
    for path,anchors in SPECS.items():
        data=(ROOT/path).read_bytes();source=data.decode('utf-8-sig');nodes=definitions(source)
        current[path]={key:semantic(nodes[key]) for key in anchors}
        audit['sources'].append(dict(path=path,sha256=digest(data),
            references=[source_ref(path,key,nodes[key],digest(data)) for key in anchors]))
    for entry in entries:
        mpath=ROOT/entry['metadata'];meta=json.loads(mpath.read_text(encoding='utf-8-sig'));cfg=meta['config']
        row=dict(id=entry['id'],metadata=entry['metadata'],metadata_sha256=digest(mpath.read_bytes()),
            git=meta['git'],git_dirty=meta.get('git_dirty'),configured_stream_rate_hz=cfg['rate_hz'],
            log_timing=meta.get('log_timing'),arms={})
        for side,key in [('L','left'),('R','right')]:
            a=cfg[key];n=max(1,int(round(float(a['ft_rate_hz'])/cfg['rate_hz'])))
            assert a['wrench_source']=='ft' and a['ft_topic'].endswith('/ft_data_avg')
            assert cfg['rate_hz']==100 and a['ft_rate_hz']==200 and n==2
            row['arms'][side]=dict(wrench_source=a['wrench_source'],ft_topic=a['ft_topic'],
                declared_ft_rate_hz=a['ft_rate_hz'],backend_mean_samples=n)
        audit['runs'].append(row)
        for path,anchors in SPECS.items():
            proc=subprocess.run(['git','show',meta['git']+':'+path],cwd=ROOT,capture_output=True,timeout=10)
            check=dict(id=entry['id'],revision=meta['git'],path=path,available=proc.returncode==0)
            if proc.returncode:
                check['error']=proc.stderr.decode(errors='replace')[:250]
            else:
                source=proc.stdout.decode('utf-8-sig');nodes=definitions(source)
                check['historical_bytes_sha256']=digest(proc.stdout)
                check['anchors']=[dict(anchor=k,semantic_match=k in nodes and semantic(nodes[k])==current[path][k]) for k in anchors]
                check['all_selected_anchors_match']=all(x['semantic_match'] for x in check['anchors'])
            audit['historical_source_checks'].append(check)
    frequencies=np.array([.8,1.,1.5,2.5,5.,10.])
    assumptions='Ideal uniform 1kHz sensor input, every=5 block means at200Hz and last2 consecutive block means; launch defaults, not runtime sensor-rate/every evidence. No queueing, sensor firmware filter or consume-age included.'
    audit['filter_hypothesis']=dict(assumptions=assumptions,raw_sample_count=10,ideal_group_delay_ms=4.5,
        metrics=[dict(frequency_Hz=float(f),gain=float(g),amplitude_reduction_percent=float(100*(1-g)))
                 for f,g in zip(frequencies,fir(frequencies,10,1000.))])
    audit['source_checks_summary']=dict(file_revision_comparisons=len(audit['historical_source_checks']),
        available=sum(x['available'] for x in audit['historical_source_checks']),
        all_selected_anchors_match=sum(x.get('all_selected_anchors_match',False) for x in audit['historical_source_checks']),
        selected_anchors_per_revision=sum(len(v) for v in SPECS.values()))
    # Independent known FIR limits and cascade identity, no controller simulation.
    ff=np.linspace(0,20,100)
    assert np.allclose(fir(ff,5,1000)*fir(ff,2,200),fir(ff,10,1000),atol=2e-14)
    assert fir([0],10,1000)[0]==1.
    audit['software_checks']=dict(dc_gain=1.,cascade_matches_ten_consecutive_samples=True,
        max_cascade_gain_difference=float(max(abs(fir(ff,5,1000)*fir(ff,2,200)-fir(ff,10,1000)))))
    (OUT/'SIGNAL_PIPELINE_AUDIT.json').write_text(json.dumps(audit,indent=2)+'\n',encoding='utf-8')
    report(audit)
    print('12 original metadata,24 arm configurations verified; backend mean always2 messages.')
    print('Historical source:',audit['source_checks_summary'])
    print('Ideal filter:',audit['filter_hypothesis'])


def report(audit):
    references={r['anchor']:r for item in audit['sources'] for r in item['references']}
    def cite(anchor):
        r=references[anchor];p=(ROOT/r['path']).as_posix()
        return f"[{anchor}]({p}:{r['line']})"
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.2,'axes.spines.top':False,'axes.spines.right':False})
    fig,axs=plt.subplots(1,2,figsize=(14,5.8),layout='constrained')
    f=np.linspace(.01,4,1000);h=fir(f,10,1000)
    for width,col,label in [(0,'#202630','Medie attuali ideali: 10 campioni / 1 kHz'),(.1,'#087e8b','Se aggiunta una media ideale di 100 ms'),(.2,'#c06030','Se aggiunta una media ideale di 200 ms')]:
        gain=h*abs(np.sinc(f*width));delay=.0045+width/2
        axs[0].plot(f,100*(1-gain),label=label,color=col)
        axs[1].plot(f,360*f*delay,label=label,color=col)
    axs[0].set(xlabel='Frequenza [Hz]',ylabel='Riduzione d’ampiezza [%]',title='Un filtro molto breve attenua poco le onde lente')
    axs[1].set(xlabel='Frequenza [Hz]',ylabel='Fase aggiunta in ritardo [gradi]',title='Una media più lunga introduce più fase in ritardo')
    for ax in axs:ax.axvspan(.8,2.5,color='#aaa',alpha=.12);ax.set_xlim(0,4)
    axs[0].legend(fontsize=8,loc='upper left')
    fig.suptitle('La misura FFI è già mediata; qui si calcolano solo filtri ideali\nIpotesi 1 kHz, blocchi da 5 e media degli ultimi 2. Nessuna simulazione del robot o scelta di nuovi parametri.',fontsize=13)
    fig.savefig(OUT/'18_filtri_esistenti_e_ritardo.png',dpi=160);plt.close(fig)
    checks=audit['source_checks_summary'];m=audit['filter_hypothesis']['metrics']
    rows='\n'.join(f"| {x['frequency_Hz']:g} | {x['gain']:.6f} | {x['amplitude_reduction_percent']:.4f}% |" for x in m)
    selected=checks['selected_anchors_per_revision']
    text=f'''# Catena della misura e del comando: audit dei sorgenti

**La FFI non riceve il singolo campione ATI raw.** Tutti i 12 metadata originali
delle run FFI riuscite dichiarano, per entrambe le mani, `wrench_source=ft`,
topic `ft_data_avg` e rate configurato 200 Hz, stream 100 Hz. Il backend calcola
`n_avg=round((1/100)*200)=2` e passa al controller la media degli ultimi due
messaggi del buffer. «raw ft» nel CSV significa prima di bias/gravità/frame,
non senza media temporale. Nessun nuovo filtro o parametro è stato applicato.

Questa tranche legge sorgenti e metadata, **nessun CSV numerico né replay**.
I parametri dei controllori, split e risultati riservati restano congelati.

## Provenienza

Audit originale per run, SHA dei metadata e dei sorgenti, riferimenti alle
funzioni e confronto con `git show` delle revisioni registrate sono in
[SIGNAL_PIPELINE_AUDIT.json](SIGNAL_PIPELINE_AUDIT.json).
Confronti file/revisione: {checks['file_revision_comparisons']}; disponibili: {checks['available']};
tutti gli anchor selezionati semanticamente uguali al sorgente corrente:
{checks['all_selected_anchors_match']}. Sono {selected} definizioni per revisione,
confrontate come AST senza commenti/numero di riga, non solo i nomi dei file.
Questo conferma la congruenza con la revisione registrata; un metadata git_dirty
non conserva il diff del file dispiegato. Il sorgente locale immutato rispetto
allo snapshot non è da solo una prova bit-per-bit del processo sul robot.

## Percorso documentato

1. {cite('AtiNode._rx')}: accumula `every` pacchetti e pubblica la media su
   `ft_data_avg`. Il default del nodo e del launch è 5. Il raw separato viene
   pubblicato solo se abilitato. `configure_sensor=false`: il rate hardware
   non viene dimostrato dal solo default del launch.
2. {cite('AtiNode._msg')}: il timestamp ROS è l'istante in cui si crea il
   messaggio medio, non il centro temporale dei campioni hardware. I primi
   due campi del pacchetto RDT sono scartati dalla routine di ricezione.
3. {cite('_Arm._ft')} e {cite('_Arm.ft_last')}: nel buffer entrano vettori
   scalati in N/Nm; il timestamp è tenuto separatamente per l'ultimo messaggio,
   senza cronologia per ogni vettore. L'età usa il clock monotono alla callback.
4. {cite('RosBackend.read')}: media gli ultimi n_avg vettori; n_avg dipende
   dal periodo configurato dello stream, **non dal dt reale FFI**. Se il loop
   rallenta non diventa automaticamente una media più lunga. Poi rimuove
   bias/peso, trasforma il wrench e usa la stessa FK snapshot per posa e frame
   del wrench di ciascun braccio ({cite('RosBackend.ft_on_tool')}).
5. {cite('Insertion.normal_force')}: proietta la forza lungo l'asse x misurato
   di ogni mano, taglia le componenti negative a 0 e somma le due normali.
6. {cite('FFIMixin._squeeze_update')}: il trim I usa quella somma.
   {cite('NominalIntegral')} non aggiunge un filtro della forza; limita dt,
   velocità, corsa e trim. CONTROL_PERIOD 20 ms è una soglia di scheduling,
   non prova di tick realmente equispaziati.

Il controller interno `kinova_control/kinova_controller.py` non è implementato
qui: il launch passa cart_k/cart_d/kalman/friction, ma non dimostra la legge
o il filtro velocità effettivi. I commenti del launch sul Kalman non sostituiscono
quel sorgente. Non si può derivare un effetto quantitativo del damping da questa
catena esterna.

## Quanto filtrano queste medie, sotto ipotesi dichiarate

Se arrivano uniformemente 1000 campioni/s e ogni messaggio 200 Hz è davvero la
media di 5 consecutivi non sovrapposti, gli ultimi due messaggi equivalgono a
10 campioni raw consecutivi. Guadagno ideale:

`|H(f)| = |sin(pi*10*f/1000) / (10*sin(pi*f/1000))|`.

| Frequenza [Hz] | Guadagno | Riduzione d'ampiezza |
| --- | --- | --- |
{rows}

Il ritardo di gruppo del solo filtro ideale è 4,5 ms rispetto al campione raw
più recente incluso nella media. La finestra dei blocchi è già in quei 4,5 ms;
non si somma una seconda volta. Sono esclusi coda ROS, età del messaggio alla
lettura, firmware, scheduling e servo. Per ogni ramo servono i campioni consumati.
I metadata confermano il topic e il rate **dichiarato**, non il valore runtime
di `every`, il rate hardware effettivo o l'antialiasing del sensore.
L'identità della cascata è verificata numericamente, non sui campioni delle run.

Queste medie brevi, nell'ipotesi nominale, attenuano pochissimo 0,8–2,5 Hz:
non c'è motivo matematico per aspettarsi che da sole eliminino onde di 1 Hz.
Una media aggiuntiva più lunga può attenuarle, ma cambia la fase dell'anello.
La figura illustra finestre ideali 100/200 ms, senza scegliere un controller,
senza prova di stabilità/overshoot e senza fingere una simulazione delle run.

![Filtri ideali e fase](18_filtri_esistenti_e_ritardo.png)

## Il tempo del CSV non è il tempo di applicazione del nuovo ds

{cite('Insertion.drive')} usa ds per generare i target. In
{cite('Insertion.tick')} l'ordine è: pubblicazione dei target → wait → lettura
dei messaggi → eventuale update FFI → scrittura della riga. Quindi i target
cartesiani della riga sono già stati inviati; ds/target/diagnostica FFI possono
essere nuovi valori per i comandi successivi. Non assegnare al nuovo ds il
timestamp della pubblicazione precedente. I cicli senza update ripetono la
stessa ffi_seq. Il rate 100 Hz configurato e la soglia 20 ms non vanno confusi.

{cite('RosBackend.command')} mette nel messaggio il tempo ROS di pubblicazione,
ma non legge un acknowledgment del servo o il tempo effettivo di consumo.
{cite('Log')} salva `t` monotono all'atto del log (arrotondato a1ms), `t_wall`
wall clock a 0,1 ms, gli ultimi joint_stamp/ft_stamp e le età dalla ricezione
locale. La misura FFI e gli stati non hanno un unico timestamp hardware comune.
ft_stamp identifica l'ultimo messaggio visto dalla callback, non tutti i
campioni della media; una callback può intervenire durante una lettura.
Il metadata descrive già esplicitamente questa sequenza in `log_timing`.

Nei log dunque esistono stamp e indicatori di freschezza utili, ma non l'intera
cronologia acquisizione→media→consumo→applicazione. Un valore basso di ft_age
non certifica bassa latenza hardware-end-to-end: misura il tempo dall'ultima
callback locale. Il joint_age usa la ricezione, non il timestamp di acquisizione.
Non si convertono automaticamente questi campi in ritardo del servo.

## Conseguenza pratica

Alla proposta «mettiamo una moving average sul raw ATI» va aggiunta questa
precisazione: due medie sono già previste nella pipeline; per una nuova prova
occorre fissare **su quale segnale, con quale finestra e con quale timestamp**.
Non è stata modificata l'interpretazione dei risultati congelati né rilanciato
il tuning con uno shift temporale favorevole.

Per il damping la conclusione resta aperta: il confronto prioritario è
[D-only con FFI invariata](DAMPING_AB_PROTOCOL.md). Per distinguere il contributo
di misura e servo, il [protocollo dati](MINIMUM_IDENTIFICATION_PROTOCOL.md)
richiede i timestamp per campione/consumo e il comando realmente applicato.
Nessuna deduzione causale certa dell'origine delle onde dai soli sorgenti.
'''
    (OUT/'SIGNAL_PIPELINE_REPORT.md').write_text(text,encoding='utf-8')


if __name__=='__main__':main()
