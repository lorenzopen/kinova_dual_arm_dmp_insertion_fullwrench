"""Generate a readable Italian report and a linked plot index from actual metrics."""
import json
from collections import Counter
from pathlib import Path

OUT=Path(__file__).resolve().parent


def fmt(x,n=2):return f'{x:.{n}f}'.replace('.',',')


def span(xs,n=2):return fmt(min(xs),n)+'–'+fmt(max(xs),n)


def main():
    metrics=json.loads((OUT/'successful_and_reference_metrics.json').read_text())
    catalog=json.loads((OUT/'run_catalog.json').read_text())
    checks=json.loads((OUT/'bag_validation.json').read_text())
    probe=json.loads((OUT/'integrator_probe.json').read_text())
    static=json.loads((OUT/'stationary_bags.json').read_text())
    manifest=json.loads((OUT/'archive_manifest_before.json').read_text())
    core=[r for r in metrics if r['status']=='curated_seat_found' and 'ffi' in r]
    assert len(core)==12
    rows=[]
    for pn in 'ZLST':
        rr=[r for r in core if r['piece']==pn]
        rows.append('| '+pn+' | '+str(len(rr))+' | '+
                    span([r['bands_mean_hand']['0.8_2.5']['rms_N'] for r in rr])+' | '+
                    span([100*r['bands_mean_hand']['0.8_2.5']['rms_N']/r['mean_force_per_hand_N'] for r in rr])+' | '+
                    span([r['tracking_rmse_mean_hand_N'] for r in rr])+' | '+
                    span([r['phases']['slow_seat']['rmse_mean_hand_N'] for r in rr])+' | '+
                    span([100*r['ffi']['route']['rate_limited_fraction'] for r in rr],1)+' |')
    table='\n'.join(rows)
    stat=static[0]['fixed_window_20_60s']
    text=f'''# Analisi dell’archivio NVIDIA_RUNS — 23 ottobre 2026

Pull completato sul branch `codex/feedforward-i-20261023`, da `c80dd78` a `51834b7`.
La manutenzione automatica Git ha allungato il pull; l’aggiornamento è stato fast-forward.
Nessun codice di controllo è stato modificato da questa analisi e nessun robot è stato comandato.

**Valutazione:** ci sono 12 prove FFI archiviate come riuscite su quattro pezzi,
con tre ripetizioni per pezzo e tutti i risultati registrati senza JAM. È una base
sperimentale utilizzabile per descrivere il funzionamento del sistema e i suoi limiti.
Non dimostra un controllo di forza preciso in ogni fase: rimangono onde durante
il movimento, inseguimento insufficiente dei picchi sulla S e sovraelongazioni
sistematiche della L nello slow_seat. Non serve presentare queste prove come perfette.

## Risultati delle 12 FFI selezionate

Tutte usano SOCP offline, k_eff 680 N/m, I = 1,6 mm/(N s) sull’errore della somma,
velocità complessiva nominale+trim 2 mm/s per mano e trim ±3 mm. Le istantanee dei
controller registrano K 1500/20, D 60/7, friction/Kalman attivi, 1 kHz. La normale
di ciascuna mano è ricostruita proiettando il wrench ATI sull’asse normale del tool
misurato, con la convenzione del logger; la somma differisce dal CSV di meno di 0,015 N.

Le metriche in newton della tabella riguardano **la media delle mani**,
`(N_L+N_R)/2`; non sono la somma e non sono medie temporali dei picchi.
Le singole mani sono disponibili nel JSON e nei grafici individuali.

| Pezzo | Run archiviate | RMS 0,8–2,5 Hz [N] | RMS banda / forza media [%] | Errore RMS intera route [N] | Errore RMS slow_seat [N] | Aggiornamenti rate limited in route [%] |
|---|---:|---:|---:|---:|---:|---:|
{table}

L’RMS di banda è una misura dell’oscillazione a frequenze selezionate. L’errore RMS
completo include anche offset, transitori e componenti fuori banda. Il minimo o
massimo dentro una finestra di cinque secondi non è l’ampiezza di ogni ciclo.

- **Z:** forza media vicina al target e onda residua di circa 0,11–0,12 N RMS.
  Il cambio di target finale lascia una carenza istantanea di circa 2 N per mano
  prima del recupero. Il tratto centrale è più regolare del transitorio.
- **L:** tracking in route migliorato rispetto alle L storiche; nello slow_seat
  la forza media delle mani arriva a circa 6,2–6,6 N su un target di circa 3,8 N.
  L’errore RMS in quella fase è circa 1,4–1,5 N: è ripetuto in tutte e tre e
  merita una discussione separata dal ripple durante il trasporto. I log mostrano
  l’evento; non identificano da soli la causa meccanica del contatto.
  L’[approfondimento cronologico successivo](L_TRANSIENT.md) mostra che, dopo il
  calo del target, il comando apre mentre i tool misurati si avvicinano ancora
  per circa 0,34–0,45 s fino al picco. Non va attribuito tutto automaticamente
  a un urto con la board: è evidente anche il transitorio di inseguimento.
- **S:** è quella con meno RMS nella banda scelta, ma con più errore nel seguire
  i picchi del riferimento. La richiesta cresce fino a circa 7,4 N/mano e il
  movimento nominale arriva in ritardo; circa il 18–19% degli aggiornamenti di
  route incontra il limite di velocità. Il tratto centrale include questo evento,
  quindi escludere solo i bordi non rende automaticamente piccolo il tracking RMS.
  Il minimo di stretta nello slow_seat arriva a 5,64 N sulla somma su almeno 7,10 N
  richiesti nella terza prova. Il margine reale al contatto va verificato, non
  dedotto soltanto da `seat_found` o dalla positività della normale.
- **T:** onda in banda di circa 0,09–0,11 N RMS sulla media mani e tracking route
  circa 0,27–0,28 N RMS. Nello slow_seat compaiono carenze transitorie: nella
  seconda prova la somma minima è 5,64 N contro un target minimo di 7,68 N.

Nessuna delle 12 prove raggiunge il limite di corsa del trim o del comando totale.
Il trim massimo assoluto in route è circa 1,64 mm, inferiore a 3 mm. Questo non
significa assenza di saturazioni: il limite di velocità interviene, soprattutto
sulla S e sulle transizioni. `ffi_i_limited` può indicare un incremento limitato
dalla velocità condivisa; non va interpretato automaticamente come trim a fondo corsa.

![Le 12 run](01_12_FFI_forze_errori.png)

![Finali confrontati](05_finali_per_pezzo.png)

## Cosa aggiungono davvero le bag

Sono state inventariate tutte le **114 MCAP** e decodificate le bag associate a
**31 run** del confronto, oltre alle due diagnostiche con I quasi spenta e alle
due registrazioni denominate “fermo”. L’associazione alle run usa la sovrapposizione
degli intervalli temporali reali della route, non il solo nome della cartella.

Le forze delle bag sono ricostruite dai messaggi `ft_data_avg`, con conversione delle
unità native, ultima tara registrata, trasformazioni sensore/tool e orientamenti
misurati nel CSV. Le bag e i CSV condividono il sensore: è una verifica della
registrazione e della trasformazione, non una misura indipendente della forza.
Per le run moderne ho verificato anche la media degli ultimi campioni ATI contro
le colonne sensoriali grezze del CSV, usando i timestamp sorgente. I vecchi CSV
senza quei timestamp permettono il confronto del segnale, ma non quella verifica
campione per campione.

Il rapporto RMS bag/CSV nei 31 confronti va da **{fmt(min(r['band_rms_bag_over_csv'] for r in checks),3)}
a {fmt(max(r['band_rms_bag_over_csv'] for r in checks),3)}**: meno dell’1% di differenza.
Le onde viste nel CSV sono presenti anche nei segnali ATI a circa 200 Hz. Non
sono spiegate dalla sola frequenza del CSV di circa 100 Hz. I topic raw da 1 kHz
non hanno campioni nelle bag delle 12 FFI: “bag completa” non significa ATI raw
registrato a 1 kHz.

![Verifica bag e CSV](03_verifica_bag_vs_csv.png)

### La registrazione a pezzo fermo

La bag `fermo_20261023_122511` dura circa 81 s di campioni bilaterali utili e
contiene movimento all’inizio e alla fine. Il tratto fisso **20–60 s** ha articolazioni
quasi ferme: deviazione standard massima delle posizioni articolari circa
{fmt(max(stat['joint_position_max_std_rad_L_R']),6)} rad. Sulla proiezione normale
sensoriale la deviazione standard della media delle mani è circa
**{fmt(stat['std_force_mean_hand_N'],3)} N** e l’RMS nella banda 0,8–2,5 Hz è circa
**{fmt(stat['bands_mean_hand']['0.8_2.5']['rms_N'],4)} N**.

Il tratto è definito esplicitamente e non scelto minimizzando il rumore di forza.
Il montaggio e le unità sono quelli della vicina run delle 12:32; senza una tara
dedicata non uso questi dati per dichiarare una forza assoluta. Il README della
run descrive l’esperimento come pezzo fermo senza loop. Questo sostiene che il
livello di variabilità durante le run non sia un inevitabile rumore di misura
costante; non isola però, a parità di posa/carico, tutti i contributi dinamici.

La bag `fermo_20261023_123102` contiene soltanto `/rosout`: non può fornire una
seconda verifica quantitativa della forza. Non è stata usata come se avesse ATI.

## Un’informazione nuova: l’onda rimane con I quasi spenta

Le Z delle 16:55 e 16:59 usano lo stesso codice `1c25bd7`, k_eff 680, K/D e
impostazioni dei controller della prima FFI riuscita. L’I vale **0,001 mm/(N s)**,
1600 volte meno del valore attuale di 1,6 mm/(N s). Sono state interrotte: servono
come diagnostica e non entrano nel gruppo dei successi.

Su una finestra comune 0–{fmt(probe[0]['window_end_s'],2)} s, prima di applicare lo stesso filtro:

| Run | I [mm/(N s)] | Forza media per mano [N] | RMS banda media mani [N] | RMS / media [%] |
|---|---:|---:|---:|---:|
'''
    for r in probe:
        text+=f'| {r["label"]} | {fmt(1000*r["ki_m_per_N_s"],3)} | {fmt(r["mean_force_mean_hand_N"])} | {fmt(r["band_rms_mean_hand_N"],3)} | {fmt(r["band_percent_mean"])} |\n'
    text+='''
Con I quasi spenta il trim varia complessivamente solo di circa 0,014–0,023 mm;
la sua componente in banda è circa 0,00002 mm RMS. L’onda misurata rimane di
circa 0,08–0,10 N RMS. Le bag corrispondenti confermano anche questi segnali.
La variazione della distanza misurata fra i tool è fortemente correlata con la
forza in banda (circa 0,98), ma questa correlazione non identifica una funzione
di trasferimento o una rigidezza indipendente.

**Quindi ridurre ancora I non promette di eliminare tutta l’onda.** La correzione
I può comunque amplificarla: nelle prove con I maggiore l’RMS è più alto e la
forza media è più vicina al target. Le prese e le forze medie non sono identiche,
perciò non attribuisco tutta la differenza al solo gain. Il target offline stesso
ha componenti in banda e il nominale continua a variare anche con I quasi nulla:
non è una prova di comando di chiusura perfettamente costante durante il movimento.

![Confronto integratore quasi spento](06_I_quasi_spenta.png)

## Il limite dinamico del piano

Con k_eff 680 N/m e 2 mm/s il nominale può seguire, nel proprio modello, una
variazione di forza di circa **1,36 N/s per mano**. Tutti e quattro i piani hanno
segmenti che chiedono variazioni più rapide. L’ammissibilità delle allocazioni
SOCP in ogni punto non dimostra la seguibilità dinamica tra punti e fasi.

Esempio già verificato sulla Z: 3,64 → 5,66 N/mano in 0,22 s richiede circa
2,97 mm di chiusura nominale per mano, cioè almeno 1,49 s al limite di velocità.
Nei log il nominale applicato è limitato mentre la richiesta cresce e poi torna
giù. La S presenta un problema simile durante il picco di fine route; la L ha
anche un evento ripetuto di forza eccessiva in slow_seat.

Questo è un limite verificabile del piano e dell’esecuzione impostata; non è
la prova che tutto dipenda da una rigidezza “vera” già identificata. Non conviene
abbassare o filtrare arbitrariamente un target necessario alla presa. Prima va
capito perché compare quel picco e se sia possibile anticipare la stretta o
raccordare i tempi mantenendo i vincoli del compito.

## Cosa fare adesso, senza cambiare parametri alla cieca

1. **Congelare la configurazione attuale come riferimento sperimentale.** È già
   riuscita su quattro route; conservarne tutti i tentativi rende il confronto
   successivo interpretabile. Non inseguire un grafico piatto sacrificando la forza
   minima o nascondendo i transitori.
2. **Analizzare il raccordo del piano e la fase di posa.** Verificare la motivazione
   dei picchi e la compatibilità con velocità/corsa, includendo to_mouth e slow_seat.
   La L richiede una verifica dell’evento di contatto sistematico; la S richiede
   soprattutto un riferimento dinamicamente seguibile. Non aumenterei I per
   compensare una richiesta che il limite del comando impedisce di eseguire.
3. **Se la priorità resta l’onda durante il trasporto, usare una prova mirata.**
   Il confronto con I quasi spenta esiste già. Il passo successivo utile è separare
   il nominale variabile, il movimento dei bracci e l’interazione col pezzo:
   confrontare un tratto ripetuto a chiusura comandata costante e il nominale
   attuale, a forza iniziale e posa comparabili, mantenendo i controlli di forza
   e tracking esistenti. Le bag hanno target_pose e joint_states per studiare
   la risposta; questi log non giustificano da soli nuovi valori di damping,
   una derivata della forza o un filtro aggiunto all’anello.
4. **Validare il risultato del pezzo con una misura esterna.** `believed_mm` è
   una stima del robot. Sulle 12 FFI è circa −0,45…+0,20 mm per Z, +2,0…+3,0 mm
   per L, +2,45…+2,90 mm per S e +0,32…+0,49 mm per T. Non va presentata come
   precisione metrologica del pezzo o come accuratezza uniforme entro 0,5 mm.

## Come scriverlo in tesi

Una formulazione sostenuta dai dati:

> Sono state analizzate dodici esecuzioni archiviate come riuscite, distribuite
> su quattro traiettorie con tre ripetizioni per traiettoria. Il controllo con
> nominale SOCP e correzione integrale mantiene la forza media prossima al
> riferimento lungo buona parte della route e presenta oscillazioni residue
> nella banda 0,8–2,5 Hz pari a circa 1,35–2,97% della forza media. La riduzione
> dell’oscillazione rispetto ai riferimenti storici non elimina i transitori:
> i picchi del piano possono superare la velocità nominale disponibile e nella
> fase di posa persistono eventi di carenza o eccesso di forza dipendenti dal
> pezzo. I risultati dimostrano il completamento ripetuto di queste traiettorie;
> non costituiscono una prova di tracking preciso in ogni fase o di robustezza
> generale del sistema.

Non dire “12/12 tentativi riusciti”: l’archivio contiene **24 record metadata FFI**,
di cui 13 con `seat_found` (uno aggiuntivo S delle 18:54, non nella selezione delle
riuscite) e 11 con arresto/interruzione, anche prima della route. Alcune prove
usano parametri diversi, quindi nemmeno 13/24 è una stima pulita della probabilità
di successo della configurazione finale. Un Ctrl-C durante release non dimostra
che l’inserimento sia fallito; un `seat_found` isolato non sostituisce l’esito
osservato dall’operatore. Il catalogo mantiene le due informazioni distinte.

Separare assestamento, tracking/disturbi ed esito del compito è coerente con
[NISTIR 8097](https://www.nist.gov/publications/benchmarking-robot-force-control-capabilities-experimental-results).
Non uso le prestazioni di altri robot come soglia universale di accettabilità
per questo setup. Il contributo difendibile è l’esecuzione sperimentale con
limiti quantificati, non l’affermazione che le oscillazioni siano sparite.

## Copertura, integrità e riproduzione

L’inventario protetto comprende **765 file, circa 4,08 GB decimali**: 258 CSV,
161 JSON, 114 MCAP e relativi documenti. Ho scritto una regola in `AGENTS.md`
alla root affinché `NVIDIA_RUNS/` sia trattata come sorgente in sola lettura,
senza cancellazioni, spostamenti o sovrascritture. Tutti gli output sono qui.
Gli SHA-256, le dimensioni e gli mtime iniziali di ogni file sono in
[archive_manifest_before.json](archive_manifest_before.json); il confronto
finale è in [archive_integrity.json](archive_integrity.json).

I 160 metadata presenti in copie diverse corrispondono a 110 nomi run unici;
le copie CSV duplicate coincidono dopo la sola normalizzazione CRLF/LF.
Aggiungendo quattro inserimenti storici senza metadata, il catalogo ha 114 record.
Sono stati analizzati 39 profili: 27 nella selezione riuscite con `seat_found`,
5 con il solo flag metadata, 3 riferimenti interrotti archiviati insieme alle
riuscite, 3 storici senza metadata e uno storico esplicitamente parziale.
I `follow2`, le baseline e i profili offline non sono contati come inserimenti.
I CSV del 16/10 sono a circa 6,7 Hz, con forza stimata pre-ATI: la banda 2,5–10 Hz
non è calcolabile e non vengono usati come confronto quantitativo delle ATI moderne.

- [Indice dei 39 grafici individuali e delle metriche](RUNS.md).
- [Confronto per pezzo con le run ATI storiche](02_confronto_per_pezzo.png).
- [Catalogo di tutti i record](run_catalog.csv), [metriche dei 39 profili](successful_summary.csv).
- [Metriche complete](successful_and_reference_metrics.json), [inventario bag](bag_inventory.json).
- [Verifiche bag](bag_validation.json), [diagnostica I quasi spenta](integrator_probe.json),
  [contenuto delle bag ferme](stationary_bags.json).
- [Esiti della lettura dei CSV](analysis_issues.json), [esiti della lettura delle bag](bag_analysis_issues.json).

Metodo: timestamp reali, forze proiettate sugli orientamenti misurati quando
disponibili, nessun filtro aggiunto ai grafici di tracking. Per la banda:
ricampionamento al dt mediano, detrend cubico, Butterworth ordine 2 SOS
avanti/indietro, esclusione di 1,5 s ai bordi. Errori: misurata meno riferimento
registrato, senza traslare il target per “cancellare” il ritardo. Il logger registra
la lettura della forza e poi aggiorna target/stati per i comandi successivi;
i transitori includono questa temporizzazione, che resta documentata nei metadata.
Le percentuali di saturazione usano cambi distinti di `ffi_seq`.

Per riprodurre, con numpy/scipy/matplotlib e mcap/mcap-ros2-support disponibili:

```text
python docs/analysis_20261023/archive_review/analyze.py
python docs/analysis_20261023/archive_review/bags.py
python docs/analysis_20261023/archive_review/integrator_probe.py
python docs/analysis_20261023/archive_review/report.py
python docs/analysis_20261023/archive_review/inventory.py verify
```

Le cache decodificate sono in `bag_cache/`, fuori dall’archivio originale.
Il file manifest iniziale non viene sovrascritto dallo script di inventario.
'''
    (OUT/'REPORT.md').write_text(text,encoding='utf-8')
    labels={'curated_seat_found':'Archivio riuscite + seat_found',
            'metadata_seat_found_only':'Solo flag metadata',
            'curated_interrupted':'Riferimento interrotto',
            'legacy_curated_no_metadata':'Storica senza metadata',
            'legacy_partial':'Esito parziale'}
    index=['# Indice dei grafici individuali','',
           'Una sola riga per run, indipendentemente dalle copie. RMS ed errore riguardano la media delle mani.',
           '','| Pezzo | Run | Stato | RMS banda [N] | Errore RMS route [N] | Bag associata |',
           '|---|---|---|---:|---:|---|']
    for r in sorted(metrics,key=lambda r:(r['piece'],r['id'])):
        bag=Path(r.get('bag_match',{}).get('path','')).parent.name if r.get('bag_match') else 'non associata'
        index.append(f'| {r["piece"]} | [{r["label"]}](per_run/{r["id"]}.png) | {labels[r["status"]]} | '
                     f'{fmt(r["bands_mean_hand"]["0.8_2.5"]["rms_N"],3)} | {fmt(r["tracking_rmse_mean_hand_N"],3)} | {bag} |')
    (OUT/'RUNS.md').write_text('\n'.join(index)+'\n',encoding='utf-8')
    print('Report e indice scritti; core',len(core),'profili',len(metrics),'bag verificate',len(checks))


if __name__=='__main__':main()
