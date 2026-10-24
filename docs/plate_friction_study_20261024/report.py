"""Italian study report from frozen campaign outputs; no simulation changes."""
from analyze import *
from xml.sax.saxutils import escape
from reportlab.platypus import SimpleDocTemplate,Paragraph,Spacer,Table,TableStyle,Image,PageBreak,KeepTogether
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet,ParagraphStyle
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import A4

def number(value,digits=2):return '-' if value is None else f'{value:.{digits}f}'
NAMES={'welded':'Tripod rigido','floating':'Floating plate','floating2700':'Floating, Kc=2700'}
METHOD={'none':'Nessun margine','fixed':'Margine fisso','adaptive':'Margine adattivo','wong':'Wong-inspired'}

def refinement_summary():
    records=[]
    for path in sorted((HERE/'refinement').glob('*/COMPLETED.json')):
        finer=process(path.parent);coarse=process(HERE/'runs'/finer['id'])
        fields=['endpoint_ok','outcome','final_position_mm','final_attitude_error_deg','force_wrench_rmse_N','moment_wrench_rmse_Nm','psi_min_loaded_N','margin_deficit_percent']
        records.append(dict(id=finer['id'],coarse={k:coarse[k] for k in fields},fine={k:finer[k] for k in fields}))
    assert len(records)==4
    (HERE/'REFINEMENT_COMPARISON.json').write_text(json.dumps(records,indent=2)+'\n')
    return records

def main():
    state=json.loads((HERE/'STATE.json').read_text());assert state.get('complete') and state['total']==111
    raw=json.loads((HERE/'metrics.json').read_text());records=raw['records'];assert len(records)+len(raw['exceptions'])==111
    groups=json.loads((HERE/'groups.json').read_text());audit=json.loads((HERE/'AUDIT.json').read_text())
    ref=refinement_summary()
    out=HERE/'output/pdf';out.mkdir(parents=True,exist_ok=True)
    styles=getSampleStyleSheet()
    styles.add(ParagraphStyle(name='BodyStudy',fontName='Times-Roman',fontSize=10.3,leading=13.4,spaceAfter=7))
    styles.add(ParagraphStyle(name='TitleStudy',fontName='Times-Bold',fontSize=22,leading=25,spaceAfter=14,textColor=colors.HexColor('#1f6fb2')))
    styles.add(ParagraphStyle(name='HeadStudy',fontName='Times-Bold',fontSize=15,leading=18,spaceAfter=11,textColor=colors.HexColor('#1f6fb2')))
    styles.add(ParagraphStyle(name='CaptionStudy',fontName='Times-Roman',fontSize=9,leading=11.5,spaceAfter=11))
    styles.add(ParagraphStyle(name='SmallStudy',fontName='Times-Roman',fontSize=8.5,leading=10.5,spaceAfter=5))
    story=[];markdown=['# Coni di attrito e floating plate: studio PyBullet\n']
    def text(value,style='BodyStudy'):
        story.append(Paragraph(value,styles[style]));markdown.append(value.replace('<b>','**').replace('</b>','**').replace('<br/>','\n')+'\n')
    def heading(value):
        text(value,'HeadStudy');markdown[-1]='## '+value+'\n'
    def page():story.append(PageBreak())
    def figure(name,caption,width=151*mm):
        path=HERE/'figures'/(name+'.png');assert path.exists()
        from PIL import Image as PIL
        with PIL.open(path) as im:w,h=im.size
        story.append(Image(str(path),width=width,height=width*h/w))
        story.append(Spacer(1,3*mm));text(caption,'CaptionStudy')
        markdown.append(f'![{name}](figures/{name}.png)\n')
    def table(rows,widths):
        body=[[Paragraph(escape(str(c)),styles['SmallStudy']) for c in row] for row in rows]
        t=Table(body,colWidths=widths,repeatRows=1,hAlign='LEFT')
        t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#edf3f8')),('LINEBELOW',(0,0),(-1,0),.6,colors.HexColor('#1f6fb2')),
            ('VALIGN',(0,0),(-1,-1),'TOP'),('BOTTOMPADDING',(0,0),(-1,-1),5),('TOPPADDING',(0,0),(-1,-1),5),('LINEBELOW',(0,-1),(-1,-1),.4,colors.HexColor('#bbbbbb'))]))
        story.append(t);story.append(Spacer(1,4*mm))
        markdown.extend(['| '+' | '.join(map(str,row))+' |' for row in rows[:1]])
        markdown.append('| '+' | '.join('---' for _ in rows[0])+' |')
        markdown.extend(['| '+' | '.join(map(str,row))+' |' for row in rows[1:]]);markdown.append('')

    text('Coni di attrito e floating plate','TitleStudy')
    text('Studio PyBullet - 24 ottobre 2026<br/>Confronto tra forze pianificate, contatti realizzati e wrench risultante.','CaptionStudy')
    text('<b>Risultato:</b> nel banco di prova nominale il margine fisso e quello adattivo completano tutti i 9 casi per ciascun meccanismo. La compliance modifica la distribuzione dei carichi: il floating plate mantiene i contatti ma realizza un wrench meno vicino al riferimento. La fattibilita del SOCP sul modello nominale non certifica le forze dei pad del meccanismo passivo.')
    completed=sum(r['outcome']=='complete' for r in records);endpoints=sum(r['endpoint_ok'] for r in records)
    text(f'Sono conservati <b>111 casi</b>, inclusi tutti i fallimenti: {completed} raggiungono la fine temporale e {endpoints} soddisfano le soglie finali di 20 mm / 5 gradi. Quattro pilot e quattro verifiche a passo dimezzato sono separati dal denominatore. I tre assetti iniziali sono deterministici; questi conteggi non sono probabilita sperimentali.')
    rows=[['Meccanismo / metodo','Endpoint / 9','RMSE F [N]','RMSE M [Nm]','Errore finale [mm / deg]']]
    for g in groups:
        if g['mechanism']=='floating2700':continue
        rows.append([NAMES[g['mechanism']]+' / '+METHOD[g['method']],str(g['endpoint_ok']),number(g['force_wrench_rmse_N']),number(g['moment_wrench_rmse_Nm'],3),number(g['final_position_mm'])+' / '+number(g['final_attitude_error_deg'])])
    table(rows,[64*mm,19*mm,20*mm,21*mm,27*mm])
    text('Valori continui: mediana dei 9 casi. Per le cadute, RMSE sul solo intervallo in aria registrato prima dello stop: non e un confronto a durata uguale con le run complete. Le soglie 20 mm / 5 deg sono descrittive, definite nell\'analisi e non preregistrate. Endpoint significa posa a fine movimento in aria, non inserimento o rilascio dopo appoggio.','CaptionStudy')
    text('<b>Ambito:</b> carico noto al pianificatore, traiettoria prescritta, due proxy dinamici del polso e sei contatti sferici. Non e una simulazione dei Gen3, dell\'ATI, dello stimatore di Wong o dell\'inserimento Kinova. In particolare D=15 Ns/m nel proxy non e il damping 60/80 del controller di laboratorio.')

    page();heading('1. Modello, informazione e confronto controllato')
    figure('11_information_flow','Il pianificatore riceve massa, CoM, geometria e attrito nominali. Le forze di contatto lette dal simulatore sono diagnostiche: nessun comando indipendente ai pad e nessun ritorno della posa reale al pianificatore.')
    table([['Parametro','Valore fissato'],['Oggetto','Cubo 120 mm; 1.9 kg; CoM [0,0,0], [15,0,0], [15,15,0] mm'],['Tool','3 sfere per mano, R=10 mm; triangolo r=30 mm; offset 20 mm'],['Floating plate','Periferiche 500 N/m per asse, l0=0; centrale 20000 N/m trasl.; 40/0/0 Nm/rad rot.'],['Polso proxy','K=1500 N/m, D=15 Ns/m; Krot=40 Nm/rad, Drot=1.15 Nms/rad'],['Contatto','mu fisico 0.75; nessun momento locale, rolling/spinning friction nulli; collisione plate disattivata'],['Campionamento','Fisica 0.5 ms; trace 100 Hz; 4.5 s massimi'],['Movimento','Approccio fino a 0.8 s; fixture rimossa a 1 s; tilt 0-15 deg tra 1.5 e 4 s; hold finale']], [35*mm,116*mm])
    text('Per isolare la compliance, il tripod rigido usa la stessa plate con una giunzione saldata al flange: geometria, massa, materiali e controllo del polso restano uguali. Nel floating la plate si muove per effetto delle molle; i tre pad sono solidali alla plate. La rigidezza centrale di tilt e nulla, ma le periferiche danno circa 0.675 Nm/rad: il momento flettente totale non e automaticamente zero.')
    text('Realizzazione comune: offset di posizione Fdes/K e offset angolare Mdes/Krot, poi azione elastica e dissipativa sul flange. Si tratta di un canale di impedenza idealizzato senza feedback esterno di forza. Gravita del tool compensata; nessun pavimento o supporto dell\'oggetto dopo il rilascio.')

    page();heading('2. Cosa viene ripreso dai due paper')
    text('<b>[1] Keeping Contact.</b> Si riprendono distanza dal cono, margine fisso/adattivo, confronto con assenza di margine e misure di scarico/slittamento. Il paper usa un problema planare con pivot ambientale; qui il corpo e in aria con sei contatti puntiformi e bilancio completo. Il costo resta quello TP-GMM di sessione C, solo precisione: non si sostituisce con il costo di sforzo del paper.')
    text('Per il pad i, n e la normale uscente dall\'oggetto, f la forza sull\'oggetto, N=-n dot f e T=||f+N n||. Si valutano eta=T/(mu N) e psi=(mu N-T)/sqrt(1+mu^2). Il vincolo e psi >= rho. Margine adattivo del codice corrente: rho_task=0.25 ||F_task||+1 N, con pesi dei contatti; il termine costante nel paper e 0.5 N. Il margine fisso e calibrato solo sui profili dell\'ottimizzatore, prima delle prove fisiche, per pareggiare la media di ||F_L||+||F_R||.')
    fixed=[p for p in audit['profiles'] if p['method']=='fixed']
    table([['CoM','rho fisso [N]','Forza media adattiva [N]','Forza media fissa [N]']]+[[p['case'],number(p['fixed_calibration']['rho_per_pad_N'],4),number(p['fixed_calibration']['target_mean_force_N'],3),number(p['fixed_calibration']['matched_mean_force_N'],3)] for p in sorted(fixed,key=lambda x:x['case'])],[33*mm,31*mm,44*mm,43*mm])
    text('In questo movimento la rho adattiva cambia poco con l\'angolo: circa 2.014 N al CoM centrato e 2.192-2.202 N al CoM spaziale. Il confronto quasi equivalente al fisso e un risultato atteso di questo banco; non dimostra una superiorita generale del margine adattivo. Le prove di mismatch usano soltanto l\'adattivo, quindi non confrontano la robustezza dei tre metodi.')
    text('<b>[2] Wong e colleghi.</b> Si riprende il limite aggregato sqrt(||Ft||^2+(tau_n/Reff)^2) &lt;= (1-rs) mu N, con Reff=32.8 mm e rs=0.10, e il costo quadratico sui wrench con momenti scalati da Reff. Il paper usa due patch estese 7 x 10 cm; qui e una baseline dichiarata, con lo stesso tool fisico a tre sfere. Non si assegna una coppia locale alle sfere. Il suo vincolo di bending nullo al polo aggregato e una scelta della baseline, non una proprieta generale del floating plate.')
    text('Il confronto Wong cambia insieme modello di contatto, costo e restrizione sul bending. Non isola il solo effetto dell\'ellisse, e non riproduce le fasi di stima inerziale o gli esperimenti del paper.')

    page();heading('3. I sei coni: piano e simulazione')
    figure('01_per_pad_cones','Caso dichiarato: CoM [15,15,0] mm, assetto iniziale 0 deg, margine adattivo. Punti blu/arancio: forze realizzate, t>=1.3 s, decomposte usando le normali effettive; tratteggio: testimone SOCP nominale. Area grigia: T<=0.75 N. Le forze ausiliarie del solver non sono sei setpoint attuati.',width=151*mm)
    text('Le forze possono restare dentro i coni pur essendo diverse dal testimone del solver. La distribuzione viene scelta anche dalla deformazione passiva, dalle pose dei due polsi e dalla dinamica. Nel caso rigido un pad sinistro si avvicina alla frontiera mentre altri mantengono riserva; la compliance cambia questa distribuzione.')
    text('Bullet impone gia il proprio modello di attrito. Un basso conteggio di forze fuori cono non e una validazione indipendente della legge di Coulomb e non prova assenza di slip. Per questo sono salvati anche carico normale, velocita tangenziale relativa, perdita di contatto e deriva/caduta dell\'oggetto.')

    page();heading('4. La riserva realizzata non coincide con quella imposta')
    figure('02_per_pad_margin','Stesso caso della pagina precedente. psi e calcolata dalle forze reali e dalle normali di contatto; rho e quella richiesta al modello nominale. Una curva sotto rho ha perso la riserva progettata; sotto zero violerebbe il cono. Contatto assente e riserva nulla non sono la stessa condizione.',width=151*mm)
    rigid=next(g for g in groups if g['mechanism']=='welded' and g['method']=='adaptive');floating=next(g for g in groups if g['mechanism']=='floating' and g['method']=='adaptive')
    text(f'Nel gruppo adattivo, la mediana della frazione campioni-pad con psi&lt;rho e {number(rigid["margin_deficit_percent"],1)}% nel rigido e {number(floating["margin_deficit_percent"],1)}% nel floating. In questi gruppi tutti e sei i contatti rimangono presenti; questi numeri <b>non sono percentuali di slip o fallimento</b>.')
    text('A mu e normale fissati vale psi(f+delta) >= psi(f)-||delta||. Il codice controlla questa disuguaglianza sui dati salvati. Un margine rho copre una perturbazione di forza entro rho rispetto al testimone nominale; cambiamenti di normale, geometria o mu richiedono verifiche separate. La condizione e sufficiente, non necessaria: una differenza maggiore di rho non implica automaticamente slip.')

    page();heading('5. Forze risultanti al polso')
    figure('10_wrist_forces','Sei componenti di forza, world frame, stesso caso adattivo. Le risultanti vengono ricostruite dai contatti. Il riferimento e comune ai due meccanismi; l\'RMSE valuta la norma del vettore per mano e poi aggrega tempo e mani.',width=151*mm)
    text(f'Con gli stessi profili di carico e lo stesso proxy di polso, la mediana RMSE di forza passa da {number(rigid["force_wrench_rmse_N"])} N a {number(floating["force_wrench_rmse_N"])} N. La differenza e osservata in questo sistema simulato e non identifica una causa delle oscillazioni dei motori reali.')
    text('La risultante virtuale contiene le forze di interazione ricostruite dai contatti. Non e una misura di un giunto ATI simulato: gravita e inerzia del tool non sono incluse come sarebbero in un sensore fisico non compensato. Il dataset mantiene separate le forze native dei pad e la risultante.')

    page();heading('6. Momenti e trasporto del polo')
    figure('10_wrist_moments','Momenti world frame trasportati al medesimo flange nominale, con M_nom=M_actual+(p_actual-p_nom) x F. Confrontare momenti presi in poli diversi introdurrebbe un errore geometrico. Non sono momenti locali sulle sfere.',width=151*mm)
    text(f'La mediana RMSE di momento e {number(rigid["moment_wrench_rmse_Nm"],3)} Nm nel rigido e {number(floating["moment_wrench_rmse_Nm"],3)} Nm nel floating. Il fatto che ogni pad trasmetta solo una forza non annulla la coppia risultante del tripod: M=sum((p_i-p_polo) x f_i).')
    text('Il pannello supplementare 03_wrist_components mostra compressione, forza tangenziale e torsione al centroide dei tre contatti. Il dataset conserva tutte le sei componenti al flange e i punti di contatto effettivi.')

    page();heading('7. Tre coni puntiformi e patch aggregata')
    figure('04_exact_vs_ellipse','Sezioni del wrench set ottenute con i tre coni sulle geometrie flat, tilted e faceted del precedente banco di contatto. Forza assiale fissata a 15 N, Fy=0 e bending nullo al centroide. Il confronto con R=30 mm separa parte dell\'effetto geometrico da Reff=32.8 mm. Nel caso faceted 15 N e la componente assiale, non la somma delle compressioni locali. La finestra di Ft e limitata a +/-11.25 N.')
    text('La rotazione rigida di una superficie piana preserva questa sezione nel frame locale. Le normali non parallele della superficie sfaccettata la rendono asimmetrica: l\'ellisse non e automaticamente un limite interno conservativo. Queste sono sezioni di fattibilita dei contatti a geometria fissata, non l\'intero insieme raggiungibile dal meccanismo passivo e dalle sue molle.')
    wong_profile=next(p for p in audit['profiles'] if p['method']=='wong' and p['case']=='spatial')
    wdiag=json.loads((HERE/'WONG_DISCRETE_AUDIT.json').read_text())['records'];largest=max(r['diagnostic_minimum_cone_excess_N'] for r in wdiag)
    text(f'Nel profilo Wong-inspired con CoM spaziale, soltanto {wong_profile["discrete_witness_knots"]}/7 nodi angolari hanno un testimone nei sei coni puntiformi. Una seconda ottimizzazione diagnostica quantifica fino a {number(largest,3)} N di eccesso minimo richiesto sul limite tangenziale. Questa rilassazione serve solo a verificare l\'incompatibilita e non viene usata dal controllore.')
    figure('05_aggregate_vs_local','Caso floating / Wong-inspired / CoM spaziale / assetto 0 deg. A sinistra il risultante rispetto all\'ellisse; a destra la massima utilizzazione tra i tre pad. Un indice aggregato favorevole non certifica ciascun cono locale.',width=140*mm)

    page();heading('8. Perche l\'ATI non identifica i tre coni')
    figure('07_same_wrist_different_cones','Esempio algebrico sulla geometria nominale: una coppia di forze interne lungo il segmento fra due pad lascia invariato il wrench al polso ma modifica la distanza dei singoli contatti dal cono. Le ipotesi oltre il limite non rappresentano un equilibrio aderente realizzabile; mostrano cosa il solo wrench non puo escludere.')
    text('Per una mano f ha 9 componenti e w=Hf ne ha 6. Qui rank(H)=6, quindi lo spazio nullo ha dimensione 3. Anche conoscendo esattamente il risultante, restano possibili redistribuzioni interne. Prima di attraversare il limite esistono gia ripartizioni diverse tutte ammissibili. Sull\'hardware l\'ATI non distingue queste ripartizioni; in simulazione le possiamo leggere e confrontare direttamente.')
    figure('06_core_outcomes','Confronto su tutti i 72 casi principali. Endpoint: entro 20 mm e 5 deg, nessuno stop di perdita oggetto. Errori: mediana su 9 assetti/CoM. La baseline Wong non e una replica del sistema del paper; usa la sua patch aggregata sul nostro tool discreto.',width=151*mm)
    text('La baseline senza margine perde l\'oggetto in 6/9 casi rigidi e 9/9 floating. La fattibilita nominale ai nodi del piano non basta, da sola, per l\'esecuzione con un canale di impedenza e un tool passivo. Il risultato e specifico al banco e ai suoi guadagni.')
    text('Le mediane possono nascondere i sottogruppi: con Wong-inspired il floating perde i 3 casi centrati; i 3 casi planari sono entro soglia; i 3 casi con CoM spaziale arrivano alla fine ma hanno circa 30-32 mm e 38-42 deg di errore. Nel rigido il CoM spaziale porta a 28-31 mm e 49-55 deg. Non si puo interpretare la piccola mediana angolare globale come buon comportamento in ogni condizione.','CaptionStudy')

    page();heading('9. Mismatch e molla centrale 2700 N/m')
    figure('08_stress_conditions','30 casi aggiuntivi: solo margine adattivo, 3 assetti per condizione e meccanismo. Massa 1.7/2.2 kg nota; massa sottostimata del 10%; CoM reale spaziale ma pianificato centrato; mu pianificato 0.825 contro 0.75 fisico. Non sono prove esaustive di robustezza.')
    g2700=next(g for g in groups if g['mechanism']=='floating2700')
    text(f'Nella sensibilita separata con <b>molla centrale assiale 2700 N/m</b>, le tre periferiche restano 500 N/m: rigidezza assiale totale 4200 N/m, non 2700. Gli endpoint entro soglia sono {g2700["endpoint_ok"]}/9; RMSE F mediano {number(g2700["force_wrench_rmse_N"])} N, errore finale {number(g2700["final_position_mm"])} mm / {number(g2700["final_attitude_error_deg"])} deg. Non si e ritoccato il controllo per compensare la cedevolezza.')
    figure('09_recorded_end_states','Screenshot realmente salvati a fine run; CoM spaziale, assetto iniziale 0 deg. Stessa camera: floating con margine adattivo, floating senza margine, floating con centrale 2700. Sono stati registrati anche i checkpoint dopo il rilascio.',width=151*mm)
    stress=[]
    for mechanism in ['welded','floating']:
        for case in ['light_known','heavy_known','mass_under10','com_unmodelled','mu_over10']:
            batch=[r for r in records if r['group']=='stress' and r['mechanism']==mechanism and r['case']==case]
            stress.append([NAMES[mechanism],case,str(sum(r['endpoint_ok'] for r in batch))+'/3'])
    markdown.append('\n## Dettaglio stress\n');markdown.extend(' / '.join(row) for row in stress)

    page();heading('10. Verifiche numeriche e limiti della conclusione')
    dense={k:max(p['dense_interpolation'][k] for p in audit['profiles'] if p['dense_interpolation'][k] is not None) for k in ['wrench_balance_force_N','wrench_balance_moment_Nm','margin_deficit_N']}
    text('Le allocazioni dei profili sono state ricontrollate indipendentemente ai 7 nodi angolari: bilancio, compressione e margini per i tre metodi puntiformi; equilibrio, ellisse e bending per Wong. Gli originali e le copie congelate coincidono tramite SHA-256. Il segno delle forze native A/B e verificato dal bilancio di Newton/Eulero e dalla ricostruzione del peso 1.9*9.81=18.639 N dopo rimozione della fixture.')
    text(f'<b>Limite del comando interpolato:</b> i wrench world frame vengono interpolati linearmente tra nodi. Su 601 angoli, residuo massimo di forza {dense["wrench_balance_force_N"]:.2g} N, momento {number(dense["wrench_balance_moment_Nm"],5)} Nm, deficit massimo di margine {number(dense["margin_deficit_N"],4)} N. Quindi la fattibilita esatta ai nodi non e una garanzia continua. Le tracce originali restano inalterate; non e stata corretta retroattivamente la traiettoria.')
    rows=[['Caso','Endpoint dt / dt/2','Delta posizione [mm]','Delta RMSE F [N]']]
    for r in ref:
        rows.append([r['id'].replace('core_','').replace('_adaptive',''),str(r['coarse']['endpoint_ok'])+' / '+str(r['fine']['endpoint_ok']),number(r['fine']['final_position_mm']-r['coarse']['final_position_mm'],3),number(r['fine']['force_wrench_rmse_N']-r['coarse']['force_wrench_rmse_N'],3)])
    table(rows,[64*mm,31*mm,29*mm,27*mm])
    same=all(r['coarse']['endpoint_ok']==r['fine']['endpoint_ok'] for r in ref)
    text('Verifica a 0.25 ms: '+('le quattro classificazioni endpoint restano uguali.' if same else '<b>alcune classificazioni cambiano: la sensibilita numerica limita la conclusione.</b>')+' I coefficienti fisici degli smorzatori e la registrazione a 100 Hz restano fissi. Il confronto e controllo numerico a posteriori, non tuning o replica indipendente; non prova convergenza di tutti i 111 casi.')
    text('<b>Conclusione utilizzabile in tesi:</b> "Il modello SOCP fornisce un\'allocazione nominalmente ammissibile per una geometria di contatto assunta. Il trasferimento al floating plate viene valutato tramite esecuzione dinamica e analisi delle forze per pad in simulazione; la redistribuzione dovuta alla compliance puo ridurre i margini realizzati. Il wrench risultante al polso, da solo, non certifica i singoli vincoli locali."')
    text('Lo studio non identifica il coefficiente reale di attrito, il contatto deformabile dei pad, l\'attrito dei motori o la causa delle oscillazioni hardware. Non dimostra validita per oggetti sconosciuti arbitrari. I tre assetti sono deterministici e le superfici sfaccettate entrano nella sola diagnostica geometrica, non nei 111 sollevamenti del cubo. La verifica sperimentale richiede una misura distribuita o stime con incertezza, e confronto D-only separato per il controller Kinova.')

    page();heading('Riferimenti e riproducibilita')
    text('[1] Anonymous authors, <i>Keeping Contact: Task-Adaptive Friction Margins for Bimanual Non-Prehensile Manipulation</i>. PDF locale fornito, 8 pagine. SHA-256: 5cf869f81ff87155... [papers/sources.json].')
    text('[2] K. I. Wong, L. Yang, Q. Y. Lee, D. Campolo, <i>Industrial Dual-Arm Box Handling via Online Inertial Estimation and Convex Wrench Optimization</i>. PDF locale fornito, 14 pagine. SHA-256: 6b63459ca9e3aa19... [papers/sources.json]. Nessuna assunzione sullo stato di pubblicazione.')
    text('Versioni: Python 3.12.14, NumPy 2.5.4, CVXPY 1.9.3, CLARABEL; PyBullet nel runtime locale con engine DIRECT, 120 iterazioni solver e enableConeFriction=1. Un worker fisico per volta; BLAS a un thread. Run manifest e checkpoint identificano input, sorgenti e casi; i dati originali NVIDIA_RUNS non sono stati modificati.')
    table([['Artefatto','Contenuto'],['PROTOCOL.md / runs_manifest.json','Disegno e 111 condizioni congelate prima della campagna'],['study.py / socp_reference.py / vendor/','Sorgenti di simulazione; copie inalterate del SOCP e tool'],['runs/<id>/','Trace e manifold nativi, screenshot, job, esito e hash sorgente'],['metrics.csv / metrics.json / groups.json','Metriche per caso, eccezioni e aggregati; fallimenti inclusi'],['AUDIT.json / WONG_DISCRETE_AUDIT.json','Verifiche indipendenti e incompatibilita patch/discreto'],['capability.py / capability.json','Sezioni geometriche e esempio spazio nullo'],['refinement/ / REFINEMENT_COMPARISON.json','Quattro casi a passo dimezzato, separati dal principale'],['plots.py / figures/','Grafici vettoriali PDF e PNG'],['SHA256.json','Manifest consegna, include sorgenti, tracce, figure e report']], [63*mm,88*mm])
    text('Riproduzione: usare il Python e le dipendenze documentati in study.py. Il worker preserva le directory gia completate e rifiuta directory incomplete; prima di rilanciare controllare lock, PID e checkpoint. analyze.py, audit.py, audit_wong.py, plots.py e report.py rigenerano soltanto derivati. Non ripulire o sovrascrivere i risultati registrati.')
    doc=SimpleDocTemplate(str(out/'studio_coni_floating_plate.pdf'),pagesize=A4,leftMargin=32*mm,rightMargin=27*mm,topMargin=25*mm,bottomMargin=23*mm,
        title='Coni di attrito e floating plate - studio PyBullet',author='Lorenzo - analisi riproducibile')
    def footer(canvas,doc):
        canvas.setStrokeColor(colors.HexColor('#ccd5dc'));canvas.setLineWidth(.4);canvas.line(32*mm,19*mm,A4[0]-27*mm,19*mm)
        canvas.setFont('Times-Roman',8);canvas.setFillColor(colors.HexColor('#555555'));canvas.drawString(32*mm,14*mm,'PyBullet / floating plate / 24 ottobre 2026');canvas.drawRightString(A4[0]-27*mm,14*mm,str(doc.page))
    doc.build(story,onFirstPage=footer,onLaterPages=footer)
    (HERE/'RESULTS.md').write_text('\n'.join(markdown),encoding='utf-8')
    print('Report and RESULTS.md written.')

if __name__=='__main__':main()
