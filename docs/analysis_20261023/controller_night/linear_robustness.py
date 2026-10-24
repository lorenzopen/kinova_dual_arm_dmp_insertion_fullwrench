"""Local sampled-loop analysis of frozen hypotheses, NOT robot certification.

No CSV/holdout loading, no replay, no tuning, no mapping cart_d to model poles.
Exact held-input discretization including fractional delay for lag/resonance.
"""
import sys
sys.dont_write_bytecode=True
import conditional_controllers as c
import json
import csv
import hashlib
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
from numpy.polynomial import polynomial as poly
from scipy.linalg import expm
from scipy.optimize import brentq
from evaluate_locked_holdout import verify_freeze
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=c.OUT


def trim(a):
    a=np.asarray(a,dtype=float)
    while len(a)>1 and abs(a[-1])<1e-16:a=a[:-1]
    return a
def add(a,b):return trim(poly.polyadd(a,b))
def mul(a,b):return trim(poly.polymul(a,b))
def shift(a,n):return np.r_[np.zeros(n),a]
def ratadd(a,b):return add(mul(a[0],b[1]),mul(b[0],a[1])),mul(a[1],b[1])
def evaluate(r,q):return poly.polyval(q,r[0])/poly.polyval(q,r[1])
def now():return datetime.now(timezone.utc).isoformat()
def dump(name,value):(OUT/name).write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')


def controller(cfg,T):
    kp,ki,kd=(cfg[k] for k in ['kp_m_N','ki_m_N_s','kd_m_s_N'])
    pd=(np.array([kp]),np.array([1.]))
    if kd:pd=ratadd(pd,(np.array([kd,-kd]),np.array([T+cfg['derivative_tau_s'],-cfg['derivative_tau_s']])))
    r=cfg.get('pd_rolloff_tau_s',0.)
    if r:pd=(pd[0]*T,mul(pd[1],[T+r,-r]))
    if ki:return ratadd(pd,(np.array([ki*T]),np.array([1.,-1.])))
    return pd


def discretization(model,T):
    ratio=model['delay_s']/T;m=int(np.floor(ratio+1e-10));delta=model['delay_s']-m*T
    if abs(delta)<1e-10:delta=0.
    if model['kind']=='delay':
        return dict(kind='delay',steps=m+(delta>0),gain=2*model['stiffness_N_m_hand'])
    if model['kind']=='lag':
        A=np.array([[-1/model['tau_s']]]);B=np.array([[1/model['tau_s']]])
    else:
        w=2*np.pi*model['frequency_Hz'];z=model['damping_ratio']
        A=np.array([[0.,1.],[-w*w,-2*z*w]]);B=np.array([[0.],[w*w]])
    n=len(A);block=np.zeros((n+1,n+1));block[:n,:n]=A;block[:n,n:]=B
    def parts(t):
        e=expm(block*t);return e[:n,:n],e[:n,n]
    Ad,_=parts(T);tail,B0=parts(T-delta);_,b_head=parts(delta)
    return dict(kind=model['kind'],A=Ad,B0=B0,B1=tail@b_head,m=m,delta=delta,
                gain=2*model['stiffness_N_m_hand'])


def plant(d):
    if d['kind']=='delay':return shift([d['gain']],d['steps']),np.array([1.])
    A=d['A']
    if len(A)==1:
        den=[1.,-A[0,0]]
        def numerator(B):return np.array([0.,B[0]])
    else:
        den=[1.,-np.trace(A),np.linalg.det(A)]
        def numerator(B):return np.array([0.,B[0],A[0,1]*B[1]-A[1,1]*B[0]])
    return d['gain']*add(shift(numerator(d['B0']),d['m']),shift(numerator(d['B1']),d['m']+1)),trim(den)


def state_radius(cfg,d,T):
    """Independent explicit controller + delay history state update, then eig."""
    n=0 if d['kind']=='delay' else len(d['A'])
    hlen=d['steps'] if d['kind']=='delay' else d['m']+1
    ki=cfg['ki_m_N_s'];kd=cfg['kd_m_s_N'];r=cfg.get('pd_rolloff_tau_s',0.)
    ci=n+hlen;ix_i=ci if ki else None;ci+=bool(ki)
    ix_d=ci if kd else None;ci+=2*bool(kd)
    ix_r=ci if r else None;ci+=bool(r)
    def step(v):
        out=np.zeros_like(v);h=v[n:n+hlen]
        y=d['gain']*(h[d['steps']-1] if d['kind']=='delay' else v[0])
        deriv=0.
        if kd:
            tau=cfg['derivative_tau_s'];deriv=(tau*v[ix_d]+y-v[ix_d+1])/(tau+T)
            out[ix_d]=deriv;out[ix_d+1]=y
        pd=-cfg['kp_m_N']*y-kd*deriv
        if r:pd=(r*v[ix_r]+T*pd)/(r+T);out[ix_r]=pd
        integ=v[ix_i]-ki*T*y if ki else 0.
        if ki:out[ix_i]=integ
        u=integ+pd
        if n:
            recent=u if d['m']==0 else h[d['m']-1]
            older=h[d['m']]
            out[:n]=d['A']@v[:n]+d['B0']*recent+d['B1']*older
        out[n]=u
        if hlen>1:out[n+1:n+hlen]=h[:-1]
        return out
    matrix=np.column_stack([step(e) for e in np.eye(ci)])
    return float(max(abs(np.linalg.eigvals(matrix)),default=0))


def closed_radius(P,C):
    characteristic=add(mul(P[1],C[1]),mul(P[0],C[0]))
    roots=np.roots(characteristic)
    return float(max(abs(roots),default=0))


def software_checks():
    worst=0.
    # Exact pure-delay I roots: m=1 => z=1-g*ki*T; stability boundary
    # for m>=1 is g*ki*T=2*sin(pi/(4*m-2)). No Pade approximation.
    for m in [1,2,4,9]:
        boundary=2*np.sin(np.pi/(4*m-2))
        for factor in [.7,1.,1.2]:
            gamma=boundary*factor;char=add([1.,-1.],shift([gamma],m))
            rho=max(abs(np.roots(char)))
            if factor<1:assert rho<1
            elif factor>1:assert rho>1
            else:worst=max(worst,abs(rho-1));assert abs(rho-1)<1e-10
    cfg=dict(kp_m_N=0.,ki_m_N_s=.0016,kd_m_s_N=0.,derivative_tau_s=.05)
    d=dict(kind='delay',steps=1,gain=1360.)
    assert abs(closed_radius(plant(d),controller(cfg,.025))-(1-1360*.0016*.025))<1e-12
    # First-order ZOH without delay has known (1-a)*q/(1-a*q).
    model=dict(kind='lag',delay_s=0.,stiffness_N_m_hand=680.,tau_s=.12)
    q=np.exp(-1j*np.array([.01,.5,2.]));T=.025;P=plant(discretization(model,T));a=np.exp(-T/.12)
    assert np.max(abs(evaluate(P,q)-1360*(1-a)*q/(1-a*q)))<1e-10
    return dict(pure_I_boundary_max_root_error=worst,checks='Closed-form I stability boundaries and one-tick pole; exact no-delay lag transfer; all336 independent state-matrix radius comparisons follow.')


def case(cfg,model,T):
    d=discretization(model,T);P=plant(d);C=controller(cfg,T)
    rho=closed_radius(P,C);rho_state=state_radius(cfg,d,T)
    assert abs(rho-rho_state)<2e-7,(cfg['id'],model['id'],T,rho,rho_state)
    frequencies=np.geomspace(.0001,(.5/T)*(1-1e-7),14000)
    q=np.exp(-2j*np.pi*frequencies*T);L=evaluate(P,q)*evaluate(C,q)
    phase=np.unwrap(np.angle(L));mag=abs(L);S=1/(1+L);Tc=L/(1+L)
    crossings=[]
    for j in np.flatnonzero((mag[:-1]-1)*(mag[1:]-1)<0):
        f=brentq(lambda x:abs(evaluate(P,np.exp(-2j*np.pi*x*T))*evaluate(C,np.exp(-2j*np.pi*x*T)))-1,
                 frequencies[j],frequencies[j+1])
        ph=float(np.interp(f,frequencies,phase));crossings.append(dict(frequency_Hz=f,phase_margin_deg=180+np.degrees(ph)))
    row=dict(candidate=cfg['id'],model=model['id'],control_period_s=T,
        closed_loop_spectral_radius=rho,independent_state_radius=rho_state,
        locally_stable=bool(rho<1-1e-8),gain_crossings=crossings,
        minimum_crossing_phase_margin_deg=min([x['phase_margin_deg'] for x in crossings],default=None),
        S_DC=0. if cfg['ki_m_N_s'] else 1/(1+2*model['stiffness_N_m_hand']*cfg['kp_m_N']),
        maximum_S=float(max(abs(S))),maximum_S_frequency_Hz=float(frequencies[np.argmax(abs(S))]))
    for label,(lo,hi) in dict(low=(.2,.8),mid=(.8,2.5),high=(2.5,10)).items():
        ix=(frequencies>=lo)&(frequencies<=hi)
        row['maximum_S_'+label+'_band']=float(max(abs(S[ix])))
        row['maximum_T_'+label+'_band']=float(max(abs(Tc[ix])))
    qq=np.exp(-2j*np.pi*1.5*T);ll=evaluate(P,qq)*evaluate(C,qq)
    row['S_at_1p5Hz']=float(abs(1/(1+ll)))
    return row,frequencies,S


def main():
    frozen,sha,verification=verify_freeze()
    target=OUT/'LINEAR_ROBUSTNESS_DESIGN.json'
    design=dict(created_utc=now(),freeze_sha256=sha,candidates=frozen['candidates'],models=frozen['models'],
        control_periods_s=[.020,.025,.030],cases=336,
        scope='Local unsaturated scalar loop, fixed nominal/target, no contact/geometry evolution. Exact ZOH plant and backward-Euler discrete filters/I. No CSV, heldout tuning or robot access.',
        exclusions='No saturation, jitter certificate, measurement prefilters, actuator limits, full-wrench margin or mapping from cart_d. Closed-loop poles certify only stated sampled hypothetical LTI model.',
        verification=verification,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    with target.open('x',encoding='utf-8') as f:json.dump(design,f,indent=2)
    checks=software_checks();rows=[];curves={}
    for cfg in design['candidates']:
        for model in design['models']:
            for T in design['control_periods_s']:
                row,f,S=case(cfg,model,T);rows.append(row)
                if T==.025 and model['id']=='delay_k680_d0.18':curves[cfg['id']]=(f,abs(S))
    checks['max_state_vs_polynomial_radius_difference']=max(abs(x['closed_loop_spectral_radius']-x['independent_state_radius']) for x in rows)
    checks['all_cases']=len(rows);assert len(rows)==336
    checks['all_passed']=True
    dump('linear_robustness_validation.json',checks);dump('linear_robustness_metrics.json',rows)
    with (OUT/'linear_robustness_metrics.csv').open('w',newline='',encoding='utf-8') as f:
        flat=[{**r,'gain_crossings':json.dumps(r['gain_crossings'])} for r in rows]
        w=csv.DictWriter(f,fieldnames=list(flat[0]));w.writeheader();w.writerows(flat)
    summarize(design,rows,curves,checks)
    print('Local fixed-loop analysis complete:',len(rows),'cases; independent matrix verification passed.')


def summarize(design,rows,curves,checks):
    labels=['FFI','FF solo','P','PI filtrata','PD','PID','PI semplice'];summ=[]
    for cfg,label in zip(design['candidates'],labels):
        rr=[x for x in rows if x['candidate']==cfg['id']];stable=[x for x in rr if x['locally_stable']]
        pm=[x['minimum_crossing_phase_margin_deg'] for x in rr if x['minimum_crossing_phase_margin_deg'] is not None]
        summ.append(dict(id=cfg['id'],label=label,stable_cases=len(stable),cases=len(rr),
            worst_radius=max(x['closed_loop_spectral_radius'] for x in rr),
            minimum_crossing_phase_margin_deg=min(pm,default=None),
            maximum_S_mid_band_over_stable_models=max((x['maximum_S_mid_band'] for x in stable),default=None),
            maximum_T_high_band_over_stable_models=max((x['maximum_T_high_band'] for x in stable),default=None),
            central=next(x for x in rr if x['model']=='delay_k680_d0.18' and x['control_period_s']==.025)))
    dump('linear_robustness_summary.json',summ)
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.2,'axes.spines.top':False,'axes.spines.right':False})
    fig,axs=plt.subplots(1,2,figsize=(14,5.8),layout='constrained')
    colors=['#202630','#b18b38','#667e34','#087e8b','#b34d95','#d26227','#4169b0']
    for cfg,label,col in zip(design['candidates'],labels,colors):
        f,S=curves[cfg['id']];axs[0].semilogx(f,S,label=label,color=col,lw=1.3)
    axs[0].axhline(1,color='#888',ls='--');axs[0].axvspan(.8,2.5,color='#aaa',alpha=.12)
    axs[0].set(xlim=(.05,10),xlabel='Frequenza [Hz]',ylabel='|S|: forza con anello / disturbo applicato',
        title='Disturbo sulla forza — caso centrale, tick 25 ms');axs[0].legend(ncol=2,fontsize=8)
    for k,col in zip([480,680,964,1500],['#679b37','#087e8b','#d26227','#b34d95']):
        delays=np.array([.12,.18,.23]);a=2*k*.0016
        axs[1].plot(delays*1000,90-np.degrees(a*delays),'o-',label=f'k={k} N/m/mano',color=col)
    axs[1].set(xlabel='Ritardo puro ipotizzato [ms]',ylabel='Margine di fase continuo [gradi]',
        title='FFI con solo I: riferimento analitico continuo')
    axs[1].axhline(0,color='#777',ls='--');axs[1].legend(fontsize=8)
    fig.suptitle('Stabilità locale e attenuazione delle onde sono proprietà diverse\nModelli ipotizzati, nominale fisso e limiti inattivi. Nessuna previsione del robot o di cart_d.',fontsize=14)
    fig.savefig(OUT/'17_robustezza_locale_sensibilita.png',dpi=160);plt.close(fig)
    lines=['| Legge | Stabili /48 | ρ peggiore | Min PM attraversamenti [°] | Max ampiezza S 0,8–2,5 Hz, modelli stabili | Ampiezza S(0) centrale |',
           '| --- | --- | --- | --- | --- | --- |']
    for s in summ:
        pm='—' if s['minimum_crossing_phase_margin_deg'] is None else f"{s['minimum_crossing_phase_margin_deg']:.1f}"
        lines.append(f"| {s['label']} | {s['stable_cases']} | {s['worst_radius']:.4f} | {pm} | {s['maximum_S_mid_band_over_stable_models']:.3f} | {s['central']['S_DC']:.3f} |")
    tab='\n'.join(lines)
    text=f'''# Robustezza locale: analisi delle sette leggi congelate

Questa tranche non carica CSV né riapre il tuning: **336 modelli di anello
lineare locale**, sette leggi fisse ×16 impianti ipotizzati ×3 tick (20/25/30 ms).
Non è una nuova simulazione delle run. Il damping interno cart_d resta estraneo
ai parametri di questi impianti. Il confronto D-only resta prioritario e richiede
la semantica del servo interno e nuove prove controllate già protocollate.

## Derivazione e perimetro

Si linearizza a nominale/target costanti, contatto invariato e limiti inattivi.
Ingresso posizione per mano u, uscita forza somma y: G=2k·H·ritardo.
Con q=z⁻¹ e tick T, il regolatore della misura è

`C(q) = Ki*T/(1-q) + [Kp + Kd*(1-q)/(T+tauD-tauD*q)] * T/(T+r-r*q)`.

L'ultimo fattore è 1 se r=0; I e derivata assenti vengono rimossi, la derivata
è sulla misura. Ki integra l'errore della somma, non della media. È l'algebra
locale della legge congelata quando velocità/trim/allocazione non limitano.
Le variazioni temporali e il dt clamped del log non sono sostituiti da questi tick.

Per il ritardo puro τ, il comando tenuto dà G(q)=2k·q^ceil(τ/T), con l'intero
esatto quando τ/T è intero. Per lag/risonante, con τ=mT+δ e 0≤δ<T:

`x[n+1]=A(T)x[n]+B(T-δ)u[n-m]+A(T-δ)B(δ)u[n-m-1]`.

A e B sono esponenziali/integrali esatti del sistema continuo; non si usa Padé.
I poli vengono dalle radici di `den(G)den(C)+num(G)num(C)` in q,
trasformato in z. Una matrice di stato costruita separatamente aggiornando
controller, memoria dei comandi e impianto verifica il raggio spettrale su
tutti i 336 casi: differenza massima {checks['max_state_vs_polynomial_radius_difference']:.2e}.
Verificate anche soglie analitiche del ritardo puro con I e lag ZOH noto.
I risultati e sorgenti sono in `linear_robustness_*.json/csv` e nel design esclusivo.

{tab}

ρ<1 certifica solo stabilità del sistema discreto LTI dichiarato. Non certifica
il robot, il contatto, la risposta con saturazioni né jitter variabile.
Il PM riporta il minimo fra tutti gli attraversamenti di guadagno unitario;
se molteplici non basta da solo come criterio: la stabilità qui è decisa dai poli.
FF solo non ha anello esterno e non ha PM: si riportano i poli dell'impianto.
I massimi sono su griglia in frequenza, non una norma H∞ certificata.

## Perché un anello stabile può ancora mostrare onde

Per un disturbo aggiunto alla forza, S=1/(1+GC); per rumore aggiunto alla misura,
la forza fisica risponde con −GC/(1+GC). Queste sono due vie diverse: più onda
misurata non identifica automaticamente rumore ATI. |S|>1 amplifica una
componente sinusoidale imposta anche se tutti i poli sono stabili. |S| è un
rapporto d'ampiezza, non la RMS futura della run; la forma/spettro dei disturbi
reali cambia anche con moto, geometria e controllore. La banda della tabella
non è il band RMS del replay e non ne modifica lo score.

![Sensibilità locale e margine analitico](17_robustezza_locale_sensibilita.png)

Con sola I e guadagno statico a=2kKi, il modello continuo a ritardo puro ha
attraversamento ω=a e PM=π/2−aτ. Con k=680 e Ki=0,0016, a=2,176 s⁻¹:
PM≈67,6° a180 ms, 61,3° a230 ms. Con k=1500, a=4,8 s⁻¹ e230 ms, PM≈26,7°.
La soglia continua è aτ=π/2: non è una stima del ritardo critico del robot.
Nel discreto esatto aT=γ, m tick di ritardo, la soglia è
`γcrit=2*sin(pi/(4*m-2))`; l'attraversamento è `Ω=2*asin(γ/2)` e
`PM=pi/2-(m-1/2)*Ω`. Queste formule forniscono controlli indipendenti del codice.

P/PD senza I lasciano una frazione DC `1/(1+2kKp)` del disturbo costante sulla
forza: con k=680 e P=0,05 mm/N è circa0,936. La D vale zero in DC.
Questo spiega un possibile compromesso del modello: meno oscillazione in una
banda può accompagnarsi a offset grande. Non identifica la causa degli offset
misurati; il nominale FF varia durante le run e non è incluso in questo test.
La I porta S(0) a0 solo nel sistema stabile idealizzato senza limiti attivi;
non promette errore nullo reale né rapidità arbitraria.

## Conseguenze per il damping e il prossimo dato utile

Non è stato scelto nessun nuovo gain dopo l'holdout. Nessuna compensazione del
ritardo emerge da questo calcolo, perché l'impianto non ha superato la verifica
predittiva. cart_d modifica il servo interno, non il parametro τ o ζ scelto qui.
Un eventuale aumento può attenuare risonanza e modificare fase/rapidità: serve
il sorgente effettivo o un confronto D-only con FFI invariata. La stabilità
locale ipotizzata non sostiene né esclude «bastava alzare il damping».

Misure minime: comando cartesiano effettivamente applicato, posa/twist misurati,
wrench ATI per mano con timestamp di acquisizione e di consumo nel controller,
stato/limiti FFI, runtime D/K e clock comune. Sono necessarie per distinguere
ritardo puro, filtro della misura, risonanza del servo e variazioni del contatto.
Con i log attuali l'effetto di un controllore alternativo resta condizionato;
nessuna acquisizione o esecuzione sul robot è stata fatta da questa analisi.
'''
    (OUT/'LINEAR_ROBUSTNESS_REPORT.md').write_text(text,encoding='utf-8')
    for s in summ:print(s['label'],s['stable_cases'],'/48; max S mid=',s['maximum_S_mid_band_over_stable_models'])


if __name__=='__main__':main()
