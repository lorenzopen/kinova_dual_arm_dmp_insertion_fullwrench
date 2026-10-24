"""Publication figures from completed traces and independent derived metrics."""
from analyze import *
os.environ['MPLCONFIGDIR']=str(HERE/'mplconfig')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=HERE/'figures';OUT.mkdir(exist_ok=True)
BLUE='#1f6fb2';ORANGE='#d9622b'
COLOURS={'welded':BLUE,'floating':ORANGE,'floating2700':'#5a7d58'}
LABELS={'welded':'Welded tripod','floating':'Floating plate','floating2700':'Central 2700'}
plt.rcParams.update({'font.family':'serif','font.size':9,'axes.titlesize':9,'axes.labelsize':9,
    'legend.fontsize':7,'xtick.labelsize':8,'ytick.labelsize':8,'axes.spines.top':False,
    'axes.spines.right':False,'pdf.fonttype':42,'mathtext.fontset':'cm','legend.frameon':False})

def export(fig,name):
    fig.savefig(OUT/(name+'.pdf'),bbox_inches='tight')
    fig.savefig(OUT/(name+'.png'),dpi=180,bbox_inches='tight');plt.close(fig)

def load(mechanism,method='adaptive',case='spatial',seed=0):
    name=f'core_{mechanism}_{method}_{case}_{seed}'
    folder=HERE/'runs'/name
    summary=json.loads((folder/'COMPLETED.json').read_text())
    d=dict(np.load(folder/'trace.npz',allow_pickle=False));m,x=measure(d,summary['job'])
    return d,m,x,summary

def cones():
    traces={mech:load(mech) for mech in ['welded','floating']}
    fig,axes=plt.subplots(2,3,figsize=(6.8,4.9),layout='constrained')
    for j,ax in enumerate(axes.flat):
        values=[]
        for mech,(d,m,x,s) in traces.items():
            mask=d['t']>=1.3;force=d['force'][mask,j];n=d['normals'][mask,j]
            N=-np.sum(force*n,axis=1);T=np.linalg.norm(force+N[:,None]*n,axis=1)
            ax.scatter(N,T,s=3,alpha=.42,c=COLOURS[mech],label=LABELS[mech],rasterized=True)
            values.extend(N)
        d=traces['welded'][0];mask=d['t']>=1.3
        force=d['witness'][mask,j];n=d['nominal_normals'][mask,j];N=-np.sum(force*n,axis=1)
        T=np.linalg.norm(force+N[:,None]*n,axis=1)
        ax.plot(N,T,c='.15',lw=.85,ls='--',label='SOCP witness')
        xmax=max(values)*1.15;xx=np.linspace(0,xmax,50)
        ax.fill_between(xx,0,.75*xx,color='.94',zorder=-3);ax.plot(xx,.75*xx,c='.4',lw=.75)
        ax.set_xlim(0,xmax);ax.set_ylim(0,.75*xmax);ax.set_title(('L' if j<3 else 'R')+str(j%3+1))
        ax.set_xlabel('Compression (N)');ax.set_ylabel('Tangential norm (N)')
    axes[0,0].legend(loc='upper left',handlelength=1.2)
    export(fig,'01_per_pad_cones')

    fig,axes=plt.subplots(2,3,figsize=(6.8,4.7),sharex=True,layout='constrained')
    for j,ax in enumerate(axes.flat):
        for mech,(d,m,x,s) in traces.items():
            mask=d['t']>=1.0
            y=x['actual_psi'][:,j].copy();y[~x['present'][:,j]]=np.nan
            ax.plot(d['t'][mask],y[mask],c=COLOURS[mech],lw=.9,label=LABELS[mech])
        d=traces['welded'][0];mask=d['t']>=1
        ax.plot(d['t'][mask],d['rho'][mask,j],c='.15',lw=.8,ls='--',label='Required distance')
        ax.axhline(0,c='.5',lw=.6);ax.set_title(('L' if j<3 else 'R')+str(j%3+1))
        ax.set_ylabel(r'$\psi_\mu$ (N)');ax.set_xlabel('Time (s)');ax.grid(axis='y',color='.93',lw=.5)
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='lower center',bbox_to_anchor=(.5,-.055),ncol=3)
    export(fig,'02_per_pad_margin')

    fig,axes=plt.subplots(3,2,figsize=(6.8,5.8),sharex=True,layout='constrained')
    for h in range(2):
        for mech,(d,m,x,s) in traces.items():
            mask=d['t']>=1
            for row,key in enumerate(['hand_N','hand_Ft','hand_Mn']):
                axes[row,h].plot(d['t'][mask],x[key][mask,h],c=COLOURS[mech],lw=.9,label=LABELS[mech])
        d=traces['welded'][0];mask=d['t']>=1;n=d['nominal_normals'][:,3*h]
        N=-np.sum(d['desired_wrist'][:,h,:3]*n,axis=1)
        Ft=np.linalg.norm(d['desired_wrist'][:,h,:3]+N[:,None]*n,axis=1)
        T=np.sum(traces['welded'][2]['contact_moment_plan'][:,h]*n,axis=1)
        for row,y in enumerate([N,Ft,T]):
            ax=axes[row,h];ax.plot(d['t'][mask],y[mask],c='.1',ls='--',lw=.85,label='SOCP reference')
            ax.grid(axis='y',color='.93',lw=.5);ax.set_xlabel('Time (s)')
        axes[0,h].set_title('Left hand' if h==0 else 'Right hand')
    for ax,label in zip(axes[:,0],['Axial force (N)','Tangential force (N)','Axial moment (Nm)']):ax.set_ylabel(label)
    axes[0,0].legend(loc='upper left');export(fig,'03_wrist_components')

def capability():
    data=json.loads((HERE/'capability.json').read_text())
    fig,axes=plt.subplots(1,3,figsize=(7.0,2.8),layout='constrained')
    for ax,(name,d) in zip(axes,data['slices'].items()):
        T=np.array(d['T']);lo=np.array(d['lower']);hi=np.array(d['upper']);mu=d['mu'];N=d['normal_force']
        ax.fill_between(T,lo,hi,color=BLUE,alpha=.15);ax.plot(T,hi,c=BLUE,lw=1,label='3-point cone slice');ax.plot(T,lo,c=BLUE,lw=1)
        for radius,rs,style,label,colour in [(.0328,0,'--','Patch ellipse',ORANGE),(.0328,.1,':','Patch, rs=0.10',ORANGE),(.03,0,'-.','Ellipse, R=30 mm','.4')]:
            t=np.linspace(-(1-rs)*mu*N,(1-rs)*mu*N,150)
            m=radius*np.sqrt(np.maximum(0,((1-rs)*mu*N)**2-t*t))
            ax.plot(t,m,c=colour,ls=style,lw=.8,label=label);ax.plot(t,-m,c=colour,ls=style,lw=.8)
        ax.set_title(name.capitalize());ax.set_xlabel('Tangential force (N)');ax.set_ylabel('Axial moment (Nm)');ax.grid(color='.94',lw=.5)
    handles,labels=axes[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='lower center',bbox_to_anchor=(.5,-.17),ncol=2,fontsize=7)
    export(fig,'04_exact_vs_ellipse')

    if not (HERE/'runs/core_floating_wong_spatial_0/COMPLETED.json').exists():return

    fig,axes=plt.subplots(1,2,figsize=(6.8,2.9),layout='constrained')
    d,m,x,summary=load('floating','wong')
    mask=d['t']>=1.3
    theta=np.linspace(0,2*np.pi,200)
    axes[0].plot(np.cos(theta),np.sin(theta),c='.3',lw=.8,label='Nominal ellipse')
    axes[0].plot(.9*np.cos(theta),.9*np.sin(theta),c='.3',lw=.8,ls='--',label='rs=0.10')
    for h,c in enumerate([BLUE,ORANGE]):
        X=x['hand_Ft'][mask,h]/(.75*x['hand_N'][mask,h]);Y=x['hand_Mn'][mask,h]/(.75*.0328*x['hand_N'][mask,h])
        axes[0].scatter(X,Y,s=4,c=c,alpha=.6,label='Left' if h==0 else 'Right',rasterized=True)
    axes[0].set(xlabel='Tangential / (mu N)',ylabel='Torsion / (mu N Reff)',xlim=(-.1,1.1),ylim=(-1.05,1.05));axes[0].legend(loc='lower left',fontsize=6.5)
    for h,c in enumerate([BLUE,ORANGE]):
        yy=np.nanmax(x['eta'][mask,3*h:3*h+3],axis=1)
        axes[1].scatter(x['hand_eta'][mask,h],yy,c=c,s=4,alpha=.6,label='Left' if h==0 else 'Right',rasterized=True)
    axes[1].axhline(1,c='.4',lw=.7);axes[1].axvline(.9,c='.4',lw=.7,ls='--')
    axes[1].set(xlabel='Aggregate ellipse utilization',ylabel='Largest per-pad utilization');axes[1].grid(color='.94',lw=.5)
    export(fig,'05_aggregate_vs_local')

    obs=data['observability'];alpha=np.array(obs['alpha']);psi=np.array(obs['signed_distance'])
    fig,axes=plt.subplots(1,2,figsize=(6.6,2.65),layout='constrained')
    for j,c in enumerate([BLUE,ORANGE,'.4']):axes[0].plot(alpha,psi[:,j],c=c,lw=.9,label='Pad '+str(j+1))
    axes[0].axhline(0,c='.3',lw=.7);axes[0].set(xlabel='Internal-force change (N)',ylabel='Signed distance (N)');axes[0].legend()
    axes[1].axis('off')
    axes[1].text(.5,.62,'Same wrist resultant',ha='center',va='center',transform=axes[1].transAxes,fontsize=12,color=BLUE)
    axes[1].text(.5,.40,'Max force change: '+f"{max(obs['force_error_N']):.1e}"+' N\nMax moment change: '+f"{max(obs['moment_error_Nm']):.1e}"+' Nm\n(numerical roundoff)',
        ha='center',va='center',transform=axes[1].transAxes,fontsize=9,linespacing=1.8)
    for ax in axes:ax.grid(color='.94',lw=.5)
    export(fig,'07_same_wrist_different_cones')

def outcomes():
    groups=json.loads((HERE/'groups.json').read_text())
    fig,axes=plt.subplots(1,3,figsize=(7.2,2.9),layout='constrained')
    methods=['none','fixed','adaptive','wong'];labels=['No margin','Fixed','Adaptive','Wong model']
    for k,mech in enumerate(['welded','floating']):
        values=[next(g for g in groups if g['mechanism']==mech and g['method']==method) for method in methods]
        xx=np.arange(4)+(k-.5)*.34
        for ax,key in zip(axes,['endpoint_ok','final_attitude_error_deg','force_wrench_rmse_N']):
            heights=[g[key] for g in values]
            ax.bar(xx,heights,width=.32,color=COLOURS[mech],label=LABELS[mech])
            ax.set_xticks(np.arange(4),labels,rotation=25,ha='right');ax.grid(axis='y',color='.94',lw=.5)
    axes[0].set_ylabel('Endpoints within 20 mm / 5 deg (of 9)');axes[1].set_ylabel('Final attitude error (deg), median')
    axes[2].set_ylabel('Wrist force RMSE (N), median');axes[0].set_ylim(0,11);axes[0].set_yticks([0,3,6,9]);axes[0].legend(fontsize=6.5);export(fig,'06_core_outcomes')

    records=json.loads((HERE/'metrics.json').read_text())['records']
    if sum(r['group']=='stress' for r in records)!=30:return
    cases=['light_known','heavy_known','mass_under10','com_unmodelled','mu_over10']
    fig,axes=plt.subplots(1,2,figsize=(6.8,3.15),layout='constrained')
    for k,mech in enumerate(['welded','floating']):
        groups=[[r for r in records if r['group']=='stress' and r['mechanism']==mech and r['case']==case] for case in cases]
        xx=np.arange(5)+(k-.5)*.32
        axes[0].bar(xx,[sum(r['endpoint_ok'] for r in g) for g in groups],width=.3,color=COLOURS[mech],label=LABELS[mech])
        axes[1].bar(xx,[np.median([r['minimum_pad_normal_N'] for r in g]) for g in groups],width=.3,color=COLOURS[mech])
    for ax in axes:ax.set_xticks(np.arange(5),['1.7 kg','2.2 kg','Mass -10%','CoM omitted','mu +10%'],rotation=28,ha='right');ax.grid(axis='y',color='.94',lw=.5)
    axes[0].set_ylabel('Endpoints within limits (of 3)');axes[0].set_ylim(0,3.75);axes[0].set_yticks([0,1,2,3]);axes[1].set_ylabel('Lowest pad load (N), median');axes[0].legend()
    export(fig,'08_stress_conditions')

def snapshots():
    fig,axes=plt.subplots(1,3,figsize=(7.,2.5),layout='constrained')
    paths=['core_floating_adaptive_spatial_0','core_floating_none_spatial_0','central2700_floating2700_adaptive_spatial_0']
    for ax,path,title in zip(axes,paths,['Adaptive, floating','No margin, floating','Adaptive, central 2700']):
        ax.imshow(plt.imread(HERE/'runs'/path/'final.png'));ax.set_xlim(280,620);ax.set_ylim(520,150);ax.set_title(title);ax.axis('off')
    export(fig,'09_recorded_end_states')

def six_axes():
    traces={mech:load(mech) for mech in ['welded','floating']}
    for kind,offset,unit in [('forces',0,'N'),('moments',3,'Nm')]:
        fig,axes=plt.subplots(3,2,figsize=(6.8,5.65),sharex=True,layout='constrained')
        for h in range(2):
            for row in range(3):
                ax=axes[row,h]
                for mech,(d,m,x,s) in traces.items():
                    mask=d['t']>=1
                    ax.plot(d['t'][mask],x['matched_wrist'][mask,h,offset+row],c=COLOURS[mech],lw=.9,label=LABELS[mech])
                d=traces['welded'][0];mask=d['t']>=1
                ax.plot(d['t'][mask],d['desired_wrist'][mask,h,offset+row],c='.15',ls='--',lw=.8,label='SOCP reference')
                ax.grid(axis='y',color='.94',lw=.5);ax.set_xlabel('Time (s)')
                ax.set_ylabel(('F' if offset==0 else 'M')+['x','y','z'][row]+' ('+unit+')')
            axes[0,h].set_title('Left, world axes' if h==0 else 'Right, world axes')
        axes[0,0].legend(loc='best',fontsize=6.5)
        export(fig,'10_wrist_'+kind)

def diagram():
    from matplotlib.patches import FancyBboxPatch,FancyArrowPatch
    fig,ax=plt.subplots(figsize=(7,2.45));ax.set(xlim=(-.12,10.12),ylim=(0,3.3));ax.axis('off')
    nodes=[(.0,1.8,2.0,.8,'Known load\nNominal contacts'),(2.6,1.8,1.65,.8,'SOCP\n6 point cones'),
           (4.85,1.8,1.9,.8,'Wrist pose bias\nImpedance proxy'),(7.35,1.8,2.6,.8,'PyBullet plant\nWelded / floating plate'),
           (6.8,.2,3.15,.8,'Per-pad forces and normals\nWrist resultant, slip, pose')]
    for i,(x,y,w,h,label) in enumerate(nodes):
        colour=ORANGE if i>=3 else BLUE
        ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.035',ec=colour,fc=colour+'12',lw=.8))
        ax.text(x+w/2,y+h/2,label,ha='center',va='center',fontsize=8)
    for a,b in [((2,2.2),(2.6,2.2)),((4.25,2.2),(4.85,2.2)),((6.75,2.2),(7.35,2.2)),((8.65,1.8),(8.65,1.0))]:
        ax.add_patch(FancyArrowPatch(a,b,arrowstyle='-|>',mutation_scale=10,color='.3',lw=.8))
    ax.text(4.5,.65,'Simulation truth is used for scoring.\nNo individual pad commands.',ha='right',va='center',fontsize=8,color='.25')
    export(fig,'11_information_flow')

def main():
    diagram()
    capability()
    if not (HERE/'runs/core_floating_adaptive_spatial_0/COMPLETED.json').exists():return
    cones()
    six_axes()
    groups=json.loads((HERE/'groups.json').read_text())
    if all(any(g['method']==method and g['mechanism']=='floating' and g['n']==9 for g in groups) for method in ['none','fixed','adaptive','wong']):outcomes()
    if (HERE/'runs/central2700_floating2700_adaptive_spatial_0/COMPLETED.json').exists():snapshots()

if __name__=='__main__':main()
