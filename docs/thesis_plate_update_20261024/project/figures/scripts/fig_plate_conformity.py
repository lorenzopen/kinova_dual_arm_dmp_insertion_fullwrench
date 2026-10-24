"""Chapter 4: geometry, zero-free-length springs and recorded PyBullet conformity.

Rebuilds publication figures from compact saved bench data; no dynamics run.
"""
from pathlib import Path
import json
import numpy as np
from matplotlib.patches import Circle,Polygon,Rectangle
from style import FIG,LEFT,RIGHT,plt,save

DATA=FIG/'data/plate_conformity_20261024'
plt.rcParams.update({'mathtext.fontset':'cm'})
GOLD='#d5ae4d'

def spring(ax,start,end,colour=LEFT,amp=.8,n=6):
    a=np.asarray(start,float);b=np.asarray(end,float);v=b-a
    tangent=np.array([-v[1],v[0]])/np.linalg.norm(v)
    t=np.r_[0,.12,np.linspace(.17,.83,2*n),.88,1.]
    d=np.zeros_like(t);d[2:-2]=amp*np.tile([1,-1],n)
    xy=a+t[:,None]*v+d[:,None]*tangent
    ax.plot(xy[:,0],xy[:,1],c=colour,lw=1.1,zorder=5)

def architecture():
    fig,(a,b)=plt.subplots(1,2,figsize=(6.6,3.25),gridspec_kw={'width_ratios':[1,1.45]},layout='constrained')
    vertices=np.array([[0,30],[30*np.sqrt(3)/2,-15],[-30*np.sqrt(3)/2,-15]])
    a.add_patch(Polygon(vertices,fc=GOLD,ec='.45',alpha=.48))
    offsets=[(12,10),(12,-1),(-23,-1)]
    for i,(point,offset) in enumerate(zip(vertices,offsets)):
        a.plot([0,point[0]],[0,point[1]],c='.65',ls='--',lw=.6)
        a.add_patch(Circle(point,10,fc=RIGHT,ec=RIGHT,alpha=.85,zorder=3))
        a.text(*(point+offset),r'$\mathbf{c}_'+str(i+1)+'$',fontsize=9)
        direction=np.array([-point[1],point[0]])/30
        a.annotate('',xy=point+20*direction,xytext=point,arrowprops=dict(arrowstyle='->',color=LEFT,lw=1.0),zorder=6)
    a.plot(0,0,'o',ms=2,c='.3')
    a.annotate('',xy=(13,0),xytext=(0,0),arrowprops=dict(arrowstyle='->',color='.3',lw=.7))
    a.annotate('',xy=(0,13),xytext=(0,0),arrowprops=dict(arrowstyle='->',color='.3',lw=.7))
    a.text(14,-2,r'$y$');a.text(1.5,11,r'$z$')
    a.text(-13,14,r'$r=30$ mm',fontsize=8,ha='center')
    a.text(-23,33,r'$\mathbf{f}_{t,1}$',fontsize=9,c=LEFT)
    a.text(0,-43,'Push axis $x$ normal to the page',ha='center',fontsize=8)
    a.set(xlim=(-51,51),ylim=(-47,53),aspect='equal')
    a.set_title('(a) Front view',pad=9)
    a.axis('off')

    b.add_patch(Rectangle((-3,-20),3,42,fc='.35'))
    b.text(-6,0,'Flange',rotation=90,ha='center',va='center',fontsize=8,c='.35')
    b.plot([26,26],[-17,24],ls='--',c='.7',lw=.7)
    anchors=np.array([[26,14],[26,-7]])
    spheres=np.array([[36,14],[24,-7]])
    b.add_patch(Polygon([[34.9,14.6],[37.1,13.4],[25.1,-7.6],[22.9,-6.4]],fc=GOLD,ec='.45',zorder=2))
    for anchor,pad in zip(anchors,spheres):
        b.plot([0,anchor[0]],[anchor[1],anchor[1]],c='.55',lw=3,zorder=1)
        b.add_patch(Circle(pad,4.8,fc=RIGHT,ec=RIGHT,alpha=.35,zorder=3))
        spring(b,anchor,pad,amp=.65,n=4)
        b.plot(*anchor,'o',ms=3,c=LEFT,zorder=7)
        b.plot(*pad,'o',ms=2,c=LEFT,zorder=7)
    spring(b,[0,0],[28,0],colour=RIGHT,amp=1.2,n=7)
    b.plot(28,0,'s',ms=3,c=RIGHT,zorder=8)
    b.text(12,4,r'$K_c^{(6)}$',ha='center',color=RIGHT,fontsize=9)
    b.text(12,18,'Rigid support',ha='center',fontsize=8,c='.45')
    b.text(31,21,r'$K_i$',color=LEFT,fontsize=9)
    b.annotate('Sphere centre $s_i$',xy=(36,14),xytext=(47,26),fontsize=8,
               arrowprops=dict(arrowstyle='-',color='.4',lw=.6),ha='center')
    b.annotate('Anchor $a_i$',xy=(26,14),xytext=(19,31),fontsize=8,
               arrowprops=dict(arrowstyle='-',color='.4',lw=.6),ha='center')
    b.annotate('Rigid plate',xy=(30,3),xytext=(52,4),fontsize=8,
               arrowprops=dict(arrowstyle='-',color='.4',lw=.6),ha='center')
    b.annotate('',xy=(26,-20),xytext=(0,-20),arrowprops=dict(arrowstyle='<->',color='.5',lw=.6))
    b.text(13,-24,r'$d=20$ mm (nominal)',ha='center',fontsize=8)
    b.text(31,-32,r'Unloaded: $a_i=s_i$, $\ell_0=0$',ha='center',fontsize=9,c=LEFT)
    b.text(31,-38,r'Deformed: $\Delta_i=R_f^\top(s_i-a_i)$',ha='center',fontsize=9)
    b.set(xlim=(-10,69),ylim=(-43,36),aspect='equal')
    b.set_title('(b) Side view (deformation exaggerated)',pad=9)
    b.axis('off')
    save(fig,'ch04_contacts/plate_front_side')

def main():
    architecture()
    records=json.loads((DATA/'results.json').read_text())['cases']
    fig,axes=plt.subplots(2,3,figsize=(6.6,4.35),layout='constrained')
    names={'flat':'Flat face','tilted':'Tilted face (12 deg)','faceted':'Faceted surface'}
    for j,record in enumerate(records):
        for i,phase in enumerate(['first_load','settled']):
            ax=axes[i,j];image=plt.imread(DATA/(record['case']+'_'+phase+'.png'))
            ax.imshow(image);ax.set_xticks([]);ax.set_yticks([])
            for spine in ax.spines.values():spine.set_visible(False)
            if i==0:ax.set_title(names[record['case']])
            st=record[phase]
            ax.set_xlabel(f"{st['loaded_pads']}/3 loaded; plate tilt {st['plate_tilt_deg']:.1f} deg",fontsize=8,labelpad=3)
            if j==0:ax.set_ylabel('First load' if i==0 else 'Settled',fontsize=9,labelpad=4)
    save(fig,'ch04_contacts/plate_conformity_bullet')

    d=np.genfromtxt(DATA/'tilted_trace.csv',delimiter=',',names=True)
    fig,axes=plt.subplots(2,1,figsize=(6.2,3.45),sharex=True,layout='constrained')
    for i,c in enumerate([LEFT,RIGHT,'.35']):
        axes[0].plot(d['t'],d['N'+str(i+1)],lw=.85,c=c,label='Pad '+str(i+1))
    axes[0].set_ylabel('Normal force (N)');axes[0].legend(ncol=3,loc='upper left')
    axes[1].plot(d['t'],d['plate_tilt_deg'],c=LEFT,lw=1,label='Plate tilt')
    axes[1].axhline(12,c='.4',lw=.8,ls='--',label='Face tilt')
    axes[1].set_ylabel('Tilt (deg)');axes[1].set_xlabel('Bench time (s)');axes[1].legend(ncol=2,loc='upper left')
    for ax in axes:
        ax.axvline(1.4,c='.65',ls=':',lw=.7);ax.set_xlim(0,3);ax.grid(axis='y',color='.92',lw=.5)
    save(fig,'ch04_contacts/plate_conformity_trace')

if __name__=='__main__':main()
