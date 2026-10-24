"""Regenerate Chapter 11 comparisons from frozen compact arrays, without physics.

Run this script with Python plus numpy, scipy and matplotlib. Source identities,
scope and selected original run IDs are in data/kinova_comparison_20261024.
"""
import json
import hashlib
from pathlib import Path
import numpy as np
from scipy.signal import butter, sosfiltfilt
from style import FIG, LEFT, RIGHT, plt, save

DATA = FIG / 'data/kinova_comparison_20261024'
ACTIVE = ['to_mouth','route','slow_seat','seat_by_feel']
COL = dict(L=LEFT, R=RIGHT)
plt.rcParams.update({'mathtext.fontset':'cm'})

def load(name):
    return dict(np.load(DATA/(name+'.npz'), allow_pickle=False))

def clock(d):
    return d['t']-d['t'][np.flatnonzero(d['phase']=='to_mouth')[0]]

def decorate(ax, d, xlim=None):
    t = clock(d)
    active = np.isin(d['phase'], ACTIVE)
    for i in np.flatnonzero(np.r_[True, d['phase'][1:] != d['phase'][:-1]]):
        if active[i]: ax.axvline(t[i], color='.70', ls=':', lw=.55, zorder=0)
    ax.set_xlim(*(xlim if xlim else (0, float(t[active][-1])+.1)))
    ax.set_xlabel('Time from departure (s)')
    ax.grid(axis='y', color='.92', lw=.5)

def normals(ax,d):
    t=clock(d); m=np.isin(d['phase'], ACTIVE)
    for side in 'LR': ax.plot(t[m], d[side+'_normal'][m], c=COL[side], lw=.65, label=side)
    ax.plot(t[m], d['target'][m], c='.15', ls='--', lw=.9, label=r'$N^*$')
    ax.set_ylabel('Normal force (N)')
    decorate(ax,d)

def band(values,lo,hi):
    y=sosfiltfilt(butter(3,[lo,hi],btype='bandpass',fs=100,output='sos'),values)
    return float(np.sqrt(np.mean(y*y)))

def metrics(d):
    route=d['phase']=='route'; t=d['t'][route]
    grid=np.arange(t[0],t[-1],.01)
    force=np.interp(grid,t,d['mean_normal'][route])
    target=np.interp(grid,t,d['target'][route]);e=force-target
    seat=np.isin(d['phase'],['slow_seat','seat_by_feel'])
    active=np.isin(d['phase'],ACTIVE)
    result=dict(route_rmse_N=float(np.sqrt(np.mean(e*e))),
        route_force_band_08_25_N=band(force,.8,2.5),
        route_error_band_08_25_N=band(e,.8,2.5),
        route_force_band_25_10_N=band(force,2.5,10),
        seat_positive_error_N=float((d['mean_normal']-d['target'])[seat].max()),
        active_min_L_N=float(d['L_normal'][active].min()),
        active_min_R_N=float(d['R_normal'][active].min()))
    return result

def main():
    manifest=json.loads((DATA/'manifest.json').read_text())
    for record in manifest['local_arrays']:
        assert hashlib.sha256((DATA/record['path']).read_bytes()).hexdigest()==record['sha256']
    pairs={piece: [load(piece+'_'+origin) for origin in ['real','sim']] for piece in ['Z','L']}
    records={piece:dict(zip(['real','simulation'],[metrics(d) for d in pair])) for piece,pair in pairs.items()}
    (DATA/'metrics.json').write_text(json.dumps(dict(definition=manifest['metrics'],cases=records),indent=2)+'\n')

    fig,axes=plt.subplots(2,2,figsize=(6.6,4.9),layout='constrained')
    for i,(piece,pair) in enumerate(pairs.items()):
        for j,d in enumerate(pair):
            normals(axes[i,j],d)
            axes[i,j].set_title(piece+(' - real ATI' if j==0 else ' - simulated, approximate servo'))
            axes[i,j].legend(loc='upper right',ncol=3,handlelength=1.2,columnspacing=.8)
        lim=(0,max(ax.get_ylim()[1] for ax in axes[i]))
        end=max(ax.get_xlim()[1] for ax in axes[i])
        for ax in axes[i]:ax.set_ylim(*lim);ax.set_xlim(0,end)
    save(fig,'ch11_kinova/real_sim_normal')

    fig,axes=plt.subplots(1,2,figsize=(6.6,2.65),layout='constrained')
    vals=[]
    for d in pairs['Z']:
        mask=(clock(d)>=12)&(clock(d)<=17)
        vals.extend(np.r_[d['L_normal'][mask],d['R_normal'][mask],d['target'][mask]])
    for ax,d,title in zip(axes,pairs['Z'],['Z - real ATI','Z - simulated, approximate servo']):
        normals(ax,d);ax.set_xlim(12,17);ax.set_ylim(min(vals)-.07,max(vals)+.07)
        ax.set_title(title);ax.legend(ncol=3,loc='upper right',handlelength=1.2,columnspacing=.8)
    save(fig,'ch11_kinova/real_sim_zoom')

    fig,axes=plt.subplots(2,2,figsize=(6.6,4.8),layout='constrained')
    for i,(piece,pair) in enumerate(pairs.items()):
        for j,d in enumerate(pair):
            t=clock(d);mask=np.isin(d['phase'],ACTIVE);ax=axes[i,j]
            for side in 'LR':ax.plot(t[mask],np.linalg.norm(d[side+'_moment'][mask],axis=1),c=COL[side],lw=.65,label=side)
            ax.set_ylabel('Wrist moment (Nm)');decorate(ax,d)
            ax.set_title(piece+(' - real ATI' if j==0 else ' - simulated, approximate servo'))
            ax.legend(ncol=2,loc='upper right',handlelength=1.2)
        lim=(0,max(ax.get_ylim()[1] for ax in axes[i]));end=max(ax.get_xlim()[1] for ax in axes[i])
        for ax in axes[i]:ax.set_ylim(*lim);ax.set_xlim(0,end)
    save(fig,'ch11_kinova/real_sim_moment')

    raw=load('original_source_stop')['raw_wrist_1kHz']
    result=json.loads((DATA/'original_source_result.json').read_text())
    fig,axes=plt.subplots(2,1,figsize=(6.0,3.6),sharex=True,layout='constrained')
    for side,start in [('L',13),('R',19)]:
        axes[0].plot(raw[:,0]*1000,np.linalg.norm(raw[:,start:start+3],axis=1),c=COL[side],lw=.8,label=side)
        axes[1].plot(raw[:,0]*1000,np.linalg.norm(raw[:,start+3:start+6],axis=1),c=COL[side],lw=.8,label=side)
    for ax in axes:
        for event in result['laboratory_events']:ax.axvline(1000*event['t'],c=COL[event['side']],ls=':',lw=.8)
        ax.legend(ncol=2,loc='upper left');ax.grid(axis='y',color='.92',lw=.5)
        ax.set_xlim(0,120);ax.set_ylim(bottom=0)
    axes[0].set_ylabel('Force magnitude (N)');axes[1].set_ylabel('Moment magnitude (Nm)')
    axes[1].set_xlabel('Simulated time (ms)')
    save(fig,'ch11_kinova/original_source_stop')
    print(json.dumps(records,indent=2))

if __name__=='__main__': main()
