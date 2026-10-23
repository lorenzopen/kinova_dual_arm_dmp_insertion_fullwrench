"""Recompute Z force tracking on real CSVs; does not identify a dynamic plant.

No ROS, hardware, control changes or force reconstruction using the current SOCP.
"""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfiltfilt
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from kinova_dual_arm_dmp_insertion.audit_run import load, spectrum

OUT = Path(__file__).resolve().parent
BASE = ROOT/'docs/runs'
SOURCES = [
    ('22 Oct 19:32 offline, k=730', '20261022_F75_Zt1_s9012_offline/1_1932_keff730', '*offline_*.csv'),
    ('22 Oct 19:53 offline, k=480', '20261022_F75_Zt1_s9012_offline/2_1953_keff480', '*offline_*.csv'),
    ('22 Oct 20:13 offline, k=580', '20261022_F75_Zt1_s9012_offline/3_2013_K1500_keff580', '*offline_*.csv'),
    ('22 Oct 20:20 online, gain=0.5', '20261022_F75_Zt1_s9012_offline/4_2020_K1500_online_gain05', '*online_*.csv'),
    ('23 Oct 12:32 offline, k=964', '20261023_F75_Zt1_s9012_offline', '*offline_*.csv'),
]


def main():
    fig, axes = plt.subplots(len(SOURCES), 3, figsize=(14, 13), layout='constrained')
    results = []
    for row, (label, folder, glob) in enumerate(SOURCES):
        paths = [p for p in (BASE/folder).glob(glob) if '_profile_' not in p.name]
        assert len(paths) == 1, paths
        path = paths[0]
        d = load(path)
        meta = json.loads(path.with_name(path.stem+'_meta.json').read_text())
        route = d['phase'] == 'route'
        g = {k:v[route] for k,v in d.items()}
        t = g['t_wall'] - g['t_wall'][0]
        dt = np.diff(t)
        assert (dt > 0).all()
        target, actual = 2*g['target_N'], g['fn_N']
        splits = np.minimum((6*t/max(t[-1], .001)).astype(int), 5)
        bins = []
        for j in range(6):
            keep = splits == j
            bins.append(dict(target_sum_N=float(np.median(target[keep])),
                             actual_sum_N=float(np.median(actual[keep])),
                             ratio=float(np.median(actual[keep]/target[keep]))))
        hand_normals, tangential = [], []
        for side in 'LR':
            q = np.column_stack([g['mq'+c+side] for c in ('x','y','z','w')])
            a = Rotation.from_quat(q).apply(np.tile([1.,0.,0.], (len(t),1)))
            force = np.column_stack([g['F'+c+side] for c in 'xyz'])
            n = np.einsum('ij,ij->i', force, a)
            hand_normals.append(np.maximum(n, 0.))
            tangential.append(np.linalg.norm(force-n[:,None]*a, axis=1))
        normal_check = np.max(np.abs(actual - np.sum(hand_normals, axis=0)))
        f, amp = spectrum(t, actual)
        band = (f >= .8) & (f <= 2.5)
        peak = np.flatnonzero(band)[np.argmax(amp[band])]
        tu = np.arange(0, t[-1], np.median(dt))
        y = np.interp(tu, t, actual)
        y -= np.polyval(np.polyfit(tu,y,3),tu)
        filtered = sosfiltfilt(butter(2,[.8,2.5],fs=1/np.median(dt),btype='band',output='sos'),y)
        keep = (tu >= 1.5) & (tu <= tu[-1]-1.5)
        rms_band = float(np.std(filtered[keep]))
        axis = Rotation.from_quat(np.column_stack([g[k+'L'] for k in ('qx','qy','qz','qw')])).apply(
            np.tile([1.,0.,0.], (len(t),1)))
        command_gap = np.einsum('ij,ij->i',np.column_stack([g[c+'R']-g[c+'L'] for c in 'xyz']),axis)
        force_external = np.column_stack([g['ext_'+c] for c in 'xyz'])
        # Component perpendicular to gravity is a useful contact-load proxy;
        # it does not determine exact contact onset or measure object slip.
        horizontal_load = np.linalg.norm(force_external[:,:2],axis=1)
        axes[row,0].plot(t,actual,label='ATI sum')
        axes[row,0].plot(t,target,'--',label='SOCP sum')
        axes[row,0].set_title(label); axes[row,0].set_ylabel('normal force [N]')
        axes[row,1].plot(t,actual/target,label='measured / target')
        axes[row,1].axhline(1,color='k',ls='--',lw=.8)
        axes[row,1].set_ylabel('force ratio')
        axes[row,2].plot(t,1000*(command_gap-command_gap[0]),label='commanded opening change [mm]')
        axes[row,2].plot(t,horizontal_load,label='horizontal external load [N]')
        for ax in axes[row]:
            ax.grid(alpha=.25); ax.legend(fontsize=7); ax.set_xlabel('route elapsed wall time [s]')
        result = dict(label=label,path=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            argv=meta.get('argv'), recorded_controller_parameters=meta.get('controller_parameters'),
            execution=meta.get('execution'), result=meta.get('result'), bins=bins,
            total_force_min_N=float(actual.min()), total_force_median_N=float(np.median(actual)),
            total_force_error_rms_N=float(np.sqrt(np.mean((actual-target)**2))),
            median_ratio=float(np.median(actual/target)),
            normal_projection_vs_logged_max_difference_N=float(normal_check),
            band_0p8_2p5_rms_N=rms_band, band_rms_over_mean_force=rms_band/float(actual.mean()),
            spectral_peak_Hz=float(f[peak]), spectral_peak_total_N=float(amp[peak]),
            per_hand_normal_min_N=[float(n.min()) for n in hand_normals],
            per_hand_Ft_over_N_p95=[float(np.percentile(ft/np.maximum(n,.01),95))
                                      for ft,n in zip(tangential,hand_normals)],
            horizontal_load_max_N=float(horizontal_load.max()),
            csv_dt_ms_p50_p99_max=(1000*np.percentile(dt,[50,99,100])).tolist())
        results.append(result)
        print(label, 'ratio', round(result['median_ratio'],3), 'min',round(result['total_force_min_N'],2),
              'band RMS', round(rms_band,3), 'relative',round(result['band_rms_over_mean_force'],3),flush=True)
    fig.savefig(OUT/'new_runs_force_tracking.png',dpi=140)
    (OUT/'new_runs_metrics.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')


if __name__ == '__main__':
    main()
