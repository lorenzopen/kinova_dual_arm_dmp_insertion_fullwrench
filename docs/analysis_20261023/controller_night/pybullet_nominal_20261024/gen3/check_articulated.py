"""Independent stationary and fixed-object compression checks, no gain tuning."""
from run_gen3 import *

def main():
    cfg=json.loads((ROOT/B.META['Z']).read_text())['config']
    GE.set_tool(**{k:cfg['tool'][k] for k in ('pad_x','r_tri','pad_r')})
    r=Route('F75_Zt1_s9012','c100_five_cut800',scale=cfg['scale'],mass=cfg['piece_mass'])
    dest=HERE/'sensor_check_implicit';dest.mkdir(exist_ok=False)
    save(dest/'STARTED.json',dict(pid=os.getpid(),argv=sys.argv))
    b=ArticulatedBackend(r,cfg,dest,.0005,servo='implicit')
    initial={s:b.tools[s].pose()[0] for s in 'LR'}
    for _ in range(100):b.wait()
    free={}
    for s in 'LR':
        raw,gc=b.sample_sensor(s)
        free[s]=dict(force_norm_N=float(np.linalg.norm(gc[:3])),torque_norm_Nm=float(np.linalg.norm(gc[3:])),
            position_drift_m=float(np.linalg.norm(b.tools[s].pose()[0]-initial[s])))
        assert free[s]['force_norm_N']<.02 and free[s]['position_drift_m']<.0001,free
    axis=r.axis/np.linalg.norm(r.axis);reach=.5*r.opening+GE.PAD_X+GE.PAD_R
    closed=[r.centre-(reach-.004)*axis,GE.quat_from_x(axis),r.centre+(reach-.004)*axis,GE.quat_from_x(-axis)]
    start=b.cmd
    # Smooth 0.8s command avoids interpreting an impulsive numerical step as sensor calibration.
    for i in range(80):
        x=(i+1)/80.;x=x*x*(3-2*x)
        b.command((1-x)*start[0]+x*closed[0],closed[1],(1-x)*start[2]+x*closed[2],closed[3],8.)
        b.wait()
    samples={s:[] for s in 'LR'}
    for i in range(200):
        b.wait()
        if i<150:continue
        m=b.read()
        for s in 'LR':
            pf,q=b.tools[s].pose();cw,pads,plate=B.contact_wrench(b.body,b.tools[s],pf)
            a=Rot.from_quat(q).apply([1.,0,0]);w=np.r_[m['F'+s],m['T'+s]]
            samples[s].append(np.r_[w,cw,w[:3]@a,cw[:3]@a,pads,plate])
    stat={}
    for s in 'LR':
        x=np.array(samples[s]);avg=x.mean(axis=0)
        stat[s]=dict(wrist_wrench_mean=avg[:6],contact_wrench_mean=avg[6:12],
            wrist_normal_N=float(avg[12]),contact_normal_N=float(avg[13]),
            wrench_mean_difference=avg[:6]-avg[6:12],pad_normal_N=avg[14:17],
            wrist_normal_std_N=float(x[:,12].std()),plate_normal_N=float(avg[17]))
        assert min(avg[12],avg[13])>1,stat
        assert np.linalg.norm(avg[:3]-avg[6:9])<.1,stat
        # Sensor and contact moments are expressed about the same flange point;
        # use a separate moment tolerance rather than mixing N and Nm.
        assert np.linalg.norm(avg[3:6]-avg[9:12])<.01,stat
    result=dict(free_air=free,stationary_compression=stat,central_k=2700.,pad_each_k=500.,
        spring_total_k=4200.,servo='implicit',dt=.0005,
        limitation='Known static test checks sensor sign/gravity/pole; not dynamic hardware identification')
    save(HERE/'articulated_sensor_checks.json',result)
    save(dest/'COMPLETED.json',dict(pid=None));p.disconnect()
    print(json.dumps(result,default=serial),flush=True)
if __name__=='__main__':main()
