"""Decode actual MCAP samples, preserve all source files, cache outside archive.

Reconstruct ATI normal forces using recorded units, tare, sensor/tool mounts and
CSV measured orientations. This checks the recording pipeline; it is not an
independent sensor validation. No ROS installation or robot connection is used.
"""
from analyze import *
from mcap_ros2.decoder import DecoderFactory

FORCE_UNITS={'N':1.,'lbf':4.4482216,'Lbf':4.4482216,'kN':1000.,'kgf':9.80665}


def stamp(x):
    return x.header.stamp.sec+1e-9*x.header.stamp.nanosec


def extract(path):
    cache=OUT/'bag_cache';cache.mkdir(exist_ok=True)
    dest=cache/(Path(path).stem+'.npz')
    if dest.exists():
        with np.load(dest) as a:return {k:a[k] for k in a.files}
    topics=[]
    for side in ['left','right']:
        topics.extend(f'/{side}_arm/{s}' for s in ['ft_data_avg','joint_states','target_pose'])
    data=defaultdict(list)
    with (ROOT/path).open('rb') as f:
        reader=make_reader(f,decoder_factories=[DecoderFactory()])
        for schema,ch,msg,x in reader.iter_decoded_messages(topics=topics):
            side='L' if ch.topic.startswith('/left') else 'R'
            row=[stamp(x),msg.log_time/1e9]
            if ch.topic.endswith('ft_data_avg'):
                w=x.wrench;row.extend([w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z]);key='ft'+side
            elif ch.topic.endswith('joint_states'):
                row.extend(list(x.position[:7])+list(x.velocity[:7]));key='joints'+side
            elif ch.topic.endswith('target_pose'):
                p=x.pose;row.extend([p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]);key='pose'+side
            data[key].append(row)
    arrays={k:np.array(v,float) for k,v in data.items()}
    np.savez_compressed(dest,**arrays)
    return arrays


def normals_from_bag(run,arrays):
    d=run['d'];meta=run['rec']['meta'];clock=d['t_wall'];result={};validation={}
    tare=meta['tares'][-1]
    for side,name in [('L','left'),('R','right')]:
        a=arrays['ft'+side];order=np.argsort(a[:,0],kind='stable');a=a[order]
        tt=a[:,0];cfg=meta['config'][name]
        scale=FORCE_UNITS[meta['ft_units'][name]['force']]
        sensor=Rotation.from_quat(cfg['ft_in_ee']['quat']);tool=Rotation.from_quat(cfg['tool_in_ee']['quat'])
        normal_sensor_axis=sensor.inv().apply(tool.apply([1.,0.,0.]))
        bias=np.array(tare[name]['bias_F'])
        q=np.column_stack([np.interp(tt,clock,d['mq'+c+side]) for c in 'xyzw'])
        axis=Rotation.from_quat(q).apply(np.tile([1.,0.,0.],(len(q),1)))
        y=-((cfg.get('ft_sign',1.)*scale*a[:,2:5]-bias)@normal_sensor_axis)
        y+=(-9.81*cfg.get('tool_mass',0.))*axis[:,2]
        result[side]=(tt,np.maximum(y,0.))
        # Match the mean of the last two 200 Hz samples to the logged raw ATI.
        if 'ft_stamp_'+side not in d:
            validation[side]={'native_force_unit':meta['ft_units'][name]['force'],
                              'raw_average_comparison':'unavailable: source CSV lacks ATI message stamps'}
            continue
        k=run['route']&np.isfinite(d['ft_stamp_'+side])
        ix=np.searchsorted(tt,d['ft_stamp_'+side][k]+1e-6,side='right')-1
        valid=(ix>=1)&(ix<len(tt))
        ix=ix[valid]
        logged=np.column_stack([d['ft'+side+'_'+c][k][valid] for c in ['fx','fy','fz']])
        n=max(1,int(round(cfg.get('ft_rate_hz',200)/meta['config'].get('rate_hz',100))))
        # Last n samples in the backend buffer, in physical force units.
        enough=ix>=n-1;ix=ix[enough];logged=logged[enough]
        avg=np.mean([a[ix-j,2:5] for j in range(n)],axis=0)*scale
        delta=logged-avg
        validation[side]={'native_force_unit':meta['ft_units'][name]['force'],
                          'raw_average_comparison_samples':len(ix),
                          'raw_average_component_rmse_N':float(np.sqrt(np.mean(delta**2))),
                          'raw_average_component_abs_p99_N':float(np.percentile(abs(delta),99)),
                          'timestamp_match_abs_max_ms':float(1000*abs(tt[ix]-d['ft_stamp_'+side][k][valid][enough]).max())}
    return result,validation


def analyse_bag(run):
    m=run['metrics'];path=m['bag_match']['path'];arrays=extract(path)
    result={'run':m['id'],'bag':path,'topics':{}}
    lo,hi=m['route_epoch']
    for key,a in arrays.items():
        k=(a[:,0]>=lo)&(a[:,0]<=hi);tt=a[k,0];dt=np.diff(tt);dt=dt[dt>0]
        result['topics'][key]={'route_messages':int(k.sum()),
                                'median_rate_Hz':float(1/np.median(dt)) if len(dt) else None,
                                'max_interval_ms':float(1000*dt.max()) if len(dt) else None}
    normals,validation=normals_from_bag(run,arrays)
    result['validation']=validation
    u=np.arange(lo,hi,.005)
    left=np.interp(u,*normals['L']);right=np.interp(u,*normals['R'])
    mean=(left+right)/2
    b=bands(u-u[0],mean)
    result['normal_mean_hand_bands']=b
    csv_rms=m['bands_mean_hand']['0.8_2.5']['rms_N']
    result['csv_band_rms_N']=csv_rms
    result['band_rms_bag_over_csv']=b['0.8_2.5']['rms_N']/csv_rms
    _,lb,k=previous.bandpass(u-u[0],left,[.8,2.5]);_,rb,_=previous.bandpass(u-u[0],right,[.8,2.5])
    result['left_right_band_correlation']=float(np.corrcoef(lb[k],rb[k])[0,1])
    run['bag_normals']=normals
    return result


def plot_validation(runs,results):
    selected=[r for r in runs if 'bag_normals' in r and r['metrics']['status']=='curated_seat_found' and 'ffi' in r['metrics']]
    fig,axes=plt.subplots(4,2,figsize=(14,12))
    fig.subplots_adjust(left=.08,right=.98,top=.89,bottom=.13,hspace=.57,wspace=.25)
    fig.suptitle('Le bag confermano le oscillazioni registrate nei CSV',x=.08,ha='left',y=.975,fontsize=20,weight='bold',color='#172b40')
    fig.text(.08,.935,'Una prova FFI per pezzo · ATI 200 Hz dalla bag contro CSV circa 100 Hz · zoom 5–10 s',fontsize=11)
    for row,pn in enumerate('ZLST'):
        run=next(r for r in selected if r['metrics']['piece']==pn)
        m,d,t=run['metrics'],run['d'],run['t'];epoch=m['route_epoch'][0]
        u=np.arange(epoch+5,epoch+10,.005)
        y=sum(np.interp(u,*run['bag_normals'][s]) for s in 'LR')/2
        k=run['route']&(t>=5)&(t<=10)
        a=axes[row,0];a.plot(u-epoch,y,color='#1675bb',lw=1,label='Bag ATI 200 Hz')
        a.plot(t[k],d['fn_N'][k]/2,color='#df711c',lw=.85,alpha=.85,label='CSV media mani')
        a.set(title=pn+' · '+m['label'],ylabel='Forza media mani [N]',xlim=(5,10));a.legend(fontsize=8,ncol=2)
        a=axes[row,1]
        uu=np.arange(epoch,m['route_epoch'][1],.005)
        yy=sum(np.interp(uu,*run['bag_normals'][s]) for s in 'LR')/2
        f,amp=previous.spectrum(uu-epoch,yy)
        a.plot(f,amp,color='#1675bb',lw=1,label='Bag')
        f,amp=previous.spectrum(t[run['route']],d['fn_N'][run['route']]/2)
        a.plot(f,amp,color='#df711c',lw=.8,alpha=.8,label='CSV')
        a.set(title='Spettro detrendizzato, finestra Hann',xlim=(0,10),ylabel='Ampiezza [N]');a.legend(fontsize=8)
        axes[row,0].set_xlabel('Tempo da inizio route [s]');a.set_xlabel('Frequenza [Hz]')
        for a in axes[row]:a.grid(alpha=.2)
    fig.text(.08,.035,'Ricostruzione con unità native, ultima tara, montaggi sensore/tool e orientamenti misurati del CSV.\n'
             'Sono due registrazioni della stessa catena di misura, non due sensori indipendenti. Nessun filtro aggiunto nello zoom.',
             fontsize=10,color='#526173',linespacing=1.5)
    fig.savefig(OUT/'03_verifica_bag_vs_csv.png',dpi=155);plt.close(fig)


def stationary():
    # No tare/absolute-force claim for these bags: show only projection variations.
    bag_list=json.loads((OUT/'bag_inventory.json').read_text())
    reference=next((SOURCE/'runs_riuscitee/20261023_F75_Zt1_s9012_offline').glob('*meta.json'))
    meta=json.loads(reference.read_text())
    records=[]
    fig,axes=plt.subplots(2,1,figsize=(12,8))
    fig.subplots_adjust(left=.09,right=.98,top=.85,bottom=.18,hspace=.45)
    fig.suptitle('Bag denominate “fermo”: cosa contengono davvero',x=.09,ha='left',y=.97,fontsize=20,weight='bold')
    fig.text(.09,.915,'Proiezione sensoriale della normale, centrata sulla mediana: nessuna stima della forza assoluta.',fontsize=10)
    for ax,b in zip(axes,[b for b in bag_list if 'fermo_' in b['path']]):
        arrays=extract(b['path']);times=[];ys=[]
        if not all('ft'+s in arrays for s in 'LR'):
            record={'bag':b['path'],'usable_force_data':False,
                    'reason':'No bilateral ft_data_avg samples; this bag contains rosout only.'}
            records.append(record)
            ax.text(.5,.5,'Solo /rosout: nessun campione di forza disponibile',ha='center',va='center',transform=ax.transAxes)
            ax.set_title(Path(b['path']).parent.name)
            continue
        for side,name in [('L','left'),('R','right')]:
            a=arrays['ft'+side];cfg=meta['config'][name]
            c=Rotation.from_quat(cfg['ft_in_ee']['quat']).inv().apply(Rotation.from_quat(cfg['tool_in_ee']['quat']).apply([1.,0.,0.]))
            y=-(a[:,2:5]@c)*FORCE_UNITS[meta['ft_units'][name]['force']]
            times.append(a[:,0]);ys.append(y)
        lo=max(a[0] for a in times);hi=min(a[-1] for a in times);u=np.arange(lo,hi,.005)
        y=sum(np.interp(u,tt,yy) for tt,yy in zip(times,ys))/2
        rel=u-lo;centered=y-np.median(y)
        ax.plot(rel,centered,color='#1675bb',lw=.7);ax.set(title=Path(b['path']).parent.name,ylabel='Variazione media mani [N]',xlabel='Tempo da primo campione comune [s]');ax.grid(alpha=.2)
        record={'bag':b['path'],'duration_s':float(rel[-1]),'std_whole_N':float(centered.std()),
                'range_whole_N':float(np.ptp(centered)),'candidate_quiet_windows':[]}
        fixed=(rel>=20)&(rel<60)
        if fixed.sum()>100:
            movement=[]
            for side in 'LR':
                a=arrays['joints'+side];j=(a[:,0]>=lo+20)&(a[:,0]<lo+60)
                movement.append(float(np.max(np.std(np.unwrap(a[j,2:9],axis=0),axis=0))))
            record['fixed_window_20_60s']={'std_force_mean_hand_N':float(y[fixed].std()),
                   'bands_mean_hand':bands(rel[fixed],y[fixed]),'joint_position_max_std_rad_L_R':movement}
            ax.axvspan(20,60,color='#dce6ef',alpha=.5)
            ax.text(.03,.9,f'20–60 s: deviazione standard {y[fixed].std():.3f} N sulla media mani',
                    transform=ax.transAxes,fontsize=9,va='top')
        # Fixed consecutive 10s windows, not selected by lowest force noise.
        for start in np.arange(0,rel[-1]-10,10):
            k=(rel>=start)&(rel<start+10);motion=[]
            for side in 'LR':
                a=arrays['joints'+side];j=(a[:,0]>=lo+start)&(a[:,0]<lo+start+10)
                motion.append(float(np.max(np.std(a[j,2:9],axis=0))) if j.any() else None)
            bd=bands(rel[k],y[k]).get('0.8_2.5',{}).get('rms_N')
            record['candidate_quiet_windows'].append({'start_s':float(start),'end_s':float(start+10),
                 'std_force_N':float(y[k].std()),'band_rms_N':bd,'joint_position_max_std_rad_L_R':motion})
        records.append(record)
    fig.text(.09,.035,'Nome della cartella ≠ prova automatica di posa, carico o contatto costanti. Finestre fisse di 10 s nel JSON\n'
             'riportano sia variabilità della forza sia movimento articolare. Nessuna finestra è stata scelta minimizzando il rumore di forza.',
             fontsize=10,color='#526173',linespacing=1.5)
    fig.savefig(OUT/'04_bag_fermo.png',dpi=155);plt.close(fig)
    dump('stationary_bags.json',records)


def main():
    recs=catalog();metrics=json.loads((OUT/'successful_and_reference_metrics.json').read_text())
    by_id={r['id']:r for r in recs};runs=[];results=[];issues=[]
    for m in metrics:
        if 'bag_match' not in m or m['status'] not in ['curated_seat_found','metadata_seat_found_only']:continue
        rec=by_id[m['id']]
        if not rec['meta'].get('tares') or not all(v=='ft' for v in (rec['meta'].get('wrench_source') or {}).values()):continue
        try:
            run=analyse_run(rec);run['metrics']['bag_match']=m['bag_match'];result=analyse_bag(run)
            results.append(result);runs.append(run)
            print(m['id'],'bag/CSV RMS',round(result['band_rms_bag_over_csv'],3),
                  'raw RMSE', {s:round(v['raw_average_component_rmse_N'],4) if 'raw_average_component_rmse_N' in v else 'no stamps' for s,v in result['validation'].items()},flush=True)
        except Exception as e:
            issues.append({'id':m['id'],'error':repr(e)});print('BAG ISSUE',m['id'],repr(e),flush=True)
    dump('bag_validation.json',results);dump('bag_analysis_issues.json',issues)
    previous.style();plot_validation(runs,results);stationary()
    print('BAGS ANALYSED',len(results),'ISSUES',len(issues),flush=True)


if __name__=='__main__':main()
