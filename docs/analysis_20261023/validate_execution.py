"""Integration matrix on the toy backend. It has no neighbours/delay/slip dynamics."""
import contextlib
import copy
import io
import json
import time
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))

from kinova_dual_arm_dmp_insertion import main as M
from kinova_dual_arm_dmp_insertion.controller import Insertion
from kinova_dual_arm_dmp_insertion.dryrun import DryRun
from kinova_dual_arm_dmp_insertion.insert_open import OpenInsertion, ProfileInsertion, offline_profile, series_stiffness
from kinova_dual_arm_dmp_insertion.route import Route

OUT = Path(__file__).resolve().parent
cfg0,_ = M.load_config(None)
results=[]
for stiffness in (1000.,1500.):
    for world in ('F75_Zt1_s9012','F75_Tt1_s9007','F75_Lt1_s9014'):
        cfg=copy.deepcopy(cfg0)
        cfg['stiffness']['trans']=stiffness
        cfg['log_dir']=str(ROOT/'tmp/controller_review_profiles')
        r=Route(world,'c100_five_cut800',cfg['scale'],cfg['piece_mass'])
        k=series_stiffness(stiffness)
        with contextlib.redirect_stdout(io.StringIO()):
            profile=offline_profile(SimpleNamespace(slow=3.,arm='c100_five_cut800'),r,k,cfg)
        for mode in ('online_scalar','online_forces','online_contact','offline_feedforward','offline_forces','offline_contact'):
            b=DryRun(r,k_face=k,period=.01)
            kw=dict(confirm=lambda msg:None,squeeze_gain=.25,**M.socp_kwargs(cfg))
            kw.update(full_wrench=('forces' in mode or 'contact' in mode),
                      full_wrench_channels='contact' if 'contact' in mode else 'forces')
            ticklog=[]
            class Record:
                def row(self,ins,*args):
                    if ins.frame is not None:
                        ticklog.append((ins.target or 0.,ins.normal_force(),ins.ds,
                                        max(np.linalg.norm(o[:3]) for o in ins.off.values())))
            kw['log']=Record()
            with contextlib.redirect_stdout(io.StringIO()):
                if mode=='offline_feedforward': ins=OpenInsertion(r,b,k_eff=k,profile=profile,**kw)
                elif mode in ('offline_forces','offline_contact'): ins=ProfileInsertion(r,b,profile=profile,**kw)
                else: ins=Insertion(r,b,**kw)
                ins=M.slowed(ins,3.)
                start=time.perf_counter()
                out=ins.run()
            arr=np.array(ticklog)
            assert np.isfinite(arr).all()
            assert out['route_followed'] and out['seat_found'] and not out['jammed']
            assert abs(arr[:,2]).max()<=.06000001 and arr[:,3].max()<=.02000001
            if 'forces' in mode or 'contact' in mode: assert arr[:,3].max()<=.00200001
            if mode.startswith('offline'): assert ins.stats['socp_ticks']==0
            results.append(dict(world=world,arm_stiffness_N_per_m=stiffness,k_eff=k,mode=mode,
                result=out,execution_seconds=time.perf_counter()-start,
                max_internal_translation_mm=1000*float(arr[:,3].max()),
                max_squeeze_model_error_N=float(np.max(np.abs(arr[:,1]-2*arr[:,0]))),
                planned_phases=list(profile.tab)))
            print(world,stiffness,mode,'OK',flush=True)
(OUT/'dryrun_matrix.json').write_text(json.dumps(results,indent=2))
print(f'{len(results)} dry-run cases passed',flush=True)
