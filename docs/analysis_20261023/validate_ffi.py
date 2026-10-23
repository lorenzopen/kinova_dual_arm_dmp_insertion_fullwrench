"""Synthetic full-sequence FFI checks, including a stiffness-model mismatch.

No hardware dynamics, neighbour contacts, delay or slip. CSVs are explicitly
SYNTHETIC and live in tmp/ffi_validation; this does not validate Kinova stability.
"""
import contextlib
import copy
import csv
import io
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))

from kinova_dual_arm_dmp_insertion import main as M
from kinova_dual_arm_dmp_insertion.dryrun import DryRun
from kinova_dual_arm_dmp_insertion.insert_ffi import FFIInsertion, FFIProfileInsertion
from kinova_dual_arm_dmp_insertion.insert_open import offline_profile, series_stiffness
from kinova_dual_arm_dmp_insertion.route import Route


def validate():
    cfg0,cfgpath = M.load_config(None)
    dest = ROOT/'tmp/ffi_validation'
    dest.mkdir(parents=True,exist_ok=True)
    results = []
    for arm_k,actual_k in ((1000.,series_stiffness(1000.)),
                            (1500.,series_stiffness(1500.)),(1500.,680.)):
        for world in ('F75_Zt1_s9012','F75_Tt1_s9007','F75_Lt1_s9014'):
            cfg = copy.deepcopy(cfg0)
            cfg['stiffness']['trans'] = arm_k
            cfg['log_dir'] = str(dest)
            r = Route(world,'c100_five_cut800',cfg['scale'],cfg['piece_mass'])
            model_k = series_stiffness(arm_k)
            with contextlib.redirect_stdout(io.StringIO()):
                prof = offline_profile(SimpleNamespace(slow=3.,arm='c100_five_cut800'),r,model_k,cfg)
            for offline in (False,True):
                for channels in ('scalar','contact'):
                    stem = f'SYNTHETIC_{world}_{int(arm_k)}_actual{actual_k:.0f}_{offline}_{channels}'
                    path = dest/(stem+'.csv')
                    log = M.Log(path)
                    b = DryRun(r,period=.01,k_face=actual_k)
                    kw = dict(log=log,confirm=lambda msg:None,squeeze_gain=.1,**M.socp_kwargs(cfg))
                    kw.update(full_wrench=channels=='contact',full_wrench_channels='contact')
                    with contextlib.redirect_stdout(io.StringIO()):
                        cls = FFIProfileInsertion if offline else FFIInsertion
                        ins = cls(r,b,k_eff=model_k,**({'profile':prof} if offline else {}),**kw)
                        ins = M.slowed(ins,3.)
                        try:
                            result = ins.run()
                        finally:
                            log.close()
                    assert result['route_followed'] and result['seat_found'] and not result['jammed'],result
                    assert ins.stats['socp_fail']==0
                    assert ins.stats['socp_ticks']==(0 if offline else ins.stats['socp_ok'])
                    with path.open() as fh:
                        rows = list(csv.DictReader(fh))
                    diagnostics = [row for row in rows if np.isfinite(float(row['ffi_seq']))]
                    assert diagnostics
                    assert all(np.isfinite(float(row[key])) for row in diagnostics for key in M.FFI_COLS)
                    # Distinct controller updates only; release repeats the last diagnostics.
                    distinct = [row for j,row in enumerate(diagnostics)
                                if j==0 or row['ffi_seq']!=diagnostics[j-1]['ffi_seq']]
                    out = np.array([float(row['ffi_output_mm']) for row in distinct])
                    trim = np.array([float(row['ffi_i_mm']) for row in distinct])
                    ff = np.array([float(row['ffi_ff_mm']) for row in distinct])
                    dt = np.array([float(row['ffi_dt_ms'])/1000 for row in distinct])
                    np.testing.assert_allclose(out,ff+trim,atol=1e-10)
                    assert abs(out).max()<=12.000001 and abs(trim).max()<=3.000001
                    assert abs(out[0])<1e-12
                    assert (np.abs(np.diff(out))<=2*dt[1:]+1e-9).all()
                    assert (np.abs(np.diff(trim))<=dt[1:]+1e-9).all()
                    assert max(np.linalg.norm(o[:3]) for o in ins.off.values())<=.0020001
                    M.write_meta(str(path.with_name(stem+'_meta.json')),cfg,cfgpath,SimpleNamespace(),
                                 result=dict(**result,source='synthetic DryRun'),ins=ins)
                    results.append(dict(world=world,arm_k=arm_k,model_k=model_k,toy_actual_k=actual_k,
                                        offline=offline,channels=channels,result=result,
                                        max_integral_trim_mm=float(abs(trim).max()),
                                        max_total_correction_mm=float(abs(out).max()),
                                        source='synthetic DryRun; not hardware stability validation'))
                    print(stem,'OK',flush=True)
    (Path(__file__).parent/'ffi_dryrun_matrix.json').write_text(json.dumps(results,indent=2)+'\n')
    print(f'{len(results)} FFI toy executions passed',flush=True)


if __name__=='__main__':
    validate()
