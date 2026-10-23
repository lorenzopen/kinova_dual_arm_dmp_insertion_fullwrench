"""PI execution contracts on the toy backend, NOT a physical stability assessment.

Run from the repository with numpy/scipy/cvxpy installed. CSVs under tmp/pi_validation
are synthetic. Reproduce online/offline, scalar/tangential, three routes and two axial
stiffnesses. This backend has no neighbour contact, delay, resonance or slip dynamics.
"""
import contextlib
import copy
import io
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from kinova_dual_arm_dmp_insertion import main as M
from kinova_dual_arm_dmp_insertion.dryrun import DryRun
from kinova_dual_arm_dmp_insertion.insert_open import offline_profile, series_stiffness
from kinova_dual_arm_dmp_insertion.insert_pi import PIInsertion, PIProfileInsertion
from kinova_dual_arm_dmp_insertion.route import Route


def validate():
    cfg0, cfgpath = M.load_config(None)
    dest = ROOT/'tmp/pi_validation'
    dest.mkdir(parents=True, exist_ok=True)
    results = []
    for arm_k in (1000., 1500.):
        for world in ('F75_Zt1_s9012', 'F75_Tt1_s9007', 'F75_Lt1_s9014'):
            cfg = copy.deepcopy(cfg0)
            cfg['stiffness']['trans'] = arm_k
            cfg['log_dir'] = str(dest)
            route = Route(world, 'c100_five_cut800', cfg['scale'], cfg['piece_mass'])
            with contextlib.redirect_stdout(io.StringIO()):
                profile = offline_profile(SimpleNamespace(slow=3., arm='c100_five_cut800'),
                                          route, series_stiffness(arm_k), cfg)
            for offline in (False, True):
                for channels in ('scalar', 'forces'):
                    stem = f'SYNTHETIC_{world}_{int(arm_k)}_{"offline" if offline else "online"}_{channels}'
                    path = dest/(stem+'.csv')
                    log = M.Log(path)
                    b = DryRun(route, period=.01, k_face=series_stiffness(arm_k))
                    kw = dict(log=log, confirm=lambda msg:None, squeeze_gain=.25,
                              **M.socp_kwargs(cfg))
                    kw.update(full_wrench=channels=='forces', full_wrench_channels='forces')
                    with contextlib.redirect_stdout(io.StringIO()):
                        cls = PIProfileInsertion if offline else PIInsertion
                        ins = cls(route, b, **({'profile':profile} if offline else {}), **kw)
                        ins = M.slowed(ins, 3.)
                        try:
                            result = ins.run()
                        finally:
                            log.close()
                    assert result['route_followed'] and result['seat_found'] and not result['jammed'], result
                    assert ins.stats['socp_fail'] == 0
                    assert ins.stats['socp_ticks'] == (0 if offline else ins.stats['socp_ok'])
                    import csv
                    with path.open() as fh:
                        rows = list(csv.DictReader(fh))
                    diag = [row for row in rows if np.isfinite(float(row['pi_seq']))]
                    assert diag and all(np.isfinite(float(row[c])) for row in diag for c in M.PI_COLS)
                    assert all(abs(float(row['pi_output_mm'])) <=
                               (15.00001 if row['phase']=='squeeze' else 6.00001)
                               for row in diag if row['phase'] not in ('approach', 'to_open_pose'))
                    assert max(np.linalg.norm(o[:3]) for o in ins.off.values()) <= .00200001
                    M.write_meta(str(path.with_name(stem+'_meta.json')), cfg, cfgpath,
                                 SimpleNamespace(), result=dict(**result, source='synthetic DryRun'), ins=ins)
                    results.append(dict(world=world, arm_k=arm_k, offline=offline, channels=channels,
                                        result=result, pi_samples=ins.squeeze_pi.sequence,
                                        source='synthetic DryRun; no hardware stability claim'))
                    print(stem, 'OK', flush=True)
    (Path(__file__).parent/'pi_dryrun_matrix.json').write_text(json.dumps(results, indent=2)+'\n')
    print(f'{len(results)} PI toy executions passed', flush=True)


if __name__ == '__main__':
    validate()
