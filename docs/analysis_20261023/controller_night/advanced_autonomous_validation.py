"""Compare autonomous baseline updates to the actual preserved production class.

Only two pure numerical classes are extracted by AST: no ROS imports/execution.
"""
import advanced_controllers as a
import ast
import json
from dataclasses import dataclass,asdict
import numpy as np


def main():
    tree=ast.parse((a.c.ROOT/'kinova_dual_arm_dmp_insertion/insert_ffi.py').read_text())
    ns=dict(dataclass=dataclass,asdict=asdict,np=np,Abort=RuntimeError)
    classes=[n for n in tree.body if isinstance(n,ast.ClassDef) and n.name in ['FFISettings','NominalIntegral']]
    exec(compile(ast.Module(body=classes,type_ignores=[]),'<pure FFI classes>','exec'),ns)
    with np.load(a.OUT/'advanced_autonomous_traces.npz') as archive:
        data={key:archive[key] for key in archive.files}
    dt=.0025
    baseline=list(data['candidate_ids']).index('I_P0_I1.6_D0')
    variants={v['id']:v for v in json.loads((a.OUT/'advanced_autonomous_design.json').read_text())['variants']}
    rng=np.random.default_rng(23442026);jitter_ticks=[0]
    while jitter_ticks[-1]*dt<32:jitter_ticks.append(jitter_ticks[-1]+int(rng.integers(6,15)))
    jitter=np.array(jitter_ticks)*dt;fixed=np.arange(0,32.0001,.025)
    noise=np.random.default_rng(23442027).normal(size=2500)
    largest=0.;updates=0
    for m,variant in enumerate(data['variant_ids']):
        params=variants[str(variant)];schedule=jitter if params['jitter'] else fixed
        indices=np.rint(schedule/dt).astype(int)
        ctrl=ns['NominalIntegral'](ns['FFISettings'](),680.,4.)
        for k,j in enumerate(indices):
            if j>=len(data['t']):break
            measured=2*data['force'][j,baseline,m]+2*params['noise_N_hand']*noise[k]
            value=ctrl.update(float(data['target'][j]),float(measured),float(data['t'][j]),ctrl.output)
            largest=max(largest,abs(value*1000-data['commands'][j,baseline,m]));updates+=1
    result=dict(scenarios=len(data['model_ids']),actual_FFI_updates_checked=updates,
        maximum_baseline_command_difference_mm=largest,all_passed=largest<1e-9,
        note='Production class numerical update only, extracted by AST without controller imports or robot connection.')
    a.c.dp.dump('advanced_autonomous_software_validation.json',result)
    assert largest<1e-9,result
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
