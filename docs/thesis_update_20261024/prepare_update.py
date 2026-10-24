"""Stage thesis edits and compact immutable evidence; never write source archives."""
from pathlib import Path
import csv
import hashlib
import json
import os
import shutil
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PROJECT = HERE / 'project'
ORIGINAL = Path('C:/Users/Lorenzo/Desktop/TESI_antiG/thesis_english_2026')
SIM = ROOT / 'docs/analysis_20261023/controller_night/pybullet_nominal_20261024'
sys.path.insert(0, str(ROOT / 'tmp/dryrun_deps'))
sys.dont_write_bytecode = True
import numpy as np
from scipy.spatial.transform import Rotation

DATA = PROJECT / 'figures/data/kinova_comparison_20261024'
DATA.mkdir(parents=True, exist_ok=True)
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

CHANGED = ['chapters/07_socp_control.tex', 'chapters/10_tetris_dmp.tex',
           'chapters/11_kinova_hardware.tex', 'chapters/B_reproducibility.tex',
           'figures/README.md']
base_manifest = HERE / 'original_manifest.json'
if not base_manifest.exists():
    saved = []
    for rel in CHANGED + ['build/main.pdf']:
        src = ORIGINAL / rel
        dst = HERE / 'originals' / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        saved.append(dict(path=rel, sha256=sha(src)))
    base_manifest.write_text(json.dumps(dict(root=str(ORIGINAL), files=saved), indent=2)+'\n')

def source(rel):
    return (HERE / 'originals' / rel).read_text(encoding='utf-8')

def snippet(name):
    return (HERE / name).read_text(encoding='utf-8') + '\n'

def put(rel, text):
    (PROJECT / rel).write_text(text, encoding='utf-8', newline='\n')

s = source(CHANGED[0])
anchor = '\\section{Decision variables: two wrist wrenches}'
assert s.count(anchor) == 1
s = s.replace(anchor, snippet('socp_overview.tex') + anchor)
put(CHANGED[0], s)

s = source(CHANGED[1])
anchor = '\\section{Haptic distance}'
assert s.count(anchor) == 1
s = s.replace(anchor, snippet('insertion_contacts.tex') + anchor)
put(CHANGED[1], s)

s = source(CHANGED[2])
s = s.replace('The convex program, the scalar squeeze loop, the jam sentry and the learnt routes are the ones used in simulation; what changes is the apparatus that realizes them.',
              'The convex program, the scalar squeeze loop, the jam sentry and the learnt routes are transferred from simulation; the arm dynamics and passive tool determine how their commands are realized.')
s = s.replace('The runs reported here are a first hardware series, recorded on 21 and 22 October 2026. The piece is presented by hand at the grasp pose and the board is built by hand from the route file. Ten insertions over four worlds are not a campaign, and further runs are planned. The chapter therefore states what was executed and what was measured, and keeps the outcomes as the operator recorded them.',
              'The first hardware series was recorded on 21 and 22 October 2026, with the piece presented by hand and the board assembled from the route file. Its ten executions over four worlds are reported with the operator\'s recorded outcomes. A later series of 23 October uses offline normal-force feedforward plus integral feedback. It is presented separately, followed by comparisons with an articulated Gen3 simulation and a test of the laboratory impedance source.')
anchor = '\\subsection{The wrist sensors}'
assert s.count(anchor) == 1
s = s.replace(anchor, snippet('impedance.tex') + anchor)
s = s.replace('This is the lag of a $1000$ N/m impedance under the contact forces and the residual of the gravity model.',
              'This error includes the compliant response under contact load and possible gravity-model residuals; the logs do not separate their contributions.')
s = s.replace('These are consistent with the soft spring in series with the arm.',
              'A compliant arm--tool chain can support such motion, but these traces alone do not distinguish mechanical resonance, feedback dynamics, contact changes and measurement effects. The small median bias is not an instantaneous $0.1$ N tracking bound.')
s = s.replace('that tick fell back to the trained pickup squeeze.',
              'that historical implementation fell back to the trained pickup squeeze for that tick. This describes the recorded series, not the later allocator-failure policy.')
anchor = '\\section{What these runs show and what they do not}'
assert s.count(anchor) == 1
s = s[:s.index(anchor)] + snippet('kinova_comparison.tex') + r'''\section{What these runs show and what they do not}

The initial series shows that routes learnt in simulation can be executed by two impedance-controlled arms following one object frame, while a squeeze loop closes on wrist-sensor measurements. Its small median force bias coexists with repeated fluctuations and larger transient errors. The later FFI series gives recorded scalar-force executions, but it is not a paired ablation of the earlier full-wrench condition.

The articulated simulations make the distinction between allocation and realization explicit. An admissible SOCP reference still has to pass through pose tracking, tool deflection and changing shelf contact. The approximate model can complete selected routes with smoother forces, yet its step-size sensitivity and the original-source free-space failure prevent using it to identify the cause of the real oscillations. Raising damping to $80$ has only a modest effect in the completed approximate simulations.

Real placement accuracy and neighbour displacement remain unmeasured. The passive spring gives the twist channel little authority in the initial apparatus. A controlled same-route comparison, changing only inner damping or only the outer force law, is needed before attributing improvements to one component. A synchronized free-space validation of the robot model must precede a claim that simulation predicts the hardware transients.

\todo{Measure placement after release and neighbour displacement; repeat a fixed route with identical FFI settings at two inner damping values; validate the free-space arm response with synchronized joint, torque and timing logs; evaluate twist only with a tool whose torsional transmission is measured.}
'''
put(CHANGED[2], s)

s = source(CHANGED[3])
anchor = '\\section{Unfinished work}'
assert s.count(anchor) == 1
s = s.replace(anchor, snippet('reproducibility.tex') + anchor)
put(CHANGED[3], s)

evidence = []
def note(path):
    evidence.append(dict(path=str(path.relative_to(ROOT)), sha256=sha(path), bytes=path.stat().st_size))

cases = dict(Z='Gen3_Z_central2700_implicit_dt025', L='Gen3_L_central2700_implicit_dt05')
times = dict(Z='20261023_174143', L='20261023_182743')
for piece, run in cases.items():
    real_paths = list((ROOT / 'docs/runs').rglob('*'+times[piece]+'.csv'))
    assert len(real_paths) == 1
    path = real_paths[0]
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    col = lambda k: np.array([float(r[k]) for r in rows])
    real = dict(t=col('t_wall'), phase=np.array([r['phase'] for r in rows]), target=col('target_N'))
    for side in 'LR':
        force = np.column_stack([col('F'+a+side) for a in 'xyz'])
        quat = np.column_stack([col('mq'+a+side) for a in 'xyzw'])
        axis = Rotation.from_quat(quat).apply(np.tile([1.,0,0], (len(rows),1)))
        real[side+'_normal'] = np.sum(force*axis, axis=1)
        real[side+'_moment'] = np.column_stack([col('T'+a+side) for a in 'xyz'])
    real['mean_normal'] = .5*(real['L_normal']+real['R_normal'])
    np.savez_compressed(DATA / (piece+'_real.npz'), **real)
    note(path)
    note(path.with_name(path.stem+'_meta.json'))
    folder = SIM / 'runs' / run
    raw = dict(np.load(folder / 'trace.npz', allow_pickle=False))
    sim = {k: raw[k] for k in ['t','phase','target']}
    for side in 'LR':
        axis = Rotation.from_quat(raw[side+'_quat']).apply(np.tile([1.,0,0], (len(raw['t']),1)))
        sim[side+'_normal'] = np.sum(raw[side+'_measured'][:,:3]*axis, axis=1)
        sim[side+'_moment'] = raw[side+'_measured'][:,3:]
    sim['mean_normal'] = .5*(sim['L_normal']+sim['R_normal'])
    np.savez_compressed(DATA / (piece+'_sim.npz'), **sim)
    for file in ['trace.npz','result.json','profile.csv','runner_source.py','robot_used.urdf','socp_allocations.json']:
        note(folder/file)
    shutil.copy2(folder/'result.json', DATA/(piece+'_result.json'))

folder = SIM / 'runs/Gen3Lab_Z_central2700_warm_dt025'
raw = dict(np.load(folder/'trace.npz', allow_pickle=False))
np.savez_compressed(DATA/'original_source_stop.npz', raw_wrist_1kHz=raw['raw_wrist_1kHz'])
shutil.copy2(folder/'result.json', DATA/'original_source_result.json')
for file in ['trace.npz','result.json','runner_source.py']:
    note(folder/file)
for rel in ['gen3/lab_source/kinova_controller_original.py','gen3/controller_math.py',
            'gen3/run_lab_impedance.py','gen3/user_reference_check.json',
            'gen3/lab_model_checks.json','damping80_20261024/results.json',
            'gen3/rigid_pin_hold_comparison.png']:
    path = SIM / rel
    if path.exists(): note(path)
shutil.copy2(SIM/'damping80_20261024/results.json', DATA/'damping_comparison.json')
manifest = dict(repository=str(ROOT), evidence_class='Measured real traces and autonomous simulation; no real-force replay',
    cases=cases, real_times=times, sources=evidence,
    plotting='Own departure clocks; unwarped signals; same paired axes; no display smoothing',
    metrics='Route stage; own 100 Hz interpolation; third-order Butterworth forward-backward bandpass',
    local_arrays=[dict(path=p.name,sha256=sha(p)) for p in sorted(DATA.glob('*.npz'))])
(DATA/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')

s = source(CHANGED[4]) + '''
## Articulated Gen3 comparison (24 October 2026)

`scripts/fig_kinova_comparison.py` regenerates `ch11_kinova/real_sim_normal`,
`real_sim_zoom`, `real_sim_moment` and `original_source_stop` as PDF/PNG.
It uses the common `style.py` hand colours and typography. Compact plotted
signals and original source SHA-256 identities are in
`data/kinova_comparison_20261024/manifest.json`; `metrics.json` records the
descriptive route/seat metrics. The full insertion curves use approximate
inner impedance; the original-source trial is shown separately to its early
stop. No new physics or tuning is performed by the plotting script. The three
block diagrams are editable TikZ in Chapters 7, 10 and 11.
'''
put(CHANGED[4], s)
print(json.dumps(dict(staged=str(PROJECT),original_files_preserved=len(CHANGED)+1,evidence_sources=len(evidence)),indent=2))
