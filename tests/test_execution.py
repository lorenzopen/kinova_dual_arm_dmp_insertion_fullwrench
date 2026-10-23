"""Force-plan execution, frame/sign conventions and data integrity (no ROS/hardware)."""
import csv
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from kinova_dual_arm_dmp_insertion.controller import Insertion, Abort, GAIN
from kinova_dual_arm_dmp_insertion.insert_open import Profile, ProfileInsertion, NominalPlanner, effective_stiffness, series_stiffness
from kinova_dual_arm_dmp_insertion.main import Log, TELEMETRY_COLS, write_meta
from kinova_dual_arm_dmp_insertion.geometry import contact_face_centres


def sample_row(t, target=4., offset=0., phase="route"):
    return (phase, t, target, *(np.arange(12, dtype=float) + offset))


class Execution(unittest.TestCase):
    def test_metadata_records_effective_gains_and_fixed_plan_provenance(self):
        profile = SimpleNamespace(path='plan.csv',sha256='verified-plan-hash',k_eff=964.2857,
                                  socp_ok=100,socp_ticks=100)
        ins = SimpleNamespace(execution_mode='offline_socp_ati_feedback',gain=.25*GAIN,
                              squeeze_feedback=True,hand_wrench=True,full_wrench=True,channels='contact',
                              force_offset_limit=.002,profile=profile,socp_reason='',
                              stats=dict(socp_fail=0),_socp_job=None)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'meta.json'
            write_meta(path,{},'config/cell.yaml',SimpleNamespace(),ins=ins)
            meta = json.loads(path.read_text())
        self.assertEqual(meta['execution']['squeeze_gain_scale'], .25)
        self.assertEqual(meta['execution']['wrench_channels'], 'contact')
        self.assertEqual(meta['execution']['offline_profile_sha256'], 'verified-plan-hash')
        self.assertEqual(meta['execution']['planned_socp_ticks'], 100)
        self.assertTrue(meta['execution']['squeeze_feedback'])
        self.assertEqual(meta['allocation_failures'], 0)

    def test_path_comparison_removes_tool_depth_difference_not_real_path_error(self):
        R = Rot.from_euler('xyz', [[.1,.2,.3],[.5,.6,.7]])
        target = np.array([[1.,2,3],[4.,5,6]])
        sim = target - R.apply([.02833333,0,0])
        real = target - R.apply([.1,0,0])
        np.testing.assert_allclose(contact_face_centres(sim,R.as_quat(),.02833333),target)
        np.testing.assert_allclose(contact_face_centres(real,R.as_quat(),.1),target)
        real += [.002,0,0]
        delta = contact_face_centres(real,R.as_quat(),.1)-target
        np.testing.assert_allclose(np.linalg.norm(delta,axis=1),.002)

    def test_offline_interpolates_every_force_and_moment_component(self):
        p = Profile([sample_row(0), sample_row(2, 6., 4.)])
        target, w = p.sample("route", .5)
        self.assertAlmostEqual(target, 4.5)
        np.testing.assert_allclose(np.concatenate([*w['L'], *w['R']]), np.arange(12) + 1.)
        np.testing.assert_allclose(np.concatenate(p.sample("route", 10)[1]['R']), np.arange(6,12) + 4.)

    def test_offline_geometry_is_independent_of_virtual_squeeze_deflection(self):
        from kinova_dual_arm_dmp_insertion.geometry import pad_contacts, quat_from_x
        ins = NominalPlanner.__new__(NominalPlanner)
        ins.R_cmd = Rot.from_euler('xyz',[.1,.2,.3])
        ins.frame = dict(ax=np.array([0.,1,0]))
        axis = ins.R_cmd.apply(ins.frame['ax'])
        qL,qR=quat_from_x(axis),quat_from_x(-axis)
        nominalL,nominalR=-.15*axis,.15*axis
        ins.r=SimpleNamespace(r_plan=np.array([.01,0,0]),mass=.05,swapped=False)
        ins._q_cmd,ins._q_cmd_R=qL,qR
        ins.stats,ins.w_ext,ins.f_min=dict(socp_ticks=0),None,1.
        ins.b=SimpleNamespace(now=lambda:1.)
        ins.s_pick=.008
        inputs=[]
        for ds in (-.003,.002):
            ins.ds=ds
            ins.meas=dict(pL=nominalL+(.008+ds)*axis,pR=nominalR-(.008+ds)*axis,
                          qL=qL.as_quat(),qR=qR.as_quat())
            inputs.append(ins._socp_inputs())
        for inp in inputs:
            np.testing.assert_allclose(inp['kw']['baseL'],nominalL,atol=1e-12)
            np.testing.assert_allclose(inp['kw']['posL'],pad_contacts(nominalL,qL)[0],atol=1e-12)
        np.testing.assert_allclose(inputs[0]['kw']['posR'],inputs[1]['kw']['posR'],atol=1e-12)

    def test_offline_refuses_unplanned_phase_or_missing_wrench(self):
        p = Profile([sample_row(0)])
        p.require_phases(['route'])
        with self.assertRaises(Abort):
            p.require_phases(['route','slow_seat'])
        with self.assertRaises(Abort):
            p.sample('slow_seat', 0)
        for trace in ([], [('route', 0, 4)], [sample_row(0), sample_row(0)],
                      [sample_row(0, offset=np.nan)]):
            with self.subTest(trace=trace), self.assertRaises(ValueError):
                Profile(trace)

    def test_offline_feedback_uses_plan_without_any_solver_call(self):
        ins = ProfileInsertion.__new__(ProfileInsertion)
        ins.profile = Profile([sample_row(0), sample_row(2, 6., 4.)])
        ins.phase_t0, ins.phase = 20., 'route'
        ins.hand_wrench = True
        ins.stats = dict(asked=[])
        ins._squeeze_update = Mock()
        ins._chase_wrench = Mock()
        ins.socp = SimpleNamespace(solve=Mock(side_effect=AssertionError('online solve in offline mode')))
        ins._control(20.5)
        ins._control(21.)
        self.assertEqual(ins.stats['asked'], [4.5, 5.])
        ins.socp.solve.assert_not_called()
        self.assertEqual(ins._chase_wrench.call_count, 2)
        np.testing.assert_allclose(ins.w_des['R'][1], [11., 12., 13.])

    def test_spring_is_in_series_and_actual_controller_stiffness_wins(self):
        cfg = {'stiffness': {'trans': 1000.}}
        self.assertAlmostEqual(series_stiffness(1000), 729.7297297297)
        self.assertAlmostEqual(series_stiffness(1500), 964.2857142857)
        actual = {'L': {'cart_k': [1500.]*3 + [20.]*3}, 'R': {'cart_k': [1500.]*3+[20.]*3}}
        b = SimpleNamespace(controller_parameters=actual, arms={})
        self.assertAlmostEqual(effective_stiffness(cfg, b), series_stiffness(1500))
        self.assertEqual(effective_stiffness(cfg, b, 900.), 900.)
        actual['R']['cart_k'][0] = 1000.
        with self.assertRaises(ValueError):
            effective_stiffness(cfg, b)
        with self.assertRaises(ValueError):
            effective_stiffness(cfg, SimpleNamespace(arms={}))

    def test_bad_stiffness_is_not_used_to_generate_pose_offsets(self):
        for value in (0., -1., np.nan, np.inf):
            with self.subTest(value=value), self.assertRaises(ValueError):
                series_stiffness(value)

    def test_force_feedback_is_internal_tangential_and_uses_selected_gain(self):
        # Arbitrary orientation: a world-axis-only implementation would fail this test.
        R = Rot.from_euler('xyz', [.2,.4,.7])
        axis = R.apply([1.,0,0])
        tangent = R.apply([0.,1.,0])
        ins = Insertion.__new__(Insertion)
        ins.frame = {'ax': np.array([1.,0,0])}
        ins.R_cmd = R
        ins.full_wrench, ins.channels = True, 'forces'
        ins.gain, ins.moment_sign = .25 * GAIN, -1.
        ins.force_offset_limit = .002
        ins.off = {s: np.zeros(6) for s in 'LR'}
        ins.meas = dict(pL=-.1*axis, pR=.1*axis,
                        FL=5*axis + .3*tangent, FR=-5*axis - .3*tangent,
                        TL=np.ones(3), TR=-np.ones(3))
        ins.w_des = {'L': (5*axis, np.zeros(3)), 'R': (-5*axis, np.zeros(3))}
        ins._chase_wrench()
        np.testing.assert_allclose(ins.off['L'][:3], -.3*ins.gain*tangent, atol=1e-12)
        np.testing.assert_allclose(ins.off['L'] + ins.off['R'], 0., atol=1e-12)
        np.testing.assert_allclose(ins.off['L'][3:], 0.)
        self.assertAlmostEqual(ins.off['L'][:3] @ axis, 0.)
        # A common load belongs to object motion, not to an internal correction.
        ins.off = {s: np.zeros(6) for s in 'LR'}
        ins.meas.update(FL=5*axis+.3*tangent, FR=-5*axis+.3*tangent)
        ins._chase_wrench()
        np.testing.assert_allclose(ins.off['L'], 0., atol=1e-12)
        ins.meas.update(FL=5*axis+100*tangent, FR=-5*axis-100*tangent)
        for _ in range(30):
            ins._chase_wrench()
        self.assertAlmostEqual(np.linalg.norm(ins.off['L'][:3]), .002)
        self.assertAlmostEqual(ins.off['L'][:3] @ axis, 0., places=12)
        np.testing.assert_allclose(ins.off['L']+ins.off['R'],0.,atol=1e-12)
        ins.channels = 'contact'
        ins.off = {s:np.zeros(6) for s in 'LR'}
        ins.meas.update(FL=5*axis+.3*tangent, FR=-5*axis-.3*tangent,
                        TL=.01*axis+.2*tangent, TR=-.01*axis-.2*tangent)
        ins._chase_wrench()
        np.testing.assert_allclose(ins.off['L'][:3],-.3*ins.gain*tangent,atol=1e-12)
        np.testing.assert_allclose(ins.off['L'][3:],-.01/(4*40.)*axis,atol=1e-12)
        np.testing.assert_allclose(ins.off['L']+ins.off['R'],0.,atol=1e-12)

    def test_scalar_execution_still_retains_full_wrenches_with_swapped_hands(self):
        ins = Insertion.__new__(Insertion)
        ins.hand_wrench = False
        ins.stats = dict(socp_ok=0)
        p_a = np.tile([1.,0,0], (3,1))
        p_b = np.tile([-1.,0,0], (3,1))
        inp = dict(first='R', second='L', pA=p_a, pB=p_b, c=np.zeros(3), time=2.)
        res = dict(ok=True, fn_L=[1,1,1], fn_R=[2,2,2],
                   f_L=np.tile([0.,1.,0], (3,1)), f_R=np.tile([0.,0.,2.], (3,1)))
        ins._socp_apply(inp, res, 1.)
        self.assertEqual(ins.target, 6.)
        np.testing.assert_allclose(ins.w_des['R'][0], [0,3,0])
        np.testing.assert_allclose(ins.w_des['R'][1], [0,0,3])
        np.testing.assert_allclose(ins.w_des['L'][0], [0,0,6])
        np.testing.assert_allclose(ins.w_des['L'][1], [0,6,0])

    def test_failed_allocation_stops_without_increasing_squeeze(self):
        ins = Insertion.__new__(Insertion)
        ins.stats = dict(socp_fail=0)
        ins.target, ins.w_des = 4., {'previous': 'feasible wrench'}
        inp = dict(time=2., first='L',second='R',pA=[],pB=[])
        with self.assertRaisesRegex(Abort, 'incompatible'):
            ins._socp_apply(inp,dict(ok=False,reason='incompatible'),1.)
        self.assertEqual(ins.target, 4.)
        self.assertEqual(ins.w_des, {'previous':'feasible wrench'})
        self.assertEqual(ins.stats['socp_fail'], 1)

    def test_warmup_finishes_before_starting_the_motion_clock(self):
        ins = Insertion.__new__(Insertion)
        now = [0.]
        ins.b = SimpleNamespace(now=lambda:now[0],command=Mock())
        ins._allocate = Mock(side_effect=lambda:now.__setitem__(0,2.))
        ins._prepare_control()
        self.assertEqual(ins.t_socp, 2.)
        self.assertEqual(ins.t_ctrl, 2.)
        ins.b.command.assert_not_called()

    def test_csv_keeps_wrench_joints_source_age_and_control_status_in_matching_columns(self):
        b = SimpleNamespace(now=lambda: 5., raw={}, telemetry={c:i for i,c in enumerate(TELEMETRY_COLS)})
        m = {k:np.zeros(3) for k in ('pL','pR','FL','FR','TL','TR')}
        m.update(qL=[0.,0,0,1], qR=[0.,0,0,1])
        ins = SimpleNamespace(b=b, meas=m, phase='route', w_des={'L':(np.ones(3),2*np.ones(3))},
                              ds=.001, target=4., jammed=False, off={s:np.zeros(6) for s in 'LR'},
                              socp_ms=2., socp_input_time=4.9, socp_status=False, control_dt=.03,
                              normal_force=lambda: 8., external_force=lambda: np.zeros(3))
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'log.csv'
            log = Log(path)
            log.row(ins, np.zeros(3), Rot.identity(), np.zeros(3), Rot.identity())
            log.close()
            with path.open() as f:
                raw = list(csv.reader(f))
            self.assertEqual(len(raw[0]), len(raw[1]))
            row = dict(zip(*raw))
            self.assertEqual(float(row['wdesL_fz']), 1.)
            self.assertEqual(float(row['wdesL_tz']), 2.)
            self.assertEqual(float(row['ctrl_dt_ms']), 30.)
            self.assertAlmostEqual(float(row['socp_input_age_ms']), 100.)
            self.assertEqual(row['socp_ok'], '0')
            for c,i in b.telemetry.items():
                self.assertEqual(float(row[c]), i)


if __name__ == '__main__':
    unittest.main()
