"""PI law and integration contracts; these tests do not certify hardware stability."""
import contextlib
import io
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from kinova_dual_arm_dmp_insertion.controller import Abort
from kinova_dual_arm_dmp_insertion.insert_pi import (
    PISettings, SqueezePI, PIInsertion, PIProfileInsertion, main)
from kinova_dual_arm_dmp_insertion.insert_open import Profile


class PILaw(unittest.TestCase):
    def test_bumpless_entry_and_actual_dt(self):
        s = PISettings(kp_m_per_N=.0001, ki_m_per_N_s=.0002, slew_m_per_s=.01)
        p = SqueezePI(s)
        self.assertEqual(p.update(8, 6, 10., .001, -.006, .006), .001)
        y = p.update(8, 6, 10.03, .001, -.006, .006)
        self.assertAlmostEqual(y, .001 + .0002*2*.03)
        self.assertAlmostEqual(p.diag['pi_dt_ms'], 30.)

    def test_P_is_not_integrated_repeatedly(self):
        p = SqueezePI(PISettings(ki_m_per_N_s=1e-12, slew_m_per_s=.01))
        y = p.update(4, 4, 0., 0., -.006, .006)
        for j in range(1, 101):
            y = p.update(5, 4, .02*j, y, -.006, .006)
        self.assertAlmostEqual(y, .0001, places=10)

    def test_rate_and_position_limits_and_antiwindup_recovery(self):
        p = SqueezePI(PISettings())
        y = p.update(12, 0, 0., 0., -.006, .006)
        for j in range(1, 501):
            prev = y
            y = p.update(12, 0, .02*j, y, -.006, .006)
            self.assertLessEqual(abs(y-prev), .002*.02 + 1e-12)
            self.assertLessEqual(abs(y), .006)
            self.assertAlmostEqual(p.integral + p.diag['pi_p_mm']/1000, y)
        self.assertAlmostEqual(y, .006)
        self.assertEqual(p.diag['pi_position_limited'], 1)
        # Immediate reversal, no hidden ten seconds of accumulated integral.
        reverse = p.update(0, 12, 10.02, y, -.006, .006)
        self.assertLess(reverse, y)

    def test_constant_error_integration_independent_of_update_rate(self):
        ends = []
        for dt in (.01, .02, .025, .04):
            p = SqueezePI(PISettings(ki_m_per_N_s=.0002, slew_m_per_s=.01))
            y = p.update(5, 4, 0., 0., -.006, .006)
            for j in range(1, round(1/dt)+1):
                y = p.update(5, 4, j*dt, y, -.006, .006)
            ends.append(y)
        np.testing.assert_allclose(ends, .0002, atol=1e-12)

    def test_bad_data_and_stalled_clock_abort_before_changing_state(self):
        for dt in (0., -.02, .101, np.nan):
            p = SqueezePI(PISettings())
            p.update(4, 4, 1., 0., -.006, .006)
            with self.subTest(dt=dt), self.assertRaises(Abort):
                p.update(4, 4, 1+dt, 0., -.006, .006)
            self.assertEqual(p.output, 0.)
        p = SqueezePI(PISettings())
        for args in ((4,np.nan,0,0,-.006,.006), (4,4,0,.007,-.006,.006)):
            with self.assertRaises(Abort):
                p.update(*args)

    def test_reset_tracks_new_origin_without_carrying_pickup_integral(self):
        p = SqueezePI(PISettings())
        p.update(12, 0, 0., 0., 0., .015)
        p.update(12, 0, .02, 0., 0., .015)
        p.reset()
        self.assertEqual(p.update(8, 12, 20., 0., -.006, .006), 0.)
        self.assertAlmostEqual(p.integral, .0004)

    def test_invalid_settings(self):
        for name, value in (('kp_m_per_N', -1), ('ki_m_per_N_s', 0),
                            ('ki_m_per_N_s', np.nan), ('slew_m_per_s', .011),
                            ('correction_limit_m', .013), ('max_dt_s', .2)):
            with self.subTest(name=name), self.assertRaises(ValueError):
                replace(PISettings(), **{name:value})

    def test_cli_rejects_bad_gains_before_loading_backend(self):
        for opts in (['--pi-kp','nan'], ['--pi-ki','-1'], ['--pi-limit-mm','13']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as cm:
                main(['insert','F75_Zt1_s9012',*opts])
            self.assertEqual(cm.exception.code, 2)

    def test_offline_PI_uses_full_profile_and_never_solves_online(self):
        ins = PIProfileInsertion.__new__(PIProfileInsertion)
        ins.profile = Profile([('route', t, 4., *np.arange(12)) for t in (0., 2.)])
        ins.phase, ins.phase_t0 = 'route', 10.
        ins.b = SimpleNamespace(now=lambda:10.5)
        ins.pi_settings = PISettings()
        ins.squeeze_pi = SqueezePI(ins.pi_settings)
        ins.ds, ins.hand_wrench, ins.stats = 0., True, dict(asked=[])
        ins.normal_force = lambda: 6.
        ins._chase_wrench = Mock()
        ins.socp = SimpleNamespace(solve=Mock(side_effect=AssertionError('online solve')))
        ins._control(10.5)
        self.assertEqual(ins.target, 4.)
        np.testing.assert_array_equal(ins.w_des['R'][1], [9.,10.,11.])
        ins.socp.solve.assert_not_called()
        ins._chase_wrench.assert_called_once()

    def test_pickup_timeout_retains_support_prompt_and_does_not_freeze(self):
        ins = PIInsertion.__new__(PIInsertion)
        ins.pi_settings = PISettings()
        ins.squeeze_pi = SqueezePI(ins.pi_settings)
        clock = [0.]
        ins.b = SimpleNamespace(now=lambda:clock[0])
        ins.normal_force = lambda:0.
        ins.axis = np.array([1.,0.,0.])
        ins.cL, ins.cR = np.zeros(3), np.ones(3)
        ins.qL = ins.qR = None
        ins.meas = dict(FL=np.zeros(3),FR=np.zeros(3),qL=[0,0,0,1],qR=[0,0,1,0])
        ins.tick = lambda *args: clock.__setitem__(0,clock[0]+.01)
        with self.assertRaisesRegex(Abort, 'keep supporting'):
            ins.squeeze(6., seconds=.1)
        np.testing.assert_array_equal(ins.cL, np.zeros(3))


if __name__ == '__main__':
    unittest.main()
