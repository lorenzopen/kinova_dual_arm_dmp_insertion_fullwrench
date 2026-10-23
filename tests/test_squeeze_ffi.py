"""Nominal force + integral trim contracts; no physical stability claim."""
import contextlib
import io
import unittest
from dataclasses import replace

import numpy as np

from kinova_dual_arm_dmp_insertion.controller import Abort
from kinova_dual_arm_dmp_insertion.insert_ffi import FFISettings, NominalIntegral, main


class FFILaw(unittest.TestCase):
    def test_pickup_preload_is_not_added_twice(self):
        c = NominalIntegral(FFISettings(), 964., 6.)
        out = 0.
        for j in range(100):
            out = c.update(6., 12., .02*j, out)
            self.assertEqual(out, 0.)
            self.assertEqual(c.diag['ffi_ff_want_mm'], 0.)

    def test_nominal_change_and_summed_error_units(self):
        c = NominalIntegral(FFISettings(), 1000., 6.)
        self.assertEqual(c.update(4., 8., 0., 0.), 0.)
        out = 0.
        for j in range(1, 101):
            out = c.update(4., 8., .02*j, out)
        self.assertAlmostEqual(out, -.002)
        self.assertAlmostEqual(c.trim, 0.)
        self.assertAlmostEqual(c.diag['ffi_error_N'], 0.)

    def test_integral_actual_dt_and_sign(self):
        c = NominalIntegral(FFISettings(), 1000., 6.)
        c.update(6., 10., 1., 0.)
        y = c.update(6., 10., 1.03, 0.)
        self.assertAlmostEqual(y, .0005*2*.03)
        self.assertAlmostEqual(c.ff, 0.)

    def test_combined_limits_under_jitter_and_reversals(self):
        c = NominalIntegral(FFISettings(), 680., 6.)
        rng = np.random.default_rng(42)
        now, out = 0., c.update(6., 12., 0., 0.)
        for j in range(3000):
            dt = float(rng.uniform(.015, .04))
            now += dt
            old_ff, old_trim, old_out = c.ff, c.trim, out
            target = (30., 1., 6., 0.)[(j//200) % 4]
            measured = (0., 60.)[(j//70) % 2]
            out = c.update(target, measured, now, out)
            self.assertLessEqual(abs(out-old_out), .002*dt+1e-12)
            self.assertLessEqual(abs(c.trim-old_trim), .001*dt+1e-12)
            self.assertLessEqual(abs(c.ff-old_ff), .002*dt+1e-12)
            self.assertLessEqual(abs(c.trim), .003+1e-12)
            self.assertLessEqual(abs(c.ff), .009+1e-12)
            self.assertLessEqual(abs(out), .012+1e-12)
            self.assertAlmostEqual(out, c.ff+c.trim)

    def test_trim_saturation_recovers_without_windup(self):
        c = NominalIntegral(FFISettings(), 964., 6.)
        out = c.update(6., 0., 0., 0.)
        for j in range(1, 1001):
            out = c.update(6., 0., .02*j, out)
        self.assertAlmostEqual(c.trim, .003)
        self.assertLess(c.update(6., 24., 20.02, out), out)

    def test_ideal_static_plant_and_contact_force_disturbance(self):
        # Artificial algebraic two-hand plant. This verifies units/compensation,
        # not delayed Kinova dynamics, friction, compliance resonance or slip.
        for actual_k in (680., 964.):
            c = NominalIntegral(FFISettings(), 964., 6.)
            out = 0.
            for j in range(801):
                disturbance = 2. if j >= 400 else 0.
                measured = 12.+2*actual_k*out-disturbance
                out = c.update(4., measured, .02*j, out)
                if j == 399:
                    self.assertLess(abs(measured-8.), .05)
            self.assertLess(abs(measured-8.), .05)
            self.assertLess(abs(c.trim), .003)

    def test_integrator_invariance_to_sample_interval(self):
        ends = []
        for dt in (.01, .02, .025, .04):
            c = NominalIntegral(FFISettings(), 964., 6.)
            y = c.update(6., 11., 0., 0.)
            for j in range(1, round(1/dt)+1):
                y = c.update(6., 11., j*dt, y)
            ends.append(y)
        np.testing.assert_allclose(ends, .0005, atol=1e-12)

    def test_invalid_input_and_late_clock_do_not_change_state(self):
        for now, target, measure, current in ((.101,6.,12.,0.), (0.,6.,12.,0.),
                                              (.02,-1.,12.,0.), (.02,6.,np.nan,0.),
                                              (.02,6.,12.,.001)):
            c = NominalIntegral(FFISettings(),964.,6.)
            c.update(6.,12.,0.,0.)
            before = (c.ff,c.trim,c.output,c.time,c.sequence)
            with self.assertRaises(Abort):
                c.update(target,measure,now,current)
            self.assertEqual(before,(c.ff,c.trim,c.output,c.time,c.sequence))

    def test_invalid_settings_and_cli(self):
        for key, value in (('ki_m_per_N_s',0.), ('trim_limit_m',.013),
                           ('total_speed_m_per_s',.0001), ('max_dt_s',.2)):
            with self.assertRaises(ValueError):
                replace(FFISettings(),**{key:value})
        for args in (['--i-ki','nan'], ['--i-limit-mm','12'], ['--k-eff','-1']):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as cm:
                main(['insert','F75_Zt1_s9012',*args])
            self.assertEqual(cm.exception.code,2)


if __name__ == '__main__':
    unittest.main()
