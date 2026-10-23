import tempfile
import unittest
from pathlib import Path

import numpy as np

from kinova_dual_arm_dmp_insertion.audit_run import load, measured_common_wrench, spectrum


class Analysis(unittest.TestCase):
    def test_spectrum_uses_actual_time_with_irregular_sampling(self):
        rng = np.random.default_rng(42)
        t = np.cumsum(rng.uniform(.006,.016,5000))
        force = 8. + .6*np.cos(2*np.pi*1.2*t) + .01*t
        f, a = spectrum(t,force)
        peak = np.argmax(a)
        self.assertLess(abs(f[peak]-1.2), .025)
        self.assertLess(abs(a[peak]-.6), .1)
        f, a = spectrum(np.array([0.,.01,.01,.02,.03,.04,.05,.06]), np.zeros(8))
        self.assertEqual(len(f), 0)

    def test_plot_moments_are_transported_to_the_same_origin_as_the_plan(self):
        d = {k:np.zeros(1) for k in ('mxL','myL','mzL','mxR','myR','mzR',
                                    'FxL','FyL','FzL','TxL','TyL','TzL')}
        d['mxL'][0], d['mxR'][0] = -1., 1.
        d['FyL'][0], d['TzL'][0] = 2., 3.
        np.testing.assert_allclose(measured_common_wrench(d,'L')[0], [0,2,0,0,0,1])

    def test_empty_optional_telemetry_is_missing_not_zero(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'run.csv'
            path.write_text('t,phase,socp_ok\n1,route,\n2,route,1\n')
            d = load(path)
            self.assertTrue(np.isnan(d['socp_ok'][0]))
            self.assertEqual(d['socp_ok'][1], 1.)


if __name__ == '__main__':
    unittest.main()
