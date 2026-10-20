"""Analytical contact-load cases for the optional force-space cone translation."""
import unittest

import numpy as np

from kinova_dual_arm_dmp_insertion.socp import GraspSOCP


def geometry():
    # Identical triangles on opposite faces: no accidental torque from the setup.
    half_side = .03 * np.sqrt(3.) / 2
    face = np.array([[0., 0., .03], [half_side, 0., -.015], [-half_side, 0., -.015]])
    return dict(posL=face + [0., -.05, 0.], posR=face + [0., .05, 0.],
                baseL=np.array([0., -.12, 0.]), baseR=np.array([0., .12, 0.]),
                nrmL=np.tile([0., -1., 0.], (3, 1)), nrmR=np.tile([0., 1., 0.], (3, 1)),
                m_hat=.2, p_com=np.zeros(3), orientation=np.eye(3))


class ConeTranslation(unittest.TestCase):
    def solver(self, shift):
        return GraspSOCP(prior_mode="none", w_force=1., mu=1., r_s=.3,
                         cone_shift_N=shift)

    def test_symmetric_weight_has_additive_reserve(self):
        # Minimum per-hand compression = mg/(2*k) + 3*N0.
        for shift in (0., .25, .5, 1.):
            with self.subTest(shift=shift):
                solver = self.solver(shift)
                result = solver.solve(**geometry())
                self.assertTrue(result["ok"], result["reason"])
                expected = .2 * 9.81 / (2 * .7) + 3 * shift
                self.assertAlmostEqual(sum(result["fn_L"]), expected, places=4)
                self.assertAlmostEqual(sum(result["fn_R"]), expected, places=4)
                self.assertLessEqual(result["eta_max"], .7 + 1e-5)
                self.assertGreaterEqual(result["min_cone_slack_N"], -1e-4)
                self.assertTrue(next(iter(solver._programs.values()))["prob"].is_dpp())

    def test_zero_tangential_load_keeps_preload(self):
        result = self.solver(.5).solve(**geometry(), w_des=np.zeros(6))
        self.assertTrue(result["ok"], result["reason"])
        np.testing.assert_allclose(result["fn_L"] + result["fn_R"], .5, atol=2e-5)

    def test_triangle_transmits_pure_twist_with_reserve(self):
        solver = GraspSOCP(prior_mode="none", w_force=1., mu=1., r_s=.3,
                           cone_shift_N=.5, no_bending=True)
        # Six equal tangential forces around radius .03 generate tau_y=.02 Nm.
        # Each hand needs 3*N0 + |tau|/(2*r*k) normal force.
        result = solver.solve(**geometry(), w_des=np.array([0., 0., 0., 0., .02, 0.]))
        self.assertTrue(result["ok"], result["reason"])
        expected = 3 * .5 + .02 / (2 * .03 * .7)
        self.assertAlmostEqual(sum(result["fn_L"]), expected, places=4)
        self.assertAlmostEqual(sum(result["fn_R"]), expected, places=4)
        self.assertAlmostEqual(result["wrench_L"][4] + result["wrench_R"][4], .02, places=6)

    def test_inactive_points_have_no_preload(self):
        solver = self.solver(.5)
        result = solver.solve(**geometry(), active_L=[True, False, False],
                              active_R=[True, False, False])
        self.assertTrue(result["ok"], result["reason"])
        self.assertAlmostEqual(sum(result["fn_L"]), .2 * 9.81 / (2 * .7) + .5, places=4)
        np.testing.assert_allclose(result["f_L"][1:], 0., atol=1e-8)
        self.assertEqual(len(solver._programs), 1)
        solver.solve(**geometry(), active_L=[True, False, False], active_R=[True, False, False])
        self.assertEqual(len(solver._programs), 1)

    def test_checker_rejects_unshifted_solution(self):
        args = geometry()
        old = self.solver(0.).solve(**args)
        shifted = self.solver(.5)
        shifted.solve(**args)
        program = next(iter(shifted._programs.values()))
        from kinova_dual_arm_dmp_insertion.socp import wrench_map
        pts = np.vstack((args["posL"], args["posR"]))
        H = np.zeros((12, 18))
        H[:6, :9] = wrench_map(args["posL"], args["baseL"])
        H[6:, 9:] = wrench_map(args["posR"], args["baseR"])
        check = shifted._check(np.array(old["f_L"] + old["f_R"]).ravel(), pts,
                               np.vstack((args["nrmL"], args["nrmR"])), [True] * 6,
                               args["p_com"], np.array([0., 0., .2 * 9.81, 0., 0., 0.]),
                               H, program["prob"], "test")
        self.assertFalse(check["ok"])
        self.assertAlmostEqual(check["cone_violation"], .7 * .5, places=4)

    def test_invalid_parameters_fail_explicitly(self):
        for shift in (-.1, float("nan"), float("inf")):
            with self.subTest(shift=shift), self.assertRaises(ValueError):
                self.solver(shift)


if __name__ == "__main__":
    unittest.main()
