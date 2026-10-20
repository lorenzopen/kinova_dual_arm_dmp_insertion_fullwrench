"""Paper signed friction distance and the tri-pad task-adaptive extension."""
import unittest

import numpy as np

from kinova_dual_arm_dmp_insertion.socp import GraspSOCP, task_margin


def geometry():
    # Identical triangles on opposite faces: no accidental torque from the setup.
    half_side = .03 * np.sqrt(3.) / 2
    face = np.array([[0., 0., .03], [half_side, 0., -.015], [-half_side, 0., -.015]])
    return dict(posL=face + [0., -.05, 0.], posR=face + [0., .05, 0.],
                baseL=np.array([0., -.12, 0.]), baseR=np.array([0., .12, 0.]),
                nrmL=np.tile([0., -1., 0.], (3, 1)), nrmR=np.tile([0., 1., 0.], (3, 1)),
                m_hat=.2, p_com=np.zeros(3), orientation=np.eye(3))


class FrictionMargin(unittest.TestCase):
    def solver(self, rho, mu=1.):
        return GraspSOCP(prior_mode="none", w_force=1., mu=mu,
                         margin_mode="fixed", margin_fixed_N=rho)

    def test_symmetric_weight_has_additive_reserve(self):
        # Eq. 7-8: minimum normal = |ft|/mu + rho*sqrt(1+mu^2)/mu.
        for mu, rho in ((1., 0.), (1., .25), (1., .5), (.75, .5)):
            with self.subTest(mu=mu, rho=rho):
                solver = self.solver(rho, mu)
                result = solver.solve(**geometry())
                self.assertTrue(result["ok"], result["reason"])
                expected = .2 * 9.81 / (2 * mu) + 3 * rho * np.sqrt(1 + mu ** 2) / mu
                self.assertAlmostEqual(sum(result["fn_L"]), expected, places=4)
                self.assertAlmostEqual(sum(result["fn_R"]), expected, places=4)
                self.assertLessEqual(result["eta_max"], 1. + 1e-5)
                self.assertGreaterEqual(result["min_margin_surplus_N"], -1e-4)
                self.assertGreaterEqual(result["min_cone_slack_N"], -1e-4)
                self.assertTrue(next(iter(solver._programs.values()))["prob"].is_dpp())

    def test_zero_tangential_load_keeps_preload(self):
        result = self.solver(.5).solve(**geometry(), w_des=np.zeros(6))
        self.assertTrue(result["ok"], result["reason"])
        np.testing.assert_allclose(result["fn_L"] + result["fn_R"], .5 * np.sqrt(2), atol=2e-5)

    def test_triangle_transmits_pure_twist_with_reserve(self):
        solver = GraspSOCP(prior_mode="none", w_force=1., mu=1., margin_mode="fixed",
                           margin_fixed_N=.5, no_bending=True)
        # Six equal tangential forces around radius .03 generate tau_y=.02 Nm.
        # Each hand needs 3*rho*sqrt(2) + |tau|/(2*r*mu) normal force.
        result = solver.solve(**geometry(), w_des=np.array([0., 0., 0., 0., .02, 0.]))
        self.assertTrue(result["ok"], result["reason"])
        expected = 3 * .5 * np.sqrt(2) + .02 / (2 * .03)
        self.assertAlmostEqual(sum(result["fn_L"]), expected, places=4)
        self.assertAlmostEqual(sum(result["fn_R"]), expected, places=4)
        self.assertAlmostEqual(result["wrench_L"][4] + result["wrench_R"][4], .02, places=6)

    def test_inactive_points_have_no_preload(self):
        solver = self.solver(.5)
        result = solver.solve(**geometry(), active_L=[True, False, False],
                              active_R=[True, False, False])
        self.assertTrue(result["ok"], result["reason"])
        self.assertAlmostEqual(sum(result["fn_L"]), .2 * 9.81 / 2 + .5 * np.sqrt(2), places=4)
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
                               H, program["prob"], "test", np.full(6, .5))
        self.assertFalse(check["ok"])
        self.assertAlmostEqual(check["cone_violation"], np.sqrt(2) * .5, places=4)

    def test_invalid_parameters_fail_explicitly(self):
        for rho in (-.1, float("nan"), float("inf")):
            with self.subTest(rho=rho), self.assertRaises(ValueError):
                self.solver(rho)
        with self.assertRaisesRegex(ValueError, "removed"):
            GraspSOCP(prior_mode="none", w_force=1., r_s=.3)
        with self.assertRaisesRegex(ValueError, "removed"):
            GraspSOCP(prior_mode="none", w_force=1., cone_shift_N=.5)

    def test_scalar_task_allocation_equals_paper_eq_25(self):
        G = np.array([[.1, .2, .3, .4, -.2, .1]])
        rho, info = task_margin(G, np.array([.8]), [True, True], .25, .5)
        demand = .8 / np.linalg.norm(G)
        expected = np.array([np.linalg.norm(G[:, :3]), np.linalg.norm(G[:, 3:])]) / np.linalg.norm(G)
        np.testing.assert_allclose(rho, expected * (.25 * demand + .5), atol=1e-12)
        self.assertAlmostEqual(info["task_force_norm_N"], demand)

    def test_general_task_bounds_each_minimum_norm_correction(self):
        from kinova_dual_arm_dmp_insertion.socp import wrench_map
        args = geometry()
        G = wrench_map(np.vstack((args["posL"], args["posR"])), np.zeros(3))
        rho, info = task_margin(G, np.array([0., 0., 1.962, 0., 0., 0.]), [True] * 6, .25, .5)
        rng = np.random.default_rng(0)
        for _ in range(100):
            u = rng.normal(size=18)
            u *= info["task_margin_N"] / np.linalg.norm(u)
            correction = np.linalg.pinv(G) @ (G @ u)
            self.assertTrue(np.all(np.linalg.norm(correction.reshape(6, 3), axis=1) <= rho + 1e-12))

    def test_adaptive_margin_changes_with_task_and_remains_dpp(self):
        solver = GraspSOCP(prior_mode="none", w_force=1.)
        first = solver.solve(**geometry())
        heavier = solver.solve(**dict(geometry(), m_hat=.4))
        self.assertTrue(first["ok"], first["reason"])
        self.assertTrue(heavier["ok"], heavier["reason"])
        self.assertGreater(heavier["task_margin_N"], first["task_margin_N"])
        self.assertEqual(len(solver._programs), 1)
        self.assertTrue(next(iter(solver._programs.values()))["prob"].is_dpp())

    def test_adaptive_map_includes_no_bending_equalities(self):
        solver = GraspSOCP(prior_mode="none", w_force=1., no_bending=True)
        result = solver.solve(**geometry())
        self.assertTrue(result["ok"], result["reason"])
        self.assertGreater(result["task_margin_N"], .5)
        self.assertGreaterEqual(result["min_margin_surplus_N"], -1e-6)

    def test_general_allocation_is_invariant_to_equation_units(self):
        from kinova_dual_arm_dmp_insertion.socp import wrench_map
        args = geometry()
        G = wrench_map(np.vstack((args["posL"], args["posR"])), np.zeros(3))
        target = np.array([0., 0., 1.962, .01, 0., 0.])
        rho, info = task_margin(G, target, [True] * 6, .25, .5)
        units = np.diag([1., 1., 1., 1000., 1000., 1000.])
        converted, converted_info = task_margin(units @ G, units @ target, [True] * 6, .25, .5)
        np.testing.assert_allclose(converted, rho, atol=1e-12)
        self.assertAlmostEqual(info["task_force_norm_N"], converted_info["task_force_norm_N"])

    def test_force_perturbations_within_margin_stay_in_nominal_cone(self):
        result = self.solver(.5).solve(**geometry())
        rng = np.random.default_rng(1)
        normals = np.vstack((geometry()["nrmL"], geometry()["nrmR"]))
        for f, n in zip(result["f_L"] + result["f_R"], normals):
            for _ in range(50):
                delta = rng.normal(size=3)
                delta *= .5 / np.linalg.norm(delta)
                actual = f + delta
                normal = -n @ actual
                tangent = actual + normal * n
                self.assertGreaterEqual(normal, -1e-6)
                self.assertLessEqual(np.linalg.norm(tangent), normal + 1e-6)

    def check_forces(self, solver, forces, active):
        from kinova_dual_arm_dmp_insertion.socp import wrench_map
        args = geometry()
        pts = np.vstack((args["posL"], args["posR"]))
        H = np.zeros((12, 18))
        H[:6, :9] = wrench_map(args["posL"], args["baseL"])
        H[6:, 9:] = wrench_map(args["posR"], args["baseR"])
        program = next(iter(solver._programs.values()))
        return solver._check(forces.ravel(), pts, np.vstack((args["nrmL"], args["nrmR"])),
                             active, args["p_com"], np.array([0., 0., .2 * 9.81, 0., 0., 0.]),
                             H, program["prob"], "test", np.asarray(active) * .5)

    def test_checker_rejects_internal_bending_even_when_object_balances(self):
        solver = GraspSOCP(prior_mode="none", w_force=1., margin_mode="fixed",
                           margin_fixed_N=.5, no_bending=True)
        self.assertTrue(solver.solve(**geometry())["ok"])
        forces = np.array([[0., 2., .2 * 9.81 / 6]] * 3 + [[0., -2., .2 * 9.81 / 6]] * 3)
        self.assertTrue(self.check_forces(solver, forces, [True] * 6)["ok"])
        forces[[0, 1, 3, 4], 1] += [.1, -.1, -.1, .1]  # opposite couples, zero net object wrench
        result = self.check_forces(solver, forces, [True] * 6)
        self.assertLess(result["residual_torque"], 1e-10)
        self.assertLess(result["cone_violation"], 1e-10)
        self.assertGreater(result["bending_residual_Nm"], 1e-5)
        self.assertFalse(result["ok"])

    def test_checker_rejects_force_on_inactive_points_even_when_object_balances(self):
        solver = self.solver(.5)
        active = [True, False, False, True, False, False]
        result = solver.solve(**geometry(), active_L=active[:3], active_R=active[3:])
        self.assertTrue(result["ok"])
        forces = np.array(result["f_L"] + result["f_R"])
        forces[1] = [0., .1, 0.]
        forces[4] = [0., -.1, 0.]
        check = self.check_forces(solver, forces, active)
        self.assertLess(check["residual_force"], 1e-10)
        self.assertLess(check["residual_torque"], 1e-10)
        self.assertAlmostEqual(check["inactive_force_residual_N"], .1)
        self.assertFalse(check["ok"])

    def test_rigid_frame_change_preserves_margins_and_rotates_solution(self):
        from scipy.spatial.transform import Rotation
        args = geometry()
        solver = GraspSOCP(prior_mode="none", w_force=1.)
        reference = solver.solve(**args)
        rotation = Rotation.from_euler("xyz", [23., -39., 71.], degrees=True)
        translation = np.array([.3, -.7, .5])
        changed = dict(args)
        for key in ("posL", "posR", "baseL", "baseR", "p_com"):
            changed[key] = rotation.apply(args[key]) + translation
        for key in ("nrmL", "nrmR"):
            changed[key] = rotation.apply(args[key])
        changed["orientation"] = rotation.as_matrix()
        target = np.r_[rotation.apply([0., 0., .2 * 9.81]), np.zeros(3)]
        result = solver.solve(**changed, w_des=target)
        self.assertTrue(result["ok"], result["reason"])
        np.testing.assert_allclose(result["required_margin_N"], reference["required_margin_N"], atol=1e-10)
        expected = rotation.apply(np.array(reference["f_L"] + reference["f_R"]))
        np.testing.assert_allclose(np.array(result["f_L"] + result["f_R"]), expected, atol=1e-5)

    def test_normal_length_does_not_change_the_contact_model(self):
        solver = self.solver(.5)
        args = geometry()
        reference = solver.solve(**args)
        args["nrmL"] *= 2
        args["nrmR"] *= 3
        rescaled = solver.solve(**args)
        self.assertTrue(rescaled["ok"], rescaled["reason"])
        np.testing.assert_allclose(rescaled["fn_L"] + rescaled["fn_R"],
                                   reference["fn_L"] + reference["fn_R"], atol=1e-8)


if __name__ == "__main__":
    unittest.main()
