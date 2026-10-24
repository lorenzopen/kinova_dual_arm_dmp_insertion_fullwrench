"""Executable method bodies transcribed from Lorenzo's supplied VERBATIM reference.
Docstrings/comments omitted solely for an executable-AST equivalence check.
This file is a reference, not imported by the simulation.
"""
class CartesianImpedanceController:
    def compute_torque(self, q: np.ndarray, dq: np.ndarray,
                       tau_measured: np.ndarray, dt: float) -> np.ndarray:
        if self.target_SE3 is None:
            raise RuntimeError("Target pose not set.")
        RC = 1.0 / (2.0 * math.pi * self.filter_hz)
        alpha = dt / (RC + dt)
        self._dq_filt = alpha * dq + (1.0 - alpha) * self._dq_filt
        pin.forwardKinematics(self.model, self.data, q, self._dq_filt)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data, q)
        g = pin.computeGeneralizedGravity(self.model, self.data, q)
        self._g_cached = g
        J = pin.computeFrameJacobian(
            self.model, self.data, q, self.ee_frame_id, pin.LOCAL_WORLD_ALIGNED
        )
        current_SE3 = self.data.oMf[self.ee_frame_id]
        err_pos = self.target_SE3.translation - current_SE3.translation
        err_rot = pin.log3(self.target_SE3.rotation @ current_SE3.rotation.T)
        error = np.concatenate([err_pos, err_rot])
        v_cart = J @ self._dq_filt
        F = self.K * error - self.D * v_cart
        tau = J.T @ F - self.joint_damping * self._dq_filt + g
        if self.friction_observer is not None:
            tau = tau + self.friction_observer.compute(
                q, self._dq_filt, np.zeros(self.n_dof), dt)
        if self.null_space_stiffness > 0.0:
            J_pinv = np.linalg.pinv(J)
            N = np.eye(self.n_dof) - J_pinv @ J
            tau_null = N @ (self.null_space_stiffness * (self.q_ref - q)
                            - 2.0 * np.sqrt(self.null_space_stiffness) * self._dq_filt)
            tau = tau + tau_null
        return np.clip(tau, -self.max_torque, self.max_torque)

    def estimate_wrench(self, q: np.ndarray, tau_measured: np.ndarray,
                        dq: np.ndarray = None) -> np.ndarray:
        g = self._g_cached
        if g is None:
            g = pin.computeGeneralizedGravity(self.model, self.data, q)
        tau_residual = tau_measured + g
        if (self.friction_observer is not None) and (dq is not None):
            tau_friction_pred = self.friction_observer.compute(
                q, dq, np.zeros(self.n_dof), 0.0)
            tau_residual = tau_residual + tau_friction_pred
        pin.computeJointJacobians(self.model, self.data, q)
        J = pin.computeFrameJacobian(
            self.model, self.data, q, self.ee_frame_id, pin.LOCAL
        )
        return np.linalg.pinv(J.T) @ tau_residual
