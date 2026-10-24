# Verbatim source segments from the frozen laboratory controller.
import math
import numpy as np
import pinocchio as pin

def angle_diff(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    Shortest signed angular difference: a − b, wrapped to (−π, π].

    Works element-wise on numpy arrays.
    Example: angle_diff(0.01, 6.28) → +0.02 (not −6.27)

    Used wherever two joint angles are subtracted to avoid wrap-around spikes
    on continuous joints (e.g. Kinova joints that can pass through 0/2π).
    """
    diff = np.asarray(a, dtype=float) - np.asarray(b, dtype=float)
    return (diff + np.pi) % (2.0 * np.pi) - np.pi

class URDFStribeckFriction:
    """
    Model-based friction using identified Stribeck parameters baked into the
    URDF's <joint><dynamics>...</dynamics></joint> tags.

    Per-joint friction torque (smooth sign via tanh):
        τ_f = fc·tanh(q̇/ε)
            + fv·q̇
            + fs·exp(-|q̇|/vs)·tanh(q̇/ε)
            + offset

    Read from URDF custom dynamics attributes:
        friction → fc       (Coulomb)
        damping  → fv       (viscous)
        stribeck → fs       (Stribeck negative-slope amplitude)
        offset   → offset   (per-joint DC bias)

    Pure feedforward — no internal state. The `compute()` signature accepts
    `ddq_nom_in` and `dt` for hot-path uniformity but ignores them.

    Usage
    -----
        fric = URDFStribeckFriction('gen3_p2_F_only.urdf', n_dof=7)
        tau_f = fric.compute(q, dq, np.zeros(n_dof), dt)
    """

    DEFAULT_VS  = 0.1     # Stribeck velocity scale (rad/s) — same as identification pipeline
    DEFAULT_EPS = 0.01    # tanh smoothing width (rad/s)    — same as identification pipeline

    def __init__(self, urdf_path: str, n_dof: int = 7,
                 vs: float = DEFAULT_VS, eps: float = DEFAULT_EPS,
                 scale: float = 1.0, use_offset: bool = True):
        """
        Parameters
        ----------
        urdf_path : str   – calibrated URDF with friction in <dynamics> tags
        n_dof     : int   – number of revolute joints to read (default 7)
        vs        : float – Stribeck velocity (rad/s)
        eps       : float – tanh smoothing width (rad/s)
        scale     : float – multiplicative scale on the whole friction (safety
                            knob: 0=off, 1=full, 0.5=conservative half-strength)
        use_offset : bool – include the per-joint `offset` (DC bias) term.
                            The offset was identified at calibration time as the
                            sensor's DC bias at static. If sensor bias has
                            drifted at deployment (temperature, mounting,
                            firmware), the static offset commands a constant
                            extra torque that no longer cancels — pass
                            `use_offset=False` to disable it as a quick test.
        """
        import xml.etree.ElementTree as ET

        self.n_dof = n_dof
        self.vs    = float(vs)
        self.eps   = float(eps)
        self.scale = float(scale)
        self.use_offset = bool(use_offset)

        # Parse <dynamics> per joint
        self.phi_f = np.zeros((n_dof, 4))    # columns: [fc, fv, fs, offset]
        j = 0
        for jt in ET.parse(urdf_path).getroot().findall('joint'):
            if jt.get('type') != 'revolute':
                continue
            d = jt.find('dynamics')
            if d is not None:
                self.phi_f[j, 0] = float(d.get('friction', 0.0))   # fc
                self.phi_f[j, 1] = float(d.get('damping',  0.0))   # fv
                self.phi_f[j, 2] = float(d.get('stribeck', 0.0))   # fs
                self.phi_f[j, 3] = float(d.get('offset',   0.0))   # bias
            j += 1
            if j >= n_dof:
                break

    def reset(self, q: np.ndarray, dq: np.ndarray):
        """No internal state to reset (purely feedforward)."""
        pass

    # Hard safety cap on per-joint friction torque (Nm). The largest identified
    # |fc| in our calibration is ~1.1 Nm (J2), so 5 Nm leaves ample headroom for
    # legitimate Stribeck peaks while preventing model blow-ups (e.g. NaN q̇)
    # from injecting hazardous torque into the joint command.
    HARD_CAP_NM = 5.0

    def compute(self, q: np.ndarray, dq: np.ndarray,
                ddq_nom_in: np.ndarray, dt: float) -> np.ndarray:
        """
        Compute Stribeck friction torque from q̇ alone.
        `q`, `ddq_nom_in`, `dt` are accepted but unused (interface compatibility).
        """
        # Defensive: replace any NaN/Inf in dq with 0 before evaluating tanh/exp.
        # Prevents a single corrupted Kortex feedback frame from poisoning τ_cmd.
        dq_safe = np.where(np.isfinite(dq), dq, 0.0)

        sgn   = np.tanh(dq_safe / self.eps)
        tau_f = (self.phi_f[:, 0] * sgn
                 + self.phi_f[:, 1] * dq_safe
                 + self.phi_f[:, 2] * np.exp(-np.abs(dq_safe) / self.vs) * sgn)
        if self.use_offset:
            tau_f = tau_f + self.phi_f[:, 3]
        tau_f = self.scale * tau_f
        # Hard cap (per joint, symmetric): defends against model misuse / bad scale.
        return np.clip(tau_f, -self.HARD_CAP_NM, self.HARD_CAP_NM)

    def set_scale(self, scale: float):
        """Runtime safety knob — ramp friction in/out without rebuilding."""
        self.scale = float(scale)

class OnlineKalmanState:
    """
    Per-joint constant-acceleration Kalman filter, recursive form.

    Mirrors `signal_processing.kalman_ddq` (used during offline calibration)
    but updates **per control tick** instead of running over a recorded buffer.

    State per joint:  x = [q, dq, ddq]
    Measurement:      y = [q_meas, dq_meas]
    Model:
        q_{k+1}   = q_k + dq_k·dt + ½·ddq_k·dt²
        dq_{k+1}  = dq_k + ddq_k·dt
        ddq_{k+1} = ddq_k        (random walk, process noise q_ddq)

    Why this matters for friction
    -----------------------------
    Stribeck friction uses `tanh(q̇/ε)` with ε ≈ 0.01 rad/s. Raw Kortex
    velocity at q̇≈0 is dominated by quantisation/encoder noise (~0.005 rad/s),
    so the sign term flips on every tick → friction torque chatter. Smoothed
    q̇ from this filter cleans that up while the q̈ output is also useful
    for advanced compensation.

    Wrap-around handling
    --------------------
    The joint position innovation uses `angle_diff` so a 2π wrap on Kortex
    feedback (e.g. 6.28 → 0.01) doesn't cause a state jump. The internal
    state stays continuous (multi-turn safe).

    Default noise params match those used during calibration so the model
    behaves identically online and offline.
    """

    def __init__(self, n_dof: int,
                 r_q: float   = 1e-6,
                 r_dq: float  = 2.5e-5,
                 q_ddq: float = 1.0):
        """
        Parameters
        ----------
        n_dof : int   – number of joints
        r_q   : float – position-measurement noise variance [rad²]
        r_dq  : float – velocity-measurement noise variance [(rad/s)²]
        q_ddq : float – acceleration process noise [(rad/s²)² per step]
                        higher → trust measurements more, faster tracking
        """
        self.n_dof  = n_dof
        self.r_q    = float(r_q)
        self.r_dq   = float(r_dq)
        self.q_ddq  = float(q_ddq)
        self.H      = np.array([[1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0]])
        self.R      = np.diag([self.r_q, self.r_dq])
        # Per-joint state: shape (n_dof, 3) for x and (n_dof, 3, 3) for P
        self.x      = np.zeros((n_dof, 3))
        self.P      = np.zeros((n_dof, 3, 3))
        self._init  = False

    def reset(self, q: np.ndarray, dq: np.ndarray):
        """Re-seed state from sensor readings — call on startup / mode switch."""
        for j in range(self.n_dof):
            self.x[j] = np.array([q[j], dq[j], 0.0])
            self.P[j] = np.diag([self.r_q, self.r_dq, self.q_ddq * 10.0])
        self._init = True

    def update(self, q_meas: np.ndarray, dq_meas: np.ndarray,
               dt: float) -> tuple:
        """
        One predict-update cycle. Returns (q_filt, dq_filt, ddq_filt).

        On the first call, simply seeds state from the measurements and
        returns them unchanged (with ddq=0).
        """
        # Defensive: refuse to ingest non-finite measurements — keep last estimate
        # rather than letting NaN/Inf propagate into the controller and torque cmd.
        if not (np.all(np.isfinite(q_meas)) and np.all(np.isfinite(dq_meas))):
            return (self.x[:, 0].copy(),
                    self.x[:, 1].copy(),
                    self.x[:, 2].copy())

        if not self._init:
            self.reset(q_meas, dq_meas)
            return q_meas.copy(), dq_meas.copy(), np.zeros(self.n_dof)

        # Cap dt so an OS scheduling stall (>several ms) doesn't make F·dt²
        # blow the prediction step. 50 ms ≈ 50× nominal at 1 kHz; well above
        # legitimate jitter, well below where the linear model collapses.
        dt = float(np.clip(dt, 1e-4, 0.05))

        F = np.array([[1.0, dt,  0.5 * dt * dt],
                      [0.0, 1.0, dt           ],
                      [0.0, 0.0, 1.0          ]])
        # Process noise scales with dt (continuous-time variance ~ q_ddq per second
        # → discretised as q_ddq·dt per step). This makes the filter behave
        # consistently across control rates (200 Hz offline, 1 kHz online).
        Q = np.diag([0.0, 0.0, self.q_ddq * dt])

        for j in range(self.n_dof):
            # ---- Predict ----
            self.x[j] = F @ self.x[j]
            self.P[j] = F @ self.P[j] @ F.T + Q

            # ---- Update with wrap-aware innovation on q ----
            innov_q  = (q_meas[j] - self.x[j, 0] + np.pi) % (2.0 * np.pi) - np.pi
            innov_dq = dq_meas[j] - self.x[j, 1]
            y = np.array([innov_q, innov_dq])

            S = self.H @ self.P[j] @ self.H.T + self.R
            K = self.P[j] @ self.H.T @ np.linalg.inv(S)
            self.x[j] = self.x[j] + K @ y
            self.P[j] = (np.eye(3) - K @ self.H) @ self.P[j]

        return (self.x[:, 0].copy(),
                self.x[:, 1].copy(),
                self.x[:, 2].copy())

class JointSpaceController:
    """
    Joint-space PD controller with gravity compensation and optional friction.

    Control law:
      tau = -Kj*(q - q_des) - Kd*dq + g(q) + tau_friction

    Usage
    -----
      model = pin.buildModelFromUrdf("robot.urdf")
      ctrl  = JointSpaceController(
                  model      = model,
                  Kj         = [50, 50, 50, 50, 10, 10, 10],
                  Kd         = [5,  5,  5,  5,  2,  2,  2 ],
              )
      ctrl.set_target(q_desired)
      tau = ctrl.compute_torque(q, dq, dt)
    """

    def __init__(
        self,
        model: pin.Model,
        Kj,
        Kd,
        max_torque = 20.0,
        friction_observer: 'URDFStribeckFriction' = None,
    ):
        """
        Parameters
        ----------
        model              : Pinocchio model built from URDF
        Kj                 : Joint stiffness, shape (n_dof,) [N·m/rad]
        Kd                 : Joint damping,   shape (n_dof,) [N·m·s/rad]
        max_torque         : Per-joint safety torque clip (Nm), scalar or array (n_dof,)
        friction_observer  : Optional URDFStribeckFriction instance
        """
        self.model = model
        self.data  = model.createData()   # own data object — NOT shared
        self.n_dof = model.nv

        self.Kj = np.diag(np.asarray(Kj, dtype=float))
        self.Kd = np.diag(np.asarray(Kd, dtype=float))
        self.max_torque = np.broadcast_to(np.asarray(max_torque, dtype=float), (self.n_dof,)).copy()
        self.friction_observer = friction_observer

        self.q_des    = np.zeros(self.n_dof)
        self._g_cached = None

    # ----------------------------------------------------------
    # Target management
    # ----------------------------------------------------------

    def set_target(self, q_desired: np.ndarray):
        """Set desired joint positions (radians)."""
        self.q_des = np.asarray(q_desired, dtype=float).copy()

    def init_target_from_state(self, q: np.ndarray):
        """Set target = current joint positions (hold-in-place on startup)."""
        self.q_des = np.asarray(q, dtype=float).copy()

    # ----------------------------------------------------------
    # Control law
    # ----------------------------------------------------------

    def compute_torque(self, q: np.ndarray, dq: np.ndarray,
                        tau_measured: np.ndarray, dt: float) -> np.ndarray:
        """
        Joint-space PD + gravity comp + (optional) URDF Stribeck friction.

        `tau_measured` is accepted for API symmetry with the callsite (the
        controller node passes it for both modes) but is not used here —
        URDFStribeckFriction is purely feedforward in q̇.

            err = q_des − q                   (wrapped to (−π, π])
            tau = Kj·err − Kd·q̇ + g(q) + τ_friction(q̇)
        """
        g = pin.computeGeneralizedGravity(self.model, self.data, q)
        self._g_cached = g

        # angle_diff(q_des, q) returns the shortest signed angular distance
        # from q to q_des, wrapped to (−π, π]. This handles continuous joints
        # correctly: q=0.01, q_des=6.28 → err=−0.01 (NOT +6.27).
        err = angle_diff(self.q_des, q)
        tau = self.Kj @ err - self.Kd @ dq + g

        if self.friction_observer is not None:
            tau = tau + self.friction_observer.compute(
                q, dq, np.zeros(self.n_dof), dt)

        return np.clip(tau, -self.max_torque, self.max_torque)

    # ----------------------------------------------------------
    # Wrench estimation — not available in joint-space controller
    # Use CartesianImpedanceController.estimate_wrench() instead.
    # ----------------------------------------------------------

    # ----------------------------------------------------------
    # Runtime gain tuning
    # ----------------------------------------------------------

    def set_gains(self, Kj, Kd):
        """Update joint PD gains at runtime."""
        self.Kj = np.diag(np.asarray(Kj, dtype=float))
        self.Kd = np.diag(np.asarray(Kd, dtype=float))

class CartesianImpedanceController:
    """
    Cartesian Impedance Controller based on Pinocchio.

    Completely decoupled from hardware and middleware.
    Extends v4 with optional friction observer integration.

    Control Law
    -----------
      v_cart = J(q) @ dq_filtered
      F      = K * e_cart - D * v_cart          (task-space wrench)
      tau    = J^T @ F  -  B * dq_filtered  +  g(q)  +  tau_friction

    Velocity Filtering
    ------------------
      Adaptive RC low-pass filter:
        alpha    = dt / (RC + dt),   RC = 1 / (2*pi*filter_hz)
        dq_filt  = alpha * dq_raw + (1 - alpha) * dq_filt_prev

    Usage
    -----
      ctrl = CartesianImpedanceController(
                 model         = model,
                 ee_frame_name = "tool_frame",
                 K = [350, 350, 350, 100, 100, 100],
                 D = [  5,   5,   5,   1,   1,   1],
             )
      ctrl.init_target_from_state(q)
      tau = ctrl.compute_torque(q, dq, dt)
    """

    def __init__(
        self,
        model: pin.Model,
        ee_frame_name: str,
        K,
        D,
        joint_damping: float = 0.5,
        max_torque = 20.0,
        filter_hz: float = 30.0,
        friction_observer: 'URDFStribeckFriction' = None,
        null_space_stiffness: float = 0.0,
        q_ref: np.ndarray = None,
    ):
        """
        Parameters
        ----------
        model              : Pinocchio model built from URDF
        ee_frame_name      : End-effector frame name in the URDF
        K                  : 6-D stiffness  [kx, ky, kz, krx, kry, krz]
        D                  : 6-D damping    [dx, dy, dz, drx, dry, drz]
        joint_damping      : Scalar joint-space damping B
        max_torque         : Per-joint safety torque clip (Nm), scalar or array (n_dof,)
        filter_hz          : Cutoff frequency for joint velocity LPF (Hz)
        friction_observer  : Optional URDFStribeckFriction instance.
        """
        self.model = model
        self.data  = model.createData()   # own data object — NOT shared
        self.n_dof = model.nv

        self.ee_frame_id = model.getFrameId(ee_frame_name)
        if self.ee_frame_id == model.nframes:
            raise ValueError(f"Frame '{ee_frame_name}' not found in Pinocchio model.")

        self.K             = np.asarray(K, dtype=float)
        self.D             = np.asarray(D, dtype=float)
        self.joint_damping = float(joint_damping)
        self.max_torque = np.broadcast_to(np.asarray(max_torque, dtype=float), (self.n_dof,)).copy()
        self.filter_hz     = float(filter_hz)
        self.friction_observer = friction_observer

        self.null_space_stiffness = float(null_space_stiffness)
        self.q_ref = np.asarray(q_ref, dtype=float) if q_ref is not None else np.zeros(self.n_dof)

        self.target_SE3: pin.SE3 = None
        self._dq_filt = np.zeros(self.n_dof)
        self._g_cached = None

    # ----------------------------------------------------------
    # Target management
    # ----------------------------------------------------------

    def set_target(self, SE3_target: pin.SE3):
        """Set desired end-effector pose explicitly."""
        self.target_SE3 = SE3_target.copy()

    def init_target_from_state(self, q: np.ndarray):
        """Set target = current FK pose (hold-in-place on startup)."""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        self.target_SE3 = self.data.oMf[self.ee_frame_id].copy()

    def get_current_pose(self, q: np.ndarray) -> pin.SE3:
        """Return current end-effector SE3 (runs forward kinematics)."""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.ee_frame_id].copy()

    # ----------------------------------------------------------
    # Control law
    # ----------------------------------------------------------
    def compute_torque(self, q: np.ndarray, dq: np.ndarray,
                        tau_measured: np.ndarray, dt: float) -> np.ndarray:
        """
        Cartesian impedance + gravity + (optional) URDF Stribeck friction.

        `tau_measured` is accepted for API symmetry but unused (URDFStribeckFriction
        is purely feedforward in q̇).

            v_cart  = J(q) · q̇_filt
            F       = K · e_cart − D · v_cart
            tau     = J^T · F − B · q̇_filt + g(q) + τ_friction(q̇)
                       (+ null-space posture term if enabled)
        """
        if self.target_SE3 is None:
            raise RuntimeError("Target pose not set.")

        # Adaptive RC low-pass filter for joint velocity
        RC    = 1.0 / (2.0 * math.pi * self.filter_hz)
        alpha = dt / (RC + dt)
        self._dq_filt = alpha * dq + (1.0 - alpha) * self._dq_filt

        # Kinematics, Jacobian, Gravity
        pin.forwardKinematics(self.model, self.data, q, self._dq_filt)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data, q)
        g = pin.computeGeneralizedGravity(self.model, self.data, q)
        self._g_cached = g

        # World-aligned Jacobian
        J = pin.computeFrameJacobian(
            self.model, self.data, q, self.ee_frame_id, pin.LOCAL_WORLD_ALIGNED
        )
        current_SE3 = self.data.oMf[self.ee_frame_id]

        # Pose error → impedance wrench
        err_pos = self.target_SE3.translation - current_SE3.translation
        err_rot = pin.log3(self.target_SE3.rotation @ current_SE3.rotation.T)
        error   = np.concatenate([err_pos, err_rot])
        v_cart  = J @ self._dq_filt
        F       = self.K * error - self.D * v_cart

        # Task torque + gravity
        tau = J.T @ F - self.joint_damping * self._dq_filt + g

        # Feedforward friction (URDF Stribeck) — uses filtered q̇
        if self.friction_observer is not None:
            tau = tau + self.friction_observer.compute(
                q, self._dq_filt, np.zeros(self.n_dof), dt)

        # Null-space posture control — keeps configuration near q_ref
        # without affecting the Cartesian task (projected through null-space of J)
        if self.null_space_stiffness > 0.0:
            J_pinv = np.linalg.pinv(J)                        # (7, 6)
            N = np.eye(self.n_dof) - J_pinv @ J               # null-space projector (7, 7)
            tau_null = N @ (self.null_space_stiffness * (self.q_ref - q)
                            - 2.0 * np.sqrt(self.null_space_stiffness) * self._dq_filt)
            tau = tau + tau_null

        return np.clip(tau, -self.max_torque, self.max_torque)

    # ----------------------------------------------------------
    # Wrench estimation
    # ----------------------------------------------------------

    def estimate_wrench(self, q: np.ndarray, tau_measured: np.ndarray,
                          dq: np.ndarray = None) -> np.ndarray:
        """
        Estimate end-effector wrench from measured joint torques.

        Sign convention
        ---------------
        Pinocchio:   M(q)·q̈ + C(q,q̇)·q̇ + g(q) = τ_motor + J^T · F_ext
        Kortex:      τ_sensor = −τ_motor  (sensor reads reaction torque,
                                            verified by τ_sensor ≈ −g at rest)

        At quasi-static (q̈ = q̇ = 0):
            g(q) = τ_motor + J^T · F_ext
            J^T · F_ext = g(q) − τ_motor = g(q) + τ_sensor
            F_ext       = pinv(J^T) · (τ_sensor + g(q))

        Friction-aware refinement
        -------------------------
        If the friction observer is available and `dq` is given, we subtract
        the predicted friction from the residual:

            F_ext = pinv(J^T) · (τ_sensor + g + τ_friction(q̇))

        The sign assumes the strain gauge is *motor-side* (sensor reads the
        torque before friction loss); this also cleans up the per-joint DC
        bias that calibration's `offset` term captured. For an ideal link-side
        sensor the correction is small (just removes calibration residuals).

        Returns [fx, fy, fz, tx, ty, tz] in the end-effector local frame.
        """
        g = self._g_cached
        if g is None:
            g = pin.computeGeneralizedGravity(self.model, self.data, q)

        tau_residual = tau_measured + g

        # Subtract predicted friction so wrench reads cleanly when joints move
        # (and removes the calibrated DC offset baked into friction.offset).
        if (self.friction_observer is not None) and (dq is not None):
            tau_friction_pred = self.friction_observer.compute(
                q, dq, np.zeros(self.n_dof), 0.0)
            tau_residual = tau_residual + tau_friction_pred

        pin.computeJointJacobians(self.model, self.data, q)
        J = pin.computeFrameJacobian(
            self.model, self.data, q, self.ee_frame_id, pin.LOCAL
        )
        return np.linalg.pinv(J.T) @ tau_residual

    # ----------------------------------------------------------
    # Runtime gain tuning
    # ----------------------------------------------------------

    def set_stiffness(self, K):
        self.K = np.asarray(K, dtype=float)

    def set_damping(self, D):
        self.D = np.asarray(D, dtype=float)

    def set_joint_damping(self, B: float):
        self.joint_damping = float(B)

    def set_filter_hz(self, hz: float):
        self.filter_hz = float(hz)
