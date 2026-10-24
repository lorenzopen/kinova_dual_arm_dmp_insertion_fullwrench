#! /usr/bin/env python3

# Edited by: Kitti
# Date: 2026-03-06

"""
Multi-Mode Compliant Controller for Kinova Gen3
================================================
Three-layer architecture:

  URDFStribeckFriction         – feedforward Stribeck friction read from
                                 a calibrated URDF's <dynamics> tags.
  OnlineKalmanState            – per-joint q, q̇, q̈ smoothing.

  JointSpaceController         – joint-space PD + gravity + friction
  CartesianImpedanceController – Cartesian impedance + friction
                                 Both are pure math classes, hardware-agnostic.

  RealRobotImpedance(Node)     – ROS 2 node + Kortex hardware glue
                                 Mode switching: JOINT_SPACE ↔ CARTESIAN.

Control Laws
------------
Joint Space:
  tau = Kj*(q_des - q) + Kd*(dq_des - dq) + g(q) + tau_friction

Cartesian:
  F   = K * e_cart - D * v_cart
  tau = J^T * F - B * dq_filt + g(q) + tau_friction

Friction model (URDF Stribeck, identified offline):
  tau_friction = fc·tanh(q̇/ε) + fv·q̇ + fs·exp(-|q̇|/vs)·tanh(q̇/ε)  (+ offset)
  Parameters [fc, fv, fs, offset] are read from the URDF `<dynamics>` tag.

ROS 2 Topics / Services
-----------------------
Subscribed:
  /cmd_vel          Twist        – Cartesian teleop (cartesian mode only)
  /target_pose      PoseStamped  – Absolute Cartesian target (cartesian mode only)
  /joint_cmd        JointState   – Target joint angles in rad (joint_space mode only)
  /gripper_cmd      Float32      – Gripper position: 0.0=open, 1.0=closed

Published:
  /joint_states     JointState   – Arm (7 DOF) + Gripper mimic joints (6 joints)
  /target_pose_viz  PoseStamped
  /estimated_wrench WrenchStamped

Services:
  /switch_control_mode    std_srvs/SetBool  True=JOINT_SPACE, False=CARTESIAN
  /zero_wrench            std_srvs/Trigger  Tare /estimated_wrench (capture
                                             current reading as offset; ~200 ms
                                             averaging window).
  /clear_wrench_offset    std_srvs/Trigger  Reset wrench offset to 0 (raw).
"""

import os
import sys
import time
import threading
import math
from enum import Enum, auto

import numpy as np
import pinocchio as pin

# Python 3.10+ collections shim (required for kortex_api)
if sys.version_info >= (3, 10):
    import collections, collections.abc
    for _name in ("MutableMapping", "MutableSequence"):
        if not hasattr(collections, _name):
            setattr(collections, _name, getattr(collections.abc, _name))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped, WrenchStamped
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, Trigger

from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import ActuatorConfig_pb2, Base_pb2, BaseCyclic_pb2
from kortex_api.RouterClient import RouterClientSendOptions

from kinova_control import utilities


# ============================================================
#  Control Mode Enum
# ============================================================

class ControlMode(Enum):
    JOINT_SPACE = auto()
    CARTESIAN   = auto()


# ============================================================
#  Angle helpers
# ============================================================

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


# ============================================================
#  URDF Stribeck Friction (model-based, from identified URDF)
# ============================================================

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


# ============================================================
#  Online Kalman Filter — q̇, q̈ smoothing for real-time control
# ============================================================

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


# ============================================================
#  Joint Space Controller
# ============================================================

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


# ============================================================
#  Cartesian Impedance Controller  (v4 + optional friction)
# ============================================================

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


# ============================================================
#  ROS 2 Node  –  Kinova Gen3 hardware glue
# ============================================================

class RealRobotImpedance(Node):
    """
    ROS 2 node that wires JointSpaceController and CartesianImpedanceController
    to a Kinova Gen3 via the Kortex low-level cyclic API.

    Runtime mode switching via /switch_control_mode service.

    ROS 2 Parameters
    ----------------
    # Shared
    urdf_path          : str      path to URDF
    ee_frame           : str      end-effector frame [tool_frame]
    max_torque         : float    per-joint safety clip (Nm)    [20.0]
    control_hz         : float    control loop rate (Hz)         [750.0]
    initial_mode       : str      "cartesian" or "joint_space"   [cartesian]

    # Cartesian mode
    cart_k             : float[6] 6-D stiffness  [150,150,150,100,100,100]
    cart_d             : float[6] 6-D damping    [0.5,0.5,0.5,0.5,0.5,0.5]
    joint_damping      : float    joint-space damping B           [0.0]
    filter_hz          : float    velocity LPF cutoff (Hz)        [50.0]
    teleop_lin_scale   : float    m per /cmd_vel linear unit      [0.001]
    teleop_rot_scale   : float    rad per /cmd_vel angular unit   [0.001]

    # Joint space mode
    joint_kp           : float[7] stiffness per joint  [50,50,50,50,10,10,10]
    joint_kd           : float[7] damping per joint    [5,5,5,5,2,2,2]

    # URDF Stribeck friction (feedforward, identified offline)
    friction_enable     : bool   enable URDFStribeckFriction          [False]
    friction_urdf_path  : str    URDF with <dynamics> tags            ['' → urdf_path]
    friction_scale      : float  0=off, 1=full, 0.5=half               [0.0]
    stribeck_use_offset : bool   include per-joint DC offset           [True]
    stribeck_vs         : float  Stribeck velocity scale (rad/s)       [0.1]
    stribeck_eps        : float  tanh smoothing width (rad/s)          [0.01]

    # Online Kalman state estimator
    kalman_enable       : bool   smooth q, q̇; estimate q̈              [True]
    kalman_r_q          : float  position-meas noise variance          [1e-6]
    kalman_r_dq         : float  velocity-meas noise variance          [2.5e-5]
    kalman_q_ddq        : float  acceleration process noise            [1.0]

    # Safety
    tau_slew_per_tick       : float  max Δτ per tick (Nm)              [0.5]
    velocity_watchdog_limit : float  |q̇| trigger threshold (rad/s)    [2.0]
    velocity_watchdog_ticks : int    consecutive trigger ticks         [20]
    friction_ramp_seconds   : float  soft-start ramp duration (s)      [3.0]
    """

    def __init__(self, router, router_real_time):
        super().__init__('real_robot_controller_node')

        # ---- Default URDF path -------------------------------------------
        try:
            from ament_index_python.packages import get_package_share_directory
            _default_urdf = os.path.join(
                get_package_share_directory('kinova_description'),
                'urdf', 'kinova_gen3_cal_tool.urdf'
            )
        except Exception:
            _default_urdf = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                'kinova_gen3_cal_tool.urdf'
            )

        # ---- ROS 2 Parameters --------------------------------------------
        # Shared
        self.declare_parameter('urdf_path',      _default_urdf)
        self.declare_parameter('ee_frame',        'tool_frame')
        self.declare_parameter('max_torque',      [35.0, 35.0, 35.0, 35.0, 12.0, 12.0, 12.0])
        self.declare_parameter('control_hz',      1000.0)
        self.declare_parameter('initial_mode',    'cartesian')
        # self.declare_parameter('initial_mode',    'joint_space')

        # Cartesian mode
        self.declare_parameter('cart_k',          [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # self.declare_parameter('cart_k',          [50.0, 50.0, 50.0, 10.0, 10.0, 10.0])
        # self.declare_parameter('cart_k',          [300.0, 300.0, 300.0, 30.0, 30.0, 30.0])
        # self.declare_parameter('cart_k',          [500.0, 500.0, 500.0, 30.0, 30.0, 30.0])
        # self.declare_parameter('cart_k',          [1000.0, 1000.0, 1000.0, 30.0, 30.0, 30.0])
        # self.declare_parameter('cart_k',          [200.0, 200.0, 200.0, 100.0, 100.0, 100.0])
        # self.declare_parameter('cart_d',          [0.2,   0.2,   0.2,   0.1,   0.1,   0.1  ])
        self.declare_parameter('cart_d',          [0.1,   0.1,   0.1,   0.05,   0.05,   0.05  ])
        self.declare_parameter('joint_damping',   0.001)
        self.declare_parameter('filter_hz',       100.0)
        self.declare_parameter('teleop_lin_scale', 0.001)
        self.declare_parameter('teleop_rot_scale', 0.001)

        # Joint space mode
        # self.declare_parameter('joint_kp',        [350.0, 350.0, 350.0, 350.0, 350.0, 350.0, 350.0])
        self.declare_parameter('joint_kp',        [500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0])
        self.declare_parameter('joint_kd',        [5.0,  5.0,  5.0,  5.0,  2.0,  2.0,  2.0 ])
        # self.declare_parameter('joint_kp',        [50.0, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0])
        # self.declare_parameter('joint_kd',        [5.0,  5.0,  5.0,  5.0,  2.0,  2.0,  2.0 ])

        # Null-space posture control
        self.declare_parameter('null_space_stiffness', 0.0)
        self.declare_parameter('null_space_q_ref',     [0.00, 0.35, 3.14, -1.50, 0.00, 1.05, 1.57])

        # ─── URDF Stribeck friction (feedforward, calibrated) ───────────
        self.declare_parameter('friction_enable',  False)
        # URDF that contains identified <dynamics> tags. Empty → fall back
        # to the same urdf_path used for kinematics.
        self.declare_parameter('friction_urdf_path', '')
        # Safety knob: 0.0 = off, 1.0 = full identified friction, 0.5 = half.
        # Default = 0.0 → EXPLICITLY OPT-IN; first robot test should ramp
        # 0.0 → 0.3 → 0.5 → 1.0 while watching for drift / chatter.
        self.declare_parameter('friction_scale',   0.0)
        self.declare_parameter('stribeck_vs',      URDFStribeckFriction.DEFAULT_VS)
        self.declare_parameter('stribeck_eps',     URDFStribeckFriction.DEFAULT_EPS)
        # Stribeck `offset` term = per-joint DC sensor bias from calibration.
        # Drift / temperature / firmware can change the live bias; if the
        # robot drifts in gravity-only mode, set this False to disable the
        # offset and keep only the velocity-dependent part of friction.
        self.declare_parameter('stribeck_use_offset', True)

        # Online Kalman filter (smooths q, q̇; estimates q̈)
        # Noise defaults match the offline calibration filter (signal_processing.kalman_ddq).
        # Strongly recommended together with the URDF Stribeck friction —
        # tanh(q̇/ε) chatters on raw encoder velocity near zero, the Kalman
        # filter cleans that up.
        self.declare_parameter('kalman_enable', True)
        self.declare_parameter('kalman_r_q',    1e-6)
        self.declare_parameter('kalman_r_dq',   2.5e-5)
        self.declare_parameter('kalman_q_ddq',  1.0)

        # Gripper: set False for an arm with NO gripper / interconnect module so
        # the controller never adds or sends a gripper command in the cyclic
        # frame (avoids any firmware complaint on a gripper-less arm).
        self.declare_parameter('gripper_enable', True)

        # Homing: instead of Kinova's high-level "Home" action (which needs the
        # action database + session control + high-level servoing, all of which
        # can fail with DATABASE_ERROR / SESSION_NOT_IN_CONTROL / WRONG_SERVOING_
        # MODE), drive to a home joint configuration COMPLIANTLY using the
        # joint-space PD controller after entering torque mode.
        self.declare_parameter('home_on_start', True)
        # Kinova Gen3 home, measured in degrees [0, 15, 179, 230, 360, 55, 90]
        # and converted to radians. NOTE: J4 230°→4.0143 rad (≡ -2.269) and
        # J5 360°→6.2832 (≡ 0); angle_diff() handles the 2π wrap, so these
        # raw-degree-to-radian values give the correct shortest-path error.
        self.declare_parameter('home_joint_angles',
                               [0.0, 0.2618, 3.1241, 4.0143, 6.2832, 0.9599, 1.5708])
        self.declare_parameter('home_seconds', 5.0)   # duration of the homing drive

        # ─── Safety layer ──────────────────────────────────────────────
        # Maximum torque change per control tick (Nm). Limits dτ/dt so a
        # discontinuity in any term (mode switch, friction-scale ramp, command
        # step) cannot inject an unbounded impulse into the joint.
        self.declare_parameter('tau_slew_per_tick', 0.5)

        # Velocity watchdog: if any joint exceeds this magnitude for more than
        # `velocity_watchdog_ticks` consecutive ticks, the controller drops
        # tau_cmd to gravity-only and logs a fault.
        self.declare_parameter('velocity_watchdog_limit',  2.0)   # rad/s
        self.declare_parameter('velocity_watchdog_ticks',  20)    # consecutive

        # Friction soft-start: ramp friction_scale from 0 to its commanded
        # value over `friction_ramp_seconds` seconds after init / mode switch.
        # Avoids the ~0.6 Nm step on J1 at t=0.
        self.declare_parameter('friction_ramp_seconds', 3.0)
        
        # ---- Read parameters ---------------------------------------------
        urdf_path       = self.get_parameter('urdf_path').value
        ee_frame        = self.get_parameter('ee_frame').value
        max_torque      = list(self.get_parameter('max_torque').value)
        filter_hz       = float(self.get_parameter('filter_hz').value)
        cart_k          = list(self.get_parameter('cart_k').value)
        cart_d          = list(self.get_parameter('cart_d').value)
        joint_damping   = float(self.get_parameter('joint_damping').value)
        joint_kp        = list(self.get_parameter('joint_kp').value)
        joint_kd        = list(self.get_parameter('joint_kd').value)
        friction_enable        = bool(self.get_parameter('friction_enable').value)
        friction_urdf_path     = str(self.get_parameter('friction_urdf_path').value)
        friction_scale         = float(self.get_parameter('friction_scale').value)
        stribeck_vs            = float(self.get_parameter('stribeck_vs').value)
        stribeck_eps           = float(self.get_parameter('stribeck_eps').value)
        stribeck_use_offset    = bool(self.get_parameter('stribeck_use_offset').value)
        kalman_enable          = bool(self.get_parameter('kalman_enable').value)
        self.gripper_enable    = bool(self.get_parameter('gripper_enable').value)
        self.home_on_start     = bool(self.get_parameter('home_on_start').value)
        self.home_joint_angles = list(self.get_parameter('home_joint_angles').value)
        self.home_seconds      = float(self.get_parameter('home_seconds').value)
        kalman_r_q             = float(self.get_parameter('kalman_r_q').value)
        kalman_r_dq            = float(self.get_parameter('kalman_r_dq').value)
        kalman_q_ddq           = float(self.get_parameter('kalman_q_ddq').value)
        self.tau_slew_per_tick      = float(self.get_parameter('tau_slew_per_tick').value)
        self.velocity_watchdog_lim  = float(self.get_parameter('velocity_watchdog_limit').value)
        self.velocity_watchdog_max  = int(self.get_parameter('velocity_watchdog_ticks').value)
        self.friction_ramp_seconds  = float(self.get_parameter('friction_ramp_seconds').value)
        self._friction_scale_target = friction_scale   # remember target; scale is ramped from 0
        self._velocity_watchdog_count = 0
        self._safe_mode = False                        # set True if watchdog trips
        null_space_stiffness   = float(self.get_parameter('null_space_stiffness').value)
        null_space_q_ref       = list(self.get_parameter('null_space_q_ref').value)
        initial_mode_str = self.get_parameter('initial_mode').value

        self.control_dt        = 1.0 / float(self.get_parameter('control_hz').value)
        self.teleop_lin_scale  = float(self.get_parameter('teleop_lin_scale').value)
        self.teleop_rot_scale  = float(self.get_parameter('teleop_rot_scale').value)

        if not os.path.isfile(urdf_path):
            raise FileNotFoundError(
                f"URDF not found: '{urdf_path}'\n"
                "Set urdf_path parameter or correct the default path."
            )

        # ── Parameter shape validation (fail fast at startup) ─────────────
        def _expect_len(name, val, expected):
            if len(val) != expected:
                raise ValueError(
                    f"Parameter '{name}' must have {expected} elements, got {len(val)}: {val}"
                )
        _expect_len('cart_k',     cart_k,     6)   # 6-D Cartesian: x,y,z,rx,ry,rz
        _expect_len('cart_d',     cart_d,     6)
        _expect_len('joint_kp',   joint_kp,   7)   # Kinova Gen3 has 7 joints
        _expect_len('joint_kd',   joint_kd,   7)
        _expect_len('max_torque', max_torque, 7)

        # ---- Initial mode ------------------------------------------------
        if initial_mode_str.lower() == 'joint_space':
            self._mode = ControlMode.JOINT_SPACE
        else:
            self._mode = ControlMode.CARTESIAN
        # Remember the requested mode: homing temporarily forces JOINT_SPACE,
        # then we switch back to this once home is reached.
        self._initial_mode = self._mode

        # ---- Pinocchio model (shared, read-only) -------------------------
        model = pin.buildModelFromUrdf(urdf_path)
        # n_dof = arm joints only (7 DOF) — gripper is controlled separately
        # via high-level Kortex API and does NOT appear in actuators[].
        # Kortex actuator_count = 7, so we must keep n_dof = 7.
        N_ARM_DOF = 7
        self.n_dof       = N_ARM_DOF
        self.joint_names = [model.names[i] for i in range(1, N_ARM_DOF + 1)]

        # Build a reduced Pinocchio model with only the 7 arm joints so that
        # gravity / Jacobian computations use the correct (7-DOF) model.
        joints_to_lock = list(range(N_ARM_DOF + 1, model.njoints))  # gripper joint ids
        if joints_to_lock:
            q0 = pin.neutral(model)
            model = pin.buildReducedModel(model, joints_to_lock, q0)

        # ---- URDF Stribeck friction (optional, shared between both controllers)
        if friction_enable:
            fric_urdf = friction_urdf_path if friction_urdf_path else urdf_path
            if not os.path.isfile(fric_urdf):
                raise FileNotFoundError(
                    f"friction_urdf_path not found: '{fric_urdf}'.\n"
                    "Point to a calibrated URDF (e.g. gen3_p2_F_only.urdf) "
                    "with identified <dynamics> tags."
                )
            self.friction_obs = URDFStribeckFriction(
                urdf_path  = fric_urdf,
                n_dof      = self.n_dof,
                vs         = stribeck_vs,
                eps        = stribeck_eps,
                scale      = friction_scale,
                use_offset = stribeck_use_offset,
            )
            self.get_logger().info(
                f"Friction: URDF Stribeck from '{os.path.basename(fric_urdf)}'  "
                f"(scale={friction_scale}, vs={stribeck_vs}, eps={stribeck_eps}, "
                f"use_offset={stribeck_use_offset})"
            )
            for j in range(self.n_dof):
                fc, fv, fs, off = self.friction_obs.phi_f[j]
                self.get_logger().info(
                    f"  J{j+1}: fc={fc:+.3f}  fv={fv:+.3f}  "
                    f"fs={fs:+.3f}  off={off:+.3f}"
                )
        else:
            self.friction_obs = None
            self.get_logger().info("Friction: disabled")

        # ---- Online Kalman filter for q, q̇, q̈ ---------------------------
        if kalman_enable:
            self.kalman = OnlineKalmanState(
                n_dof = self.n_dof,
                r_q   = kalman_r_q,
                r_dq  = kalman_r_dq,
                q_ddq = kalman_q_ddq,
            )
            self.get_logger().info(
                f"Online Kalman: ENABLED  (r_q={kalman_r_q}, r_dq={kalman_r_dq}, q_ddq={kalman_q_ddq})"
            )
        else:
            self.kalman = None
            self.get_logger().info("Online Kalman: disabled (using raw q, dq)")

        # State holders for filtered values — populated in run_control_loop
        self.q_filt   = np.zeros(self.n_dof)
        self.dq_filt  = np.zeros(self.n_dof)
        self.ddq_filt = np.zeros(self.n_dof)

        # ---- Controllers (each has its own pin.Data) ----------------------
        self.joint_ctrl = JointSpaceController(
            model             = model,
            Kj                = joint_kp,
            Kd                = joint_kd,
            max_torque        = max_torque,
            friction_observer = self.friction_obs,
        )

        self.cart_ctrl = CartesianImpedanceController(
            model                 = model,
            ee_frame_name         = ee_frame,
            K                     = cart_k,
            D                     = cart_d,
            joint_damping         = joint_damping,
            max_torque            = max_torque,
            filter_hz             = filter_hz,
            friction_observer     = self.friction_obs,
            null_space_stiffness  = null_space_stiffness,
            q_ref                 = np.array(null_space_q_ref),
        )
        if null_space_stiffness > 0.0:
            self.get_logger().info(
                f'Null-space posture control: ON (Kn={null_space_stiffness}, q_ref={null_space_q_ref})')

        # ---- State -------------------------------------------------------
        self.q       = np.zeros(self.n_dof)
        self.dq      = np.zeros(self.n_dof)
        self.torque  = np.zeros(self.n_dof)
        self.running = True

        # Gripper state (Robotiq 2F-85)
        # position in rad (0.0 = open, 0.8 = closed), matches URDF limits
        self.gripper_pos = 0.0   # robotiq_85_left_knuckle_joint (rad)
        self.gripper_vel = 0.0   # rad/s
        # Desired gripper position in % (0–100), sent via cyclic interconnect command
        self._gripper_cmd_pct = 0.0
        # True after homing finishes in JOINT_SPACE mode, until an external
        # /joint_cmd arrives. Used to keep recovery from "helpfully" changing
        # the target to a sagged current pose.
        self._hold_home_target = False

        # Single lock covering mode, all targets, and friction observer state
        self._ctrl_lock = threading.Lock()

        # ---- Kortex -------------------------------------------------------
        self.base            = BaseClient(router)
        self.base_cyclic     = BaseCyclicClient(router_real_time)
        self.actuator_config = ActuatorConfigClient(router)

        self.base_command  = BaseCyclic_pb2.Command()
        self.base_feedback = BaseCyclic_pb2.Feedback()
        self.actuator_count = self.base.GetActuatorCount().count

        self.send_opt            = RouterClientSendOptions()
        self.send_opt.andForget  = False
        self.send_opt.delay_ms   = 0
        self.send_opt.timeout_ms = 3

        # ---- ROS 2 Publishers --------------------------------------------
        # NOTE: topic/service names are RELATIVE (no leading '/') so that a
        # launch-time namespace (e.g. __ns:=/master) prefixes them automatically.
        # This lets two arms run as two namespaced instances without name clashes.
        # A single-arm run in the root namespace still resolves to '/joint_states'.
        self.pub_joint        = self.create_publisher(JointState,    'joint_states',          10)
        self.pub_target       = self.create_publisher(PoseStamped,   'target_pose_viz',       10)
        self.pub_joint_target = self.create_publisher(JointState,    'joint_target_viz',      10)
        self.pub_wrench       = self.create_publisher(WrenchStamped, 'estimated_wrench',      10)
        # Diagnostics: q_filt, dq_filt, ddq_filt, tau_cmd, gravity, friction —
        # the test logger (test_logger.py) subscribes here to capture the full
        # internal state for offline analysis.
        self.pub_diag         = self.create_publisher(JointState,    'control_diagnostics',   10)

        # ---- ROS 2 Subscribers -------------------------------------------
        self.create_subscription(Twist,      'cmd_vel',     self._cb_teleop,       10)
        self.create_subscription(PoseStamped,'target_pose', self._cb_target_pose,  10)
        self.create_subscription(JointState, 'joint_cmd',   self._cb_joint_cmd,    10)
        self.create_subscription(Float32,    'gripper_cmd', self._cb_gripper_cmd,  10)

        # ---- ROS 2 Service -----------------------------------------------
        self.create_service(SetBool, 'switch_control_mode', self._cb_switch_mode)

        # ── Wrench-tare services ───────────────────────────────────────
        # /zero_wrench           : capture an N-sample average of the current
        #                           f_ext and store it as a permanent offset
        #                           (subtract from future readings).
        # /clear_wrench_offset   : reset offset to 0 (raw readings).
        # Call /zero_wrench when the robot is stationary AND no external force
        # is applied, to remove the calibration residual.
        self.wrench_offset             = np.zeros(6)
        self._wrench_zero_collecting   = False
        self._wrench_zero_buffer       = []
        self._WRENCH_ZERO_N            = 50   # ≈200 ms at 250 Hz publish rate
        self.create_service(Trigger, 'zero_wrench',
                             self._cb_zero_wrench)
        self.create_service(Trigger, 'clear_wrench_offset',
                             self._cb_clear_wrench_offset)

        self.get_logger().info(
            f"RealRobotImpedance initialized. Initial mode: {self._mode.name}"
        )

    # ----------------------------------------------------------
    # Mode switching
    # ----------------------------------------------------------

    def _switch_mode(self, new_mode: ControlMode):
        """
        Bumpless mode switch: seeds the new controller from actual state
        and resets the friction observer to prevent torque spikes.
        Must be called with self._ctrl_lock held OR before the control loop starts.
        """
        if new_mode == self._mode:
            return

        if new_mode == ControlMode.JOINT_SPACE:
            self.joint_ctrl.set_target(self.q.copy())
        else:
            # Reset velocity filter to actual velocity so the first tick of
            # Cartesian mode does not compute a stale v_cart = J @ old_dq_filt
            self.cart_ctrl._dq_filt = self.dq.copy()
            self.cart_ctrl.init_target_from_state(self.q)

        if self.friction_obs is not None:
            self.friction_obs.reset(self.q, self.dq)
            if isinstance(self.friction_obs, URDFStribeckFriction):
                # Restart the soft-start ramp so the friction model fades in
                # rather than stepping in across the mode boundary.
                self.friction_obs.set_scale(0.0)
                self._ramp_t0 = time.monotonic()
        if self.kalman is not None:
            self.kalman.reset(self.q, self.dq)
        self._velocity_watchdog_count = 0
        self._safe_mode = False

        self._mode = new_mode
        self.get_logger().info(f"Switched to {new_mode.name}")

    def _cb_switch_mode(self, request: SetBool.Request, response: SetBool.Response):
        """
        Service callback: True = JOINT_SPACE, False = CARTESIAN
        Called from spin_once, so the control loop is between ticks.
        """
        new_mode = ControlMode.JOINT_SPACE if request.data else ControlMode.CARTESIAN
        with self._ctrl_lock:
            self._switch_mode(new_mode)
        response.success = True
        response.message = f"Switched to {new_mode.name}"
        return response

    # ----------------------------------------------------------
    # Wrench-tare services
    # ----------------------------------------------------------

    def _cb_zero_wrench(self, request: Trigger.Request, response: Trigger.Response):
        """Capture next N publishes of f_ext and store the average as offset."""
        with self._ctrl_lock:
            self._wrench_zero_buffer = []
            self._wrench_zero_collecting = True
        response.success = True
        response.message = (f"Started averaging next {self._WRENCH_ZERO_N} "
                            f"wrench samples (≈{self._WRENCH_ZERO_N*4} ms). "
                            f"Hold the robot still with no external force.")
        self.get_logger().info(response.message)
        return response

    def _cb_clear_wrench_offset(self, request: Trigger.Request, response: Trigger.Response):
        """Reset wrench offset to zero (raw readings)."""
        with self._ctrl_lock:
            self.wrench_offset = np.zeros(6)
            self._wrench_zero_collecting = False
            self._wrench_zero_buffer = []
        response.success = True
        response.message = "Wrench offset cleared."
        self.get_logger().info(response.message)
        return response

    # ----------------------------------------------------------
    # ROS 2 Callbacks
    # ----------------------------------------------------------

    def _cb_teleop(self, msg: Twist):
        """Increment Cartesian target from /cmd_vel (keyboard / joystick)."""
        with self._ctrl_lock:
            if self._mode != ControlMode.CARTESIAN:
                return
            if self.cart_ctrl.target_SE3 is None:
                return
            self.cart_ctrl.target_SE3.translation += (
                np.array([msg.linear.x, msg.linear.y, msg.linear.z])
                * self.teleop_lin_scale
            )
            delta_rot = pin.exp3(
                np.array([msg.angular.x, msg.angular.y, msg.angular.z])
                * self.teleop_rot_scale
            )
            self.cart_ctrl.target_SE3.rotation = (
                self.cart_ctrl.target_SE3.rotation @ delta_rot
            )

    def _cb_target_pose(self, msg: PoseStamped):
        """Set absolute Cartesian target from /target_pose."""
        t    = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         msg.pose.position.z])
        quat = pin.Quaternion(
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        )
        new_target = pin.SE3(quat.toRotationMatrix(), t)
        with self._ctrl_lock:
            if self._mode != ControlMode.CARTESIAN:
                return
            self.cart_ctrl.set_target(new_target)

    def _cb_joint_cmd(self, msg: JointState):
        """Set target joint positions from /joint_cmd (radians in msg.position)."""
        if len(msg.position) < self.n_dof:
            self.get_logger().warn(
                f"/joint_cmd has {len(msg.position)} positions, expected {self.n_dof}"
            )
            return
        q_des = np.array(msg.position[:self.n_dof], dtype=float)
        with self._ctrl_lock:
            if self._mode != ControlMode.JOINT_SPACE:
                return
            self._hold_home_target = False
            self.joint_ctrl.set_target(q_des)

    def _cb_gripper_cmd(self, msg: Float32):
        """
        Set gripper position from /gripper_cmd.

        msg.data: 0.0 = fully open, 1.0 = fully closed
        Stores desired position (%) to be sent via low-level cyclic interconnect
        command on the next control loop tick (compatible with LOW_LEVEL_SERVOING).
        """
        value = float(np.clip(msg.data, 0.0, 1.0))
        self._gripper_cmd_pct = value * 100.0  # convert to % (0–100)

    # ----------------------------------------------------------
    # Hardware interface – Kortex
    # ----------------------------------------------------------

    def clear_faults(self):
        """Clear arm fault state."""
        if self.base.GetArmState().active_state == Base_pb2.ARMSTATE_IN_FAULT:
            self.get_logger().warn("Arm is in fault — clearing faults...")
            self.base.ClearFaults()
            time.sleep(2.0)
            if self.base.GetArmState().active_state == Base_pb2.ARMSTATE_IN_FAULT:
                self.get_logger().error("Fault could not be cleared.")
                return False
            self.get_logger().info("Faults cleared.")
        return True

    def move_to_home(self) -> bool:
        """Execute Kinova's pre-defined 'Home' joint-angle action."""
        if not self.clear_faults():
            return False

        self.get_logger().info("Moving to Home position...")

        srv = Base_pb2.ServoingModeInformation()
        srv.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(srv)

        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list   = self.base.ReadAllActions(action_type)
        action_handle = next(
            (a.handle for a in action_list.action_list if a.name == "Home"), None
        )
        if action_handle is None:
            self.get_logger().error("'Home' action not found on the robot.")
            return False

        done = threading.Event()

        def _on_notification(notification, ev=done):
            if notification.action_event in (Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT):
                ev.set()

        handle = self.base.OnNotificationActionTopic(
            _on_notification, Base_pb2.NotificationOptions()
        )
        self.base.ExecuteActionFromReference(action_handle)
        finished = done.wait(timeout=20.0)
        self.base.Unsubscribe(handle)

        if not finished:
            self.get_logger().warn("Timeout waiting for Home action.")
        return finished

    def recover_servoing_mode(self):
        """
        Force the arm back to a CLEAN high-level (SINGLE_LEVEL) state at startup.

        SAFE: this is a pure mode switch — the arm stays powered and holds its
        position. It does NOT power-cycle anything (unlike a hardware reboot).

        A previous run that exited uncleanly can leave the arm stuck in
        LOW_LEVEL_SERVOING and/or with a latched actuator fault; the next run
        then fails with WRONG_SERVOING_MODE. A single blind SetServoingMode is
        enough for a healthy arm (e.g. the newer .200) but NOT for a worn arm
        (e.g. .107), which needs faults cleared AND a *verified* mode switch to
        re-enter low-level without a physical power-cycle.

        So this now: (1) logs the real arm_state + servoing_mode for diagnosis,
        (2) clears faults if any, (3) sets SINGLE_LEVEL, (4) VERIFIES the switch
        actually took via GetServoingMode/GetArmState, retrying the whole
        sequence up to 5×.
        """
        for attempt in range(5):
            try:
                arm_state = self.base.GetArmState().active_state
                srv_mode  = self.base.GetServoingMode().servoing_mode
                self.get_logger().info(
                    f"Startup state (try {attempt+1}/5): "
                    f"arm={Base_pb2.ArmState.Name(arm_state)}, "
                    f"servoing={Base_pb2.ServoingMode.Name(srv_mode)}")

                # Clear faults UNCONDITIONALLY. A latched actuator
                # following-error fault (the .107 symptom) can block LOW_LEVEL
                # entry on the next run WITHOUT surfacing as ARMSTATE_IN_FAULT,
                # so a state-gated clear (the old behaviour) would miss it.
                # ClearFaults on a healthy arm is a harmless no-op.
                if arm_state == Base_pb2.ARMSTATE_IN_FAULT:
                    self.get_logger().warn("Arm is IN_FAULT — clearing faults...")
                self.base.ClearFaults()
                time.sleep(1.0 if arm_state == Base_pb2.ARMSTATE_IN_FAULT else 0.3)

                # Force SINGLE_LEVEL.
                srv = Base_pb2.ServoingModeInformation()
                srv.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
                self.base.SetServoingMode(srv)
                time.sleep(0.3)

                # Verify it actually switched and we are not in fault.
                now_mode  = self.base.GetServoingMode().servoing_mode
                now_state = self.base.GetArmState().active_state
                if (now_mode == Base_pb2.SINGLE_LEVEL_SERVOING
                        and now_state != Base_pb2.ARMSTATE_IN_FAULT):
                    self.get_logger().info(
                        "Servoing mode reset to SINGLE_LEVEL (clean state, verified).")
                    return True

                self.get_logger().warn(
                    f"Still not clean after reset "
                    f"(servoing={Base_pb2.ServoingMode.Name(now_mode)}, "
                    f"arm={Base_pb2.ArmState.Name(now_state)}); retrying...")
            except Exception as e:
                self.get_logger().warn(
                    f"recover_servoing_mode attempt {attempt+1}/5 failed: {e}")
            time.sleep(0.5)

        self.get_logger().error(
            "Could not reach a clean SINGLE_LEVEL state at startup. "
            "The arm may need a physical power-cycle.")
        return False

    def home_with_impedance(self) -> bool:
        """
        Compliantly drive to `home_joint_angles` using the joint-space PD
        controller (NO high-level action — avoids the action database). Must be
        called AFTER init_kortex_control() (i.e. already in TORQUE / LOW_LEVEL).

        Runs a short dedicated control loop, then switches to the requested
        initial mode (bumpless, seeded from the reached pose).
        """
        if not self.home_on_start:
            return True
        if len(self.home_joint_angles) != self.n_dof:
            self.get_logger().error(
                f"home_joint_angles must have {self.n_dof} elements; skipping homing.")
            return False

        home    = np.array(self.home_joint_angles, dtype=float)
        q_start = self.q.copy()
        delta_q = angle_diff(home, q_start)   # shortest signed path per joint
        self.get_logger().info(
            f"Homing: current q = {np.round(q_start, 3).tolist()}")
        self.get_logger().info(
            f"Homing: target    = {self.home_joint_angles}  "
            f"(max |Δ| = {float(np.max(np.abs(delta_q))):.3f} rad over {self.home_seconds}s)")
        with self._ctrl_lock:
            self._mode = ControlMode.JOINT_SPACE
            self.joint_ctrl.set_target(q_start)   # start at current pose: zero error

        pre = time.monotonic()
        t0  = pre
        while rclpy.ok() and (time.monotonic() - t0) < self.home_seconds:
            now = time.monotonic()
            sleep_t = self.control_dt - (now - pre)
            if sleep_t > 0:
                time.sleep(sleep_t)
            now = time.monotonic()
            dt  = min(now - pre, 3.0 * self.control_dt)
            pre = now

            # Smoothly ramp the PD target from q_start to home so tracking error
            # (and thus motion) stays small and gentle the whole way — even if
            # the home target is far from the current pose.
            alpha    = min(1.0, (now - t0) / self.home_seconds)
            q_target = q_start + alpha * delta_q

            try:
                self.base_feedback = self.base_cyclic.Refresh(
                    self.base_command, 0, self.send_opt)
                self._update_state(self.base_feedback)

                if self.kalman is not None:
                    q_use, dq_use, _ = self.kalman.update(self.q, self.dq, dt)
                else:
                    q_use, dq_use = self.q, self.dq

                with self._ctrl_lock:
                    self.joint_ctrl.set_target(q_target)
                    tau_cmd = self.joint_ctrl.compute_torque(q_use, dq_use, self.torque, dt)

                delta   = np.clip(tau_cmd - self._tau_cmd_prev,
                                  -self.tau_slew_per_tick, self.tau_slew_per_tick)
                tau_cmd = self._tau_cmd_prev + delta
                self._tau_cmd_prev = tau_cmd.copy()

                self.base_command.frame_id = (self.base_command.frame_id + 1) % 65536
                for i in range(self.n_dof):
                    self.base_command.actuators[i].position     = self.base_feedback.actuators[i].position
                    self.base_command.actuators[i].torque_joint = float(tau_cmd[i])
                    self.base_command.actuators[i].command_id   = self.base_command.frame_id
                if self.gripper_enable:
                    self.base_command.interconnect.gripper_command.motor_cmd[0].position = self._gripper_cmd_pct
            except Exception as e:
                self.get_logger().error(
                    f"Homing tick error: {e}. Aborting startup before main loop.")
                return False

        # Switch to the requested mode.
        with self._ctrl_lock:
            self._mode = self._initial_mode
            self._hold_home_target = False
            if self._initial_mode == ControlMode.CARTESIAN:
                # Cartesian: hold the pose actually reached (bumpless).
                self.cart_ctrl._dq_filt = self.dq.copy()
                self.cart_ctrl.init_target_from_state(self.q)
            else:
                # Joint space: keep commanding HOME (not the reached pose) so the
                # PD term keeps pulling toward home and counteracts any residual
                # sag from un-modelled end-effector mass. Steady-state error is
                # ~ tau_unmodelled / Kj, small instead of letting the arm settle
                # wherever it sagged to.
                self.joint_ctrl.set_target(home)
                self._hold_home_target = True
        self.get_logger().info(f"Homing done. Mode: {self._initial_mode.name}")
        return True

    def init_kortex_control(self) -> bool:
        """Switch to low-level torque control and seed all controllers."""
        feedback = self.base_cyclic.RefreshFeedback()

        # Compute gravity torque at current position for bumpless mode switch
        q_init = np.array([math.radians(feedback.actuators[i].position) for i in range(self.actuator_count)])
        g_init = pin.computeGeneralizedGravity(self.joint_ctrl.model, self.joint_ctrl.data, q_init)

        for x in range(self.actuator_count):
            self.base_command.actuators.add()
            self.base_command.actuators[x].flags        = 1
            self.base_command.actuators[x].position     = feedback.actuators[x].position
            self.base_command.actuators[x].torque_joint = float(g_init[x])

        # Init gripper interconnect command (position mode, hold current position).
        # Skipped entirely when gripper_enable is False (arm has no gripper) so
        # the cyclic frame carries no gripper_command at all.
        if self.gripper_enable:
            try:
                pct = feedback.interconnect.gripper_feedback.motor[0].position
            except (IndexError, AttributeError):
                pct = 0.0
            self._gripper_cmd_pct = pct
            motor_cmd = self.base_command.interconnect.gripper_command.motor_cmd.add()
            motor_cmd.motor_id = 1
            motor_cmd.position = pct
            motor_cmd.velocity = 100.0
            motor_cmd.force    = 100.0

        # Set TORQUE mode BEFORE entering LOW_LEVEL_SERVOING
        ctrl = ActuatorConfig_pb2.ControlModeInformation()
        ctrl.control_mode = ActuatorConfig_pb2.ControlMode.Value('TORQUE')
        for dev_id in range(1, self.actuator_count + 1):
            self.actuator_config.SetControlMode(ctrl, dev_id)

        srv = Base_pb2.ServoingModeInformation()
        srv.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
        self.base.SetServoingMode(srv)

        # Warm-up: send gravity torque for ~100 ms so firmware has valid commands
        # before run_control_loop() starts. Keep the feedforward position equal
        # to the latest encoder reading every tick; leaving it at the first
        # sample lets a small gravity sag look like a position-following error
        # on the worn .107 arm.
        for _ in range(100):
            q_warm = np.array([
                math.radians(feedback.actuators[i].position)
                for i in range(self.n_dof)
            ])
            g_warm = pin.computeGeneralizedGravity(
                self.joint_ctrl.model, self.joint_ctrl.data, q_warm)
            self.base_command.frame_id = (self.base_command.frame_id + 1) % 65536
            for i in range(self.n_dof):
                self.base_command.actuators[i].position = feedback.actuators[i].position
                self.base_command.actuators[i].torque_joint = float(g_warm[i])
                self.base_command.actuators[i].command_id = self.base_command.frame_id
            feedback = self.base_cyclic.Refresh(self.base_command, 0, self.send_opt)
            time.sleep(0.001)

        now_mode = self.base.GetServoingMode().servoing_mode
        now_state = self.base.GetArmState().active_state
        if now_mode != Base_pb2.LOW_LEVEL_SERVOING or now_state == Base_pb2.ARMSTATE_IN_FAULT:
            self.get_logger().error(
                f"LOW_LEVEL did not stay engaged after warm-up "
                f"(servoing={Base_pb2.ServoingMode.Name(now_mode)}, "
                f"arm={Base_pb2.ArmState.Name(now_state)}).")
            return False

        self._update_state(feedback)

        # Seed ALL controllers from actual state (bumpless startup)
        with self._ctrl_lock:
            self.cart_ctrl._dq_filt = self.dq.copy()   # seed LPF to actual velocity
            self.cart_ctrl.init_target_from_state(self.q)
            self.joint_ctrl.set_target(self.q.copy())
            if self.friction_obs is not None:
                self.friction_obs.reset(self.q, self.dq)
                # Soft-start: hold friction_scale at 0 and let _ramp_friction_scale
                # in the control loop drive it up to the target over N seconds.
                if isinstance(self.friction_obs, URDFStribeckFriction):
                    self.friction_obs.set_scale(0.0)
            if self.kalman is not None:
                self.kalman.reset(self.q, self.dq)
            # Initialise filtered state to raw so the first control tick has valid values
            self.q_filt[:]   = self.q
            self.dq_filt[:]  = self.dq
            self.ddq_filt[:] = 0.0
            # Mark the ramp start time
            self._ramp_t0 = time.monotonic()
            # Tau command history (for slew limiter) seeded at gravity torque
            self._tau_cmd_prev = np.array([float(g_init[i]) for i in range(self.actuator_count)])

        self.get_logger().info(
            f"Switched to TORQUE control – active mode: {self._mode.name}"
        )
        return True

    def _update_state(self, feedback):
        """Parse Kortex feedback (degrees) into numpy arrays (radians)."""
        for i in range(self.n_dof):
            self.q[i]      = math.radians(feedback.actuators[i].position)
            self.dq[i]     = math.radians(feedback.actuators[i].velocity)
            self.torque[i] = feedback.actuators[i].torque

        # Gripper feedback via interconnect (Robotiq 2F-85)
        # Kortex reports position as % (0 = open, 100 = closed)
        # URDF robotiq_85_left_knuckle_joint limit: 0.0 – 0.8 rad
        try:
            ic_fb = feedback.interconnect
            # Debug: log raw interconnect feedback every 500 cycles (~1 s at 500 Hz)
            if not hasattr(self, '_gripper_dbg_count'):
                self._gripper_dbg_count = 0
            self._gripper_dbg_count += 1

            # Commented out to reduce log spam, but can be re-enabled for debugging interconnect feedback issues.
            # if self._gripper_dbg_count % 500 == 1:
            #     self.get_logger().info(
            #         f"[gripper_dbg] interconnect feedback: {ic_fb}"
            #     )
            gripper_fb = ic_fb.gripper_feedback
            pct = gripper_fb.motor[0].position          # 0.0 – 100.0 %
            vel_pct = gripper_fb.motor[0].velocity      # %/s
            self.gripper_pos = (pct / 100.0) * 0.8     # → rad
            self.gripper_vel = (vel_pct / 100.0) * 0.8 # → rad/s
        except (IndexError, AttributeError):
            pass  # keep last known value if interconnect data is unavailable

    # ----------------------------------------------------------
    # Publishing
    # ----------------------------------------------------------

    def _publish_state(self):
        now = self.get_clock().now().to_msg()

        # JointState — arm (7 DOF) + gripper (robotiq_85, mimic joints included)
        # Mimic joint relations from URDF:
        #   left_inner_knuckle  = +1.0 * left_knuckle
        #   left_finger_tip     = -1.0 * left_knuckle
        #   right_knuckle       = -1.0 * left_knuckle
        #   right_inner_knuckle = -1.0 * left_knuckle
        #   right_finger_tip    = +1.0 * left_knuckle
        gp  = self.gripper_pos
        gv  = self.gripper_vel
        gripper_names = [
            'robotiq_85_left_knuckle_joint',
            'robotiq_85_left_inner_knuckle_joint',
            'robotiq_85_left_finger_tip_joint',
            'robotiq_85_right_knuckle_joint',
            'robotiq_85_right_inner_knuckle_joint',
            'robotiq_85_right_finger_tip_joint',
        ]
        gripper_positions = [ gp,  gp, -gp, -gp, -gp,  gp]
        gripper_velocities = [ gv,  gv, -gv, -gv, -gv,  gv]

        js              = JointState()
        js.header.stamp = now
        js.name         = self.joint_names #+ gripper_names
        js.position     = self.q.tolist()  #   + gripper_positions
        js.velocity     = self.dq.tolist() #   + gripper_velocities
        js.effort       = self.torque.tolist() #+ [0.0] * len(gripper_names)
        self.pub_joint.publish(js)

        # Current target — publish ทุก mode
        if self._mode == ControlMode.CARTESIAN:
            with self._ctrl_lock:
                target = self.cart_ctrl.target_SE3
                target_copy = target.copy() if target is not None else None

            if target_copy is not None:
                ps                    = PoseStamped()
                ps.header.stamp       = now
                ps.header.frame_id    = "base_link"
                ps.pose.position.x    = float(target_copy.translation[0])
                ps.pose.position.y    = float(target_copy.translation[1])
                ps.pose.position.z    = float(target_copy.translation[2])
                q_pin                 = pin.Quaternion(target_copy.rotation)
                ps.pose.orientation.x = float(q_pin.x)
                ps.pose.orientation.y = float(q_pin.y)
                ps.pose.orientation.z = float(q_pin.z)
                ps.pose.orientation.w = float(q_pin.w)
                self.pub_target.publish(ps)

        elif self._mode == ControlMode.JOINT_SPACE:
            with self._ctrl_lock:
                q_des = self.joint_ctrl.q_des.copy()

            jt                = JointState()
            jt.header.stamp   = now
            jt.name           = self.joint_names
            jt.position       = q_des.tolist()
            self.pub_joint_target.publish(jt)

        # Estimated external wrench (always from Cartesian controller — diagnostic).
        # Pass dq_filt so the friction-aware correction can subtract τ_friction(q̇).
        f_ext_raw          = self.cart_ctrl.estimate_wrench(self.q, self.torque, self.dq_filt)

        # Accumulate samples if a /zero_wrench was just requested
        if self._wrench_zero_collecting:
            self._wrench_zero_buffer.append(f_ext_raw.copy())
            if len(self._wrench_zero_buffer) >= self._WRENCH_ZERO_N:
                self.wrench_offset = np.mean(self._wrench_zero_buffer, axis=0)
                self._wrench_zero_collecting = False
                self._wrench_zero_buffer = []
                self.get_logger().info(
                    f"Wrench tared. Offset (Fxyz, Txyz) = "
                    f"({self.wrench_offset[0]:+.3f}, {self.wrench_offset[1]:+.3f}, "
                    f"{self.wrench_offset[2]:+.3f}, {self.wrench_offset[3]:+.3f}, "
                    f"{self.wrench_offset[4]:+.3f}, {self.wrench_offset[5]:+.3f})"
                )

        f_ext              = f_ext_raw - self.wrench_offset
        ws                 = WrenchStamped()
        ws.header.stamp    = now
        ws.header.frame_id = "tool_frame"
        ws.wrench.force.x  = float(f_ext[0])
        ws.wrench.force.y  = float(f_ext[1])
        ws.wrench.force.z  = float(f_ext[2])
        ws.wrench.torque.x = float(f_ext[3])
        ws.wrench.torque.y = float(f_ext[4])
        ws.wrench.torque.z = float(f_ext[5])
        self.pub_wrench.publish(ws)

    # ----------------------------------------------------------
    # Control loop
    # ----------------------------------------------------------

    def run_control_loop(self):
        """
        Real-time control loop at `control_hz`.

        Timing: hybrid sleep + busy-wait (same strategy as v4):
          - sleep(remaining - BUSY_WAIT_MARGIN) to yield CPU
          - busy-wait the last 0.8 ms for precision

        To hit 1 ms cleanly, secondary work is throttled:
          - /joint_states + /estimated_wrench → every 4 ticks (≈250 Hz)
          - /control_diagnostics              → every 4 ticks (≈250 Hz)
          - debug print                       → every 1000 ticks (≈1 Hz)
        Only Refresh + compute + send-torque runs every tick.
        """
        BUSY_WAIT_MARGIN = 0.0008  # 0.8 ms

        self.get_logger().info("Control loop started.")
        pre_time = time.monotonic()
        _cycle = 0
        _DEBUG_EVERY     = 1000     # ≈1 Hz at 1 kHz
        _PUBLISH_EVERY   = 4        # ≈250 Hz — joint_states + diag + wrench
        _tau_cmd_last = np.zeros(self.n_dof)  # hold-last torque on refresh failure

        try:
            while self.running and rclpy.ok():
                now     = time.monotonic()
                elapsed = now - pre_time

                # Phase 1: sleep bulk of remaining time
                remaining  = self.control_dt - elapsed
                sleep_time = remaining - BUSY_WAIT_MARGIN
                if sleep_time > 0:
                    time.sleep(sleep_time)

                # Phase 2: busy-wait to precise deadline
                while time.monotonic() - pre_time < self.control_dt:
                    pass

                now      = time.monotonic()
                dt       = now - pre_time
                pre_time = now

                # Cap dt to 3× nominal to prevent large filter jumps on
                # occasional OS scheduling hiccups (does not affect steady-state)
                dt = min(dt, 3.0 * self.control_dt)

                # Refresh sensor feedback
                try:
                    _t_refresh_start = time.monotonic()
                    self.base_feedback = self.base_cyclic.Refresh(
                        self.base_command, 0, self.send_opt
                    )
                    _t_refresh_ms = (time.monotonic() - _t_refresh_start) * 1000.0
                    self._update_state(self.base_feedback)

                    # ── Velocity watchdog ─────────────────────────────────
                    # If any joint is screaming, drop to gravity-only after
                    # `velocity_watchdog_max` consecutive bad ticks.
                    if np.any(np.abs(self.dq) > self.velocity_watchdog_lim):
                        self._velocity_watchdog_count += 1
                        if self._velocity_watchdog_count == self.velocity_watchdog_max:
                            self.get_logger().error(
                                f"VELOCITY WATCHDOG TRIPPED — |dq|>{self.velocity_watchdog_lim} rad/s for "
                                f"{self.velocity_watchdog_max} ticks. Dropping to gravity-only."
                            )
                            self._safe_mode = True
                    else:
                        self._velocity_watchdog_count = 0

                    # ── Friction soft-start ramp ──────────────────────────
                    # Linearly drive scale 0 → target over `friction_ramp_seconds`.
                    if isinstance(self.friction_obs, URDFStribeckFriction) and self.friction_ramp_seconds > 0:
                        ramp_t = time.monotonic() - self._ramp_t0
                        scale_now = min(1.0, ramp_t / self.friction_ramp_seconds) * self._friction_scale_target
                        self.friction_obs.set_scale(scale_now)

                    # Online Kalman: smooth q, q̇ and estimate q̈ before
                    # passing to the friction model + controllers. Reduces
                    # Stribeck tanh chatter near zero velocity.
                    if self.kalman is not None:
                        q_use, dq_use, ddq_use = self.kalman.update(self.q, self.dq, dt)
                    else:
                        q_use, dq_use, ddq_use = self.q, self.dq, np.zeros(self.n_dof)
                    self.q_filt[:]   = q_use
                    self.dq_filt[:]  = dq_use
                    self.ddq_filt[:] = ddq_use

                    # ── Compute torque (or gravity-only if safe-mode) ────
                    with self._ctrl_lock:
                        if self._safe_mode:
                            # Gravity-only fallback — no PD, no friction, no impedance
                            tau_cmd = pin.computeGeneralizedGravity(
                                self.joint_ctrl.model, self.joint_ctrl.data, q_use)
                            self.joint_ctrl._g_cached = tau_cmd
                        elif self._mode == ControlMode.JOINT_SPACE:
                            tau_cmd = self.joint_ctrl.compute_torque(q_use, dq_use, self.torque, dt)
                        else:
                            tau_cmd = self.cart_ctrl.compute_torque(q_use, dq_use, self.torque, dt)

                    # ── Torque slew rate limit (last line of defence) ────
                    # Caps |Δτ| per tick. With default 0.5 Nm at 1 kHz that's
                    # 500 Nm/s — fast enough for control, slow enough to never
                    # be a step input on the actuator.
                    delta = tau_cmd - self._tau_cmd_prev
                    delta = np.clip(delta, -self.tau_slew_per_tick, self.tau_slew_per_tick)
                    tau_cmd = self._tau_cmd_prev + delta
                    self._tau_cmd_prev = tau_cmd.copy()

                    # ── Defensive NaN scrub (if anything upstream blew up) ─
                    if not np.all(np.isfinite(tau_cmd)):
                        self.get_logger().error(
                            f"NaN/Inf in tau_cmd: {tau_cmd}. Holding last good torque.")
                        tau_cmd = _tau_cmd_last

                    _tau_cmd_last = tau_cmd

                except Exception as e:
                    err_str = str(e)
                    tau_cmd = _tau_cmd_last  # hold last torque to avoid drop

                    if 'WRONG_SERVOING_MODE' in err_str:
                        # Robot kicked itself out of LOW_LEVEL_SERVOING — almost
                        # always due to a fault on the robot side (over-velocity,
                        # over-torque, joint limit, or external e-stop). Try to
                        # recover automatically, rate-limited to once / 2 s.
                        now_w = time.monotonic()
                        if now_w - getattr(self, '_last_recovery_t', 0.0) > 2.0:
                            self._last_recovery_t = now_w
                            self.get_logger().warn(
                                "Servoing-mode mismatch detected. "
                                "Likely cause: robot fault (joint vel/torque/limit "
                                "exceeded, or e-stop). Attempting auto-recovery..."
                            )
                            try:
                                # Clear any latched fault first
                                if self.base.GetArmState().active_state == Base_pb2.ARMSTATE_IN_FAULT:
                                    self.base.ClearFaults()
                                    time.sleep(0.3)

                                # Re-enter LOW_LEVEL_SERVOING
                                _srv = Base_pb2.ServoingModeInformation()
                                _srv.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
                                self.base.SetServoingMode(_srv)
                                time.sleep(0.05)

                                # Reseed controllers + filters from current state
                                fb_recov = self.base_cyclic.RefreshFeedback()
                                for i in range(self.n_dof):
                                    self.q[i]  = math.radians(fb_recov.actuators[i].position)
                                    self.dq[i] = math.radians(fb_recov.actuators[i].velocity)
                                with self._ctrl_lock:
                                    if self._mode == ControlMode.JOINT_SPACE:
                                        if self._hold_home_target:
                                            self.joint_ctrl.set_target(
                                                np.array(self.home_joint_angles, dtype=float))
                                        else:
                                            self.joint_ctrl.set_target(self.q.copy())
                                    else:
                                        self.cart_ctrl.init_target_from_state(self.q)
                                        self.cart_ctrl._dq_filt = self.dq.copy()
                                    if self.friction_obs is not None:
                                        self.friction_obs.reset(self.q, self.dq)
                                        if isinstance(self.friction_obs, URDFStribeckFriction):
                                            self.friction_obs.set_scale(0.0)
                                            self._ramp_t0 = time.monotonic()
                                    if self.kalman is not None:
                                        self.kalman.reset(self.q, self.dq)
                                self._velocity_watchdog_count = 0
                                self._safe_mode = False
                                self.get_logger().info("Recovery succeeded.")
                            except Exception as recov_e:
                                self.get_logger().error(
                                    f"Recovery failed: {recov_e}. "
                                    "You may need to restart the controller "
                                    "(robot may have a hard fault)."
                                )
                    else:
                        self.get_logger().warn(
                            f"Cyclic refresh error [{type(e).__name__}]: {e}"
                        )

                _cycle += 1

                # --- Debug print every _DEBUG_EVERY cycles (≈1 Hz) -------
                if _cycle % _DEBUG_EVERY == 0:
                    ctrl = self.joint_ctrl if self._mode == ControlMode.JOINT_SPACE else self.cart_ctrl
                    fobs = self.friction_obs

                    # Compute per-term breakdown for diagnosis
                    g_vec  = ctrl._g_cached if ctrl._g_cached is not None else np.zeros(self.n_dof)
                    if self._mode == ControlMode.JOINT_SPACE:
                        pd_term = tau_cmd - g_vec  # approximate (before friction add)
                    else:
                        pd_term = np.zeros(self.n_dof)

                    # URDF Stribeck friction breakdown (None if friction disabled)
                    fric_comp_coul = fric_comp_visc = fric_comp_strib = fric_comp_off = None
                    if fobs is not None:
                        tau_fric = fobs.compute(self.q_filt, self.dq_filt,
                                                 np.zeros(self.n_dof), dt)
                        sgn   = np.tanh(self.dq_filt / fobs.eps)
                        fric_comp_coul  = fobs.scale * fobs.phi_f[:, 0] * sgn
                        fric_comp_visc  = fobs.scale * fobs.phi_f[:, 1] * self.dq_filt
                        fric_comp_strib = fobs.scale * fobs.phi_f[:, 2] * np.exp(
                            -np.abs(self.dq_filt) / fobs.vs) * sgn
                        fric_comp_off   = (fobs.scale * fobs.phi_f[:, 3]
                                            if fobs.use_offset else np.zeros(self.n_dof))
                    else:
                        tau_fric = np.zeros(self.n_dof)

                    np.set_printoptions(precision=3, suppress=True)
                    print("=" * 60)
                    print(f"[DEBUG] mode={self._mode.name}  dt={dt*1000:.2f}ms  "
                          f"(Kortex Refresh={_t_refresh_ms:.2f}ms)")
                    print(f"  q          (rad): {self.q}")
                    print(f"  dq         (r/s): {self.dq}")
                    print(f"  tau_sensor  (Nm): {self.torque}")
                    print(f"  tau_sen_neg (Nm): {-self.torque}  <- compare with gravity")
                    print(f"  tau_cmd     (Nm): {tau_cmd}")
                    print(f"  gravity     (Nm): {g_vec}")
                    print(f"  tau_friction(Nm): {tau_fric}")
                    # tau_error = (tau_cmd − gravity) + (tau_sensor + gravity)
                    # ≈ 0 when robot is still (Kortex sensor reads reaction torque,
                    # which gravity should counter).
                    tau_err = tau_cmd + self.torque
                    print(f"  tau_sensor      : {self.torque}")
                    print(f"  tau_error(~)    : {tau_err}")
                    if fobs is not None:
                        print(f"  fric Coulomb    : {fric_comp_coul}")
                        print(f"  fric viscous    : {fric_comp_visc}")
                        print(f"  fric Stribeck   : {fric_comp_strib}")
                        print(f"  fric offset     : {fric_comp_off}")
                    print("=" * 60, flush=True)

                # --- Throttled publishes (every _PUBLISH_EVERY ticks ≈250 Hz) ─
                # /control_diagnostics carries the full controller state for
                # offline analysis. Layout (each field = 4 channels × 7 joints):
                #
                #   name[12]      = labels for the 12 channels
                #   position[28]  = q_filt | dq_filt | ddq_filt | gravity
                #   velocity[28]  = tau_meas | tau_cmd | tau_fric_total | fric_coul
                #   effort[28]    = fric_visc | fric_strib | fric_off | tau_error
                #
                # tau_error = tau_cmd + tau_sensor (≈ 0 when calibration is good
                # and there is no external force).
                if _cycle % _PUBLISH_EVERY == 0:
                    nj = self.n_dof
                    gvec = (self.joint_ctrl._g_cached
                            if self.joint_ctrl._g_cached is not None
                            else np.zeros(nj))
                    if self.friction_obs is not None:
                        f = self.friction_obs
                        sgn   = np.tanh(self.dq_filt / f.eps)
                        f_coul  = f.scale * f.phi_f[:, 0] * sgn
                        f_visc  = f.scale * f.phi_f[:, 1] * self.dq_filt
                        f_strib = f.scale * f.phi_f[:, 2] * np.exp(
                            -np.abs(self.dq_filt) / f.vs) * sgn
                        f_off   = (f.scale * f.phi_f[:, 3]
                                    if f.use_offset else np.zeros(nj))
                        f_total = f_coul + f_visc + f_strib + f_off
                    else:
                        f_coul = f_visc = f_strib = f_off = f_total = np.zeros(nj)
                    tau_error = tau_cmd + self.torque

                    diag = JointState()
                    diag.header.stamp = self.get_clock().now().to_msg()
                    diag.name = [
                        'q_filt', 'dq_filt', 'ddq_filt', 'gravity',
                        'tau_meas', 'tau_cmd', 'tau_fric_total', 'fric_coul',
                        'fric_visc', 'fric_strib', 'fric_off', 'tau_error',
                    ]
                    diag.position = (list(self.q_filt) + list(self.dq_filt)
                                      + list(self.ddq_filt) + list(gvec))
                    diag.velocity = (list(self.torque) + list(tau_cmd)
                                      + list(f_total) + list(f_coul))
                    diag.effort   = (list(f_visc) + list(f_strib)
                                      + list(f_off) + list(tau_error))
                    self.pub_diag.publish(diag)

                # Send torque command to Kortex
                self.base_command.frame_id = (self.base_command.frame_id + 1) % 65536
                for i in range(self.n_dof):
                    # Feedforward position stays as encoder reading (degrees)
                    # This prevents "following error" faults — do NOT write q_des here
                    self.base_command.actuators[i].position     = self.base_feedback.actuators[i].position
                    self.base_command.actuators[i].torque_joint = float(tau_cmd[i])
                    self.base_command.actuators[i].command_id   = self.base_command.frame_id

                # Gripper position command via cyclic interconnect (only if this
                # arm has a gripper; otherwise no motor_cmd was ever added).
                if self.gripper_enable:
                    self.base_command.interconnect.gripper_command.motor_cmd[0].position = self._gripper_cmd_pct

                # Non-blocking ROS spin (handles callbacks) — every tick (cheap)
                rclpy.spin_once(self, timeout_sec=0)

                # /joint_states + /estimated_wrench at 250 Hz (every 4th tick).
                # estimate_wrench duplicates Jacobian computation that compute_torque
                # already did → expensive (~300 μs). Throttling here is the biggest
                # win for hitting 1 ms.
                if _cycle % _PUBLISH_EVERY == 0:
                    self._publish_state()

        except KeyboardInterrupt:
            pass
        finally:
            self.stop_control()

    def stop_control(self):
        """
        Switch back to POSITION control + SINGLE_LEVEL servoing (safe exit).

        Uses print() (NOT the ROS logger) and verifies the servoing switch with
        retries, because on Ctrl-C rclpy tears the context down first (the
        logger then fails with "publisher's context is invalid") AND this method
        is called from two finally blocks. A *verified* exit matters: if the
        arm is left in LOW_LEVEL by a silently-failed switch, the NEXT run can't
        re-enter low-level and needs a power-cycle (the .107 symptom). Idempotent
        — running it twice just re-confirms.
        """
        if getattr(self, '_stopped_clean', False):
            return
        print("Shutting down – switching to POSITION / SINGLE_LEVEL...", flush=True)
        self.running = False

        # Actuators → POSITION (hold current position) before leaving low level.
        ctrl = ActuatorConfig_pb2.ControlModeInformation()
        ctrl.control_mode = ActuatorConfig_pb2.ControlMode.Value('POSITION')
        for dev_id in range(1, self.actuator_count + 1):
            try:
                self.actuator_config.SetControlMode(ctrl, dev_id)
            except Exception:
                pass

        # Servoing → SINGLE_LEVEL, verified, with retries.
        for attempt in range(5):
            try:
                srv = Base_pb2.ServoingModeInformation()
                srv.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
                self.base.SetServoingMode(srv)
                time.sleep(0.1)
                if self.base.GetServoingMode().servoing_mode == Base_pb2.SINGLE_LEVEL_SERVOING:
                    self._stopped_clean = True
                    print("Clean exit — arm verified in SINGLE_LEVEL.", flush=True)
                    return
            except Exception as e:
                print(f"stop_control retry {attempt+1}/5: {e}", flush=True)
            time.sleep(0.2)

        print("WARNING: could not confirm SINGLE_LEVEL on exit — "
              "the next run may need a power-cycle.", flush=True)


# ============================================================
#  Entry point
# ============================================================

def main():
    parser = utilities.argparse.ArgumentParser()
    # Connection args are plain CLI flags (NOT ROS params), parsed before
    # rclpy sees the remaining --ros-args. Each arm instance passes its own
    # --ip so two controllers can talk to two different robots.
    parser.add_argument("--ip",       type=str, default="192.180.0.200",
                        help="IP address of this Kinova arm")
    parser.add_argument("--username", type=str, default="admin")
    parser.add_argument("--password", type=str, default="admin")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    try:
        with utilities.DeviceConnection.createTcpConnection(args) as router:
            with utilities.DeviceConnection.createUdpConnection(args) as router_rt:
                node = RealRobotImpedance(router, router_rt)

                # Robust startup that does NOT depend on a clean previous exit
                # and does NOT use the fragile high-level "Home" action:
                #   1) recover_servoing_mode  – safe mode reset (no power-cycle),
                #      unsticks an arm left in LOW_LEVEL by a prior crash.
                #   2) clear_faults
                #   3) init_kortex_control    – enter TORQUE / LOW_LEVEL, seed
                #      controllers from current pose (bumpless).
                #   4) home_with_impedance    – compliantly drive to home, then
                #      switch to the requested initial mode.
                if not node.recover_servoing_mode():
                    return
                if not node.clear_faults():
                    return
                if not node.init_kortex_control():
                    # init_kortex_control may already have switched actuators to
                    # TORQUE before discovering that LOW_LEVEL did not stay
                    # engaged, so always put the arm back into a holding mode.
                    node.stop_control()
                    return

                # From here the arm is in TORQUE / LOW_LEVEL. Guarantee that
                # stop_control() runs on ANY exit (Ctrl-C during homing OR the
                # main loop, or any error) so the arm switches back to POSITION
                # mode and HOLDS its pose instead of going limp and dropping.
                try:
                    if not node.home_with_impedance():
                        return
                    node.run_control_loop()
                finally:
                    node.stop_control()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Connection failed: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
