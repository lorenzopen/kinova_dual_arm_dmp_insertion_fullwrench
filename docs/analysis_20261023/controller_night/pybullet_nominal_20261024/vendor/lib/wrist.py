"""
Virtual coupling at the wrist: a translational spring and an ANISOTROPIC
rotational one, both applied to the tool base.

Same as the C++ rig (`13-..._torsional.cpp:808`, `compute_PD`) except the
rotational spring is anisotropic in the tool frame instead of isotropic:

    rotation about local x   = TORSION      -> STIFF (k_tors = 40)
    rotation about local y,z = TILT of the pad plane -> COMPLIANT (k_tilt)

The pad plane's normal is the tool's local x, so those two channels are
orthogonal: softening tilt does not touch torsion. A single scalar gain would
soften both, and torsion is the load the three points exist to resist.
"""
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as Rot

# Rig gains (exp13, sessions B and C). NOT free knobs: the human prior was
# recorded with kpr = 40 and kdr = 1.15, so changing them makes that data a
# description of a different tool.
KP, KD = 1500.0, 15.0          # translational spring [N/m] and damper [Ns/m]
KPR, KDR = 40.0, 1.15          # torsional spring [Nm/rad] and damper

# Tilt compliance: softer while seating the pads, firmer while holding.
K_TILT_SEAT, K_TILT_HOLD = 3.0, 8.0

# Opt-in pipeline feedback. None preserves the reference command path.
WRENCH_CONTROLLER = None


def quat_from_x(xdir, roll=0.0):
    """Tool orientation with local x (the push direction) along `xdir`.

    Roll about that axis has period 120 deg (the triangle has 3-fold symmetry).
    Measured at fixed roll on 8 objects: no detectable effect, so it stays 0
    and is not a search dimension.
    """
    x = np.asarray(xdir, float)
    x = x / np.linalg.norm(x)
    a = np.array([0.0, 0.0, 1.0])          # any vector not parallel to x
    if abs(a @ x) > 0.9:
        a = np.array([0.0, 1.0, 0.0])
    y = np.cross(a, x)
    y /= np.linalg.norm(y)
    q = Rot.from_matrix(np.stack([x, y, np.cross(x, y)], axis=1))
    return Rot.from_rotvec(roll * x) * q if roll else q


def vc(tool, target, q_des, k_tilt, kp=KP, kd=KD, kpr=KPR, kdr=KDR,
       gravity_comp=True):
    """One virtual-coupling step on the tool BASE.

    `target` is the commanded base position, `q_des` the commanded orientation.
    `k_tilt` is the only gain the caller changes between seating and holding.

    Gravity compensation uses the tool's REAL mass: with pad springs the pads
    have mass of their own, and compensating only the base would leave a steady
    droop that reads as "the springs are yielding".
    """
    if WRENCH_CONTROLLER is not None:
        WRENCH_CONTROLLER.k_tilt = float(k_tilt)
        target, q_des = WRENCH_CONTROLLER.command(tool, target, q_des)
    pos, ori = tool.pose()
    v, w = p.getBaseVelocity(tool.id)
    remote = getattr(tool, "remote_impedance", None)
    if remote is not None:
        F, T, _energy = remote.evaluate(pos, ori, v, w, target, q_des,
                                       kp, kd, kpr, k_tilt, kdr)
        if gravity_comp:
            F = F + np.array([0.0, 0.0, tool.mass * 9.81])
        p.applyExternalForce(tool.id, -1, F.tolist(), pos.tolist(), p.WORLD_FRAME)
        p.applyExternalTorque(tool.id, -1, T.tolist(), p.WORLD_FRAME)
        return
    F = kp * (np.asarray(target, float) - pos) - kd * np.asarray(v, float)
    if gravity_comp:
        F = F + np.array([0.0, 0.0, tool.mass * 9.81])

    Rw = Rot.from_quat(ori)
    M = Rw.as_matrix()                            # columns = local axes in world
    e_local = M.T @ (q_des * Rw.inv()).as_rotvec()   # orientation error, tool frame
    T = M @ (np.array([kpr, k_tilt, k_tilt]) * e_local) - kdr * np.asarray(w, float)

    p.applyExternalForce(tool.id, -1, F.tolist(), pos.tolist(), p.WORLD_FRAME)
    p.applyExternalTorque(tool.id, -1, T.tolist(), p.WORLD_FRAME)


def tilt_deg(tool, q_des):
    """[deg] how far the REAL push axis has drifted from the commanded one.
    This is the CLAUDE.md 7.2 variable: with the rigid tool it runs away as you
    squeeze harder."""
    x_cmd = q_des.as_matrix()[:, 0]
    x_now = Rot.from_quat(tool.pose()[1]).as_matrix()[:, 0]
    return float(np.degrees(np.arccos(np.clip(x_cmd @ x_now, -1.0, 1.0))))
