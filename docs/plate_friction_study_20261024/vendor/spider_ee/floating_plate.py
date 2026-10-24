"""
Flange (with 3 cylindrical supports) -> Strong 6-DoF Central Spring ->
TRIANGULAR PLATE (welded directly to the centers of the 3 spheres) +
3 Zero-Length Springs (between the tips of the cylindrical supports and the sphere centers).

Topology:
  * FLANGE: base mount at x = 0 with 3 cylindrical supports extending to x = PAD_X
    at each pad location (y_i, z_i). The tip of each cylinder is the spring anchor.
  * TRIANGULAR PLATE + 3 SPHERES: ONE single rigid body. The triangular plate
    joins the 3 sphere centers (y_i, z_i) at x = PAD_X, and the 3 contact spheres
    are rigidly welded (fixed joints) directly to the vertices/centers of the plate.
  * CENTRAL SPRING: 6-DoF strong spring connecting the flange center (x = 0)
    to the triangular plate center (x = PAD_X).
  * 3 ZERO-LENGTH SPRINGS (l0 = 0): connect the tip of each flange cylinder
    support to the center of each sphere.
"""
import argparse
import os
import sys
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as Rot

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
from lib.tool import PADS_LOCAL, PAD_X, PAD_R, R_TRI, BASE_MASS, scratch as SCRATCH   # noqa: E402


# Moved here from the old `spider.py` when the unused tools were removed: this
# was the only thing anything still imported from it.
_TRAVEL_A = np.column_stack([np.ones(3), PADS_LOCAL[:, 2], -PADS_LOCAL[:, 1]])


def tilt_from_travel(q):
    """[deg] the pad plane's own rotation, read from the three axial travels.

    Works for either mechanism - the rail through its three prismatic joints,
    the plate through the frame turning on its springs - so the same column
    compares them instead of flattering whichever one it was written for.
    """
    sol = np.linalg.solve(_TRAVEL_A, np.asarray(q, float))
    return float(np.degrees(np.linalg.norm(sol[1:])))

G = 9.81
PLATE_X = PAD_X                       # [m] plate mid-plane, 20 mm in front
PLATE_T = 0.004                       # [m] plate thickness
PLATE_MASS = 0.080                    # [kg] triangular plate mass
PAD_MASS = 0.020                      # [kg] mass of each sphere
K_STOP = 10000.0                      # end-of-travel penalty if needed
CAP = 0.5                             # integrator stability damping cap
# Does the PLATE collide with the object, or only the three spheres?
# MEASURED consequence of True, on the jittered cubes: where the plate touches,
# it carries load that `lib.tool.wrench_on_body` does not count (it asks the
# three sphere links), the net contact force stops balancing gravity, and the
# estimator rejects every sample. Object 1: one plate contact carrying 10.7 N
# of 64.2, net force 34.7 deg off vertical, 2050/2050 samples rejected, mass
# stuck at the 3.0 kg prior. Object 0, no plate contact: 2050/2050 accepted,
# mass error 0.0%, CoM 1.7 mm.
PLATE_COLLISION = True


def plate_inertia(plate_mass=PLATE_MASS):
    """Inertia of the triangular plate about its own centre, sampled.

    NOT a round number: with 0.025 (the value an override was forcing) the tilt
    mode is sqrt(K_tilt/I) = 5.2 rad/s, i.e. the tool needs over a SECOND to
    conform - during a normal push it just looks rigid and the wrist takes the
    misalignment. The honest value is ~7e-5 and it conforms in 60 ms.
    """
    w = np.linspace(0.0, 1.0, 30)
    P = np.array([[0.0, y, z] for _x, y, z in PADS_LOCAL])
    pts = np.array([a * P[0] + b * P[1] + (1 - a - b) * P[2]
                    for a in w for b in w if a + b <= 1.0])
    m = plate_mass / len(pts)
    I = np.zeros((3, 3))
    for q in pts:
        I += m * ((q @ q) * np.eye(3) - np.outer(q, q))
    return I


def _series(a, b):
    """Two springs in series. `inf` means rigid."""
    if a <= 0 or b <= 0:
        return 0.0
    return 1.0 / (1.0 / a + 1.0 / b)


class PadSprings:
    """The three zero-length springs, between each flange cylinder tip and its sphere center."""

    def __init__(self, k_push=500.0, k_lateral=500.0, travel=(-0.008, 0.008),
                 zeta=0.7, dt=0.001):
        # k_lateral 500, not 15000. MEASURED: at 15000 the three springs put
        # 45000 N/m on a 37 g reduced mass, omega*dt = 1.11, and with a FREE
        # flange the pair diverges - driven to the same target, spider.py holds
        # (-0.025, 0, 0.050) exactly while this one ended up on the floor at
        # (-0.075, 0, -0.001). Torsion is then carried by the central spring,
        # which is what it is there for.
        self.dt = float(dt)
        self.k_push = float(k_push)
        self.k_lateral = float(k_lateral)
        self.travel = (float(travel[0]), float(travel[1]))
        # MEASURED rule. Each spring gets 0.7 of critical on the pair's
        # REDUCED mass, capped at half the explicit integrator's limit.
        # Do NOT divide by three: measured on the pinned-flange bench with a
        # 10 N side load, k_lat = 15000,
        #     c_lat 6.1  -> diverges (dy runs to -192 mm)
        #     c_lat 0    -> diverges
        #     c_lat 18.4 -> stable, dy = 0.22 mm against 0.22 predicted
        # 18.4 is exactly the cap, i.e. the undivided value.
        m_red = (3 * PAD_MASS + PLATE_MASS) * BASE_MASS / (3 * PAD_MASS + PLATE_MASS + BASE_MASS)
        cap = CAP * m_red / dt
        self.c_push = min(2 * zeta * np.sqrt(self.k_push * m_red), cap)
        self.c_lat = min(2 * zeta * np.sqrt(max(self.k_lateral, 0.0) * m_red), cap)
        self.m_red = m_red
        self.c_stop = min(2 * zeta * np.sqrt(K_STOP * m_red), cap)

    @property
    def K_local(self):
        return np.array([self.k_push, self.k_lateral, self.k_lateral])

    @property
    def C_local(self):
        return np.array([self.c_push, self.c_lat, self.c_lat])

    @property
    def k_axial(self):
        return 3.0 * self.k_push

    @property
    def k_lat_total(self):
        return 3.0 * self.k_lateral

    @property
    def k_tilt(self):
        return 1.5 * self.k_push * R_TRI ** 2

    @property
    def k_torsion(self):
        return 3.0 * self.k_lateral * R_TRI ** 2

    def force_at_stop(self):
        """[N] squeeze that would use up the NOMINAL range. With no end stop
        this is no longer a wall the tool hits, only the force at which the
        excursion leaves the range the device was sized for."""
        return 3.0 * self.k_push * min(abs(self.travel[0]), abs(self.travel[1]))

    def absorbed_tilt_deg(self):
        c = min(abs(self.travel[0]), abs(self.travel[1]))
        return float(np.degrees(np.arcsin(np.clip(c / R_TRI, 0.0, 1.0))))

    def describe(self):
        return (f"pads: k_push {self.k_push:.0f}, k_lat {self.k_lateral:.0f} N/m, "
                f"stroke +-{1000 * self.travel[1]:.0f} mm -> "
                f"axial {self.k_axial:.0f}, tilt {self.k_tilt:.2f} Nm/rad")


class CentralSpring:
    """The strong 6-DoF spring, flange -> plate center."""

    def __init__(self, k_push=20000.0, k_lateral=20000.0, k_torsion=40.0,
                 k_tilt=0.0, zeta=0.7, dt=0.001):
        # k_tilt STAYS ZERO. Everything else may be strong: with the springs in
        # parallel, any tilt stiffness here covers the pads' 0.675 sixty times
        # over and the tool stops conforming. The other three are what keep the
        # frame from sinking, sliding and twisting on its mount.
        self.k_lin = np.array([float(k_push), float(k_lateral), float(k_lateral)])
        self.k_rot = np.array([float(k_torsion), float(k_tilt), float(k_tilt)])
        total_m = PLATE_MASS + 3 * PAD_MASS
        m_red = BASE_MASS * total_m / (BASE_MASS + total_m)
        # reduced INERTIA of the pair, with the plate's real value - not 0.025
        I_a = np.diag(plate_inertia()) + 3 * PAD_MASS * R_TRI ** 2
        I_red = 1.0 / (1.0 / 0.05 + 1.0 / I_a)
        self.c_lin = np.minimum(2 * zeta * np.sqrt(self.k_lin * m_red),
                                CAP * m_red / dt)
        self.c_rot = np.minimum(2 * zeta * np.sqrt(self.k_rot * I_red),
                                CAP * I_red / dt)
        self.m_red, self.I_red = m_red, I_red

    def describe(self):
        return (f"centre: push {self.k_lin[0]:.0f}, lat {self.k_lin[1]:.0f} N/m, "
                f"tors {self.k_rot[0]:.1f}, tilt {self.k_rot[1]:.1f} Nm/rad")


def stiffness_6x6(pads, centre):
    """[6x6] about the plate centre, tool frame. Same formula as `spider.py`:
    the mechanism is the same, so the matrix has to be too."""
    def skew(v):
        return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    K = np.zeros((6, 6))
    K[:3, :3] += np.diag(centre.k_lin)
    K[3:, 3:] += np.diag(centre.k_rot)
    Ki = np.diag(pads.K_local)
    for c in PADS_LOCAL - np.array([PAD_X, 0.0, 0.0]):     # anchors about the plate
        B = np.hstack([np.eye(3), -skew(c)])
        K += B.T @ Ki @ B
    return K


def channels(pads, centre):
    """Read as COMPLIANCES, so the off-diagonal coupling is included. Adding the
    diagonals instead over-reports every channel."""
    C = np.linalg.inv(stiffness_6x6(pads, centre))
    return dict(axial=1.0 / C[0, 0], lateral=1.0 / C[1, 1],
                tilt=1.0 / C[4, 4], torsion=1.0 / C[3, 3])


def stability(pads, centre, dt=0.001):
    """omega*dt per channel on the reduced mass. Explicit integrator: stable
    below 2, quiet below 1."""
    K = np.diag(stiffness_6x6(pads, centre))
    scale = np.concatenate([centre.m_red * np.ones(3), centre.I_red])
    return dict(zip(["dx", "dy", "dz", "rx", "ry", "rz"], np.sqrt(K / scale) * dt))


# =========================================================================
#  GEOMETRY & URDF GENERATION
# =========================================================================
def _prism(outline, x0, x1):
    n = len(outline)
    V = ([[x0, float(y), float(z)] for y, z in outline]
         + [[x1, float(y), float(z)] for y, z in outline])
    T = []
    for k in range(1, n - 1):
        T += [0, k, k + 1, n, n + k + 1, n + k]
    for k in range(n):
        a, b = k, (k + 1) % n
        T += [a, n + a, n + b, a, n + b, b]
    return V, T


def _flange_shapes():
    """Flange body: base disc at x=0 plus 3 cylindrical supports extending from x=0 to x=PAD_X."""
    theta_base = np.linspace(0, 2 * np.pi, 16, endpoint=False)
    V, T = _prism([(0.012 * np.cos(t), 0.012 * np.sin(t)) for t in theta_base], -0.004, 0.0)

    def add(outline, x0, x1):
        nonlocal V, T
        Vs, Ts = _prism(outline, x0, x1)
        T = T + [i + len(V) for i in Ts]
        V = V + Vs

    rod_r = 0.003
    theta = np.linspace(0, 2 * np.pi, 12, endpoint=False)
    for _x, ys, zs in PADS_LOCAL:
        circle = [(ys + rod_r * np.cos(t), zs + rod_r * np.sin(t)) for t in theta]
        add(circle, 0.0, PAD_X)

    return p.createVisualShape(p.GEOM_MESH, vertices=V, indices=T,
                               rgbaColor=[0.2, 0.7, 0.9, 1.0])


def _write_plate_mesh(obj_path):
    """Triangular plate mesh centered at origin, joining the three sphere centers (y_i, z_i)."""
    V, F = [], []
    corners = [(float(y), float(z)) for _x, y, z in PADS_LOCAL]
    
    def quad(a, b, c, d):
        F.append((a, b, c))
        F.append((a, c, d))

    x_back, x_front = -0.5 * PLATE_T, 0.5 * PLATE_T
    for x in (x_back, x_front):
        for y, z in corners:
            V.append((x, y, z))
    F.append((1, 2, 3))
    F.append((6, 5, 4))
    for k in range(3):
        a, b = k + 1, (k + 1) % 3 + 1
        quad(a, a + 3, b + 3, b)

    with open(obj_path, "w") as f:
        f.write("# triangular plate mesh joining sphere centers\n")
        for v in V:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for face in F:
            f.write(f"f {face[0]} {face[1]} {face[2]}\n")


def write_plate_urdf(path, pad_mass=PAD_MASS, plate_mass=PLATE_MASS,
                     plate_collision=None):
    """URDF: center_hub (triangular plate) + 3 fixed links for the spheres centered at the vertices."""
    obj_path = os.path.splitext(path)[0] + "_mesh.obj"
    _write_plate_mesh(obj_path)

    if plate_collision is None:
        plate_collision = PLATE_COLLISION
    plate_col_xml = ('<collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry>'
                     f'<mesh filename="{os.path.basename(obj_path)}"/>'
                     '</geometry></collision>') if plate_collision else                     '<!-- plate visual only: CONSTRAINT 1, three spheres -->'
    I_plate = np.diag(plate_inertia(plate_mass))       # ~7e-5, not 1e-3
    i_pad = 0.4 * pad_mass * PAD_R ** 2                # solid sphere, 8e-7

    pads_xml = ""
    for i, (_x, ys, zs) in enumerate(PADS_LOCAL):
        pads_xml += f"""
  <link name="pad{i}">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><sphere radius="{PAD_R}"/></geometry>
      <material name="pad_red"><color rgba="0.9 0.25 0.2 1.0"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><sphere radius="{PAD_R}"/></geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{pad_mass}"/>
      <inertia ixx="{i_pad:.9f}" ixy="0" ixz="0" iyy="{i_pad:.9f}" iyz="0" izz="{i_pad:.9f}"/>
    </inertial>
  </link>
  <joint name="joint_pad{i}" type="fixed">
    <parent link="center_hub"/>
    <child link="pad{i}"/>
    <origin xyz="0 {ys:.6f} {zs:.6f}" rpy="0 0 0"/>
  </joint>"""

    urdf_content = f"""<?xml version="1.0"?>
<robot name="floating_plate_triad">
  <link name="center_hub">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="{os.path.basename(obj_path)}"/></geometry>
      <material name="gold"><color rgba="0.85 0.75 0.25 1.0"/></material>
    </visual>
    {plate_col_xml}
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{plate_mass}"/>
      <inertia ixx="{I_plate[0]:.9f}" ixy="0" ixz="0" iyy="{I_plate[1]:.9f}" iyz="0" izz="{I_plate[2]:.9f}"/>
    </inertial>
  </link>
  {pads_xml}
</robot>"""

    with open(path, "w") as f:
        f.write(urdf_content)
    return path


# =========================================================================
#  TOOL CLASS
# =========================================================================
class FloatingPlateTool:
    """Flange (with cylinder supports) + Rigid Triangular Plate (welded to 3 sphere centers)."""

    def __init__(self, position, orientation, pads=None, centre=None, mu=0.7,
                 dt=0.001, quiet=False, **_):
        self.pads_spring = pads or PadSprings()
        self.centre = centre or CentralSpring()
        self.dt = float(dt)
        position = np.asarray(position, float)
        R = Rot.from_quat(orientation)

        # refuse to run past the explicit integrator instead of producing a run
        # that looks fine and is not
        w = stability(self.pads_spring, self.centre, dt)
        worst = max(w.values())
        if worst >= 2.0:
            raise RuntimeError(f"unstable at dt = {1000 * dt:.1f} ms: "
                               f"omega*dt = {worst:.2f} on {max(w, key=w.get)}")
        if worst >= 1.0 and not quiet:
            print(f"  [warn] omega*dt = {worst:.2f} on {max(w, key=w.get)}")

        # Flange body with 3 cylinder supports
        self.id = p.createMultiBody(BASE_MASS, -1, _flange_shapes(),
                                    position.tolist(), list(orientation))
        p.changeDynamics(self.id, -1, localInertiaDiagonal=[0.05] * 3,
                         linearDamping=0.0, angularDamping=0.0)

        # Front body: Triangular plate with 3 welded spheres
        urdf_path = SCRATCH(f"_plate_{os.getpid()}.urdf")
        write_plate_urdf(urdf_path, PAD_MASS, PLATE_MASS)
        plate_init_pos = position + R.apply([PAD_X, 0.0, 0.0])
        self.plate_id = p.loadURDF(urdf_path, plate_init_pos.tolist(),
                                   list(orientation),
                                   flags=p.URDF_USE_INERTIA_FROM_FILE)
        p.changeDynamics(self.plate_id, -1, linearDamping=0.0, angularDamping=0.0)

        self.pads = [0, 1, 2]
        self.pad_ids = [self.plate_id] * 3
        for i in self.pads:
            p.changeDynamics(self.plate_id, i, lateralFriction=float(mu),
                             spinningFriction=0.0, rollingFriction=0.0)

        # Disable collision between flange and plate
        p.setCollisionFilterPair(self.id, self.plate_id, -1, -1, enableCollision=0)
        for i in self.pads:
            p.setCollisionFilterPair(self.id, self.plate_id, -1, i, enableCollision=0)

        # the spheres are links of the PLATE body, not of the flange: tell
        # `lib.tool.wrench_on_body` where to look for the contacts
        self.contact_body = self.plate_id
        self.mass = BASE_MASS + PLATE_MASS + 3 * PAD_MASS
        self.constraint = None

    def pose(self):
        """(position, quaternion) of the FLANGE."""
        pos, ori = p.getBasePositionAndOrientation(self.id)
        return np.asarray(pos, float), np.asarray(ori, float)

    def plate_pose(self):
        """(position, quaternion) of the TRIANGULAR PLATE."""
        pos, ori = p.getBasePositionAndOrientation(self.plate_id)
        return np.asarray(pos, float), np.asarray(ori, float)

    def anchors_world(self):
        """(3,3) Tips of the 3 cylindrical supports on the flange (nominal rest positions)."""
        pos_f, ori_f = self.pose()
        Rf = Rot.from_quat(ori_f)
        return pos_f + Rf.apply(PADS_LOCAL)

    def pads_world(self):
        """(3,3) World centers of the 3 spheres welded to the triangular plate."""
        out = []
        for i in range(3):
            st = p.getLinkState(self.plate_id, i)
            out.append(st[0])
        return np.array(out)

    def deflection_local(self):
        """(3,3) sphere center minus cylinder tip anchor, in the FLANGE frame."""
        _pos_f, ori_f = self.pose()
        Rf = Rot.from_quat(ori_f)
        return Rf.inv().apply(self.pads_world() - self.anchors_world())

    def deflection6(self):
        """(6,) Deflection of the plate center relative to flange."""
        pos_f, ori_f = self.pose()
        pos_p, ori_p = self.plate_pose()
        Rf = Rot.from_quat(ori_f)
        rest_plate = pos_f + Rf.apply([PAD_X, 0.0, 0.0])
        d = Rf.inv().apply(pos_p - rest_plate)
        phi = Rf.inv().apply((Rot.from_quat(ori_p) * Rf.inv()).as_rotvec())
        return np.concatenate([d, phi])

    def travel(self):
        return self.deflection_local()[:, 0]

    def lateral_drift(self):
        return np.linalg.norm(self.deflection_local()[:, 1:], axis=1)

    def conform_deg(self):
        return tilt_from_travel(self.travel())

    def bottomed(self, margin=1e-4):
        """Always 0: since 19 August 2026 the springs have no end stop, so
        there is nothing to bottom out against. Kept so the benches and the
        pipeline, which ask every tool this, keep working."""
        return 0

    def place(self, position, orientation):
        """Teleport flange AND plate together, springs unloaded. `approach`
        calls this instead of moving the flange on its own."""
        pos = np.asarray(position, float)
        R = Rot.from_quat(orientation)
        p.resetBasePositionAndOrientation(self.id, pos.tolist(),
                                          list(orientation))
        p.resetBaseVelocity(self.id, [0, 0, 0], [0, 0, 0])
        p.resetBasePositionAndOrientation(
            self.plate_id, (pos + R.apply([PAD_X, 0.0, 0.0])).tolist(),
            list(orientation))
        p.resetBaseVelocity(self.plate_id, [0, 0, 0], [0, 0, 0])

    def pin_base(self):
        """Bolt the flange to the world. Bench only: it takes the wrist out so
        the springs are measured on their own."""
        pos, ori = self.pose()
        self.constraint = p.createConstraint(
            self.id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
            pos.tolist(), childFrameOrientation=ori.tolist())
        p.changeConstraint(self.constraint, maxForce=1e6)

    def update_springs(self):
        """Applies:
        1. Strong 6-DoF central spring between flange and plate center.
        2. Three zero-length springs between flange cylinder tips and sphere centers.
        """
        pos_f, ori_f = self.pose()
        Rf = Rot.from_quat(ori_f)
        pos_p, ori_p = self.plate_pose()
        
        v_f, w_f = p.getBaseVelocity(self.id)
        v_p, w_p = p.getBaseVelocity(self.plate_id)
        v_rel = np.asarray(v_p, float) - np.asarray(v_f, float)
        w_rel = np.asarray(w_p, float) - np.asarray(w_f, float)

        # 1. Central Spring: Flange <-> Plate Center
        x = self.deflection6()
        F_c = Rf.apply(-self.centre.k_lin * x[:3] - self.centre.c_lin * Rf.inv().apply(v_rel))
        T_c = Rf.apply(-self.centre.k_rot * x[3:] - self.centre.c_rot * Rf.inv().apply(w_rel))
        p.applyExternalForce(self.plate_id, -1, F_c.tolist(), pos_p.tolist(), p.WORLD_FRAME)
        p.applyExternalTorque(self.plate_id, -1, T_c.tolist(), p.WORLD_FRAME)
        p.applyExternalForce(self.id, -1, (-F_c).tolist(), pos_p.tolist(), p.WORLD_FRAME)
        p.applyExternalTorque(self.id, -1, (-T_c).tolist(), p.WORLD_FRAME)

        # 2. Three Zero-Length Springs: Flange Cylinder Tips <-> Sphere Centers
        s = self.pads_spring
        d_local = self.deflection_local()
        anchors = self.anchors_world()
        pads = self.pads_world()
        
        for i in range(3):
            # Velocity of sphere i
            v_pad = np.asarray(v_p, float) + np.cross(np.asarray(w_p, float), pads[i] - pos_p)
            v_anc = np.asarray(v_f, float) + np.cross(np.asarray(w_f, float), anchors[i] - pos_f)
            v_rel_i = Rf.inv().apply(v_pad - v_anc)

            f_local = -s.K_local * d_local[i] - s.C_local * v_rel_i
            f_world = Rf.apply(f_local)

            # NO END OF TRAVEL. Decision of 19 August 2026: the zero-length
            # springs run unbounded, F = -K d everywhere, no wall at any
            # excursion. It is also the honest model of the element that
            # realises them - a flexure has no hard stop until it yields - and
            # in the PARALLEL arrangement it costs nothing: the push is
            # 21500 N/m, so 60 N of squeeze is 2.8 mm of common mode and the
            # excursion never runs away. `travel` survives only as the nominal
            # range the HUD colours against; `bottomed()` is now always 0.
            # NO gravity term here. The spheres are LINKS of the plate body, so
            # their weight is already carried by the wrist through `self.mass`;
            # adding m*g again was 0.59 N of lift with no reaction anywhere.
            p.applyExternalForce(self.plate_id, i, f_world.tolist(),
                                 pads[i].tolist(), p.WORLD_FRAME)
            # Reaction on the flange cylinder tip
            p.applyExternalForce(self.id, -1, (-f_world).tolist(),
                                 anchors[i].tolist(), p.WORLD_FRAME)

    def contacts(self, body):
        """Reads contact per sphere link from the triangular plate body."""
        out = []
        for i in range(3):
            pts = p.getContactPoints(bodyA=body, bodyB=self.plate_id, linkIndexB=i)
            if not pts:
                out.append((False, None, None, np.zeros(3)))
                continue
            pos = np.zeros(3)
            nrm = np.zeros(3)
            f = np.zeros(3)
            w = 0.0
            for c in pts:
                fn = c[9]
                n_into = np.asarray(c[7], float)
                weight = max(fn, 1e-6)
                pos += np.asarray(c[5], float) * weight
                nrm += (-n_into) * weight
                f += (fn * (-n_into) + c[10] * np.asarray(c[11], float)
                      + c[12] * np.asarray(c[13], float))
                w += weight
            pos /= w
            n = np.linalg.norm(nrm)
            nrm = nrm / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
            out.append((True, pos, nrm, f))
        return out

    def normal_force(self, body):
        total = 0.0
        for touching, _pos, n, f in self.contacts(body):
            if touching:
                total += max(float(np.asarray(f) @ np.asarray(n)), 0.0)
        return float(total)

    def n_contacts(self, body):
        return int(sum(1 for c in self.contacts(body) if c[0]))


# =========================================================================
#  SELF-TEST
# =========================================================================
def self_test(dt=0.001):
    """Print the four channels and CHECK the simulated tool against K^-1 w.

    The check is the whole point: put a wrench of known size on the plate, let
    it settle, compare the six-vector of deflection with what the matrix
    predicts. If they disagree, the code and the formula are two different
    machines and only one of them is the device.
    """
    pads, centre = PadSprings(), CentralSpring()
    K = stiffness_6x6(pads, centre)
    ch = channels(pads, centre)
    w = stability(pads, centre, dt)

    print()
    print("=" * 72)
    print("FLANGE + 3 SUPPORTS -> l0=0 SPRINGS -> SPHERES WELDED TO THE PLATE")
    print("=" * 72)
    print()
    print("  " + centre.describe())
    print("  " + pads.describe())
    print()
    print(f"  channels: axial {ch['axial']:.0f}  lateral {ch['lateral']:.0f} N/m"
          f"  |  tilt {ch['tilt']:.2f}  torsion {ch['torsion']:.2f} Nm/rad")
    print(f"  who does the tilt:    pads {pads.k_tilt:.3f}   centre "
          f"{centre.k_rot[1]:.3f}   <- the centre must be ZERO here")
    print(f"  who does the torsion: pads {pads.k_torsion:.2f}   centre "
          f"{centre.k_rot[0]:.2f}")
    print()
    print("  omega*dt  " + "  ".join(f"{k} {v:.2f}" for k, v in w.items())
          + "   (stable < 2, quiet < 1)")
    print()
    print(f"  {'wrench':<14} {'channel':<9} {'predicted':>11} {'measured':>11} "
          f"{'err':>7}")

    tests = [("Fx = 10 N", np.array([10., 0, 0, 0, 0, 0]), 0, "dx [mm]", 1e3),
             ("Fy = 10 N", np.array([0, 10., 0, 0, 0, 0]), 1, "dy [mm]", 1e3),
             ("Tx = 0.5 Nm", np.array([0, 0, 0, .5, 0, 0]), 3, "rx [deg]",
              180 / np.pi),
             # SMALL on purpose. The 6x6 is the linearisation about the rest
             # pose, and the tilt channel is the softest, so it leaves the
             # linear range first. Measured error against K^-1 w:
             #     0.005 Nm -> 0.42 deg -> 0.1%
             #     0.020    -> 1.70     -> 1.2%
             #     0.050    -> 3.97     -> 6.4%
             #     0.100    -> 7.00     -> 17.6%
             # The device stiffens as it tilts, which is real: the pad springs
             # start taking the load sideways. A bigger probe here would be
             # measuring the linearisation, not the code.
             ("Ty = 0.005 Nm", np.array([0, 0, 0, 0, .005, 0]), 4, "ry [deg]",
              180 / np.pi)]
    worst = 0.0
    for name, w6, idx, label, sc in tests:
        p.connect(p.DIRECT)
        p.setGravity(0, 0, 0)                    # springs only, no weight
        p.setTimeStep(dt)
        p.setPhysicsEngineParameter(numSolverIterations=200, enableFileCaching=0)
        tool = FloatingPlateTool([0, 0, 0], [0, 0, 0, 1], pads, centre, dt=dt,
                                 quiet=True)
        tool.pin_base()
        for _ in range(int(2.5 / dt)):
            pos_p, _ = tool.plate_pose()
            p.applyExternalForce(tool.plate_id, -1, w6[:3].tolist(),
                                 pos_p.tolist(), p.WORLD_FRAME)
            p.applyExternalTorque(tool.plate_id, -1, w6[3:].tolist(),
                                  p.WORLD_FRAME)
            tool.update_springs()
            p.stepSimulation()
        got = tool.deflection6()
        pred = np.linalg.solve(K, w6)
        err = abs(got[idx] - pred[idx]) / max(abs(pred[idx]), 1e-12)
        worst = max(worst, err)
        print(f"  {name:<14} {label:<9} {sc * pred[idx]:>11.3f} "
              f"{sc * got[idx]:>11.3f} {100 * err:>6.1f}%")
        p.disconnect()
    print()
    print(f"  worst error {100 * worst:.1f}%  ->  "
          + ("the simulated tool IS the matrix" if worst < 0.05
             else "MISMATCH between code and formula"))
    print()
    return worst


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dt", type=float, default=0.001)
    self_test(ap.parse_args().dt)
