"""
The tri-pad tool, rigid or with passive springs, in one place and behind one
flag.

RIGID (springs=None) reproduces the C++ rig exactly: three 10 mm spheres at the
corners of a 30 mm circumradius triangle, 20 mm ahead of the wrist, on FIXED
joints. Zero springs in the tool. The only compliance is the wrist virtual
coupling, which is a controller gain, not a mechanism: conforming costs
`theta = F * r_tri / k_tilt`, i.e. the grasp geometry is a function of the
force (CLAUDE.md 7.2).

WITH SPRINGS (Campolo, 11 August 2026). Along the pad-plane normal (local x,
the push direction), each pad sits on a SUPPORT of length `l` sticking out of
the wrist, with a WEAK spring `k` between support and sphere. A STRONG spring
`K_c` sits at the centre of the triangle.

    base_link --------- support l ---------o pad_i        (joint coordinate q_i)
                   +---- spring k ----+
                   + spring K_c on the common mode of the three

WHY THE SUPPORT, i.e. why `l0` must not appear. A spring between two points
with rest length `l0` has energy U = 1/2 K (||p-q|| - l0)^2, which is NOT a
quadratic form: for l0 > 0 the minimum is a sphere rather than a point, the
Hessian is configuration-dependent and indefinite, and the gradient is singular
at p = q. With l0 = 0, U = 1/2 K ||p-q||^2: linear gradient, constant PSD
Hessian, and everything downstream (equilibrium, stability via the Hessian of
the potential, any elastic term inside the convex program) stays convex.
The support is the mechanical realisation: mount the spring between the sphere
and a point carried `l` forward, so the DEFORMATION COORDINATE is directly the
travel q_i measured from nominal. Here that is literally a PRISMATIC JOINT with
zero coordinate at the nominal geometry and force F_i = -k q_i - c qd_i. No
norm of a position difference, so no l0.

WHAT IT BUYS, in numbers (r = R_TRI):

    axial (push)        K_ax   = 3k + K_c
    tilt (conforming)   K_tilt = (3/2) k r^2      <-- the peripheral ones only
    torsion about x     0 from the springs        <-- friction only
    lateral             rigid (the prismatic joint)

A spring at the CENTRE produces no restoring moment about the centre, so K_c
raises push stiffness WITHOUT touching conforming compliance. That is Whitney's
remote centre of compliance (1982) obtained by construction: the compliance
centre IS the pad plane, because that is where the springs act.

The third row is what protects the thesis: the springs are AXIAL, so a rotation
about local x does not deform them. Torsional resistance still comes only from
the three contacts' lever arms (CONSTRAINT 1) and the wrist's torsional channel
stays stiff at k_tors = 40, so CLAUDE.md 9 is untouched and the TORSIONAL_C
prior stays valid.

THE LIMIT, to be looked for and not hidden: above a certain squeeze the springs
BOTTOM OUT and the tool is rigid again. The threshold is set by the TRAVEL, not
by k. `Tool.bottomed()` measures it at every instant.
"""
import os

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as Rot

# --------------------------------------------------------------- geometry
PAD_X = 0.020      # [m] how far the pads stick out ahead of the wrist (C++ pad_x)
R_TRI = 0.030      # [m] circumradius of the pad triangle              (C++ r)
PAD_R = 0.010      # [m] radius of each little sphere                  (C++ pad_r)
# [m] the tri-pad's footprint on a surface: its SIZE, not a requirement on the
# object. The tri-pad does NOT ask for a flat facet that contains it - it
# adapts, which is exactly why it exists (see Springs).
R_FOOTPRINT = R_TRI + PAD_R

# The three pads in the hand frame. The pad plane is x = PAD_X, so its normal
# is the local x axis, which is the push direction.
PADS_LOCAL = np.array([[PAD_X, 0.0, R_TRI],
                       [PAD_X, +R_TRI * np.sqrt(3) / 2, -R_TRI / 2],
                       [PAD_X, -R_TRI * np.sqrt(3) / 2, -R_TRI / 2]])
PAD_NORMAL = np.array([1.0, 0.0, 0.0])

# Grasp opening of the rig: the hands start at +-0.15 m and the pads stick out.
# The minimum is the DIAMETER of the tri-pad footprint - below that we would be
# grasping an object smaller than the tool itself.
OPENING_MIN, OPENING_MAX = 2.0 * R_FOOTPRINT, 0.26
# Mass band where the haptic display is linear and gravity is above the noise.
MASS_MIN, MASS_MAX = 1.0, 4.0

# URDF masses. The base inertia is INFLATED to 0.05 isotropic: it is what keeps
# the explicit virtual coupling numerically stable, and it is like that in C++.
BASE_MASS = 0.05
BASE_INERTIA = 0.05
# Pads are massless on the rigid tool (they inherit the base). With springs they
# have a degree of freedom of their own, so they need a mass or the mass matrix
# is singular.
PAD_MASS_SPRUNG = 0.02


class Springs:
    """Parameters of the passive composition. Instantiating it TURNS THE SPRINGS ON.

    THE VALUES ARE MEASURED, NOT CHOSEN. Bench: press the tri-pad against a face
    tilted by `delta`, 25 mm commanded push (~35 N), wrist k_tilt at 3 Nm/rad
    i.e. today's rig. Count seated pads and wrist rotation:

        delta     RIGID          k=2000      k=1000       k=500
          0    3/3  0.0 deg   3/3  0.0    3/3  0.0    3/3  0.0
         10    2/3  9.8       3/3  4.7    3/3  3.2    3/3  2.1
         15    2/3 10.6       3/3  7.1    3/3  4.7    3/3  2.9
         25    2/3 11.7       2/3 10.2    3/3  7.9    3/3  6.8

    Read it in two columns. THE PADS: the rigid tool loses the third pad
    already at 10 deg; with k = 500 it keeps all three out to 25. THE TILT: the
    rigid wrist rotates 10-12 deg - and rotating DRAGS the pads, which is the
    7.2 defect - while with springs it stays at 2-7, because the travel does
    the conforming.

    WHY k = 2000 IS NOT ENOUGH, and it is the lesson: k = 2000 gives
    K_tilt = 1.5 k r^2 = 2.70 Nm/rad, i.e. EXACTLY today's wrist tilt spring
    (3 Nm/rad). Same stiffness = same behaviour, moved 30 mm forward: you gain
    the compliance centre (less drag, the RCC result) but NOT the conforming.
    To actually conform, K_tilt must drop BELOW the wrist spring, and that is
    possible without losing push stiffness ONLY thanks to the central spring,
    which raises K_ax without touching K_tilt. That is why the strong central
    spring is not an accessory.

    Defaults:
      k_pad = 500 N/m   K_tilt = 0.67 Nm/rad, 4.4x more compliant than the
                        wrist; the differential cost of 15 deg is
                        k*R_TRI*sin15 = 3.9 N per pad, a fraction of the ~12 N
                        per pad of a normal squeeze;
      k_centre = 20000  K_ax = 21500 N/m, 14x the wrist spring (kp = 1500): in
                        series they give 1402 N/m, 6% softer than today - the
                        wrist still governs the squeeze, the tool the conforming;
      travel = +-8 mm   SYMMETRIC, because the pad plane must be able to rotate
                        about its own centre: the far pad EXTENDS by
                        R_TRI sin(delta) while the other two retract. +-8 mm
                        covers 15.5 deg; at 25 deg the far pad bottoms out
                        (measured: +8.1 mm) and there the tool goes rigid again
                        - prediction P2, to be reported, not avoided;
      c_pad = 4.5 Ns/m  0.7 of critical damping, 2 sqrt(k m).
    """

    def __init__(self, k_pad=500.0, k_centre=20000.0, c_pad=4.5,
                 travel=(-0.008, 0.008), support_len=0.020,
                 pad_mass=PAD_MASS_SPRUNG):
        self.k_pad = float(k_pad)
        self.k_centre = float(k_centre)
        self.c_pad = float(c_pad)
        self.travel = (float(travel[0]), float(travel[1]))
        self.support_len = float(support_len)
        self.pad_mass = float(pad_mass)

    @property
    def k_axial(self):
        """[N/m] push mode: 3k + K_c."""
        return 3.0 * self.k_pad + self.k_centre

    @property
    def k_tilt(self):
        """[Nm/rad] conforming mode: (3/2) k r^2. Independent of K_c, because a
        spring at the centre makes no moment about the centre."""
        return 1.5 * self.k_pad * R_TRI ** 2

    @property
    def k_torsion(self):
        """[Nm/rad] ZERO: the springs are axial, a rotation about x does not
        deform them. Torsion stays entirely at the contacts (CONSTRAINT 1)."""
        return 0.0

    def absorbed_tilt_deg(self):
        """[deg] misalignment the travel covers. Rotating the pad plane about
        its own centre, the far pad must EXTEND by R_TRI sin(delta), so the
        extension travel is what sets the limit."""
        c = min(abs(self.travel[0]), abs(self.travel[1]))
        return float(np.degrees(np.arcsin(np.clip(c / R_TRI, 0.0, 1.0))))

    def describe(self):
        return (f"springs: k_pad {self.k_pad:.0f} N/m, k_centre "
                f"{self.k_centre:.0f} N/m, travel {1000*self.travel[0]:+.1f}/"
                f"{1000*self.travel[1]:+.1f} mm  ->  K_ax {self.k_axial:.0f} N/m, "
                f"K_tilt {self.k_tilt:.2f} Nm/rad, K_tors 0, absorbs "
                f"{self.absorbed_tilt_deg():.1f} deg")


# =========================================================================
#  URDF
# =========================================================================
# Generated URDF and OBJ files are per-process throwaways. Written beside the
# source they piled up to 319 files; here they go in one _tmp/ that can be wiped.
_TMP = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "_tmp")


def scratch(name):
    """Path for a generated URDF/OBJ, inside `_tmp/`."""
    os.makedirs(_TMP, exist_ok=True)
    return os.path.join(_TMP, name)


def write_urdf(path, springs=None, draw_supports=True):
    """Write the tool URDF and return the path.

    `springs=None` -> FIXED joints and massless pads: the C++ `create3PointURDF`
    line by line. `springs=Springs(...)` -> PRISMATIC joints along local x with
    travel limits; the actual spring force is applied by `Tool.update_springs()`
    every step, because PyBullet joints have no native elastic term.

    Pad links stay at indices 0, 1, 2 in both cases: the supports are drawn as
    extra `<visual>` elements of the base, not as links, so the indexing does
    not change between the two arms of the experiment.
    """
    joint_type = "fixed" if springs is None else "prismatic"
    m_pad = 0.0 if springs is None else springs.pad_mass
    i_pad = 0.0 if springs is None else 0.4 * m_pad * PAD_R ** 2   # solid sphere

    supports = ""
    if draw_supports and springs is not None:
        # COSMETIC ONLY: the support is what makes l0 unnecessary in the model,
        # and seeing it helps read the scene. A URDF cylinder runs along z, so
        # rotate it 90 deg about y to lay it along x.
        L = springs.support_len
        for pl in PADS_LOCAL:
            supports += (
                f'<visual><origin xyz="{PAD_X - 0.5 * L:.4f} {pl[1]:.4f} '
                f'{pl[2]:.4f}" rpy="0 1.5707963 0"/>'
                f'<geometry><cylinder radius="0.0035" length="{L:.4f}"/></geometry>'
                f'<material name="gy"><color rgba="0.45 0.45 0.5 1"/></material>'
                f'</visual>')

    pads = ""
    for i, pl in enumerate(PADS_LOCAL):
        limits = ""
        if springs is not None:
            limits = (f'<axis xyz="1 0 0"/>'
                      f'<limit lower="{springs.travel[0]:.4f}" '
                      f'upper="{springs.travel[1]:.4f}" '
                      f'effort="500" velocity="10"/>')
        pads += f"""<link name="pad{i}">
<visual><geometry><sphere radius="{PAD_R}"/></geometry><material name="rd"><color rgba="0.9 0.25 0.2 1"/></material></visual>
<collision><geometry><sphere radius="{PAD_R}"/></geometry></collision>
<inertial><mass value="{m_pad}"/><inertia ixx="{i_pad}" ixy="0" ixz="0" iyy="{i_pad}" iyz="0" izz="{i_pad}"/></inertial></link>
<joint name="j{i}" type="{joint_type}"><parent link="base_link"/><child link="pad{i}"/>
<origin xyz="{pl[0]:.4f} {pl[1]:.4f} {pl[2]:.4f}"/>{limits}</joint>"""

    with open(path, "w") as f:
        f.write(f"""<?xml version="1.0"?><robot name="tripad">
<link name="base_link"><visual><geometry><sphere radius="0.015"/></geometry>
<material name="cy"><color rgba="0 0.9 1 1"/></material></visual>{supports}
<inertial><origin xyz="0 0 0"/><mass value="{BASE_MASS}"/><inertia ixx="{BASE_INERTIA}" ixy="0" ixz="0" iyy="{BASE_INERTIA}" iyz="0" izz="{BASE_INERTIA}"/></inertial></link>
{pads}</robot>""")
    return path


# =========================================================================
#  THE TOOL IN SIMULATION
# =========================================================================
class Tool:
    """A tri-pad loaded in PyBullet, rigid or sprung, same interface.

    The ONLY difference the caller sees is that with springs you must call
    `update_springs()` before every `stepSimulation()`. `grasp.py` does it for
    both arms, and on the rigid tool it is a no-op: the control loop is
    literally the same code in both cases, which is point 4 of the discussion
    (not two separate paths).
    """

    def __init__(self, urdf, position, orientation, springs=None, mu=0.7):
        self.springs = springs
        self.id = p.loadURDF(urdf, list(position), list(orientation),
                             flags=p.URDF_USE_INERTIA_FROM_FILE)
        self.pads = [0, 1, 2]
        # MEASURED TRAP. linearDamping and angularDamping are BODY properties,
        # not link ones: passing them in the SAME call as lateralFriction with a
        # link index >= 0 makes PyBullet take another branch and the friction is
        # NOT applied - no error, no warning. Isolated 2026-08-12:
        #     lateralFriction                           -> 1.0   ok
        #     lateralFriction + spinning + rolling      -> 1.0   ok
        #     lateralFriction + linearDamping + angular -> 0.5   DROPPED
        # The three pads stayed at PyBullet's default 0.5, and since PyBullet
        # MULTIPLIES, the contact had half the friction the SOCP was planning
        # with. So: friction per link, damping on the base ONLY, two calls.
        for j in range(-1, p.getNumJoints(self.id)):
            p.changeDynamics(self.id, j, lateralFriction=float(mu),
                             spinningFriction=0.0, rollingFriction=0.0)
        p.changeDynamics(self.id, -1, linearDamping=0.0, angularDamping=0.0)
        # And READ IT BACK: a wrong friction does not produce an error, it
        # produces a wrong experiment that runs perfectly.
        for j in range(-1, p.getNumJoints(self.id)):
            got = p.getDynamicsInfo(self.id, j)[1]
            if abs(got - float(mu)) > 1e-9:
                raise RuntimeError(f"friction not applied to tool link {j}: "
                                   f"asked {mu}, read {got}")
        if springs is not None:
            for j in self.pads:
                # PyBullet joints come with an active velocity motor that holds
                # them STILL: without switching it off the spring never moves
                # and the "sprung" tool is as rigid as the control arm.
                p.setJointMotorControl2(self.id, j, p.VELOCITY_CONTROL, force=0.0)
                p.changeDynamics(self.id, j, jointDamping=0.0)
        # total mass, for gravity compensation in the virtual coupling
        self.mass = BASE_MASS + (0.0 if springs is None else 3 * springs.pad_mass)

    # ---------------------------------------------------------------- state
    def pose(self):
        """(position, quaternion) of the BASE, i.e. of the wrist."""
        pos, ori = p.getBasePositionAndOrientation(self.id)
        return np.asarray(pos, float), np.asarray(ori, float)

    def travel(self):
        """[m] the three joint coordinates q_i. Zero = nominal geometry.
        Negative = pad retracted towards the wrist, i.e. spring compressed."""
        if self.springs is None:
            return np.zeros(3)
        return np.array([p.getJointState(self.id, j)[0] for j in self.pads])

    def travel_rate(self):
        if self.springs is None:
            return np.zeros(3)
        return np.array([p.getJointState(self.id, j)[1] for j in self.pads])

    def bottomed(self, margin=1e-4):
        """How many pads are at END OF TRAVEL. Above zero the tool is going
        rigid again and prediction P2 has kicked in: report it, do not hide it.
        On the rigid tool this is 3 by definition - the pads are permanently at
        the stop."""
        if self.springs is None:
            return 3
        q = self.travel()
        lo, hi = self.springs.travel
        return int(np.sum((q <= lo + margin) | (q >= hi - margin)))

    def pads_world(self):
        """(3,3) sphere centres in world, as they are NOW - so with springs it
        already accounts for the travel."""
        return np.array([p.getLinkState(self.id, j)[0] for j in self.pads])

    def pad_centre_local(self):
        """Contact centroid in the base frame. With springs it moves, so it is
        read from the state instead of using the constant."""
        pos, ori = self.pose()
        c = Rot.from_quat(ori).inv().apply(self.pads_world().mean(0) - pos)
        return c + np.array([PAD_R, 0.0, 0.0])      # sphere centre -> contact

    # --------------------------------------------------------------- springs
    def update_springs(self):
        """Apply the elastic forces to the three joints. NO-OP if rigid.

        Energy:  U = 1/2 k sum q_i^2  +  1/2 K_c qbar^2,   qbar = mean(q)
        hence    F_i = -dU/dq_i = -k q_i - (K_c/3) qbar - c qd_i

        The second term IS the central spring, written in the coordinates of
        the three pistons: on the common mode (all q_i equal) it gives -K_c q;
        on the tilt modes (sum q_i = 0, so qbar = 0) it gives exactly zero.
        That is the push/conform decoupling, not an approximation.
        """
        if self.springs is None:
            return
        q = self.travel()
        qd = self.travel_rate()
        F = (-self.springs.k_pad * q
             - (self.springs.k_centre / 3.0) * float(q.mean())
             - self.springs.c_pad * qd)
        for j, f in zip(self.pads, F):
            p.setJointMotorControl2(self.id, j, p.TORQUE_CONTROL, force=float(f))

    # -------------------------------------------------------------- contacts
    def contacts(self, body):
        """Per pad, the REAL contact read from the manifold (CONSTRAINT 3):
        `(touching, position, outward_normal, force)`.

        No mesh, no raycast: normal and force come from `getContactPoints`,
        which exposes `m_normalForce` and the two friction components. `force`
        carries the normal OUTWARD from the body (so `f . n = +f_n` gives the
        used fraction of the cone directly); to sum a WRENCH on the body use
        `wrench_on_body`, which flips it.
        """
        out = []
        for j in self.pads:
            points = p.getContactPoints(bodyA=body, bodyB=self.id, linkIndexB=j)
            if not points:
                out.append((False, None, None, np.zeros(3)))
                continue
            pos = np.zeros(3); nrm = np.zeros(3); f = np.zeros(3); w = 0.0
            for c in points:
                fn = c[9]
                n_into = np.asarray(c[7], float)     # from B(hand) towards A(body)
                weight = max(fn, 1e-6)               # weight by normal force
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

    def total_force(self, body):
        """[N] magnitude of the contact forces summed pad by pad - normal PLUS
        friction. Use it as a "I feel something" threshold, not to compare with
        the SOCP: the allocation speaks of NORMAL forces."""
        return float(sum(np.linalg.norm(c[3])
                         for c in self.contacts(body) if c[0]))

    def normal_force(self, body):
        """[N] sum of the NORMAL components only. This is what compares against
        the solver's fn_L/fn_R: closing the loop on the total magnitude would
        also count friction, which the program allocates separately, and the
        squeeze would stop early or late depending on how much friction happens
        to be engaged at that instant."""
        total = 0.0
        for touching, _pos, n, f in self.contacts(body):
            if touching:
                total += max(float(np.asarray(f) @ np.asarray(n)), 0.0)
        return float(total)

    def n_contacts(self, body):
        return int(sum(1 for c in self.contacts(body) if c[0]))


# (body, other body, other link) -> True if Bullet stores the pair with `body`
# SECOND. Read once per CONTACT EPISODE: while two objects keep touching their
# pair cannot be rebuilt, so the order cannot change; the entry is dropped the
# moment they stop, because PyBullet reuses the ids of removed bodies and a new
# body with an old id may be stored the other way round.
_STORED_SECOND = {}


def _stored_second(body, other, link):
    """Does Bullet store the pair (body, other link) with `body` SECOND?

    `getContactPoints(bodyA=body, ...)` swaps a pair stored the other way round
    so that the normal and the two positions read from `body`'s side - but it
    copies the two FRICTION directions as stored, i.e. as seen from the other
    body. Summing them as a force on `body` then gets the friction with the
    wrong sign. Only the unfiltered query shows the stored order, and it lists
    every contact in the world, so it is asked once per pair and remembered:
    asked every tick it made a rollout 2.7x slower (14.6 s against 5.4).
    """
    k = (body, other, link)
    if k not in _STORED_SECOND:
        for c in p.getContactPoints():
            if c[2] == body and c[1] == other and c[3] == link:
                _STORED_SECOND[k] = True
                break
            if c[1] == body and c[2] == other and c[4] == link:
                _STORED_SECOND[k] = False
                break
        else:
            return False                  # not touching: nothing to read anyway
    return _STORED_SECOND[k]


def forget_body(body=None):
    """Call after `p.removeBody(body)`, or with no argument after
    `p.resetSimulation()`: PyBullet reuses ids, and the next body created with
    this one may be stored the other way round."""
    for k in [k for k in _STORED_SECOND if body is None or body in (k[0], k[1])]:
        del _STORED_SECOND[k]


def wrench_on_body(body, tools, pole):
    """(F, T) net wrench applied ON the body by the given tools, about `pole`,
    with the right sign: the normal pushes INTO the body.

    THE FRICTION IS TURNED ROUND WHEN BULLET STORED THE PAIR THE OTHER WAY
    (2026-09-24). Which body is stored first depends on the shapes: a compound
    body always comes first, a single convex shape comes in creation order. So
    a piece made of ONE convex shape, created after the tools, read its
    friction reversed - the I, once it became one box: the pads read as pulling
    the piece DOWN with 2.45 N, the hang estimate came out 20-40 mm off on every
    I, and the jam sentry read the wrong side force. Where the pair was stored
    the right way round nothing changes.
    """
    F = np.zeros(3)
    T = np.zeros(3)
    for tool in tools:
        # WHERE THE SPHERES LIVE differs by tool family: links of the tool body
        # (this one), links of a separate frame (`spider_ee/`), or three free
        # bodies (the old free-pad tool). Ask the tool instead of assuming, or the
        # estimator reads a zero wrench and never takes a sample.
        pad_ids = getattr(tool, "pad_ids", None)
        if pad_ids is not None and len(set(pad_ids)) == len(tool.pads):
            pairs = [(int(b), -1) for b in pad_ids]
        else:
            cb = getattr(tool, "contact_body", tool.id)
            pairs = [(cb, j) for j in tool.pads]
        for body_b, j in pairs:
            points = p.getContactPoints(bodyA=body, bodyB=body_b, linkIndexB=j)
            if not points:                       # the episode is over
                _STORED_SECOND.pop((body, body_b, j), None)
                continue
            flip = _stored_second(body, body_b, j)
            for c in points:
                # written out twice so that the unflipped case is the very same
                # expression as before the fix, to the last bit: a game is
                # chaotic enough that a change of rounding moves it by 0.1 mm
                if flip:
                    f = (c[9] * np.asarray(c[7], float)
                         - c[10] * np.asarray(c[11], float)
                         - c[12] * np.asarray(c[13], float))
                else:
                    f = (c[9] * np.asarray(c[7], float)
                         + c[10] * np.asarray(c[11], float)
                         + c[12] * np.asarray(c[13], float))
                F += f
                T += np.cross(np.asarray(c[5], float) - pole, f)
    return F, T
