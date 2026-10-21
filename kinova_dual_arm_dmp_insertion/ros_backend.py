"""The two Kinova arms through ROS 2: flange targets out, wrench and flange poses in.

Written for the lab's own stack, `kinova_controller_pkgs` (`kinova_control`,
ROS 2 Jazzy), used AS IS - nothing of it is modified:

    ros2 launch kinova_control dual_cartesian.launch.py ee_frame:=end_effector_link cart_k:=...

which runs one `kinova_controller.py` per arm (namespaces /left .108, /right
.107): a Pinocchio Cartesian impedance in joint torque at 1 kHz on the Kortex
low-level API, gravity compensated from the URDF. From it this node uses:

    /<ns>/target_pose       PoseStamped   the ee_frame target, in the arm's base_link
    /<ns>/joint_states      JointState    250 Hz -> the flange pose, by the SAME
                                          Pinocchio forward kinematics on the SAME URDF
    /<ns>/estimated_wrench  WrenchStamped 250 Hz, from the joint torque sensors:
                                          the external wrench ON the arm, ee_frame
                                          axes, about the ee_frame origin
    /<ns>/zero_wrench       Trigger       tare (arm still, nothing touching)

That stack publishes no TF, hence the forward kinematics here.

The estimate is off by up to 8-9 N per arm away from the pose of the tare, the same
on every pass along the same path (2026-10-16: two empty runs agree within ~1 N).
`wrench_baseline:` in cell.yaml names an empty-handed `wrench_log` CSV recorded along
the route: each hand's force is then corrected by the baseline at the nearest flange
position, minus the baseline where the tare was made. Forces only, not moments.

`wrench_source: ft` (2026-10-21): the wrist ATI Net F/T, read by the lab's `ft_raw`
(tele_ros2/src/ati_ft_tools) started in the arm's namespace:

    /<ns>/ft_data_raw              WrenchStamped 1 kHz, SENSOR axes, raw (no filter, no bias)
    /<ns>/ft_raw/get_calibration   Trigger       the NetBox units (the left box is in lbf!)

The sensor pose is the flange's (forward kinematics, no TF) times `ft_in_ee`. The bias
is taken here, averaged, at each tare (tools in free air), with the tool's weight
(`tool_mass`, `tool_com_in_ft`) removed at every attitude. Each reading is the mean of
the samples of the last control period. The estimated wrench is still read and logged.

Everything inside the controller is in the SHELF frame; this file is the only
place that knows where the two robot bases are.
"""
import csv
import os
import threading
import time
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .geometry import Frame

G_WORLD = np.array([0.0, 0.0, -9.81])        # shelf z is up
# NetBox units -> N, Nm (ft_raw publishes in whatever the box is configured to)
FT_STALE = 0.050                              # [s] no ATI sample for this long: stop
FORCE_UNITS = {"N": 1.0, "lbf": 4.4482216, "Lbf": 4.4482216, "kN": 1000.0, "kgf": 9.80665}
TORQUE_UNITS = {"Nm": 1.0, "N-m": 1.0, "Nmm": 0.001, "N-mm": 0.001, "lbf-in": 0.11298483,
                "lbfin": 0.11298483, "Lbf-in": 0.11298483, "lbf-ft": 1.3558179, "kNm": 1000.0}


def _frame(cfg):
    return Frame(cfg["xyz"], cfg["quat"])


class _FK:
    """Flange pose from joint angles: Pinocchio on the controller's own URDF."""

    def __init__(self, urdf, ee_frame):
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.fid = self.model.getFrameId(ee_frame)
        if self.fid == self.model.nframes:
            raise ValueError(f"frame {ee_frame} not in {urdf}")

    def pose(self, msg):
        q = self.pin.neutral(self.model)
        for name, angle in zip(msg.name, msg.position):
            if not self.model.existJointName(name):
                continue                     # gripper / finger joints
            j = self.model.joints[self.model.getJointId(name)]
            if j.nq == 2:                    # continuous joint: (cos, sin)
                q[j.idx_q], q[j.idx_q + 1] = np.cos(angle), np.sin(angle)
            else:
                q[j.idx_q] = angle
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        M = self.data.oMf[self.fid]
        return np.array(M.translation), Rot.from_matrix(np.array(M.rotation))


class _Baseline:
    """Empty-handed wrench vs flange position, one arm, from a `wrench_log` CSV:
    the base-axes force of the K samples nearest to a position, if within RADIUS."""
    K, RADIUS = 25, 0.03

    def __init__(self, path, side):
        from scipy.spatial import cKDTree
        with open(path, newline="") as fh:
            rows = [r for r in csv.DictReader(fh) if r["arm"] == side]
        if any(r["mark"] != "start" for r in rows):
            rows = [r for r in rows if r["mark"] != "start"]     # only after the tare
        if len(rows) < self.K:
            raise ValueError(f"{path}: only {len(rows)} samples for arm {side}")
        self.P = np.array([[float(r[k]) for k in ("px", "py", "pz")] for r in rows])
        self.F = np.array([[float(r[k]) for k in ("Fbx", "Fby", "Fbz")] for r in rows])
        self.tree = cKDTree(self.P)

    def force(self, p):
        """Base-axes force at flange position `p` (base frame), or None if too far."""
        d, i = self.tree.query(np.asarray(p, float), k=self.K)
        if d[0] > self.RADIUS:
            return None
        return self.F[i].mean(0)


def _resolve(path):
    if os.path.isabs(path) or os.path.isfile(path):
        return path
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), path)


class _Arm:
    def __init__(self, node, name, cfg):
        self.name, self.cfg = name, cfg
        self.base = cfg["base_frame"]
        self.shelf_in_base = _frame(cfg["shelf_in_base"])      # pose of the shelf frame in the base
        self.base_in_shelf = self.shelf_in_base.inverse()
        self.tool_in_ee = _frame(cfg["tool_in_ee"])             # our tool frame (x = push) in the ee frame
        self.ee_in_tool = self.tool_in_ee.inverse()
        self.pub = node.create_publisher(PoseStamped, cfg["target_topic"], 10)
        self.joints = self.wrench = None
        node.create_subscription(JointState, cfg["joint_states_topic"], self._joints, 10)
        node.create_subscription(WrenchStamped, cfg["wrench_topic"], self._wrench, 10)
        self.fk = _FK(cfg["urdf_path"], cfg["ee_frame"])
        self.zero = (node.create_client(Trigger, cfg["zero_wrench_service"])
                     if cfg.get("zero_wrench_service") else None)
        self.source = cfg.get("wrench_source", "estimated")
        if self.source not in ("estimated", "ft"):
            raise ValueError(f"{name}: wrench_source must be estimated or ft")
        self.bias = (np.zeros(3), np.zeros(3))
        self.m_tool = float(cfg.get("tool_mass", 0.0))
        self.c_tool = np.asarray(cfg.get("tool_com_in_ft", [0.0, 0.0, 0.0]), float)
        self.sign = float(cfg.get("ft_sign", 1.0))
        self.ft_in_ee = _frame(cfg.get("ft_in_ee", {"xyz": [0, 0, 0], "quat": [0, 0, 0, 1]}))
        self.ft = deque(maxlen=2000)          # raw ATI samples (6,), sensor axes, N / Nm
        self.ft_scale = (1.0, 1.0)
        self.ft_t = -1e9                      # monotonic time of the last ATI sample
        self.ft_units = None
        if cfg.get("ft_topic"):
            node.create_subscription(WrenchStamped, cfg["ft_topic"], self._ft, 50)
        elif self.source == "ft":
            raise ValueError(f"{name}: wrench_source ft needs ft_topic")
        self.baseline, self.b0 = None, np.zeros(3)
        self.ticks = self.misses = 0

    def _joints(self, msg):
        self.joints = msg

    def _wrench(self, msg):
        self.wrench = msg

    def _ft(self, msg):
        w = msg.wrench
        kf, kt = self.ft_scale
        self.ft_t = time.monotonic()
        self.ft.append(np.array([kf * w.force.x, kf * w.force.y, kf * w.force.z,
                                 kt * w.torque.x, kt * w.torque.y, kt * w.torque.z]))

    def ft_last(self, n):
        """The last `n` ATI samples (sensor axes, N / Nm), [k, 6]."""
        k = min(int(n), len(self.ft))
        return np.array([self.ft[-1 - i] for i in range(k)])


class RosBackend(Node):
    def __init__(self, cfg):
        rclpy.init()
        super().__init__("kinova_dmp_insertion")
        self.cfg = cfg
        self.period = 1.0 / float(cfg["rate_hz"])
        self.arms = {"L": _Arm(self, "left", cfg["left"]), "R": _Arm(self, "right", cfg["right"])}
        if cfg.get("wrench_baseline"):
            path = _resolve(cfg["wrench_baseline"])
            for side, arm in self.arms.items():
                arm.baseline = _Baseline(path, side)
                z = arm.baseline.P[:, 2]
                print(f"  wrench baseline {arm.name}: {len(z)} samples, flange z {z.min():.2f}..{z.max():.2f} m"
                      f" (base), from {path}")
        self.raw = {}                          # last raw readings, for the log
        self.tares = []                        # every tare: the ATI biases, for the log's metadata
        self._exec = MultiThreadedExecutor()
        self._exec.add_node(self)
        threading.Thread(target=self._exec.spin, daemon=True).start()
        deadline = time.monotonic() + 10.0

        def missing():
            out = []
            for a in self.arms.values():
                out += [t for t, m in ((a.cfg["joint_states_topic"], a.joints),
                                       (a.cfg["wrench_topic"], a.wrench)) if m is None]
                if a.cfg.get("ft_topic") and not a.ft:
                    out.append(a.cfg["ft_topic"])
            return out
        while missing():
            if time.monotonic() > deadline:
                raise RuntimeError("nothing received on " + ", ".join(missing())
                                   + " - are both kinova_controller and both ft_raw running?")
            time.sleep(0.05)
        for arm in self.arms.values():
            if arm.cfg.get("ft_topic"):
                self._ft_units(arm)
        self._next = time.monotonic()

    def _ft_units(self, arm):
        """Ask ft_raw which units its NetBox is set to; everything is scaled to N, Nm."""
        srv = arm.cfg.get("ft_calibration_service")
        if not srv:
            print(f"  {arm.name} ATI: no ft_calibration_service, units taken as N / Nm")
            return
        cli = self.create_client(Trigger, srv)
        if not cli.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"{arm.name}: {srv} not available - is ft_raw running in /{arm.name}?")
        fut = cli.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 3.0:
            time.sleep(0.02)
        res = fut.result()
        if res is None or not res.success:
            raise RuntimeError(f"{arm.name}: ATI calibration not readable ({srv}): ft_raw would "
                               f"publish raw counts. Restart ft_raw, check the NetBox.")
        info = dict(line.split(": ", 1) for line in res.message.splitlines() if ": " in line)
        fu, tu = info.get("force_unit", "N").strip(), info.get("torque_unit", "Nm").strip()
        if fu not in FORCE_UNITS or tu not in TORQUE_UNITS:
            raise RuntimeError(f"{arm.name}: ATI units {fu} / {tu} not known: set the NetBox to N / Nm")
        arm.ft_scale = (FORCE_UNITS[fu], TORQUE_UNITS[tu])
        arm.ft.clear()                         # the samples so far were not scaled
        arm.ft_units = dict(force=fu, torque=tu, serial=info.get("serial"), cal_date=info.get("cal_date"))
        print(f"  {arm.name} ATI: serial {info.get('serial')}, units {fu} / {tu}"
              + ("" if arm.ft_scale == (1.0, 1.0) else
                 f" -> converted here to N / Nm (x{arm.ft_scale[0]:.4g}, x{arm.ft_scale[1]:.4g})"))
        t0 = time.monotonic()
        while len(arm.ft) < 100 and time.monotonic() - t0 < 2.0:
            time.sleep(0.02)

    # ---- clock
    def now(self):
        return time.monotonic()

    def wait(self):
        self._next += self.period
        dt = self._next - time.monotonic()
        if dt > 0:
            time.sleep(dt)
        else:
            self._next = time.monotonic()   # late: do not try to catch up

    # ---- out
    def command(self, tL, qL, tR, qR, k_tilt):
        """`k_tilt` is not sent: the lab controller takes its stiffness at launch only."""
        for arm, p, q in ((self.arms["L"], tL, qL), (self.arms["R"], tR, qR)):
            pb, qb = arm.shelf_in_base.apply(np.asarray(p, float), q)       # tool pose in the base
            pe = pb + qb.apply(arm.ee_in_tool.t)                          # ee pose in the base
            qe = qb * arm.ee_in_tool.R
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = arm.base
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pe)
            x, y, z, w = qe.as_quat()          # unit quaternion: the controller does not normalise
            msg.pose.orientation.x, msg.pose.orientation.y = float(x), float(y)
            msg.pose.orientation.z, msg.pose.orientation.w = float(z), float(w)
            arm.pub.publish(msg)

    # ---- in
    def _ee_pose(self, arm):
        """(ee position, ee rotation) in the base, by forward kinematics."""
        return arm.fk.pose(arm.joints)

    def _tool_pose(self, arm):
        pe, qe = self._ee_pose(arm)
        pt, qt = pe + qe.apply(arm.tool_in_ee.t), qe * arm.tool_in_ee.R
        return arm.base_in_shelf.apply(pt, qt)                  # tool in shelf

    def tare(self):
        """Zero both wrench readings with the tools in free air: the arm's estimate by
        its zero_wrench service (always: it is logged), the ATI by a bias averaged here."""
        time.sleep(0.5)
        for arm in self.arms.values():
            if arm.zero is not None and arm.zero.wait_for_service(timeout_sec=2.0):
                fut = arm.zero.call_async(Trigger.Request())
                t0 = time.monotonic()
                while not fut.done() and time.monotonic() - t0 < 3.0:
                    time.sleep(0.02)
            else:
                self.get_logger().warn(f"{arm.name}: no zero_wrench service, estimate not tared")
        time.sleep(0.5)                      # it averages 50 samples (~200 ms); the ATI fills its buffer
        rec = {"t": self.now(), "t_wall": time.time()}
        for arm in self.arms.values():
            if arm.source == "estimated" and arm.baseline is not None:
                b0 = arm.baseline.force(self._ee_pose(arm)[0])
                if b0 is None:
                    self.get_logger().warn(f"{arm.name}: tare pose off the wrench baseline, "
                                           f"correction relative to its own tare")
                arm.b0 = np.zeros(3) if b0 is None else b0
            if arm.source == "ft":
                w = arm.ft_last(500)
                if len(w) < 50:
                    raise RuntimeError(f"{arm.name}: only {len(w)} ATI samples for the tare")
                m, sd = w.mean(0), w.std(0)
                _ps, qs = self._ft_pose(arm)
                g_s = qs.inv().apply(arm.m_tool * G_WORLD)        # the tool's weight, sensor axes
                arm.bias = (arm.sign * m[:3] - g_s, arm.sign * m[3:] - np.cross(arm.c_tool, g_s))
                rec[arm.name] = dict(bias_F=np.round(arm.bias[0], 4).tolist(),
                                     bias_T=np.round(arm.bias[1], 5).tolist(),
                                     sd_F=np.round(sd[:3], 4).tolist(), sd_T=np.round(sd[3:], 5).tolist(),
                                     samples=len(w))
                print(f"  {arm.name} ATI tare: bias F {np.round(arm.bias[0], 2).tolist()} N; noise sd "
                      f"F {np.round(sd[:3], 3).tolist()} N, T {np.round(sd[3:], 4).tolist()} Nm")
        self.tares.append(rec)

    def _ft_pose(self, arm):
        """The ATI frame (position, rotation) in the shelf frame, by forward kinematics."""
        pe, qe = self._ee_pose(arm)
        ps, qs = pe + qe.apply(arm.ft_in_ee.t), qe * arm.ft_in_ee.R
        return arm.base_in_shelf.apply(ps, qs)

    def ft_on_tool(self, arm, w):
        """ATI sample (sensor axes, N / Nm) -> (force, moment about the sensor origin,
        sensor origin) that the world - the piece - exerts on the tool, shelf frame,
        bias and tool weight removed."""
        ps, qs = self._ft_pose(arm)
        g = arm.m_tool * G_WORLD
        f = qs.apply(arm.sign * w[:3] - arm.bias[0]) - g
        m = qs.apply(arm.sign * w[3:] - arm.bias[1]) - np.cross(qs.apply(arm.c_tool), g)
        return f, m, ps

    def read(self):
        out, raw = {}, {}
        n_avg = max(1, int(round(self.period * 1000.0)))    # ATI samples in one control period (1 kHz)
        for s, arm in self.arms.items():
            pt, qt = self._tool_pose(arm)
            w = arm.wrench
            raw["est" + s] = [w.wrench.force.x, w.wrench.force.y, w.wrench.force.z,
                              w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z]
            ft = arm.ft_last(n_avg).mean(0) if arm.ft else np.full(6, np.nan)
            if arm.source == "ft" and time.monotonic() - arm.ft_t > FT_STALE:
                from .controller import Abort
                raise Abort(f"{arm.name} ATI silent for {1000 * (time.monotonic() - arm.ft_t):.0f} ms "
                            f"({arm.cfg['ft_topic']}): is ft_raw still running?")
            raw["ft" + s] = list(ft)
            if arm.source == "estimated":
                # the external wrench ON the arm, ee axes, about the ee origin; the tool's
                # weight is the URDF's business (and what is left of it, the tare's)
                pb, qb = self._ee_pose(arm)
                pe, qe = arm.base_in_shelf.apply(pb, qb)
                f_on_tool = qe.apply([w.wrench.force.x, w.wrench.force.y, w.wrench.force.z])
                if arm.baseline is not None:
                    arm.ticks += 1
                    b = arm.baseline.force(pb)
                    if b is None:
                        arm.misses += 1
                        if arm.misses == 1:
                            self.get_logger().warn(f"{arm.name}: flange off the wrench baseline "
                                                   f"(> {1000 * _Baseline.RADIUS:.0f} mm): not corrected there")
                    else:
                        f_on_tool = f_on_tool - arm.base_in_shelf.R.apply(b - arm.b0)
                m_on_tool = qe.apply([w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z])
                origin = pe
            else:
                f_on_tool, m_on_tool, origin = self.ft_on_tool(arm, ft)
            # the hand on the piece is the opposite; moment moved to the tool origin
            F = -np.asarray(f_on_tool)
            T = -np.asarray(m_on_tool) + np.cross(origin - pt, F)
            out.update({f"p{s}": pt, f"q{s}": qt.as_quat(), f"F{s}": F, f"T{s}": T})
        self.raw = raw
        return out

    def close(self):
        for arm in self.arms.values():
            if arm.baseline is not None and arm.ticks:
                print(f"  wrench baseline {arm.name}: corrected {100 * (1 - arm.misses / arm.ticks):.0f}% "
                      f"of {arm.ticks} readings")
        self._exec.shutdown()
        self.destroy_node()
        rclpy.shutdown()
