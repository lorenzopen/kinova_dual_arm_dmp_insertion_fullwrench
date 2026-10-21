"""One wrist ATI, 1 kHz in, block means out. Its OWN process (launch/robot.launch.py starts one per arm).

    python3 ft_relay.py --in /left_arm/ft_data_raw --out /left_arm/ft_data_avg --every 5

Taking the 1 kHz stream in Python costs about a whole Jetson core (2026-10-21: ~0.5 ms per message),
and inside the `insert` process those callbacks held the interpreter while the 100 Hz loop and the
SOCP waited: ticks of 30-200 ms, the arms jerked. Here each block of `--every` samples is averaged
(a box filter, no aliasing of the 1 kHz noise) and published once, stamped with its last sample:
200 Hz for --every 5. The raw topic stays as it is, for the bag.
"""
import argparse

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="src", required=True)
    ap.add_argument("--out", dest="dst", required=True)
    ap.add_argument("--every", type=int, default=5, help="samples per published mean")
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node("ft_relay_" + a.dst.strip("/").replace("/", "_"))
    pub = node.create_publisher(WrenchStamped, a.dst, 50)
    acc = [0.0] * 6
    n = [0]

    def cb(msg):
        f, t = msg.wrench.force, msg.wrench.torque
        for i, v in enumerate((f.x, f.y, f.z, t.x, t.y, t.z)):
            acc[i] += v
        n[0] += 1
        if n[0] >= a.every:
            out = WrenchStamped()
            out.header = msg.header
            k = 1.0 / n[0]
            (out.wrench.force.x, out.wrench.force.y, out.wrench.force.z,
             out.wrench.torque.x, out.wrench.torque.y, out.wrench.torque.z) = (v * k for v in acc)
            pub.publish(out)
            acc[:] = [0.0] * 6
            n[0] = 0

    node.create_subscription(WrenchStamped, a.src, cb, 50)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():                # after Ctrl-C the launch has already shut the context down
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
