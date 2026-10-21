"""One wrist ATI, light: tele_ros2's ft_raw without its 2 kHz timer, publishing block means.

    python3 ati_node.py --ros-args -r __node:=ft_raw -r __ns:=/left_arm -p ip_address:=192.180.0.132 \\
        -p configure_sensor:=false [-p every:=5] [-p publish_raw:=false]

Same node name, calibration fetch, units and services as ft_raw (~/set_zero, ~/clear_zero,
~/get_calibration), so ros_backend and ft_check see no difference. What changes: ft_raw drains the
RDT socket from a 2 kHz rclpy timer and publishes every sample (~56% of a Jetson core per sensor),
and taking that 1 kHz back in Python cost another core and stalled `insert` (2026-10-21: ticks of
30-260 ms, the arms jerked; then the extra load pushed the controllers' 3 ms Kortex Refresh into
timeouts during homing). Here a thread blocks on the socket and publishes the mean of each block of
`every` samples on `ft_data_avg` (200 Hz for 5), the topic cell.yaml reads; the raw 1 kHz on
`ft_data_raw` only with publish_raw:=true.
"""
import socket
import struct
import sys
import threading

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import ExternalShutdownException

sys.path.insert(0, "/home/nvidia/tele_ros2/src/ati_ft_tools/ati_ft_tools")
import ft_raw as R  # noqa: E402  (tele_ros2's source, not its workspace: no kinova_* clash)


class AtiNode(R.FTRaw):
    def __init__(self):
        super().__init__()                       # calibration, sensor start, services, ft_data_raw publisher
        self.destroy_timer(self.timer)           # no 2 kHz timer: the thread below reads the socket
        self.declare_parameter("every", 5)
        self.declare_parameter("avg_topic", "ft_data_avg")
        self.declare_parameter("publish_raw", False)
        self.every = max(1, int(self.get_parameter("every").value))
        self.publish_raw = bool(self.get_parameter("publish_raw").value)
        self.pub_avg = self.create_publisher(WrenchStamped, self.get_parameter("avg_topic").value, 50)
        threading.Thread(target=self._rx, daemon=True).start()

    def _msg(self, v):
        out = WrenchStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        (out.wrench.force.x, out.wrench.force.y, out.wrench.force.z) = (x * self.force_scale for x in v[:3])
        (out.wrench.torque.x, out.wrench.torque.y, out.wrench.torque.z) = (x * self.torque_scale for x in v[3:])
        return out

    def _rx(self):
        acc, n = [0] * 6, 0
        while rclpy.ok():
            try:
                data = self.sock.recv(R.RECORD_SIZE + 1)
            except socket.timeout:
                continue
            except OSError:
                return                               # socket closed at shutdown
            if len(data) != R.RECORD_SIZE:
                continue
            _, _, status, *v = struct.unpack(R.RECORD_FMT, data)
            if not self._first_packet_logged:
                self.get_logger().info(f"Receiving RDT packets, status=0x{status:08x}")
                self._first_packet_logged = True
            if status != 0:
                self.get_logger().warn(f"Sensor status non-zero: 0x{status:08x}", throttle_duration_sec=5.0)
            if self.publish_raw:
                self.pub.publish(self._msg(v))
            for i in range(6):
                acc[i] += v[i]
            n += 1
            if n >= self.every:
                self.pub_avg.publish(self._msg([a / n for a in acc]))
                acc, n = [0] * 6, 0


def main():
    rclpy.init()
    node = AtiNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        else:
            try:
                node._send_cmd(R.CMD_STOP)
            except OSError:
                pass


if __name__ == "__main__":
    main()
