"""The two Kinova arms and their wrist ATIs, in ONE terminal (GUIDA.md, T1).

    source /opt/ros/jazzy/setup.bash && source /home/nvidia/ros2_ws/install/setup.bash
    ros2 launch /home/nvidia/lorenzo/kinova_dual_arm_dmp_insertion_fullwrench/launch/robot.launch.py [arms:=both|right|left|none] [ati:=true|false]

Arms: the lab's kinova_controller.py of ~/ros2_ws (kinova_controller_pkgs, the one of the 15-20/10
runs), one per arm in /right_arm (.107) and /left_arm (.200). It does not use the robot's Home
action: it enters torque control from the current pose and drives the arm to home IN IMPEDANCE
over `home_seconds`, then holds it in Cartesian impedance at the flange (end_effector_link).
On any exit it puts the arm back in POSITION mode. The right arm starts first; the left one
`left_delay` s later, when the right is already home and still.

ATIs: ati_node.py = ft_raw of tele_ros2 (from its SOURCE file, plain python3) made light, so tele_ros2's workspace
(kinova_* packages with the same names as ~/ros2_ws) is never sourced here. The cells are
mounted the other way round from tele_ros2's map: left .200 has the .132, right .107 the .131.

The URDF is urdf/gen3_ati_tool.urdf of this repo: gen3.urdf + the 150 g bolted on the flange.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
URDF = os.path.join(HERE, "urdf", "gen3_ati_tool.urdf")
FRICTION_URDF = "/home/nvidia/lorenzo/gen3_bare.urdf"     # the identified friction, as on 15-20/10
# one light process per ATI: tele_ros2's ft_raw (its source) without the 2 kHz timer, publishing the
# means of `ft_every` samples on <ns>/ft_data_avg, the topic cell.yaml reads; raw 1 kHz on
# <ns>/ft_data_raw only with ati_raw:=true (2026-10-21: the 1 kHz in Python cost ~a core per sensor,
# stalled insert and pushed the controllers' 3 ms Kortex Refresh into homing timeouts)
ATI_NODE = os.path.join(HERE, "kinova_dual_arm_dmp_insertion", "ati_node.py")

ARMS = {"right": {"ns": "right_arm", "ip": "192.180.0.107", "ati": "192.180.0.131"},
        "left": {"ns": "left_arm", "ip": "192.180.0.200", "ati": "192.180.0.132"}}


def _setup(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    which = lc("arms").strip().lower()
    sides = {"both": ["right", "left"], "none": []}.get(which, [which])
    if any(s not in ARMS for s in sides):
        raise RuntimeError(f"arms:={which}: use both, right, left or none")
    cart_k = [float(x) for x in lc("cart_k").split()]
    cart_d = [float(x) for x in lc("cart_d").split()]
    actions = []
    for i, side in enumerate(sides):
        a = ARMS[side]
        ctrl = Node(
            package="kinova_control", executable="kinova_controller.py",
            namespace=a["ns"], output="screen", emulate_tty=True,
            arguments=["--ip", a["ip"]],
            parameters=[{
                "initial_mode": "cartesian",
                "home_on_start": lc("home") == "true",
                "home_seconds": float(lc("home_seconds")),
                "ee_frame": "end_effector_link",
                "urdf_path": URDF,
                "friction_enable": lc("friction") == "true",
                "friction_urdf_path": FRICTION_URDF,
                "kalman_enable": lc("kalman") == "true",
                "cart_k": cart_k,
                "cart_d": cart_d,
            }])
        # the second arm waits for the first to be home and still
        actions.append(ctrl if i == 0 else TimerAction(period=float(lc("left_delay")), actions=[ctrl]))
    for side in (sides or list(ARMS)):            # arms:=none -> the two ATIs alone
        a = ARMS[side]
        if lc("ati") == "true":
            actions.append(ExecuteProcess(
                cmd=["python3", ATI_NODE, "--ros-args", "-r", "__node:=ft_raw", "-r", f"__ns:=/{a['ns']}",
                     "-p", f"ip_address:={a['ati']}", "-p", f"frame_id:={a['ns']}/tool_frame",
                     "-p", "configure_sensor:=false", "-p", f"every:={lc('ft_every')}",
                     "-p", f"publish_raw:={lc('ati_raw')}"],
                name=f"ati_{side}", output="screen", emulate_tty=True))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arms", default_value="both",
                              description="both | right | left | none (none = only the ATIs)"),
        DeclareLaunchArgument("ati", default_value="true", description="also start the wrist ATIs"),
        DeclareLaunchArgument("home", default_value="true", description="drive to home in impedance first"),
        DeclareLaunchArgument("home_seconds", default_value="8.0", description="[s] the homing drive"),
        DeclareLaunchArgument("left_delay", default_value="14.0",
                              description="[s] the left arm starts this much after the right one"),
        # both ON as in the 15-20/10 runs. Off = more passive, a few mm less accurate; the ATI
        # forces do not depend on them. Kalman filters q, dq for the friction model and the
        # torque-based wrench estimate (logged only); friction adds the joints' Stribeck torque.
        DeclareLaunchArgument("kalman", default_value="true", description="Kalman filter on q, dq"),
        DeclareLaunchArgument("friction", default_value="true", description="joint friction compensation"),
        DeclareLaunchArgument("ati_raw", default_value="false",
                              description="also publish the raw 1 kHz on ft_data_raw (more CPU)"),
        DeclareLaunchArgument("ft_every", default_value="5",
                              description="ATI samples per mean on ft_data_avg (5 -> 200 Hz; = ft_rate_hz in cell.yaml)"),
        DeclareLaunchArgument("cart_k", default_value="1000 1000 1000 20 20 20"),
        DeclareLaunchArgument("cart_d", default_value="5 5 5 1 1 1"),
        OpaqueFunction(function=_setup),
    ])
