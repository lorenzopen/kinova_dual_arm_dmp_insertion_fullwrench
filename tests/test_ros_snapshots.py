"""Exercise the ROS adapter with message/service doubles, without a ROS installation.

This tests our snapshot/decoding logic, not middleware timing or the hardware controller.
"""
import importlib.util
import contextlib
import io
import sys
import types
import time
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kinova_dual_arm_dmp_insertion.geometry import Frame
from kinova_dual_arm_dmp_insertion.controller import Abort


def module(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    return m


class RosSnapshots(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        service = type('Service', (), {'Request': type('Request', (), {})})
        doubles = {
            'rclpy': module('rclpy'),
            'rclpy.executors': module('rclpy.executors', SingleThreadedExecutor=object),
            'rclpy.node': module('rclpy.node', Node=object),
            'rclpy.parameter': module('rclpy.parameter', parameter_value_to_python=lambda v:v.value),
            'rcl_interfaces': module('rcl_interfaces'),
            'rcl_interfaces.srv': module('rcl_interfaces.srv', GetParameters=service),
            'geometry_msgs': module('geometry_msgs'),
            'geometry_msgs.msg': module('geometry_msgs.msg', PoseStamped=object, WrenchStamped=object),
            'sensor_msgs': module('sensor_msgs'),
            'sensor_msgs.msg': module('sensor_msgs.msg', JointState=object),
            'std_srvs': module('std_srvs'),
            'std_srvs.srv': module('std_srvs.srv', Trigger=service),
        }
        path = Path(__file__).parents[1]/'kinova_dual_arm_dmp_insertion/ros_backend.py'
        spec = importlib.util.spec_from_file_location('kinova_dual_arm_dmp_insertion._test_ros_backend',path)
        cls.backend_module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, doubles):
            spec.loader.exec_module(cls.backend_module)

    def test_pose_force_and_joint_log_share_one_fk_snapshot(self):
        stamp = NS(sec=1, nanosec=500)
        joints = NS(header=NS(stamp=stamp), position=list(range(7)), velocity=[.1]*7, effort=[2.]*7)
        vector = NS(x=0.,y=0.,z=0.)
        now = time.monotonic()
        arm = NS(name='left', joints=joints, joints_t=now, ft_t=now, ft_stamp=1.,
                 ft=deque([np.array([0.,1.,0.,0.,0.,0.])]*2), ft_rate=200.,
                 wrench=NS(wrench=NS(force=vector,torque=vector)), cfg={'ft_topic':'/left/ft'},
                 source='ft', fk=NS(pose=Mock(return_value=(np.zeros(3), Rot.identity()))),
                 tool_in_ee=Frame(), base_in_shelf=Frame(), ft_in_ee=Frame([.01,0,0]),
                 m_tool=0., c_tool=np.zeros(3), sign=1., bias=(np.zeros(3),np.zeros(3)))
        arm.ft_last = lambda n: np.array(list(arm.ft)[-n:])
        b = self.backend_module.RosBackend.__new__(self.backend_module.RosBackend)
        b.arms, b.period = {'L':arm}, .01
        out = b.read()
        arm.fk.pose.assert_called_once_with(joints)
        np.testing.assert_allclose(out['FL'], [0,-1,0])
        np.testing.assert_allclose(out['TL'], [0,0,-.01])
        self.assertEqual(b.telemetry['qL_j7'], 6.)
        self.assertEqual(b.telemetry['tauL_j7'], 2.)
        self.assertAlmostEqual(b.telemetry['joint_stamp_L'], 1.0000005)

    def test_stale_joint_stream_stops_before_using_geometry(self):
        b = self.backend_module.RosBackend.__new__(self.backend_module.RosBackend)
        b.arms, b.period = {'L':NS(joints=object(), joints_t=time.monotonic()-1., name='left')}, .01
        with self.assertRaisesRegex(Abort, 'joint_states silent'):
            b.read()

    def test_controller_snapshot_preserves_parameter_values_and_destroys_client(self):
        b = self.backend_module.RosBackend.__new__(self.backend_module.RosBackend)
        b.arms = {'L':NS(name='left',cfg={'target_topic':'/left_arm/target_pose'})}
        b.controller_parameters = {}
        values = [[1500.]*3+[20.]*3, [60.]*3+[7.]*3, 1000., True, True, '/robot.urdf','end_effector_link']
        future = NS(done=lambda:True, result=lambda:NS(values=[NS(value=v) for v in values]))
        cli = NS(wait_for_service=lambda timeout_sec:True, call_async=Mock(return_value=future))
        b.create_client, b.destroy_client, b.get_logger = Mock(return_value=cli), Mock(), Mock()
        with contextlib.redirect_stdout(io.StringIO()):
            b._read_controller_parameters()
        self.assertEqual(b.controller_parameters['L']['cart_k'], values[0])
        self.assertEqual(b.controller_parameters['L']['cart_d'], values[1])
        self.assertEqual(b.create_client.call_args.args[1], '/left_arm/real_robot_controller_node/get_parameters')
        self.assertEqual(cli.call_async.call_args.args[0].names[:2], ['cart_k','cart_d'])
        b.destroy_client.assert_called_once_with(cli)


if __name__ == '__main__':
    unittest.main()
