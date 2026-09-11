"""Offline tests of measured movement and velocity source arbitration."""
import math
import time
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, String

from mdp_hardware_bridge.motion_controller_node import MotionControllerNode
from mdp_interfaces.msg import MoveCommand
from mdp_interfaces.srv import ExecuteMoves


class TestMotionController(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = MotionControllerNode()
        self.node._velocity_pub = MagicMock()
        self.node._estop_pub = MagicMock()
        self.feedback()

    def tearDown(self):
        self.node.destroy_node()

    def feedback(self, x=0.2, y=0.2, yaw=math.pi / 2, us=0.5):
        pose = PoseStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.node._on_pose(pose)
        sensor = Range()
        sensor.header.stamp = pose.header.stamp
        sensor.range = us
        self.node._on_range(sensor)

    def execute(self, command="FC", value=10):
        request = ExecuteMoves.Request(commands=[MoveCommand(command=command, value=value)])
        return self.node._handle_execute_moves(request, ExecuteMoves.Response())

    def velocities(self):
        return [(c.args[0].linear.x, c.args[0].angular.z)
                for c in self.node._velocity_pub.publish.call_args_list]

    def test_measured_forward_and_reverse_without_fin(self):
        for cmd, y, sign in (("FC", 0.3, 1), ("BC", 0.1, -1)):
            self.feedback()
            self.node._velocity_pub.reset_mock()
            with patch("mdp_hardware_bridge.motion_controller_node.time.sleep",
                       side_effect=lambda _, end=y: self.feedback(y=end)):
                self.assertTrue(self.execute(cmd).success)
            self.assertGreater(self.velocities()[0][0] * sign, 0)
            self.assertEqual(self.velocities()[-1], (0, 0))

    def test_all_turn_directions_use_measured_heading(self):
        for cmd, delta, sign in (("FL", 1, 1), ("FR", -1, 1),
                                 ("BL", -1, -1), ("BR", 1, -1)):
            self.feedback()
            self.node._velocity_pub.reset_mock()
            with patch("mdp_hardware_bridge.motion_controller_node.time.sleep",
                       side_effect=lambda _, d=delta: self.feedback(yaw=math.pi / 2 + d)):
                self.assertTrue(self.execute(cmd, 57).success)
            speed, yaw = self.velocities()[0]
            self.assertGreater(speed * sign, 0)
            self.assertGreater(yaw * delta, 0)
            self.assertAlmostEqual(abs(speed / yaw), 0.21)

    def test_turn_across_wrap(self):
        self.feedback(yaw=math.radians(179))
        with patch("mdp_hardware_bridge.motion_controller_node.time.sleep",
                   side_effect=lambda _: self.feedback(yaw=math.radians(-171))):
            self.assertTrue(self.execute("FL", 10).success)

    def test_no_feedback_no_nominal_completion(self):
        self.node._telemetry_stamp -= 1
        self.assertEqual(self.execute().status, "STALE_TELEMETRY")
        self.assertTrue(self.node._estop_event.is_set())
        self.node._estop_pub.publish.assert_called_once()
        self.assertTrue(all(v == (0, 0) for v in self.velocities()))

    def test_feedback_loss_mid_move_stops(self):
        def lose(_):
            self.node._telemetry_stamp -= 1
        with patch("mdp_hardware_bridge.motion_controller_node.time.sleep", side_effect=lose):
            self.assertEqual(self.execute().status, "STALE_TELEMETRY")
        self.assertEqual(self.velocities()[-1], (0, 0))

    def test_mid_move_estop_stays_latched(self):
        with patch("mdp_hardware_bridge.motion_controller_node.time.sleep",
                   side_effect=lambda _: self.node._on_estop(Empty())):
            self.assertFalse(self.execute().success)
        self.node._on_android_cmd(String(data="ALG|1,2,3,N"))
        self.assertFalse(self.execute().success)
        self.assertTrue(self.node._estop_event.is_set())

    def test_external_estop_is_not_republished(self):
        self.node._on_estop(Empty())
        self.assertTrue(self.node._estop_event.is_set())
        self.node._estop_pub.publish.assert_not_called()
        self.assertEqual(self.velocities()[-1], (0, 0))

    def test_reset_cancels_active_generation(self):
        with patch("mdp_hardware_bridge.motion_controller_node.time.sleep",
                   side_effect=lambda _: self.node._on_android_cmd(String(data="RESET"))):
            self.assertEqual(self.execute().status, "CANCELLED")
        self.assertFalse(self.node._busy.is_set())
        self.assertFalse(self.node._telemetry_fresh())

    def test_teleop_expiration_zero_and_no_replay(self):
        msg = Twist()
        msg.linear.x = 0.1
        self.node._on_teleop(msg)
        self.node._teleop_stamp -= 1
        self.node._teleop_tick()
        self.assertEqual(self.velocities()[-1], (0, 0))
        count = len(self.velocities())
        self.feedback()
        self.node._teleop_tick()
        self.assertEqual(len(self.velocities()), count)

    def test_teleop_continuously_stops_for_raw_proximity(self):
        msg = Twist()
        msg.linear.x = 0.1
        self.node._on_teleop(msg)
        self.assertGreater(self.velocities()[-1][0], 0)
        self.feedback(us=0.12)
        self.node._teleop_tick()
        self.assertTrue(self.node._estop_event.is_set())
        self.assertEqual(self.node._stop_reason, "PROXIMITY:ULTRASONIC")
        self.assertEqual(self.velocities()[-1], (0, 0))

    def test_zero_no_echo_is_not_a_proximity_fault(self):
        self.feedback(us=0.0)
        self.assertEqual(self.node._forward_safety_error(True), "")

    def test_service_owns_output_and_drops_teleop_during_motion(self):
        msg = Twist()
        msg.linear.x = -0.3
        def arrive(_):
            self.node._on_teleop(msg)
            self.feedback(y=0.3)
        with patch("mdp_hardware_bridge.motion_controller_node.time.sleep", side_effect=arrive):
            self.assertTrue(self.execute().success)
        self.assertFalse(any(speed < 0 for speed, yaw in self.velocities()))
        self.assertIsNone(self.node._teleop_target)

    def test_active_teleop_rejects_service_and_zero_releases(self):
        msg = Twist()
        msg.linear.x = 0.1
        self.node._on_teleop(msg)
        self.assertEqual(self.execute().status, "BUSY_LOCAL")
        self.node._on_teleop(Twist())
        self.assertIsNone(self.node._teleop_target)

    def test_invalid_teleop_stops(self):
        for speed, yaw in ((math.nan, 0), (0, 1), (0.36, 0), (0.1, 1)):
            msg = Twist()
            msg.linear.x, msg.angular.z = float(speed), float(yaw)
            self.node._on_teleop(msg)
            self.assertEqual(self.velocities()[-1], (0, 0))

    def test_fu_bu_live_range_and_invalid_target(self):
        for cmd, initial in (("FU", 0.5), ("BU", 0.15)):
            self.feedback(us=initial)
            with patch("mdp_hardware_bridge.motion_controller_node.time.sleep",
                       side_effect=lambda _: self.feedback(us=0.2)):
                self.assertTrue(self.execute(cmd, 20).success)
        self.assertFalse(self.execute("FU", 8).success)

    def test_stale_range_cannot_complete_ultrasonic_move(self):
        self.node._range_stamp -= 1
        self.assertEqual(self.execute("FU", 20).status, "INVALID_RANGE")

    def test_old_pose_header_cannot_refresh_watchdog(self):
        stamp = self.node._telemetry_stamp
        self.node._on_pose(PoseStamped())
        self.assertEqual(self.node._telemetry_stamp, stamp)

    def test_maintenance_proxy_and_reject_mixed(self):
        client = MagicMock()
        client.service_is_ready.return_value = True
        future = client.call_async.return_value
        future.done.return_value = True
        future.result.return_value = ExecuteMoves.Response(success=True, status="FIN")
        self.node._maintenance_client = client
        self.assertTrue(self.execute("GC", 0).success)
        self.assertTrue(all(v == (0, 0) for v in self.velocities()))
        req = ExecuteMoves.Request(commands=[MoveCommand(command="GC", value=0),
                                             MoveCommand(command="FC", value=10)])
        self.assertFalse(self.node._handle_execute_moves(req, ExecuteMoves.Response()).success)
