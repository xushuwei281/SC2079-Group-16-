#!/usr/bin/env python3
"""Unit tests for the Android Bluetooth Bridge Node."""

import math
import os
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from mdp_android_bridge.android_bridge_node import AndroidBridgeNode
from std_msgs.msg import Empty, String


class TestAndroidBridge(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        with patch.object(AndroidBridgeNode, "_connection_worker", return_value=None):
            self.node = AndroidBridgeNode()
        # Mock serial to capture outbound messages
        self.mock_serial = MagicMock()
        self.mock_serial.is_open = True
        self.mock_serial.readline.return_value = b""
        self.node._serial = self.mock_serial

    def tearDown(self):
        self.node.destroy_node()

    def test_movement_command_conversion(self):
        """Test that FW, BW, TL, TR convert to correct MoveCommand values."""
        self.node._move_client.wait_for_service = MagicMock(return_value=True)
        self.node._move_client.call_async = MagicMock()

        # FW:20 -> FC 20
        self.node._dispatch("FW:20")
        call_args = self.node._move_client.call_async.call_args[0][0]
        self.assertEqual(call_args.commands[0].command, "FC")
        self.assertEqual(call_args.commands[0].value, 20)
        self.node._is_moving = False

        # BW:35 -> BC 35
        self.node._dispatch("BW:35")
        call_args = self.node._move_client.call_async.call_args[0][0]
        self.assertEqual(call_args.commands[0].command, "BC")
        self.assertEqual(call_args.commands[0].value, 35)
        self.node._is_moving = False

        # TL:90 -> FL 90
        self.node._dispatch("TL:90")
        call_args = self.node._move_client.call_async.call_args[0][0]
        self.assertEqual(call_args.commands[0].command, "FL")
        self.assertEqual(call_args.commands[0].value, 90)
        self.node._is_moving = False

        # TR:45 -> FR 45
        self.node._dispatch("TR:45")
        call_args = self.node._move_client.call_async.call_args[0][0]
        self.assertEqual(call_args.commands[0].command, "FR")
        self.assertEqual(call_args.commands[0].value, 45)

    def test_mm_auto_conversion(self):
        """Test that large millimeter values (e.g. FW:200) are converted to cm (20)."""
        self.node._move_client.wait_for_service = MagicMock(return_value=True)
        self.node._move_client.call_async = MagicMock()

        self.node._dispatch("FW:200")
        call_args = self.node._move_client.call_async.call_args[0][0]
        self.assertEqual(call_args.commands[0].command, "FC")
        self.assertEqual(call_args.commands[0].value, 20)

    def test_velocity_command_publishes_twist(self) -> None:
        """Android joystick velocity uses SI units on the teleop velocity topic."""
        self.node._teleop_pub.publish = MagicMock()
        self.node._dispatch("VEL:0.100,-0.400")

        msg = self.node._teleop_pub.publish.call_args.args[0]
        self.assertIsInstance(msg, Twist)
        self.assertAlmostEqual(msg.linear.x, 0.1)
        self.assertAlmostEqual(msg.angular.z, -0.4)

        self.node._dispatch("VEL:0.350,1.667")
        msg = self.node._teleop_pub.publish.call_args.args[0]
        self.assertAlmostEqual(msg.linear.x, 0.35)
        self.assertAlmostEqual(msg.angular.z, 1.667)

    def test_invalid_velocity_publishes_zero(self) -> None:
        """Malformed, excessive, and impossible Ackermann commands fail stopped."""
        self.node._teleop_pub.publish = MagicMock()
        for command in ("VEL:nan,0", "VEL:0.36,0", "VEL:0.05,1.0", "VEL:0.1"):
            self.node._dispatch(command)
            msg = self.node._teleop_pub.publish.call_args.args[0]
            self.assertEqual(msg.linear.x, 0.0)
            self.assertEqual(msg.angular.z, 0.0)

    def test_estop_dispatch(self):
        """Test that STP and STOP trigger e-stop publisher."""
        self.node._estop_pub.publish = MagicMock()
        self.node._dispatch("STP")
        self.node._estop_pub.publish.assert_called_once()

        self.node._estop_pub.publish.reset_mock()
        self.node._dispatch("STOP")
        self.node._estop_pub.publish.assert_called_once()

    def test_planner_commands_forwarding(self):
        """Test that ALG, START, and RESET commands publish to /android/cmd."""
        self.node._cmd_pub.publish = MagicMock()

        self.node._dispatch("ALG|1,10,20,N|2,50,60,E")
        self.node._cmd_pub.publish.assert_called_once()
        self.assertEqual(self.node._cmd_pub.publish.call_args[0][0].data, "ALG|1,10,20,N|2,50,60,E")

        self.node._cmd_pub.publish.reset_mock()
        self.node._dispatch("START")
        self.node._cmd_pub.publish.assert_called_once()
        self.assertEqual(self.node._cmd_pub.publish.call_args[0][0].data, "START")

        # Task 2 triggers: STM:sp (Android SP button), SP, START_TASK2
        self.node._cmd_pub.publish.reset_mock()
        self.node._dispatch("STM:sp")
        self.node._cmd_pub.publish.assert_called_once()
        self.assertEqual(self.node._cmd_pub.publish.call_args[0][0].data, "STM:sp")

        self.node._cmd_pub.publish.reset_mock()
        self.node._dispatch("SP")
        self.node._cmd_pub.publish.assert_called_once()
        self.assertEqual(self.node._cmd_pub.publish.call_args[0][0].data, "SP")

        self.node._cmd_pub.publish.reset_mock()
        self.node._dispatch("START_TASK2")
        self.node._cmd_pub.publish.assert_called_once()
        self.assertEqual(self.node._cmd_pub.publish.call_args[0][0].data, "START_TASK2")

    def test_pose_to_tablet_format(self):
        """Test that /robot_pose converts to Checklist C.10 format: ROBOT,<x>,<y>,<direction>."""
        pose = PoseStamped()
        pose.pose.position.x = 0.50  # 50 cm -> grid 5
        pose.pose.position.y = 1.20  # 120 cm -> grid 12
        # 90 degrees yaw (Z = sin(pi/4), W = cos(pi/4)) -> North
        pose.pose.orientation.z = math.sin(math.pi / 4.0)
        pose.pose.orientation.w = math.cos(math.pi / 4.0)

        self.node._on_pose(pose)

        # Default Checklist C.10 / Android format
        written = self.mock_serial.write.call_args[0][0].decode("utf-8")
        self.assertEqual(written, "ROBOT,<5>,<12>,<N>\n")

        # Test configurable legacy format (cm, degrees, no angle brackets)
        self.node._use_brackets = False
        self.node._coords_in_cm = True
        self.node._direction_as_cardinal = False
        self.node._on_pose(pose)
        written = self.mock_serial.write.call_args[0][0].decode("utf-8")
        self.assertEqual(written, "ROBOT,50,120,90\n")

    def test_hires_pose_sent_alongside_robot_line(self):
        """Every /robot_pose update should also emit a full-precision POSE
        line (Checklist C.10's ROBOT,<x>,<y>,<dir> is deliberately coarse --
        4 headings, 10cm cells -- which is too coarse to show smooth
        rotation, e.g. while circling)."""
        pose = PoseStamped()
        pose.pose.position.x = 0.523  # 52.3 cm
        pose.pose.position.y = 1.187  # 118.7 cm
        pose.pose.orientation.z = math.sin(math.pi / 4.0)
        pose.pose.orientation.w = math.cos(math.pi / 4.0)

        self.node._on_pose(pose)

        calls = [c[0][0].decode("utf-8") for c in self.mock_serial.write.call_args_list]
        self.assertEqual(calls, ["POSE,52.3,118.7,90\n", "ROBOT,<5>,<12>,<N>\n"])

    def test_hires_pose_rate_limited(self):
        """Back-to-back /robot_pose updates within one hires interval should
        only emit one POSE line, so a busy TLM feed can't flood the link."""
        pose = PoseStamped()
        pose.pose.position.x = 0.50
        pose.pose.position.y = 1.20

        self.node._on_pose(pose)
        self.mock_serial.write.reset_mock()
        pose.pose.position.x = 0.80  # change cell so ROBOT dedup doesn't suppress it
        self.node._on_pose(pose)  # immediately again -- still within the interval

        calls = [c[0][0].decode("utf-8") for c in self.mock_serial.write.call_args_list]
        self.assertEqual(len(calls), 1)
        self.assertTrue(calls[0].startswith("ROBOT,"))

    def test_target_to_tablet_format(self):
        """Test that /android/target string converts to TARGET,<payload>."""
        self.node._on_target(String(data="1,15"))
        written = self.mock_serial.write.call_args[0][0].decode("utf-8")
        self.assertEqual(written, "TARGET,1,15\n")

    def test_pending_move_queueing(self):
        """Test that rapid commands buffer into _pending_move and overwrite with latest."""
        self.node._move_client.wait_for_service = MagicMock(return_value=True)
        mock_future = MagicMock()
        self.node._move_client.call_async = MagicMock(return_value=mock_future)

        # 1st move starts
        self.node._dispatch("FC:8")
        self.assertTrue(self.node._is_moving)
        self.assertIsNone(self.node._pending_move)
        self.assertEqual(self.node._move_client.call_async.call_count, 1)

        # 2nd move while 1st is moving -> buffered
        self.node._dispatch("FC:8")
        self.assertEqual(self.node._pending_move, ("FC", 8))
        self.assertEqual(self.node._move_client.call_async.call_count, 1)

        # 3rd move with new direction -> overwrites pending slot
        self.node._dispatch("FR:45")
        self.assertEqual(self.node._pending_move, ("FR", 45))
        self.assertEqual(self.node._move_client.call_async.call_count, 1)

    def test_pending_move_dispatch_on_done(self):
        """Test that finishing a move automatically executes the pending move without idling."""
        self.node._move_client.wait_for_service = MagicMock(return_value=True)
        self.node._move_client.call_async = MagicMock()

        # Start first move
        self.node._dispatch("FC:8")
        self.node._dispatch("FR:45")  # buffered
        self.assertEqual(self.node._move_client.call_async.call_count, 1)

        # Simulate first move success
        fake_future = MagicMock()
        resp = MagicMock()
        resp.success = True
        fake_future.result.return_value = resp

        self.node._on_move_complete(fake_future)

        # Second move should now be dispatched
        self.assertEqual(self.node._move_client.call_async.call_count, 2)
        call_args = self.node._move_client.call_async.call_args[0][0]
        self.assertEqual(call_args.commands[0].command, "FR")
        self.assertEqual(call_args.commands[0].value, 45)
        self.assertIsNone(self.node._pending_move)
        self.assertTrue(self.node._is_moving)

        # Simulate second move success
        self.node._on_move_complete(fake_future)
        self.assertFalse(self.node._is_moving)
        written = self.mock_serial.write.call_args[0][0].decode("utf-8")
        self.assertEqual(written, "DONE\n")

    def test_busy_local_suppression(self):
        """Test that BUSY_LOCAL failure does not send 'Move failed' alert to tablet."""
        fake_future = MagicMock()
        resp = MagicMock()
        resp.success = False
        resp.status = "BUSY_LOCAL"
        fake_future.result.return_value = resp

        self.node._is_moving = True
        self.mock_serial.reset_mock()
        self.node._on_move_complete(fake_future)

        self.assertFalse(self.node._is_moving)
        self.mock_serial.write.assert_not_called()

    def test_estop_clears_pending_move(self):
        """Test that E-STOP clears any pending move."""
        self.node._is_moving = True
        self.node._pending_move = ("FC", 20)

        self.node._dispatch("STP")
        self.assertFalse(self.node._is_moving)
        self.assertIsNone(self.node._pending_move)

    def test_estop_reports_sensor_cause(self):
        """Test that /estop and STP report which sensor caused the estop and its reading."""
        self.node._us_cm = 8.4
        self.mock_serial.reset_mock()
        self.node._on_estop(Empty())

        calls = [c[0][0].decode("utf-8") for c in self.mock_serial.write.call_args_list]
        self.assertTrue(any("STATUS,ESTOP: Obstacle detected by Ultrasonic (8.4 cm)" in c for c in calls))
        self.assertTrue(any("ESTOP_ALERT,Ultrasonic,8.4,12.0" in c for c in calls))

    def test_forward_move_prevented_by_proximity(self):
        """Test that forward move is rejected and triggers estop if front obstacle is detected."""
        self.node._ir_l_cm = 7.5
        self.node._estop_pub.publish = MagicMock()
        self.mock_serial.reset_mock()

        self.node._dispatch("FW:20")
        self.node._estop_pub.publish.assert_called_once()
        calls = [c[0][0].decode("utf-8") for c in self.mock_serial.write.call_args_list]
        self.assertTrue(any("STATUS,ESTOP: Obstacle detected by IR Left (7.5 cm <= 12.0 cm)" in c for c in calls))
        self.assertTrue(any("ESTOP_ALERT,IR Left,7.5,12.0" in c for c in calls))


if __name__ == "__main__":
    unittest.main()
