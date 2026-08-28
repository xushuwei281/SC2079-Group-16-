#!/usr/bin/env python3
"""Unit tests for the Android Bluetooth Bridge Node."""

import math
import os
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from mdp_android_bridge.android_bridge_node import AndroidBridgeNode
from std_msgs.msg import String


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

        # BW:35 -> BC 35
        self.node._dispatch("BW:35")
        call_args = self.node._move_client.call_async.call_args[0][0]
        self.assertEqual(call_args.commands[0].command, "BC")
        self.assertEqual(call_args.commands[0].value, 35)

        # TL:90 -> FL 90
        self.node._dispatch("TL:90")
        call_args = self.node._move_client.call_async.call_args[0][0]
        self.assertEqual(call_args.commands[0].command, "FL")
        self.assertEqual(call_args.commands[0].value, 90)

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

    def test_pose_to_tablet_format(self):
        """Test that /robot_pose converts to ROBOT,<x_cm>,<y_cm>,<dir_deg>."""
        pose = PoseStamped()
        pose.pose.position.x = 0.50  # 50 cm
        pose.pose.position.y = 1.20  # 120 cm
        # 90 degrees yaw (Z = sin(pi/4), W = cos(pi/4))
        pose.pose.orientation.z = math.sin(math.pi / 4.0)
        pose.pose.orientation.w = math.cos(math.pi / 4.0)

        self.node._on_pose(pose)

        # Check last written line
        written = self.mock_serial.write.call_args[0][0].decode("utf-8")
        self.assertEqual(written, "ROBOT,50,120,90\n")

    def test_target_to_tablet_format(self):
        """Test that /android/target string converts to TARGET,<payload>."""
        self.node._on_target(String(data="1,15"))
        written = self.mock_serial.write.call_args[0][0].decode("utf-8")
        self.assertEqual(written, "TARGET,1,15\n")


if __name__ == "__main__":
    unittest.main()
