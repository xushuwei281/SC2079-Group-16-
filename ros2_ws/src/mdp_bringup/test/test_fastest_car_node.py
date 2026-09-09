#!/usr/bin/env python3
"""Unit tests for FastestCarNode (SC2079 MDP Task 2 Reactive Sprint)."""

import math
import unittest
from unittest.mock import MagicMock, patch

from geometry_msgs.msg import PoseStamped
from mdp_bringup.fastest_car_node import (
    FastestCarNode,
    Task2State,
    parse_move_command,
)
from mdp_interfaces.srv import ExecuteMoves, SampleTarget
import rclpy
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, String


class TestFastestCarNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = FastestCarNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_initial_state(self):
        """Verify initial state is IDLE and default parameters are loaded."""
        self.assertEqual(self.node._state, Task2State.IDLE)
        self.assertFalse(self.node._is_running)
        self.assertEqual(self.node._vantage_dist_cm, 30.0)
        self.assertEqual(self.node._default_turn, "LEFT")
        self.assertIn("FL045", self.node._slalom_left_cmds[0])
        self.assertIn("FR045", self.node._slalom_right_cmds[0])

    def test_parse_move_command(self):
        """Test parsing command strings into MoveCommand message objects."""
        mc_turn = parse_move_command("FL045")
        self.assertEqual(mc_turn.command, "FL")
        self.assertEqual(mc_turn.value, 45)

        mc_straight = parse_move_command("FC030")
        self.assertEqual(mc_straight.command, "FC")
        self.assertEqual(mc_straight.value, 30)

        mc_back = parse_move_command("BC015")
        self.assertEqual(mc_back.command, "BC")
        self.assertEqual(mc_back.value, 15)

    def test_cmd_triggers(self):
        """Test that STM:sp, SP, and START_TASK2 start the sprint, and RESET resets."""
        self.node.start_sprint = MagicMock()
        self.node.reset_sprint = MagicMock()

        self.node._on_cmd(String(data="STM:sp"))
        self.node.start_sprint.assert_called_once()

        self.node._on_cmd(String(data="SP"))
        self.assertEqual(self.node.start_sprint.call_count, 2)

        self.node._on_cmd(String(data="START_TASK2"))
        self.assertEqual(self.node.start_sprint.call_count, 3)

        self.node._on_cmd(String(data="RESET"))
        self.node.reset_sprint.assert_called_once()

    def test_estop_handling(self):
        """Test that E-STOP transitions to ESTOP state and stops execution."""
        self.node._is_running = True
        self.node._on_estop(Empty())
        self.assertEqual(self.node._state, Task2State.ESTOP)
        self.assertFalse(self.node._is_running)

    def test_ultrasonic_and_pose_updates(self):
        """Test live updates of ultrasonic sensor and dead-reckoned pose."""
        # Valid ultrasonic range: 0.35m = 35cm
        r_msg = Range()
        r_msg.range = 0.35
        self.node._on_us_range(r_msg)
        self.assertAlmostEqual(self.node._us_range_m, 0.35)

        # Out-of-bounds range -> treated as inf
        r_msg.range = 5.0
        self.node._on_us_range(r_msg)
        self.assertTrue(math.isinf(self.node._us_range_m))

        # Pose update
        p_msg = PoseStamped()
        p_msg.pose.position.x = 0.40
        p_msg.pose.position.y = 0.80
        p_msg.pose.orientation.z = math.sin(math.pi / 4.0)
        p_msg.pose.orientation.w = math.cos(math.pi / 4.0)
        self.node._on_pose(p_msg)
        self.assertAlmostEqual(self.node._current_x, 40.0)
        self.assertAlmostEqual(self.node._current_y, 80.0)
        self.assertAlmostEqual(self.node._current_yaw, math.pi / 2.0)

    def test_sample_arrow_classification(self):
        """Test arrow classification for LEFT (39) vs RIGHT (38) and fallback."""
        self.node._is_running = True

        # Case 1: Perception returns Left Arrow (39)
        mock_resp_left = SampleTarget.Response()
        mock_resp_left.success = True
        mock_resp_left.symbol_id = 39
        mock_resp_left.symbol_name = "arrow_left"
        mock_resp_left.confidence = 0.94

        mock_future_left = MagicMock()
        mock_future_left.done.return_value = True
        mock_future_left.result.return_value = mock_resp_left
        self.node._sample_client.service_is_ready = MagicMock(return_value=True)
        self.node._sample_client.call_async = MagicMock(
            return_value=mock_future_left
        )

        direction, sid, conf = self.node._sample_arrow(obs_num=1)
        self.assertEqual(direction, "LEFT")
        self.assertEqual(sid, 39)
        self.assertAlmostEqual(conf, 0.94)

        # Case 2: Perception returns Right Arrow (38)
        mock_resp_right = SampleTarget.Response()
        mock_resp_right.success = True
        mock_resp_right.symbol_id = 38
        mock_resp_right.symbol_name = "arrow_right"
        mock_resp_right.confidence = 0.88

        mock_future_right = MagicMock()
        mock_future_right.done.return_value = True
        mock_future_right.result.return_value = mock_resp_right
        self.node._sample_client.call_async = MagicMock(
            return_value=mock_future_right
        )

        direction, sid, conf = self.node._sample_arrow(obs_num=2)
        self.assertEqual(direction, "RIGHT")
        self.assertEqual(sid, 38)
        self.assertAlmostEqual(conf, 0.88)

        # Case 3: Perception offline/failing -> Fallback to default (LEFT)
        mock_future_fail = MagicMock()
        mock_future_fail.done.return_value = False
        self.node._sample_client.call_async = MagicMock(
            return_value=mock_future_fail
        )
        self.node._sample_client.wait_for_service = MagicMock(
            return_value=False
        )

        direction, sid, conf = self.node._sample_arrow(obs_num=1)
        self.assertEqual(direction, "LEFT")
        self.assertEqual(sid, 39)

    def test_full_sprint_sequence(self):
        """Test full reactive sprint sequence: Obs 1 Left -> Obs 2 Right -> Park."""
        self.node._move_client.wait_for_service = MagicMock(return_value=True)
        self.node._execute_move_list_sync = MagicMock(return_value=True)

        # Mock approach: simulate arriving at obstacle
        self.node._advance_to_obstacle = MagicMock(return_value=True)

        # Mock Arrow 1 = LEFT, Arrow 2 = RIGHT
        self.node._sample_arrow = MagicMock(
            side_effect=[("LEFT", 39, 0.95), ("RIGHT", 38, 0.91)]
        )

        self.node._target_pub.publish = MagicMock()
        self.node._status_pub.publish = MagicMock()

        self.node._is_running = True
        self.node._run_task2_sprint()

        # Check published targets
        target_calls = [
            c[0][0].data for c in self.node._target_pub.publish.call_args_list
        ]
        self.assertIn("1,39", target_calls)
        self.assertIn("2,38", target_calls)

        # Check that maneuvers executed
        execute_calls = [
            c[1]["label"]
            for c in self.node._execute_move_list_sync.call_args_list
        ]
        self.assertIn("Slalom Obs 1", execute_calls)
        self.assertIn("Round Obs 2 & Return", execute_calls)
        self.assertIn("Park Sprint", execute_calls)

        # Final state stands by in IDLE
        self.assertEqual(self.node._state, Task2State.IDLE)
        self.assertFalse(self.node._is_running)


if __name__ == "__main__":
    unittest.main()
