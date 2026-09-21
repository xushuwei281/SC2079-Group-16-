"""Unit tests for Checklist A.5 Approach and Face Recognition Node."""

import unittest
from unittest.mock import MagicMock, patch

from mdp_bringup.checklist_a5 import ChecklistA5Node, DEFAULT_ORBIT_MACRO, parse_macro_string
from mdp_interfaces.srv import ExecuteMoves, SampleTarget
import rclpy
from sensor_msgs.msg import Range


class TestChecklistA5Node(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = ChecklistA5Node(target_distance_cm=25.0, max_faces=4)

    def tearDown(self):
        self.node.stop_routine()
        self.node.destroy_node()

    def test_default_initialization(self):
        self.assertEqual(self.node._target_dist_cm, 25.0)
        self.assertEqual(self.node._max_faces, 4)
        self.assertEqual(self.node._macro, DEFAULT_ORBIT_MACRO)

    def test_parse_macro_string(self):
        res = parse_macro_string("BC:30,FR:90,FC:18,FL:180")
        self.assertEqual(res, [("BC", 30), ("FR", 90), ("FC", 18), ("FL", 180)])

    def test_ultrasonic_callback_updates_distance(self):
        msg = Range()
        msg.range = 0.45
        self.node._on_ultrasonic(msg)
        self.assertAlmostEqual(self.node.current_distance_cm, 45.0, places=1)

    def test_approach_obstacle_stops_when_close_enough(self):
        msg = Range()
        msg.range = 0.26  # 26 cm <= 25 + 2 cm
        self.node._on_ultrasonic(msg)

        # Should immediately succeed without moving
        self.node._execute_move_command_sync = MagicMock(return_value=True)
        ok = self.node._approach_obstacle()
        self.assertTrue(ok)
        self.node._execute_move_command_sync.assert_not_called()

    def test_approach_obstacle_advances_incrementally(self):
        # Start at 50 cm
        msg = Range()
        msg.range = 0.50
        self.node._on_ultrasonic(msg)

        executed_cmds = []

        def fake_move(cmd, val):
            executed_cmds.append((cmd, val))
            # Simulate robot getting closer after move
            new_r = max(0.25, self.node._us_range_m - (val / 100.0))
            self.node._us_range_m = new_r
            return True

        self.node._execute_move_command_sync = MagicMock(side_effect=fake_move)
        ok = self.node._approach_obstacle()
        self.assertTrue(ok)
        self.assertGreater(len(executed_cmds), 0)
        self.assertLessEqual(self.node.current_distance_cm, 27.0)

    def test_sample_target_detects_valid_symbol(self):
        self.node._sample_client.service_is_ready = MagicMock(return_value=True)

        resp = SampleTarget.Response()
        resp.success = True
        resp.symbol_id = 15
        resp.symbol_name = "arrow_up"
        resp.confidence = 0.94
        resp.is_marker = False

        done_future = MagicMock()
        done_future.done.return_value = True
        done_future.result.return_value = resp
        self.node._sample_client.call_async = MagicMock(return_value=done_future)

        result = self.node._sample_target(face_idx=1)
        self.assertIsNotNone(result)
        self.assertEqual(result[0], 15)
        self.assertEqual(result[1], "arrow_up")
        self.assertAlmostEqual(result[2], 0.94, places=2)

    def test_sample_target_rejects_bullseye_marker(self):
        self.node._sample_client.service_is_ready = MagicMock(return_value=True)

        resp = SampleTarget.Response()
        resp.success = True
        resp.symbol_id = 40
        resp.symbol_name = "bullseye"
        resp.confidence = 0.98
        resp.is_marker = True

        done_future = MagicMock()
        done_future.done.return_value = True
        done_future.result.return_value = resp
        self.node._sample_client.call_async = MagicMock(return_value=done_future)

        result = self.node._sample_target(face_idx=1)
        self.assertIsNone(result)

    def test_macro_execution(self):
        executed = []
        self.node._execute_move_command_sync = MagicMock(side_effect=lambda c, v: executed.append((c, v)) or True)

        ok = self.node._execute_macro_sync()
        self.assertTrue(ok)
        self.assertEqual(executed, DEFAULT_ORBIT_MACRO)


if __name__ == "__main__":
    unittest.main()
