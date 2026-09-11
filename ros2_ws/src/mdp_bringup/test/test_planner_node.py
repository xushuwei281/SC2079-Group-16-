#!/usr/bin/env python3
"""Unit tests for PlannerNode covering FSM, parsing, and Bull's Eye recovery."""

import math
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from geometry_msgs.msg import PoseStamped
from mdp_bringup.planner_node import MissionState, PlannerNode
from mdp_interfaces.srv import ExecuteMoves, SampleTarget
from std_msgs.msg import Empty, String


class TestPlannerNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = PlannerNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_initial_state(self):
        """Test that node initializes in IDLE state with default parameters."""
        self.assertEqual(self.node._state, MissionState.IDLE)
        self.assertTrue(self.node._auto_start)
        self.assertTrue(self.node._enable_avoidance)
        self.assertTrue(self.node._enable_orbit_recovery)

    def test_parse_android_reference_format(self):
        """Test parsing Android reference app format ALG:{Obstacle 1: [2,9,N,], ...}."""
        raw = (
            "ALG:{Obstacle 1: [2,9,N,],"
            "Obstacle 2: [4,11,N,],"
            "Obstacle 3: [],"
            "Obstacle 4: [12,14,E,],"
            "Obstacle 5: [],"
            "Obstacle 6: [],"
            "Obstacle 7: [],"
            "Obstacle 8: []}"
        )
        self.node._auto_start = False
        self.node._parse_and_plan(raw)

        self.assertEqual(len(self.node._obstacles), 3)
        obs_ids = [ob.id for ob in self.node._obstacles]
        self.assertIn(1, obs_ids)
        self.assertIn(2, obs_ids)
        self.assertIn(4, obs_ids)

        # Obstacle 1: x=2 -> 2*10+5 = 25 cm, y=9 -> 9*10+5 = 95 cm
        ob1 = next(ob for ob in self.node._obstacles if ob.id == 1)
        self.assertEqual(ob1.x, 25)
        self.assertEqual(ob1.y, 95)
        self.assertEqual(ob1.face, "N")

        self.assertEqual(self.node._state, MissionState.PLANNING)
        self.assertIsNotNone(self.node._current_plan)

    def test_parse_pipe_format(self):
        """Test parsing pipe-delimited format ALG|1,60,60,N|2,130,60,E..."""
        raw = "ALG|1,60,60,N|2,130,60,E|3,130,140,W"
        self.node._auto_start = False
        self.node._parse_and_plan(raw)

        self.assertEqual(len(self.node._obstacles), 3)
        ob1 = next(ob for ob in self.node._obstacles if ob.id == 1)
        self.assertEqual(ob1.x, 60)
        self.assertEqual(ob1.y, 60)
        self.assertEqual(ob1.face, "N")
        self.assertEqual(self.node._state, MissionState.PLANNING)

    def test_cmd_start_and_reset(self):
        """Test that START and RESET commands trigger appropriate methods."""
        self.node.start_mission = MagicMock()
        self.node.reset_mission = MagicMock()

        self.node._on_cmd(String(data="START"))
        self.node.start_mission.assert_called_once()

        self.node._on_cmd(String(data="ALG:START"))
        self.assertEqual(self.node.start_mission.call_count, 2)

        self.node._on_cmd(String(data="RESET"))
        self.node.reset_mission.assert_called_once()

    def test_reset_mission_resets_pose_and_targets(self):
        """Test that reset_mission resets _current_pose to (20, 20, pi/2) and clears recognized targets."""
        from arena import Config
        self.node._current_pose = Config(85.0, 120.0, 0.0)
        self.node._recognized_targets = {1: 15, 2: 22}
        self.node.reset_mission()
        self.assertAlmostEqual(self.node._current_pose.x, 20.0)
        self.assertAlmostEqual(self.node._current_pose.y, 20.0)
        self.assertAlmostEqual(self.node._current_pose.theta, math.pi / 2.0)
        self.assertEqual(len(self.node._recognized_targets), 0)

    def test_estop_transitions_state(self):
        """Test that E-STOP transitions to ESTOP state and cancels execution."""
        self.node._is_executing = True
        self.node._on_estop(Empty())
        self.assertEqual(self.node._state, MissionState.ESTOP)
        self.assertFalse(self.node._is_executing)

    def test_orbit_recovery_detects_target_on_adjacent_face(self):
        """Test that _inspect_adjacent_faces finds target on an adjacent face."""
        from arena import Obstacle, default_arena
        from planner import PlanLeg

        target_ob = Obstacle(id=1, x=60, y=60, face="N")
        leg = PlanLeg(
            obstacle_id=1,
            target_face="N",
            start_pose=self.node._current_pose,
            vantage_pose=self.node._current_pose,
            poses=[],
            commands=[],
            raw_strings=[],
            distance_cm=0.0
        )

        arena = default_arena()
        arena["obstacles"] = [target_ob]

        self.node._is_executing = True
        self.node._execute_commands_sync = MagicMock(return_value=True)

        # Mock perception sampler: candidate face returns valid target (symbol 15)
        self.node._query_perception_sampler = MagicMock(return_value=(15, "arrow_up", 0.92, False))

        result = self.node._inspect_adjacent_faces(leg, target_ob, arena)
        self.assertIsNotNone(result)
        self.assertEqual(result[0], 15)
        self.assertEqual(result[1], "arrow_up")

    def test_start_does_not_clear_estop(self) -> None:
        """A new START cannot restart a mission after a latched safety stop."""
        self.node._current_plan = MagicMock()
        self.node._on_estop(Empty())
        with patch("mdp_bringup.planner_node.threading.Thread") as thread:
            self.node.start_mission()
        thread.assert_not_called()
        self.assertEqual(self.node._state, MissionState.ESTOP)
        self.assertFalse(self.node._is_executing)

    def test_proximity_estop_reports_sensor_and_distance(self):
        """Test that proximity violation halts robot and reports which sensor and value caused it."""
        self.node._is_executing = True
        self.node._enable_avoidance = True
        self.node._safety_dist_cm = 12.0
        self.node._recovery_backup_cm = 8.0
        # Ultrasonic reads 8.5 cm (0.085m)
        self.node._us_range_m = 0.085
        self.node._ir_left_range_m = float("inf")
        self.node._ir_right_range_m = float("inf")

        self.node._estop_pub.publish = MagicMock()
        self.node._status_pub.publish = MagicMock()
        self.node._telemetry_pub.publish = MagicMock()

        # Mock future that never finishes on its own so proximity loop checks sensors
        mock_future = MagicMock()
        mock_future.done.return_value = False
        self.node._move_client.call_async = MagicMock(return_value=mock_future)

        result = self.node._execute_commands_sync([("FC", 20)], label="Forward Test")
        self.assertFalse(result)
        self.node._estop_pub.publish.assert_called()
        self.assertEqual(self.node._state, MissionState.ESTOP)
        self.assertFalse(self.node._is_executing)
        # A latched stop must not dispatch an automatic reverse movement.
        self.node._move_client.call_async.assert_called_once()

        # Verify status publication contains sensor name and value
        status_calls = [c[0][0].data for c in self.node._status_pub.publish.call_args_list]
        self.assertTrue(any("ESTOP: Obstacle detected by Ultrasonic (8.5 cm" in s for s in status_calls))

        # Verify telemetry publication contains ESTOP_ALERT with Ultrasonic and 8.5
        telemetry_calls = [c[0][0].data for c in self.node._telemetry_pub.publish.call_args_list]
        self.assertTrue(any("ESTOP_ALERT,Ultrasonic,8.5,12.0" in t for t in telemetry_calls))


if __name__ == "__main__":
    unittest.main()
