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

    def test_estop_transitions_state_when_idle(self):
        """An E-STOP with no mission running is a genuine external event and hard-stops."""
        self.node._is_executing = False
        self.node._on_estop(Empty())
        self.assertEqual(self.node._state, MissionState.ESTOP)
        self.assertFalse(self.node._is_executing)

    def test_estop_deferred_during_active_leg(self):
        """An E-STOP received while a leg is executing must defer to in-leg
        proximity recovery (_execute_commands_sync) instead of a terminal
        ESTOP that would require a manual RESET and wipe _current_plan."""
        self.node._is_executing = True
        self.node._on_estop(Empty())
        self.assertNotEqual(self.node._state, MissionState.ESTOP)
        self.assertTrue(self.node._is_executing)

    def test_closed_loop_orbit_request_uses_real_float_fields(self):
        """BullseyeOrbit.Request.obstacle_x_cm/obstacle_y_cm are float32 in the
        .srv; Obstacle.x/y (arena.py) are plain ints. The generated rosidl
        Python->C converter hard-asserts PyFloat_Check on these fields instead
        of coercing an int, which SIGABRTs the whole process (not a catchable
        Python exception) rather than failing an assertion here -- so this
        exercises the REAL BullseyeOrbit.Request class (not a mock) to make
        sure _try_closed_loop_orbit always passes float(...), the only way
        this class of bug can be caught by a test at all."""
        from arena import Obstacle
        from planner import PlanLeg

        target_ob = Obstacle(id=1, x=60, y=60, face="N")
        self.assertIsInstance(target_ob.x, int)
        leg = PlanLeg(
            obstacle_id=1,
            target_face="N",
            start_pose=self.node._current_pose,
            vantage_pose=self.node._current_pose,
            poses=[],
            commands=[],
            raw_strings=[],
            distance_cm=0.0,
        )
        self.node._is_executing = True
        self.node._enable_closed_loop_orbit = True
        self.node._orbit_client.service_is_ready = MagicMock(return_value=True)
        done_future = MagicMock()
        done_future.done.return_value = True
        done_future.result.return_value = MagicMock(success=False, status="NO_RESPONSE")
        self.node._orbit_client.call_async = MagicMock(return_value=done_future)

        # Would SIGABRT the whole test process on the old code (a plain int
        # assigned to obstacle_x_cm/obstacle_y_cm); reaching here at all means
        # the real message construction survived.
        self.node._try_closed_loop_orbit(leg, target_ob, "N")

        sent_req = self.node._orbit_client.call_async.call_args[0][0]
        self.assertEqual(sent_req.obstacle_x_cm, 60.0)
        self.assertEqual(sent_req.obstacle_y_cm, 60.0)

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
        self.node._execute_commands_sync.assert_called_once_with(
            [("BC", 30), ("FR", 90), ("FC", 15), ("FL", 180)], label="Orbit Right to W"
        )

    def test_unconfirmed_target_does_not_get_fabricated_id(self):
        """A failed CV sample must fail the mission, never publish a fake ID."""
        from arena import Obstacle
        from planner import FullMissionPlan, PlanLeg

        obstacle = Obstacle(id=1, x=60, y=60, face="N")
        leg = PlanLeg(
            obstacle_id=1,
            target_face="N",
            start_pose=self.node._current_pose,
            vantage_pose=self.node._current_pose,
            poses=[],
            commands=[],
            raw_strings=[],
            distance_cm=0.0,
        )
        self.node._obstacles = [obstacle]
        self.node._current_plan = FullMissionPlan(
            start_pose=self.node._current_pose,
            legs=[leg],
            total_distance_cm=0.0,
            all_commands=[],
            all_poses=[],
        )
        self.node._is_executing = True
        self.node._move_client.wait_for_service = MagicMock(return_value=True)
        self.node._query_perception_sampler = MagicMock(return_value=None)
        self.node._wait_for_recognition = MagicMock(return_value=None)
        self.node._inspect_adjacent_faces = MagicMock(return_value=None)
        self.node._target_pub.publish = MagicMock()
        self.node._status_pub.publish = MagicMock()

        self.node._execute_mission_loop()

        self.assertEqual(self.node._state, MissionState.MISSION_FAILED)
        self.node._target_pub.publish.assert_not_called()
        statuses = [call.args[0].data for call in self.node._status_pub.publish.call_args_list]
        self.assertIn("TARGET_UNCONFIRMED,1", statuses)

    def test_start_does_not_clear_estop(self) -> None:
        """A new START cannot restart a mission after a latched safety stop."""
        self.node._current_plan = MagicMock()
        self.node._on_estop(Empty())
        with patch("mdp_bringup.planner_node.threading.Thread") as thread:
            self.node.start_mission()
        thread.assert_not_called()
        self.assertEqual(self.node._state, MissionState.ESTOP)
        self.assertFalse(self.node._is_executing)

    def test_new_plan_refused_while_estopped(self) -> None:
        """A fresh ALG obstacle list must not silently resume a mission while
        ESTOP is latched: _parse_and_plan used to transition straight to
        PLANNING before start_mission()'s own ESTOP guard ever got a chance to
        run (by which point our own FSM state had already moved off ESTOP),
        letting a mission "resume" while motion_controller_node's/
        serial_bridge_node's hardware latches were still set -- every move
        then failed immediately with no visible cause."""
        self.node._on_estop(Empty())
        self.assertEqual(self.node._state, MissionState.ESTOP)
        self.node._parse_and_plan("ALG|1,60,60,N")
        self.assertEqual(self.node._state, MissionState.ESTOP)
        self.assertIsNone(self.node._current_plan)
        self.assertFalse(self.node._is_executing)

    def test_leg_movement_failure_reports_mission_failed_not_complete(self):
        """If a leg's movement never succeeds (all retries fail), the mission
        must report MISSION_FAILED, never MISSION_COMPLETE -- the loop used to
        break out of the leg loop on a movement failure without ever setting
        mission_failed, so the final check (_is_executing and not
        mission_failed) fell through to a false "all obstacles visited"."""
        from arena import Config, Obstacle
        from planner import FullMissionPlan, PlanLeg

        # vantage_pose must differ from _current_pose (and stay that way
        # across retries, since the mocked _execute_commands_sync never
        # actually moves the robot) so each retry attempt re-plans a real,
        # non-empty command sequence instead of short-circuiting via the
        # "already at vantage pose" empty-waypoints branch.
        obstacle = Obstacle(id=1, x=60, y=60, face="N")
        leg = PlanLeg(
            obstacle_id=1,
            target_face="N",
            start_pose=self.node._current_pose,
            vantage_pose=Config(150.0, 150.0, 0.0),
            poses=[],
            commands=[("FC", 20)],
            raw_strings=[],
            distance_cm=20.0,
        )
        self.node._obstacles = [obstacle]
        self.node._current_plan = FullMissionPlan(
            start_pose=self.node._current_pose,
            legs=[leg],
            total_distance_cm=20.0,
            all_commands=[("FC", 20)],
            all_poses=[],
        )
        self.node._is_executing = True
        self.node._move_client.wait_for_service = MagicMock(return_value=True)
        # Movement itself never succeeds (e.g. a hardware latch left set),
        # regardless of retries.
        self.node._execute_commands_sync = MagicMock(return_value=False)
        self.node._status_pub.publish = MagicMock()

        with patch("mdp_bringup.planner_node.time.sleep"):
            self.node._execute_mission_loop()

        self.assertEqual(self.node._state, MissionState.MISSION_FAILED)
        self.assertNotEqual(self.node._state, MissionState.MISSION_COMPLETE)
        statuses = [c[0][0].data for c in self.node._status_pub.publish.call_args_list]
        self.assertIn("Leg 1 Failed", statuses)

    def test_proximity_trip_auto_recovers_without_losing_plan(self):
        """A proximity trip during a leg approach must soft-recover (clear the
        motion_controller_node latch, back off, retry) rather than terminal-
        ESTOP the whole mission and force an operator RESET that wipes
        _current_plan -- overshoot toward our own target obstacle is expected,
        not a surprise hazard."""
        self.node._is_executing = True
        self.node._enable_avoidance = True
        self.node._safety_dist_cm = 12.0
        self.node._recovery_backup_cm = 8.0
        self.node._current_plan = MagicMock()  # must survive the recovery, unlike RESET
        # Ultrasonic reads 8.5 cm (0.085m)
        self.node._us_range_m = 0.085
        self.node._ir_left_range_m = float("inf")
        self.node._ir_right_range_m = float("inf")

        self.node._cmd_pub.publish = MagicMock()
        self.node._status_pub.publish = MagicMock()
        self.node._telemetry_pub.publish = MagicMock()

        # First call_async is the forward move; its future never finishes on
        # its own, so the proximity guard loop is what detects the trip.
        # Second call_async is the recovery backup move issued by
        # _recover_from_proximity_trip; resolve it immediately so the test
        # doesn't block on its real 3s timeout.
        forward_future = MagicMock()
        forward_future.done.return_value = False
        backup_future = MagicMock()
        backup_future.done.return_value = True
        backup_future.result.return_value = MagicMock(success=True)
        self.node._move_client.call_async = MagicMock(side_effect=[forward_future, backup_future])

        result = self.node._execute_commands_sync([("FC", 20)], label="Forward Test")

        self.assertFalse(result)
        self.assertEqual(self.node._state, MissionState.AVOIDANCE_RECOVERY)
        self.assertTrue(self.node._is_executing)
        self.assertIsNotNone(self.node._current_plan)
        # Forward move + the auto-issued backup move.
        self.assertEqual(self.node._move_client.call_async.call_count, 2)

        cmd_calls = [c[0][0].data for c in self.node._cmd_pub.publish.call_args_list]
        self.assertIn("CLEAR_ESTOP", cmd_calls)

        status_calls = [c[0][0].data for c in self.node._status_pub.publish.call_args_list]
        self.assertTrue(any("Proximity Alert: Ultrasonic at 8.5 cm" in s for s in status_calls))

        telemetry_calls = [c[0][0].data for c in self.node._telemetry_pub.publish.call_args_list]
        self.assertTrue(any("PROXIMITY_RECOVERY,Ultrasonic,8.5,12.0" in t for t in telemetry_calls))


if __name__ == "__main__":
    unittest.main()
