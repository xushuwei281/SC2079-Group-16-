#!/usr/bin/env python3
"""Unit tests for the SC2079 Autonomous Path Planner."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from arena import Config, Obstacle, default_arena, robot_collides_any
from planner import (
    DEFAULT_VIEW_DIST_CM,
    compute_vantage_pose,
    discretize_waypoints,
    plan_mission,
    sample_reeds_shepp_path,
    solve_tsp,
)


class TestPathPlanner(unittest.TestCase):

    def test_vantage_pose_north_face(self):
        """North face: target points North -> robot placed North of obstacle facing South (-pi/2)."""
        ob = Obstacle(id=1, x=100, y=100, face="N")
        vp = compute_vantage_pose(ob, d_view=25.0)
        self.assertAlmostEqual(vp.x, 100.0)
        self.assertAlmostEqual(vp.y, 141.5)  # 100 + 25 + 5 + 11.5 (front bumper clearance)
        self.assertAlmostEqual(vp.theta, -math.pi / 2.0)

    def test_vantage_pose_south_face(self):
        """South face: target points South -> robot placed South of obstacle facing North (+pi/2)."""
        ob = Obstacle(id=2, x=100, y=100, face="S")
        vp = compute_vantage_pose(ob, d_view=25.0)
        self.assertAlmostEqual(vp.x, 100.0)
        self.assertAlmostEqual(vp.y, 58.5)  # 100 - 25 - 5 - 11.5 (front bumper clearance)
        self.assertAlmostEqual(vp.theta, math.pi / 2.0)

    def test_vantage_pose_east_face(self):
        """East face: target points East -> robot placed East of obstacle facing West (pi)."""
        ob = Obstacle(id=3, x=100, y=100, face="E")
        vp = compute_vantage_pose(ob, d_view=25.0)
        self.assertAlmostEqual(vp.x, 141.5)  # 100 + 25 + 5 + 11.5 (front bumper clearance)
        self.assertAlmostEqual(vp.y, 100.0)
        self.assertAlmostEqual(abs(vp.theta), math.pi)

    def test_vantage_pose_west_face(self):
        """West face: target points West -> robot placed West of obstacle facing East (0)."""
        ob = Obstacle(id=4, x=100, y=100, face="W")
        vp = compute_vantage_pose(ob, d_view=25.0)
        self.assertAlmostEqual(vp.x, 58.5)  # 100 - 25 - 5 - 11.5 (front bumper clearance)
        self.assertAlmostEqual(vp.y, 100.0)
        self.assertAlmostEqual(vp.theta, 0.0)

    def test_adaptive_vantage_pose_near_wall(self):
        """Test adaptive view distance calculation when obstacle is placed close to a wall."""
        arena = default_arena()
        ob_near_north = Obstacle(id=5, x=100, y=180, face="N")
        vp = compute_vantage_pose(ob_near_north, d_view=25.0, arena=arena)
        # Bounded within arena margins [15, 185]
        self.assertTrue(15.0 <= vp.x <= 185.0)
        self.assertTrue(15.0 <= vp.y <= 185.0)

    def test_tsp_solver_ordering(self):
        """Test TSP permutation search with a 3-obstacle layout."""
        start = Config(20.0, 20.0, math.pi / 2.0)
        vps = [
            Config(150.0, 150.0, 0.0),
            Config(30.0, 50.0, math.pi / 2.0),
            Config(80.0, 100.0, 0.0)
        ]
        order = solve_tsp(start, vps)
        self.assertEqual(len(order), 3)
        # First visited obstacle should be the closest one (index 1)
        self.assertEqual(order[0], 1)

    def test_discretization_to_5byte_commands(self):
        """Test conversion of Reeds-Shepp waypoints into forward and backward 5-byte instructions."""
        wps = [
            ("F", 35.2, 25.0),
            ("L", math.radians(90.0), 25.0),
            ("B", 20.0, 25.0),
            ("R", math.radians(45.0), 25.0),
            ("L", math.radians(-90.0), 25.0),
            ("R", math.radians(-45.0), 25.0),
        ]
        cmds, raw_strings = discretize_waypoints(wps)
        self.assertEqual(raw_strings, ["FC035", "FL090", "BC020", "FR045", "BL090", "BR045"])
        for cmd_str in raw_strings:
            self.assertEqual(len(cmd_str), 5)
            self.assertTrue(cmd_str[:2] in ("FC", "BC", "FL", "FR", "BL", "BR"))
            self.assertTrue(cmd_str[2:].isdigit())

    def test_full_mission_planning(self):
        """Test end-to-end plan_mission on default arena."""
        arena = default_arena()
        plan = plan_mission(arena["obstacles"])
        self.assertEqual(len(plan.legs), 5)
        self.assertGreater(plan.total_distance_cm, 0.0)
        self.assertGreater(len(plan.all_commands), 0)
        self.assertGreater(len(plan.all_poses), 0)

    def test_trajectory_spatial_continuity(self):
        """Verify that every consecutive pose in all_poses has step size <= 2.5cm (no teleportation)."""
        arena = default_arena()
        plan = plan_mission(arena["obstacles"])
        self.assertGreater(len(plan.all_poses), 10)
        for i in range(len(plan.all_poses) - 1):
            p1 = plan.all_poses[i]
            p2 = plan.all_poses[i + 1]
            dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            self.assertLessEqual(dist, 2.5, f"Discontinuous jump of {dist:.2f}cm detected at index {i}")

    def test_trajectory_collision_free(self):
        """Verify that every pose in the sampled mission trajectory is strictly collision-free."""
        arena = default_arena()
        plan = plan_mission(arena["obstacles"])
        for p in plan.all_poses:
            hit = robot_collides_any(p[0], p[1], p[2], arena, safety_margin=0.0)
            self.assertIsNone(hit, f"Collision detected at pose {p} with obstacle {hit}")

    def test_forced_order_permutation(self):
        """Verify that forced_order correctly overrides TSP visit sequence."""
        arena = default_arena()
        obs = arena["obstacles"]
        forced = [4, 2, 0, 3, 1]
        plan = plan_mission(obs, forced_order=forced)
        planned_ids = [leg.obstacle_id for leg in plan.legs]
        expected_ids = [obs[i].id for i in forced]
        self.assertEqual(planned_ids, expected_ids)

    def test_astar_fallback_when_blocked(self):
        """Test A* fallback when obstacles block direct Reeds-Shepp paths."""
        from planning import plan_drive
        arena = {
            "arena_size": 200,
            "robot_w": 19,
            "robot_h": 23,
            "obstacles": [
                Obstacle(id=1, x=50, y=50),
                Obstacle(id=2, x=50, y=60),
                Obstacle(id=3, x=50, y=70),
            ]
        }
        start = Config(20.0, 60.0, 0.0)
        goal = Config(80.0, 60.0, 0.0)
        result = plan_drive(start, goal, radius=25.0, arena=arena)
        self.assertIn(result["method"], ("astar", "hybrid_astar", "astar_fallback"))
        self.assertNotEqual(result["method"], "none")
        self.assertTrue(len(result["waypoints"]) > 0)
        self.assertTrue(len(result["poses"]) > 0)

    def test_plan_leg_methods_tagged(self):
        """Verify that mission plan legs are tagged with their derivation algorithm."""
        arena = default_arena()
        plan = plan_mission(arena["obstacles"])
        for leg in plan.legs:
            self.assertIn(leg.method, ("reeds_shepp", "astar", "hybrid_astar"))


if __name__ == "__main__":
    unittest.main()
