#!/usr/bin/env python3
"""Unit tests for bullseye_control's pure P-controller / orbit-geometry math.

No rclpy dependency -- these test plain functions.
"""

import math
import unittest

from mdp_bringup.bullseye_control import (
    clamp,
    compute_orbit_radius_cm,
    distance_p_control,
    heading_p_control,
    orbit_twist,
)


class TestClamp(unittest.TestCase):
    def test_within_bounds_passthrough(self):
        self.assertAlmostEqual(clamp(0.5, 1.0), 0.5)

    def test_clamps_high(self):
        self.assertAlmostEqual(clamp(5.0, 1.0), 1.0)

    def test_clamps_low(self):
        self.assertAlmostEqual(clamp(-5.0, 1.0), -1.0)


class TestHeadingPControl(unittest.TestCase):
    def test_zero_at_center(self):
        self.assertAlmostEqual(heading_p_control(0.0, kp=1.2, max_yaw=1.0), 0.0)

    def test_positive_offset_positive_yaw(self):
        yaw = heading_p_control(0.3, kp=1.0, max_yaw=1.0)
        self.assertGreater(yaw, 0.0)

    def test_negative_offset_negative_yaw(self):
        yaw = heading_p_control(-0.3, kp=1.0, max_yaw=1.0)
        self.assertLess(yaw, 0.0)

    def test_clamped_to_max_yaw(self):
        yaw = heading_p_control(1.0, kp=5.0, max_yaw=0.8)
        self.assertAlmostEqual(yaw, 0.8)
        yaw = heading_p_control(-1.0, kp=5.0, max_yaw=0.8)
        self.assertAlmostEqual(yaw, -0.8)


class TestDistancePControl(unittest.TestCase):
    def test_zero_at_target(self):
        self.assertAlmostEqual(distance_p_control(25.0, 25.0, kp=0.02, max_speed=0.3), 0.0)

    def test_too_far_drives_forward(self):
        speed = distance_p_control(40.0, 25.0, kp=0.02, max_speed=0.3)
        self.assertGreater(speed, 0.0)

    def test_too_close_backs_off(self):
        speed = distance_p_control(10.0, 25.0, kp=0.02, max_speed=0.3)
        self.assertLess(speed, 0.0)

    def test_clamped_to_max_speed(self):
        speed = distance_p_control(200.0, 25.0, kp=1.0, max_speed=0.25)
        self.assertAlmostEqual(speed, 0.25)


class TestOrbitTwist(unittest.TestCase):
    def test_angular_z_equals_v_over_r(self):
        linear_x, angular_z = orbit_twist(orbit_radius_cm=30.0, tangential_speed_mps=0.12)
        self.assertAlmostEqual(linear_x, 0.12)
        self.assertAlmostEqual(angular_z, 0.12 / 0.30)

    def test_larger_radius_yields_smaller_yaw_for_same_speed(self):
        _, small_r_yaw = orbit_twist(orbit_radius_cm=20.0, tangential_speed_mps=0.1)
        _, large_r_yaw = orbit_twist(orbit_radius_cm=40.0, tangential_speed_mps=0.1)
        self.assertGreater(small_r_yaw, large_r_yaw)

    def test_rejects_nonpositive_radius(self):
        with self.assertRaises(ValueError):
            orbit_twist(orbit_radius_cm=0.0, tangential_speed_mps=0.1)
        with self.assertRaises(ValueError):
            orbit_twist(orbit_radius_cm=-5.0, tangential_speed_mps=0.1)


class TestComputeOrbitRadiusCm(unittest.TestCase):
    def test_matches_compute_vantage_pose_clearance_geometry(self):
        # Same shape as algorithm/planner.py:100 (dist = d + half_obs + half_robot)
        radius = compute_orbit_radius_cm(
            target_clearance_cm=25.0, obstacle_half_width_cm=5.0, robot_half_width_cm=11.5
        )
        self.assertAlmostEqual(radius, 25.0 + 5.0 + 11.5)


if __name__ == "__main__":
    unittest.main()
