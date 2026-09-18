#!/usr/bin/env python3
"""Pure control-law math for bullseye_orbit_node.

Kept free of rclpy so the P-controller and orbit-geometry math can be unit
tested without a ROS runtime. All angles are radians, all distances are cm
unless a *_mps suffix says otherwise.
"""

from __future__ import annotations

from typing import Tuple


def clamp(value: float, limit: float) -> float:
    """Clamp value to [-limit, limit]. limit must be >= 0."""
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def heading_p_control(cx_norm: float, kp: float, max_yaw: float) -> float:
    """Proportional heading correction from a normalized bbox center offset.

    cx_norm is in [-1, 1] with 0 = frame center. Positive cx_norm (target
    right of center) yields positive angular.z, matching the ROS REP-103
    convention (positive yaw = counter-clockwise, i.e. turning left) only if
    the caller's frame has cx_norm and angular.z sharing the same sign
    convention as the rest of this node -- verify against the live camera
    mount before trusting the sign on hardware.
    """
    return clamp(kp * cx_norm, abs(max_yaw))


def distance_p_control(range_cm: float, target_cm: float, kp: float, max_speed: float) -> float:
    """Proportional forward-speed correction to hold a standoff distance.

    Positive error (range_cm > target_cm, i.e. too far) drives forward
    (positive linear.x); negative error (too close) backs off.
    """
    error_cm = range_cm - target_cm
    return clamp(kp * error_cm, abs(max_speed))


def orbit_twist(orbit_radius_cm: float, tangential_speed_mps: float) -> Tuple[float, float]:
    """Twist (linear_x, angular_z) for a constant-radius arc around the obstacle.

    orbit_radius_cm must be > 0. angular_z = v / r keeps the robot moving on
    a circle of radius orbit_radius_cm at tangential_speed_mps.
    """
    if orbit_radius_cm <= 0.0:
        raise ValueError("orbit_radius_cm must be positive")
    radius_m = orbit_radius_cm / 100.0
    angular_z = tangential_speed_mps / radius_m
    return (tangential_speed_mps, angular_z)


def compute_orbit_radius_cm(target_clearance_cm: float, obstacle_half_width_cm: float,
                             robot_half_width_cm: float) -> float:
    """Orbit radius measured from the obstacle center, matching the same
    clearance geometry compute_vantage_pose() uses (algorithm/planner.py) so
    the arc holds the same standoff the discrete-hop vantage poses target."""
    return target_clearance_cm + obstacle_half_width_cm + robot_half_width_cm
