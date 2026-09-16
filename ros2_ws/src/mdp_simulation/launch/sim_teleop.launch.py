#!/usr/bin/env python3
"""Launch Gazebo simulation with manual teleoperation controls."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim_share = get_package_share_directory("mdp_simulation")
    has_display = os.environ.get("DISPLAY", "") != ""

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false" if has_display else "true",
        description="Run Gazebo simulation headless",
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "use_sim_time": "true",
        }.items(),
    )

    motion_controller = Node(
        package="mdp_hardware_bridge",
        executable="motion_controller_node",
        name="motion_controller_node",
        parameters=[{
            "use_sim_time": True,
            "velocity_speed_mps": 0.20,
            "velocity_max_speed_mps": 0.35,
            "velocity_turn_radius_m": 0.21,
            "velocity_max_yaw_rps": 1.75,
            "front_stop_distance_m": 0.12,
            "ir_stop_distance_m": 0.10,
        }],
        output="screen",
    )

    teleop_keyboard = Node(
        package="mdp_hardware_bridge",
        executable="teleop_keyboard",
        name="teleop_keyboard",
        output="screen",
        prefix="xterm -e",  # Runs in interactive window if xterm is available
    )

    return LaunchDescription([
        headless_arg,
        gz_sim_launch,
        motion_controller,
    ])
