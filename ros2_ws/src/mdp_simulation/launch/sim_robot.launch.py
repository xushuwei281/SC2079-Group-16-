#!/usr/bin/env python3
"""Launch full autonomous robot stack in Gazebo simulation."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
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
    run_perception_arg = DeclareLaunchArgument(
        "run_perception",
        default_value="true",
        description="Run YOLO/ONNX perception node in simulation",
    )
    run_planner_arg = DeclareLaunchArgument(
        "run_planner",
        default_value="true",
        description="Run autonomous Reeds-Shepp TSP planner node in simulation",
    )
    run_rviz_arg = DeclareLaunchArgument(
        "run_rviz",
        default_value="true" if has_display else "false",
        description="Launch RViz2 visualization",
    )

    # 1. Base Gazebo Simulation Bringup
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "use_sim_time": "true",
        }.items(),
    )

    # 2. Motion Controller Node (measured primitive controller & teleop arbiter)
    motion_controller = Node(
        package="mdp_hardware_bridge",
        executable="motion_controller_node",
        name="motion_controller_node",
        parameters=[{
            "use_sim_time": True,
            "velocity_speed_mps": 0.15,
            "velocity_max_speed_mps": 0.35,
            "velocity_turn_radius_m": 0.21,
            "velocity_max_yaw_rps": 1.75,
            "batch_timeout_sec": 30.0,
            "telemetry_timeout_sec": 0.40,
            "cmd_vel_timeout_sec": 0.20,
            "front_stop_distance_m": 0.12,
            "ir_stop_distance_m": 0.10,
        }],
        output="screen",
    )

    # 3. Autonomous Planner Node (Task 1)
    planner_node = Node(
        package="mdp_bringup",
        executable="planner_node",
        name="planner_node",
        parameters=[{
            "use_sim_time": True,
            "turning_radius_cm": 21.0,
            "camera_view_dist_cm": 25.0,
            "auto_start": True,
            "enable_collision_avoidance": True,
            "safety_stop_dist_cm": 12.0,
            "recovery_backup_cm": 8.0,
        }],
        condition=IfCondition(LaunchConfiguration("run_planner")),
        output="screen",
    )

    # 4. Perception Node
    perception_node = Node(
        package="mdp_perception",
        executable="perception_node",
        name="perception_node",
        parameters=[{
            "use_sim_time": True,
            "model_path": "models/best.onnx",
            "conf_threshold": 0.45,
            "preview_fps": 2.0,
        }],
        condition=IfCondition(LaunchConfiguration("run_perception")),
        output="screen",
    )

    # 5. Optional RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(pkg_sim_share, "config", "sim.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("run_rviz")),
        output="screen",
    )

    return LaunchDescription([
        headless_arg,
        run_perception_arg,
        run_planner_arg,
        run_rviz_arg,
        gz_sim_launch,
        motion_controller,
        planner_node,
        perception_node,
        rviz_node,
    ])
