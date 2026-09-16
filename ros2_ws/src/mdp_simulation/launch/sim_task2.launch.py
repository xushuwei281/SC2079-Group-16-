#!/usr/bin/env python3
"""Launch Task 2 (Fastest Car Sprint) in Gazebo simulation."""

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
    task2_world = os.path.join(pkg_sim_share, "worlds", "task2_arena.sdf")
    has_display = os.environ.get("DISPLAY", "") != ""

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false" if has_display else "true",
        description="Run Gazebo simulation headless",
    )
    run_perception_arg = DeclareLaunchArgument(
        "run_perception",
        default_value="true",
        description="Run local YOLO perception node in simulation",
    )
    run_rviz_arg = DeclareLaunchArgument(
        "run_rviz",
        default_value="true" if has_display else "false",
        description="Launch RViz2 visualization",
    )

    # 1. Base Gazebo Bringup with Task 2 Sprint World
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "world": task2_world,
            "headless": LaunchConfiguration("headless"),
            "x": "0.20",
            "y": "0.20",
            "yaw": "1.57079632679",
            "use_sim_time": "true",
        }.items(),
    )

    # 2. Motion Controller Node
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

    # 3. Fastest Car Reactive Sprint Node (Task 2)
    fastest_car_node = Node(
        package="mdp_bringup",
        executable="fastest_car_node",
        name="fastest_car_node",
        parameters=[{
            "use_sim_time": True,
            "vantage_dist_cm": 30.0,
            "approach_step_cm": 25.0,
            "min_approach_step_cm": 5.0,
            "max_approach_steps": 8,
            "default_turn": "LEFT",
        }],
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
        run_rviz_arg,
        gz_sim_launch,
        motion_controller,
        fastest_car_node,
        perception_node,
        rviz_node,
    ])
