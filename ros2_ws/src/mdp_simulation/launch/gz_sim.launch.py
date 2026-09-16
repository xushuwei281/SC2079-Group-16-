#!/usr/bin/env python3
"""Launch Gazebo Harmonic 3D simulation with SC2079 robot car and arena world."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("mdp_simulation")
    share_root = os.path.dirname(pkg_share)
    headless_val = context.perform_substitution(LaunchConfiguration("headless")).lower() in ("true", "1")
    world_path = context.perform_substitution(LaunchConfiguration("world"))
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Resource paths for Gazebo to find worlds, models, meshes, and textures
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_paths = [
        share_root,
        pkg_share,
        os.path.join(pkg_share, "worlds"),
        os.path.join(pkg_share, "models"),
        os.path.join(pkg_share, "meshes"),
    ]
    if existing_resource_path:
        resource_paths.append(existing_resource_path)
    gz_resource_path = ":".join(resource_paths)

    # Local simulation environment: isolate from remote Zenoh router to prevent bad_alloc crashes
    sim_env = {
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "ZENOH_SESSION_CONFIG_URI": "",
        "GZ_SIM_RESOURCE_PATH": gz_resource_path,
        "IGN_GAZEBO_RESOURCE_PATH": gz_resource_path,
        "SDF_PATH": gz_resource_path,
    }

    # Apply to current Python process environment so sub-invocations inherit it immediately
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ.pop("ZENOH_SESSION_CONFIG_URI", None)
    os.environ["GZ_SIM_RESOURCE_PATH"] = gz_resource_path
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = gz_resource_path
    os.environ["SDF_PATH"] = gz_resource_path

    # Process Xacro to generate robot_description URDF string
    xacro_file = os.path.join(pkg_share, "urdf", "mdp_car.urdf.xacro")
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # Gazebo sim command
    gz_flags = ["-s", "-r", "-v", "1"] if headless_val else ["-r", "-v", "1"]
    gz_cmd = ["gz", "sim"] + gz_flags + [world_path]

    gz_sim_proc = ExecuteProcess(
        cmd=gz_cmd,
        output="screen",
        additional_env=sim_env,
    )

    # Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_desc,
            "use_sim_time": use_sim_time,
        }],
        additional_env=sim_env,
        output="screen",
    )

    # Spawn robot into Gazebo
    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "mdp_car",
            "-string", robot_desc,
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
        additional_env=sim_env,
        output="screen",
    )

    # ROS <-> Gazebo parameter bridge
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/mdp_car/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/model/mdp_car/sensors/ultrasonic@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/model/mdp_car/sensors/ir_left@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/model/mdp_car/sensors/ir_right@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        additional_env=sim_env,
        output="screen",
    )

    # Sim Hardware Bridge Node (maps Gazebo topics to MDP robot topics)
    sim_bridge_node = Node(
        package="mdp_simulation",
        executable="sim_bridge_node",
        parameters=[{
            "use_sim_time": use_sim_time,
            "initial_x": 0.20,
            "initial_y": 0.20,
            "initial_yaw": 1.57079632679,
        }],
        additional_env=sim_env,
        output="screen",
    )

    return [
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("ZENOH_SESSION_CONFIG_URI", ""),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", gz_resource_path),
        SetEnvironmentVariable("SDF_PATH", gz_resource_path),
        gz_sim_proc,
        rsp_node,
        spawn_node,
        bridge_node,
        sim_bridge_node,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory("mdp_simulation")
    default_world = os.path.join(pkg_share, "worlds", "arena.sdf")
    has_display = os.environ.get("DISPLAY", "") != ""
    default_headless = "false" if has_display else "true"

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("ZENOH_SESSION_CONFIG_URI", ""),
        DeclareLaunchArgument("headless", default_value=default_headless, description="Run Gazebo headless (no GUI)"),
        DeclareLaunchArgument("world", default_value=default_world, description="Path to Gazebo SDF world file"),
        DeclareLaunchArgument("x", default_value="0.20", description="Initial robot X position (m)"),
        DeclareLaunchArgument("y", default_value="0.20", description="Initial robot Y position (m)"),
        DeclareLaunchArgument("z", default_value="0.04", description="Initial robot Z position (m)"),
        DeclareLaunchArgument("yaw", default_value="1.57079632679", description="Initial robot yaw orientation (rad)"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock (/clock)"),
        OpaqueFunction(function=launch_setup),
    ])
