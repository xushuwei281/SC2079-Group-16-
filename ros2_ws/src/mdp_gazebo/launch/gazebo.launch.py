"""Gazebo simulation launch for SC2079 MDP (runs on a laptop/PC/Mac, pixi -e sim).

Starts gz-sim with the 200x200cm arena world, spawns the robot from its xacro
description, bridges /cmd_vel (ROS->GZ) and odometry (GZ->ROS), and runs
sim_bridge_node to turn that odometry into the /robot_pose(/raw) + TF +
Range-stub contract serial_bridge_node provides on real hardware.

With run_autonomous:=true it also starts motion_controller_node and
planner_node — the same nodes robot.launch.py runs on the Pi — completely
unmodified, so the full autonomous stack can be exercised against the
simulated robot instead of hardware.

Server and GUI are launched as two separate `gz sim` processes (-s / -g)
rather than one combined process: on macOS, gz-sim cannot run server+GUI in
a single process (https://github.com/gazebosim/gz-sim/issues/44) — a
combined `gz sim -r <world>` just dies with exit code 255 there. The split
form works on Linux too, so it's used unconditionally rather than branching
on platform. headless:=true skips the GUI process for CI / driving the sim
from Foxglove/RViz only.
"""
import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mdp_gazebo")
    world_path = os.path.join(pkg_share, "worlds", "mdp_arena.sdf")
    xacro_path = os.path.join(pkg_share, "description", "mdp_robot.urdf.xacro")
    bridge_config = os.path.join(pkg_share, "config", "gz_bridge.yaml")

    run_autonomous_arg = DeclareLaunchArgument(
        "run_autonomous", default_value="false",
        description="Also launch motion_controller_node and planner_node against the sim",
    )
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Run only the gz-sim server, without the GUI client",
    )

    # Single source of truth for the spawn pose, matching the real arena's
    # 40x40cm start zone centre (algorithm/arena.py) with heading 0 == East
    # (the planner's coordinate convention). Used for both `create`'s spawn
    # arguments and sim_bridge_node's spawn_x/y/yaw parameters below — the
    # DiffDrive plugin zeroes its odometry at the spawn pose, so
    # sim_bridge_node needs this exact value to re-derive the arena-frame
    # pose (see its module docstring).
    spawn_x, spawn_y, spawn_yaw = 0.2, 0.2, 0.0

    # Rendered once, here, rather than via a `Command(["xacro ", ...])`
    # parameter substitution: `create -topic robot_description` (spawning by
    # waiting for robot_state_publisher's one-shot latched publish) is racy
    # against gz-sim's own startup and was observed to hang indefinitely
    # waiting for a message that had already gone out. Spawning from a file
    # instead removes that race entirely — both robot_state_publisher and
    # `create` end up with the exact same URDF, deterministically.
    robot_description = xacro.process_file(xacro_path).toxml()

    # gz-sim's own URDF/SDF loader (used when spawning via `-file`) doesn't
    # understand ROS's package:// mesh URI convention the way
    # robot_state_publisher/RViz do — it only resolves file:// and model://.
    # So the file handed to `create` gets package:// rewritten to an
    # absolute file:// path; robot_state_publisher keeps the original
    # package://-based string, which it resolves natively.
    robot_description_for_gz = robot_description.replace(
        "package://mdp_gazebo/", f"file://{pkg_share}/")
    robot_description_path = os.path.join(
        tempfile.gettempdir(), "mdp_gazebo_robot_description.urdf")
    with open(robot_description_path, "w") as f:
        f.write(robot_description_for_gz)

    gz_server = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", world_path],
        name="gz_server",
        output="screen",
    )
    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-g"],
        name="gz_gui",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_mdp_robot",
        arguments=["-file", robot_description_path, "-name", "mdp_robot",
                   "-x", str(spawn_x), "-y", str(spawn_y), "-z", "0.05",
                   "-Y", str(spawn_yaw)],
        output="screen",
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_config}"],
        output="screen",
    )

    sim_bridge = Node(
        package="mdp_gazebo",
        executable="sim_bridge_node",
        name="sim_bridge_node",
        parameters=[{"spawn_x": spawn_x, "spawn_y": spawn_y, "spawn_yaw": spawn_yaw}],
        output="screen",
    )

    motion_controller = Node(
        package="mdp_hardware_bridge",
        executable="motion_controller_node",
        name="motion_controller_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_autonomous")),
    )

    planner = Node(
        package="mdp_bringup",
        executable="planner_node",
        name="planner_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_autonomous")),
    )

    return LaunchDescription([
        run_autonomous_arg,
        headless_arg,
        gz_server,
        gz_gui,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
        sim_bridge,
        motion_controller,
        planner,
    ])
