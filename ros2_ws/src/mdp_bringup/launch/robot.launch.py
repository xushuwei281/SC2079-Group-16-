"""Master robot bringup launch file for SC2079 MDP (Runs on Raspberry Pi).

Launches all hardware-bound and autonomous mission nodes in one command:
1. serial_bridge_node (STM32 motor & gyro UART link)
2. android_bridge_node (Bluetooth RFCOMM link to Android remote tablet)
3. v4l2_camera_node (Pi Camera v2.1 RGB8 640x480 video streamer)
4. planner_node (Autonomous Reeds-Shepp TSP path planner & mission orchestrator)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Locate Zenoh local client config. PIXI_PROJECT_ROOT (set by pixi for
    # every task) is authoritative regardless of where this repo is checked
    # out; the installed-launch-file-relative and hardcoded paths below are
    # last-resort fallbacks for a bare `ros2 launch` outside pixi.
    config_candidates = [
        os.path.join(root, "config/zenoh_client_local.json5")
        for root in [os.environ.get("PIXI_PROJECT_ROOT")]
        if root
    ] + [
        os.path.abspath(
            os.path.join(
                os.path.dirname(__file__), "../../../config/zenoh_client_local.json5"
            )
        ),
        "/home/mdp/dev/SC2079-Group-16/ros2_ws/config/zenoh_client_local.json5",
    ]
    zenoh_cfg = config_candidates[0]
    for cp in config_candidates:
        if os.path.exists(cp):
            zenoh_cfg = cp
            break

    set_zenoh_env = SetEnvironmentVariable("ZENOH_SESSION_CONFIG_URI", zenoh_cfg)

    # Router-only config: the router process is launched inside this same
    # LaunchDescription, so it would otherwise inherit ZENOH_SESSION_CONFIG_URI
    # from either set_zenoh_env above or scripts/activate_zenoh_pi.sh (both point
    # at the mode:"client" config). A client session only connects, it never
    # listens, so the router would silently fail to bind port 7447. This override
    # forces the router process specifically onto a mode:"router" config.
    router_config_candidates = [
        os.path.join(root, "config/zenoh_router_pi.json5")
        for root in [os.environ.get("PIXI_PROJECT_ROOT")]
        if root
    ] + [
        os.path.abspath(
            os.path.join(
                os.path.dirname(__file__), "../../../config/zenoh_router_pi.json5"
            )
        ),
        "/home/mdp/dev/SC2079-Group-16/ros2_ws/config/zenoh_router_pi.json5",
    ]
    router_cfg = router_config_candidates[0]
    for cp in router_config_candidates:
        if os.path.exists(cp):
            router_cfg = cp
            break

    # Package share directories
    camera_bringup_dir = get_package_share_directory("mdp_camera_bringup")

    # Launch arguments
    stm32_port_arg = DeclareLaunchArgument(
        "stm32_port",
        default_value="/dev/ttyACM1",
        description="Serial device for STM32 UART link",
    )
    rfcomm_device_arg = DeclareLaunchArgument(
        "rfcomm_device",
        default_value="/dev/rfcomm0",
        description="RFCOMM device for Android Bluetooth link",
    )
    turning_radius_arg = DeclareLaunchArgument(
        "turning_radius_cm",
        default_value="21.0",
        description="Turning radius in cm for Reeds-Shepp path planner",
    )
    run_perception_arg = DeclareLaunchArgument(
        "run_perception",
        default_value="true",
        description="Launch local YOLO perception node on the Pi",
    )

    # 1. Zenoh Router Daemon (Port 7447 for cross-machine Tailscale/Wi-Fi communication)
    zenoh_router = ExecuteProcess(
        cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        name="zenoh_router",
        output="screen",
        additional_env={"ZENOH_SESSION_CONFIG_URI": router_cfg},
    )

    # 2. STM32 Serial Hardware Bridge Node
    motion_controller = Node(
        package="mdp_hardware_bridge",
        executable="motion_controller_node",
        name="motion_controller_node",
        output="screen",
    )

    hardware_bridge = Node(
        package="mdp_hardware_bridge",
        executable="serial_bridge_node",
        name="serial_bridge_node",
        parameters=[{"serial_port": LaunchConfiguration("stm32_port")}],
        output="screen",
    )

    # 3. Android Bluetooth Bridge Node
    android_bridge = Node(
        package="mdp_android_bridge",
        executable="android_bridge_node",
        name="android_bridge_node",
        parameters=[{"rfcomm_device": LaunchConfiguration("rfcomm_device")}],
        output="screen",
    )

    # 4. Pi Camera Driver (Includes v4l2-compat.so LD_PRELOAD)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_bringup_dir, "launch", "camera.launch.py")
        )
    )

    # 5. Autonomous Mission & Path Planner Node
    planner = Node(
        package="mdp_bringup",
        executable="planner_node",
        name="planner_node",
        parameters=[{
            "turning_radius_cm": LaunchConfiguration("turning_radius_cm"),
            "camera_view_dist_cm": 25.0,
        }],
        output="screen",
    )

    # 6. Optional On-Device YOLO Perception Node
    perception = Node(
        package="mdp_perception",
        executable="perception_node",
        name="perception_node",
        condition=IfCondition(LaunchConfiguration("run_perception")),
        output="screen",
    )

    # 7. Static Sensor Transforms (TF base_link -> ultrasonic_link, ir_left_link, ir_right_link)
    tf_ultrasonic = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_ultrasonic",
        arguments=["--x", "0.10", "--y", "0.0", "--z", "0.05",
                   "--yaw", "0", "--pitch", "0", "--roll", "0",
                   "--frame-id", "base_link", "--child-frame-id", "ultrasonic_link"],
    )
    tf_ir_left = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_ir_left",
        arguments=["--x", "0.08", "--y", "0.06", "--z", "0.04",
                   "--yaw", "0.52", "--pitch", "0", "--roll", "0",
                   "--frame-id", "base_link", "--child-frame-id", "ir_left_link"],
    )
    tf_ir_right = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_ir_right",
        arguments=["--x", "0.08", "--y", "-0.06", "--z", "0.04",
                   "--yaw", "-0.52", "--pitch", "0", "--roll", "0",
                   "--frame-id", "base_link", "--child-frame-id", "ir_right_link"],
    )

    # 8. Foxglove Studio WebSocket Bridge (Port 8765)
    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{"port": 8765, "address": "0.0.0.0", "ignore_unresponsive_param_nodes": True}],
        output="screen",
        sigterm_timeout="1.0",
        sigkill_timeout="1.0",
    )

    # Delayed launch of all ROS nodes (gives Zenoh router 2.0s to bind and start listening)
    delayed_nodes = TimerAction(
        period=2.0,
        actions=[
            hardware_bridge,
            motion_controller,
            android_bridge,
            camera_launch,
            planner,
            perception,
            tf_ultrasonic,
            tf_ir_left,
            tf_ir_right,
            foxglove,
        ],
    )

    return LaunchDescription(
        [
            set_zenoh_env,
            zenoh_router,
            stm32_port_arg,
            rfcomm_device_arg,
            turning_radius_arg,
            run_perception_arg,
            delayed_nodes,
        ]
    )
