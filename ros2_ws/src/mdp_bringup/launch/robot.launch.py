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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Locate Zenoh local client config
    config_candidates = [
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

    # 1. Zenoh Router Daemon (Port 7447 for cross-machine Tailscale/Wi-Fi communication)
    zenoh_router = ExecuteProcess(
        cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        name="zenoh_router",
        output="screen",
        additional_env={"ZENOH_SESSION_CONFIG_URI": router_cfg},
    )

    # 2. STM32 Serial Hardware Bridge Node
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
        parameters=[{"turning_radius_cm": 31.0, "camera_view_dist_cm": 25.0}],
        output="screen",
    )

    # 6. Foxglove Studio WebSocket Bridge (Port 8765)
    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{"port": 8765, "address": "0.0.0.0"}],
        output="screen",
    )

    # Delayed launch of all ROS nodes (gives Zenoh router 2.0s to bind and start listening)
    delayed_nodes = TimerAction(
        period=2.0,
        actions=[
            hardware_bridge,
            android_bridge,
            camera_launch,
            planner,
            foxglove,
        ],
    )

    return LaunchDescription(
        [
            set_zenoh_env,
            zenoh_router,
            stm32_port_arg,
            rfcomm_device_arg,
            delayed_nodes,
        ]
    )
