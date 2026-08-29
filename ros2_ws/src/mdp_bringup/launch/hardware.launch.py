"""Low-level hardware bringup launch file for SC2079 MDP (Runs on Raspberry Pi).

Launches all essential low-level hardware & communication components 100% required
for the car to move, be controlled via Android tablet, stream camera video, and communicate
over Zenoh/Tailscale (without running the autonomous planner):

1. zenoh_router (rmw_zenohd on port 7447 for cross-machine Tailscale/Wi-Fi communication)
2. serial_bridge_node (STM32 motor, servo, gyro UART link)
3. android_bridge_node (Bluetooth RFCOMM link to Android remote tablet)
4. pi_camera_node (Pi Camera v2.1 hardware ISP video streamer)
5. foxglove_bridge (Port 8765 WebSocket live telemetry and video stream)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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

    # 5. Foxglove Studio WebSocket Bridge (Port 8765)
    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{"port": 8765, "address": "0.0.0.0"}],
        output="screen",
    )

    # Delayed launch of ROS nodes (gives Zenoh router 2.0s to bind port 7447 and start listening)
    delayed_nodes = TimerAction(
        period=2.0,
        actions=[
            hardware_bridge,
            android_bridge,
            camera_launch,
            foxglove,
        ],
    )

    return LaunchDescription(
        [
            zenoh_router,
            stm32_port_arg,
            rfcomm_device_arg,
            delayed_nodes,
        ]
    )
