"""Just the Android bridge + hardware bridge + Zenoh router, for integration testing
and manual teleop control.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # The router process is launched inside this same LaunchDescription, so it
    # would otherwise inherit ZENOH_SESSION_CONFIG_URI from
    # scripts/activate_zenoh_pi.sh (mode:"client"). A client session only
    # connects, it never listens, so the router would silently fail to bind
    # port 7447. This override forces the router process onto a mode:"router"
    # config instead.
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

    stm32_port_arg = DeclareLaunchArgument(
        "stm32_port",
        default_value="/dev/ttyACM1",
        description="Serial device for the STM32 UART link",
    )
    rfcomm_device_arg = DeclareLaunchArgument(
        "rfcomm_device",
        default_value="/dev/rfcomm0",
        description="RFCOMM device android_bridge_node opens (bound by "
        "rfcomm-listen.sh -- see ros2_ws/bluetooth-setup/)",
    )

    # 1. Zenoh Router Daemon
    zenoh_router = ExecuteProcess(
        cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        name="zenoh_router",
        output="screen",
        additional_env={"ZENOH_SESSION_CONFIG_URI": router_cfg},
    )

    # 2. STM32 Hardware Serial Bridge
    hardware_bridge = Node(
        package="mdp_hardware_bridge",
        executable="serial_bridge_node",
        name="serial_bridge_node",
        parameters=[{"serial_port": LaunchConfiguration("stm32_port")}],
        output="screen",
    )

    # 3. Android Bluetooth Bridge
    android_bridge = Node(
        package="mdp_android_bridge",
        executable="android_bridge_node",
        name="android_bridge_node",
        parameters=[{"rfcomm_device": LaunchConfiguration("rfcomm_device")}],
        output="screen",
    )

    return LaunchDescription(
        [zenoh_router, stm32_port_arg, rfcomm_device_arg, hardware_bridge, android_bridge]
    )

