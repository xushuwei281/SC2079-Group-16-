"""Just the Android bridge + hardware bridge, for early integration testing
before the planner exists -- equivalent to the non-ROS implementation's
Week 2 "prove every comms link works" milestone (docs/week2-checklist.md).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stm32_port_arg = DeclareLaunchArgument(
        "stm32_port",
        default_value="/dev/ttyACM0",
        description="Serial device for the STM32 UART link",
    )
    rfcomm_device_arg = DeclareLaunchArgument(
        "rfcomm_device",
        default_value="/dev/rfcomm0",
        description="RFCOMM device android_bridge_node opens (bound by "
        "rfcomm-listen.sh -- see ros2_ws/bluetooth-setup/)",
    )

    hardware_bridge = Node(
        package="mdp_hardware_bridge",
        executable="serial_bridge_node",
        name="serial_bridge_node",
        parameters=[{"serial_port": LaunchConfiguration("stm32_port")}],
        output="screen",
    )
    android_bridge = Node(
        package="mdp_android_bridge",
        executable="android_bridge_node",
        name="android_bridge_node",
        parameters=[{"rfcomm_device": LaunchConfiguration("rfcomm_device")}],
        output="screen",
    )

    return LaunchDescription(
        [stm32_port_arg, rfcomm_device_arg, hardware_bridge, android_bridge]
    )
