"""Task 2 Fastest Car Launch File for SC2079 MDP (Runs on Raspberry Pi).

Brings up the complete reactive sprint stack:
1. rmw_zenohd router (multi-host & DDS bridge)
2. serial_bridge_node (STM32 motor & ultrasonic UART link)
3. android_bridge_node (Bluetooth RFCOMM link to Android remote tablet)
4. camera.launch.py (Pi Camera v2.1 video streamer)
5. perception_node (Local YOLO arrow detection & consensus sampler)
6. fastest_car_node (Reactive sprint FSM & slalom orchestrator)
7. Sensor TFs and Foxglove Studio bridge
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
    vantage_dist_arg = DeclareLaunchArgument(
        "vantage_dist_cm",
        default_value="30.0",
        description="Ultrasonic stopping distance at obstacles (cm)",
    )
    run_perception_arg = DeclareLaunchArgument(
        "run_perception",
        default_value="true",
        description="Launch local YOLO perception node on the Pi",
    )

    # 1. Zenoh Router Daemon
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

    # 4. Pi Camera Driver
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_bringup_dir, "launch", "camera.launch.py")
        )
    )

    # 5. Task 2 Fastest Car Reactive Sprint Node
    fastest_car = Node(
        package="mdp_bringup",
        executable="fastest_car_node",
        name="fastest_car_node",
        parameters=[{
            "vantage_dist_cm": LaunchConfiguration("vantage_dist_cm"),
            "approach_step_cm": 25.0,
            "min_approach_step_cm": 5.0,
            "return_straight_cm": 80.0,
        }],
        output="screen",
    )

    # 6. Local YOLO Perception Node
    perception = Node(
        package="mdp_perception",
        executable="perception_node",
        name="perception_node",
        condition=IfCondition(LaunchConfiguration("run_perception")),
        output="screen",
    )

    # 7. Static Sensor Transforms
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

    # 8. Foxglove Studio WebSocket Bridge
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
            fastest_car,
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
            vantage_dist_arg,
            run_perception_arg,
            delayed_nodes,
        ]
    )
