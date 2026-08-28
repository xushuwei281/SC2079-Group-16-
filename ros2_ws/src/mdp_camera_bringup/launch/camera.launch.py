"""Launch the Pi camera driver.

v4l2_camera running against Raspberry Pi's libcamera-V4L2 adaptation layer
(LD_PRELOAD of v4l2-compat.so, which runs the SYSTEM libcamera 0.7.x under
the hood). See pixi.toml [feature.pi.dependencies] for why camera_ros was
rejected. Run on the Pi only:

    pixi run -e pi camera

Publishes /image_raw (rgb8) and /camera_info for mdp_perception on the PC.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

V4L2_COMPAT_SO = "/usr/libexec/aarch64-linux-gnu/libcamera/v4l2-compat.so"


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("mdp_camera_bringup"), "config", "camera.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value=default_params,
                description="Path to the camera parameter file",
            ),
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="camera",
                output="screen",
                parameters=[LaunchConfiguration("params")],
                # Pin the topic names to the contract in ARCHITECTURE.md —
                # without this the node publishes bare /image_raw, and
                # image_transport computes the compressed name independently
                # of the raw remap, so it needs its own.
                remappings=[
                    ("image_raw", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info"),
                    ("image_raw/compressed", "/camera/image_raw/compressed"),
                ],
                additional_env={"LD_PRELOAD": V4L2_COMPAT_SO},
            ),
        ]
    )
