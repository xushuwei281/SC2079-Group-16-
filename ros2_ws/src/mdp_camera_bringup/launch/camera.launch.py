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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

V4L2_COMPAT_SO = "/usr/libexec/aarch64-linux-gnu/libcamera/v4l2-compat.so"


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

    camera_node = Node(
        package="mdp_camera_bringup",
        executable="pi_camera_node",
        name="camera",
        output="screen",
        parameters=[
            {"width": 640, "height": 480, "fps": 15.0, "frame_id": "camera_link"}
        ],
    )

    return LaunchDescription([set_zenoh_env, camera_node])

