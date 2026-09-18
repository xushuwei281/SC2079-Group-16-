"""Launch the Pi camera driver.

Spawns the system Picamera2 daemon and bridges its shared-memory frames
(see mdp_camera_bringup/pi_camera_node.py) into ROS 2 topics. Run on the
Pi only:

    pixi run -e pi camera

Publishes /image_raw (rgb8) and /camera_info for mdp_perception on the PC.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # PIXI_PROJECT_ROOT (set by pixi for every task) is authoritative
    # regardless of where this repo is checked out; the installed-launch-
    # file-relative and hardcoded paths below are last-resort fallbacks for
    # a bare `ros2 launch` outside pixi.
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

