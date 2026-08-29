import os
from glob import glob

from setuptools import setup

package_name = "mdp_camera_bringup"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ray Shao",
    maintainer_email="frieddeli@gmail.com",
    description="Launch and configuration for the Pi camera driver (v4l2_camera over the libcamera-V4L2 compat layer)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pi_camera_node = mdp_camera_bringup.pi_camera_node:main",
        ],
    },
)
