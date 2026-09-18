import os
from glob import glob

from setuptools import setup

package_name = "mdp_gazebo"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "description"), glob("description/*.xacro")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*.STL")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ray Shao",
    maintainer_email="frieddeli@gmail.com",
    description="Gazebo simulation (world, robot description, sim-to-hardware bridge) for the SC2079 MDP robot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sim_bridge_node = mdp_gazebo.sim_bridge_node:main",
        ],
    },
)
