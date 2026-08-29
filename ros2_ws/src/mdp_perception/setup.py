from setuptools import find_packages, setup

package_name = "mdp_perception"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ray Shao",
    maintainer_email="frieddeli@gmail.com",
    description="YOLOv8 target recognition and verification stitching node for MDP",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "perception_node = mdp_perception.perception_node:main",
        ],
    },
)
