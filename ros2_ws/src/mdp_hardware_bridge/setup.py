from setuptools import find_packages, setup

package_name = "mdp_hardware_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Ray Shao",
    maintainer_email="frieddeli@gmail.com",
    description="UART bridge to the STM32 motor controller (continuous velocity protocol)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "serial_bridge_node = mdp_hardware_bridge.serial_bridge_node:main",
            "motion_controller_node = mdp_hardware_bridge.motion_controller_node:main",
            "teleop_keyboard = mdp_hardware_bridge.teleop_keyboard:main",
        ],
    },
)
