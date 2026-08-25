from setuptools import find_packages, setup

package_name = "mdp_android_bridge"

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
    description="Bluetooth RFCOMM bridge to the Android remote controller tablet",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "android_bridge_node = mdp_android_bridge.android_bridge_node:main",
        ],
    },
)
