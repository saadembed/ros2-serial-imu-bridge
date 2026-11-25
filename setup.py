from glob import glob

from setuptools import find_packages, setup

package_name = "serial_imu_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Saad EL MOUSADIK",
    maintainer_email="saad.elmousadik@gmail.com",
    description="UART IMU bridge for ROS 2",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "bridge_node = serial_imu_bridge.bridge_node:main",
            "imu_filter_node = serial_imu_bridge.filter_node:main",
        ],
    },
)
