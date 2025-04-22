from setuptools import setup

package_name = "rover_telecom"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/ovif_config.json"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Renzo Damian",
    maintainer_email="renzo.damian.go@example.com",
    description="A ROS 2 package for controlling a PTZ camera.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ptz_camera_node = rover_telecom.ptz_camera_node:main",
        ],
    },
)
