from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    mavros_dir = get_package_share_directory("mavros")

    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="/dev/ttyACM0:115200",
        description="URL for connecting to the flight controller",
    )

    # Launch MAVROS using APM (ArduPilot) configuration
    apm_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(mavros_dir, "launch", "apm.launch")),
        launch_arguments={
            "fcu_url": LaunchConfiguration("fcu_url"),
        }.items(),
    )

    return LaunchDescription(
        [
            fcu_url_arg,
            apm_launch,
        ]
    )
