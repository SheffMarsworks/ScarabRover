import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time if true"
    )

    keyboard_teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_keyboard",
        output="screen",
        prefix="gnome-terminal -- bash -c",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    twist_stamper_node = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="twist_stamper",
        remappings=[
            ("cmd_vel_in", "/cmd_vel"),
            ("cmd_vel_out", "/rover_controller/cmd_vel"),
        ],
        parameters=[{"frame_id": "base_link"}],
    )

    return LaunchDescription(
        [use_sim_time_arg, keyboard_teleop_node, twist_stamper_node]
    )
