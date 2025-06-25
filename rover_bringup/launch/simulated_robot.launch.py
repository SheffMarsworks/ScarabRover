import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():
    # Declare the teleop argument with an empty default value (which will be treated as "none")
    teleop_arg = DeclareLaunchArgument(
        "teleop",
        default_value="",
        description="Type of teleop: joystick, keyboard, or none",
    )

    def launch_setup(context, *args, **kwargs):
        teleop = LaunchConfiguration("teleop").perform(context)

        gazebo = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("rover_description"),
                "launch",
                "gazebo.launch.py",
            ),
        )

        controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("rover_controller"),
                "launch",
                "controller.launch.py",
            ),
        )
        # <-- Add this Node here to spawn ros2_control_node with your YAML
        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                os.path.join(
                    get_package_share_directory("rover_controller"),
                    "config",
                    "rover_controllers.yaml",
                )
            ],
            output="screen",
        )

        launch_descriptions = [gazebo, controller, ros2_control_node]  # Add the node here
        if teleop == "joystick":
            joystick = IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory("rover_controller"),
                    "launch",
                    "joystick_teleop.launch.py",
                ),
                launch_arguments={"use_sim_time": "True"}.items(),
            )
            launch_descriptions.append(joystick)
        elif teleop == "keyboard":
            keyboard = IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory("rover_controller"),
                    "launch",
                    "keyboard_teleop.launch.py",
                )
            )
            launch_descriptions.append(keyboard)

        return launch_descriptions

    return LaunchDescription([teleop_arg, OpaqueFunction(function=launch_setup)])
