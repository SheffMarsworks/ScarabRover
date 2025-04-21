import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare the teleop argument with an empty default value (which will be treated as "none")
    teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='',
        description='Type of teleop: joystick, keyboard, or none'
    )

    def launch_setup(context, *args, **kwargs):
        teleop = LaunchConfiguration('teleop').perform(context)

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
                "controller.launch.py"
            ),
        )

        launch_descriptions = [gazebo, controller]

        if teleop == 'joystick':
            joystick = IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory("rover_controller"),
                    "launch",
                    "joystick_teleop.launch.py"
                ),
                launch_arguments={
                    "use_sim_time": "True"
                }.items()
            )
            launch_descriptions.append(joystick)
        elif teleop == 'keyboard':
            keyboard = IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory("rover_controller"),
                    "launch",
                    "keyboard_teleop.launch.py"
                )
            )
            launch_descriptions.append(keyboard)

        return launch_descriptions

    return LaunchDescription([
        teleop_arg,
        OpaqueFunction(function=launch_setup)
    ])