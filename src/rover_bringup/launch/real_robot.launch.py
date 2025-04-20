import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mavros_controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rover_controller"),
            "launch",
            "mavros_controller.launch.py"
        ),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rover_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )
    
    return LaunchDescription([
        mavros_controller,
        joystick,
    ])