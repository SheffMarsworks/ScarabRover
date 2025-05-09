import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

world_file = "mars.world.sdf"  # mars world
# world_file = "empty.sdf"  # empty world
# world_file = "warehouse.sdf"  # warehouse world


def generate_launch_description():
    rover_description = get_package_share_directory("rover_description")
    # world_file = os.path.join(rover_description, "worlds", "wro_world.sdf")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(rover_description, "urdf", "rover.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    simu_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )
    qt_qpa_platform = SetEnvironmentVariable(name="QT_QPA_PLATFORM", value="xcb")

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(rover_description).parent.resolve())],
    )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(
        Command(
            ["xacro ", LaunchConfiguration("model"), " is_ignition:=", is_ignition]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py",
            ]
        ),
        launch_arguments=[
            ("gz_args", [rover_description + "/worlds/" + world_file, " -v 4", " -r"])
        ],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "scarab",
            "-x",
            "0.0",  # X position in meters
            "-y",
            "-1.0",  # Y position in meters
            "-z",
            "0.15",  # Z position in meters
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            "/depth_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",
            "/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            "/depth_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/depth_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
        ],
        remappings=[("/depth_camera/image", "/depth_camera/image_raw")],
    )

    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        output="screen",
        parameters=[{"use_mag": False, "world_frame": "enu", "publish_tf": True}],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=os.path.join(rover_description, "models")
                + ":"
                + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
            ),
            SetEnvironmentVariable(
                name="GAZEBO_MODEL_PATH",
                value=os.path.join(rover_description, "models")
                + ":"
                + os.environ.get("GAZEBO_MODEL_PATH", ""),
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=os.path.join(rover_description, "models")
            ),
            simu_time,
            qt_qpa_platform,
            model_arg,
            gazebo_resource_path,
            robot_state_publisher_node,
            gazebo,
            gz_spawn_entity,
            gz_ros2_bridge,
            imu_filter,
        ]
    )
