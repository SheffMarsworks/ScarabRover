import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    parameters=[{
          'frame_id':'realsense_link', 
          'subscribe_depth':True,
          'subscribe_rgb':True,
          'approx_sync':True, 
          'queue_size':10,
          'qos':1, 
          'use_sim_time':True, 
          'Reg/Force3DoF':'true',  
          'RGBD/AngularUpdate':'0.01',
          'RGBD/LinearUpdate':'0.01',
          'RGBD/OptimizeFromGraphEnd':'false'
    }]

    remappings=[
          ('rgb/image', '/camera/image_raw'),
          ('rgb/camera_info', '/camera/camera_info'),
          ('depth/image', '/depth_camera'),
          ('depth/camera_info', '/depth_camera/camera_info')
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation time'),
    

        Node(
            package='rtabmap_odom', 
            executable='rgbd_odometry', 
            output='screen',
            parameters=parameters,
            remappings=remappings
        ),

        Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'] 
        ),
        
        Node(
            package='rtabmap_viz', 
            executable='rtabmap_viz', 
            output='screen',
            parameters=parameters,
            remappings=remappings
        ),
    ])
