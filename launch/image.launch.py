#!/usr/bin/env python3

'''
    Launches the ip_camera_ros2 node.
'''
import os
import xacro

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Getting directories and launch-files
    ipcam_ros2_dir = get_package_share_directory('ip_camera_ros2')
    default_params_file = os.path.join(ipcam_ros2_dir, 'config', 'params.yaml')

    # Input parameters declaration
    params_file = LaunchConfiguration('params_file')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file with detection configuration'
    )

    declare_log_level_arg = DeclareLaunchArgument(
        name='log-level',
        default_value='info',
        description='Logging level (info, debug, ...)'
    )

    # Create the launch configuration variables:
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Map these variables to arguments: can be set from the command line or a default will be used
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Prepare the ROS2 node.
    ipcam_ros2_node = Node(
        package = 'ip_camera_ros2',
        namespace = '',
        executable = 'ip_camera_ros2',
        name = 'ip_camera_ros2',
        parameters=[params_file],
        emulate_tty = True,
        output='screen', 
        arguments=[
            '--ros-args', 
            '--log-level', ['ip_camera_ros2:=', LaunchConfiguration('log-level')]]
    )

    return LaunchDescription([
        declare_params_file_arg,
        declare_log_level_arg,
        use_sim_time_launch_arg,
        ipcam_ros2_node
    ])
