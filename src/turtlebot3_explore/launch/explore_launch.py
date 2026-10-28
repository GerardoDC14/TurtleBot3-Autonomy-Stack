"""Launch the Python explorer against an already running map and Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('turtlebot3_explore'),
                          'config', 'explore_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=config),
        Node(package='turtlebot3_explore', executable='explore', name='explore_node',
             output='screen', parameters=[LaunchConfiguration('params_file'), {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
