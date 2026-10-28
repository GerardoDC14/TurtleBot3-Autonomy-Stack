"""Compose exactly one mapping/localization source and one mission client."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup_mission(context):
    mode = LaunchConfiguration('mode').perform(context)
    explorer = LaunchConfiguration('explorer').perform(context)
    model = LaunchConfiguration('model').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    params = LaunchConfiguration('nav2_params').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    if mode == 'localization' and not os.path.isfile(map_file):
        raise ValueError('localization requires map:=/absolute/path/to/map.yaml')
    if not params:
        params = os.path.join(get_package_share_directory('turtlebot3_navigation2'),
                              'param', 'humble', model + '.yaml')
    if not os.path.isfile(params):
        raise FileNotFoundError(params)
    bringup = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
    actions = [IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup),
               launch_arguments={'slam': str(mode == 'slam').lower(), 'map': map_file,
                                 'params_file': params, 'use_sim_time': str(use_sim_time).lower(),
                                 'use_composition': 'false', 'autostart': 'true'}.items())]
    if explorer != 'none':
        config = LaunchConfiguration('explore_params').perform(context)
        package, executable = (('turtlebot3_explore', 'explore') if explorer == 'python'
                               else ('explore_cpp', 'explore_node'))
        actions.append(Node(package=package, executable=executable, name='explore_node',
                            output='screen', parameters=[config, {'use_sim_time': use_sim_time}]))
    return actions


def generate_launch_description():
    config = os.path.join(get_package_share_directory('turtlebot3_explore'),
                          'config', 'explore_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='slam', choices=['slam', 'localization']),
        DeclareLaunchArgument('explorer', default_value='none', choices=['none', 'python', 'cpp']),
        DeclareLaunchArgument('model', default_value=os.environ.get('TURTLEBOT3_MODEL', 'burger'),
                              choices=['burger', 'waffle', 'waffle_pi']),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('nav2_params', default_value=''),
        DeclareLaunchArgument('explore_params', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup_mission),
    ])
