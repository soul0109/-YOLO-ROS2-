#!/usr/bin/env python3
"""
阶段 4.4：test_room + Gazebo + map_server + AMCL + RViz。

用法：
  ros2 launch navigation_config amcl_test_room.launch.py
  ros2 launch navigation_config amcl_test_room.launch.py rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_nav = get_package_share_directory('navigation_config')
    pkg_worlds = get_package_share_directory('simulation_worlds')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    world = os.path.join(pkg_worlds, 'worlds', 'test_room.world')
    map_yaml = os.path.join(pkg_nav, 'maps', 'test_room.yaml')
    amcl_params = os.path.join(pkg_nav, 'config', 'amcl_test_room.yaml')
    rviz_config = os.path.join(pkg_nav, 'config', 'amcl_localization.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_worlds, 'launch', 'gazebo_robot_v0.launch.py'),
        ),
        launch_arguments={
            'world': world,
            'spawn_x': '0.9',
            'spawn_y': '3.0',
            'spawn_yaw': '0.0',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'localization_launch.py'),
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': amcl_params,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Gazebo 仿真时钟',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='是否启动 RViz',
        ),
        gazebo,
        localization,
        rviz_node,
    ])
