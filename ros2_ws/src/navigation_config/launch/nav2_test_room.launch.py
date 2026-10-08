#!/usr/bin/env python3
"""
阶段 4.5a：test_room + Gazebo + AMCL + Nav2 navigation + RViz。

用法：
  ros2 launch navigation_config nav2_test_room.launch.py
  python3 scripts/publish_amcl_initial_pose.py   # 另开终端，发 initialpose
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
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_test_room.yaml')
    rviz_config = os.path.join(pkg_nav, 'config', 'nav2_test_room.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')

    amcl_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'amcl_test_room.launch.py'),
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': 'false',
            'gui': gui,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py'),
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
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
            description='是否启动 RViz；无人值守可设 false',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='是否启动 gzclient；无人值守可设 false',
        ),
        amcl_stack,
        navigation,
        rviz_node,
    ])
