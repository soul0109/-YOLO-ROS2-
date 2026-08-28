#!/usr/bin/env python3
"""
阶段 3.2：在 Gazebo 空世界中 spawn robot_v0。

启动内容：
  1. Gazebo（empty.world）
  2. robot_state_publisher（发布 URDF + TF，use_sim_time）
  3. joint_state_publisher（轮子初始角为 0，use_sim_time）
  4. spawn_entity（把机器人放到地面上）

阶段 3.3 再本 launch 基础上加差速驱动插件与 /cmd_vel。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_robot = get_package_share_directory('robot_description')
    pkg_worlds = get_package_share_directory('simulation_worlds')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_robot, 'urdf', 'robot_v0.urdf.xacro')
    world_file = os.path.join(pkg_worlds, 'worlds', 'empty.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time},
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # spawn 略高于地面，让物理引擎自然落下稳定
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot_v0',
            '-topic', 'robot_description',
            '-z', '0.05',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Gazebo 仿真时钟',
        ),
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
    ])
