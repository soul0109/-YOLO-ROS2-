#!/usr/bin/env python3
"""
阶段 3.3 一键启动：Gazebo + robot_v0 + 方向键遥控。

用法：
  ros2 launch simulation_worlds gazebo_robot_v0_teleop.launch.py

或项目根目录：
  bash scripts/run_gazebo_teleop.sh --build
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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

    cmd_vel_timeout = Node(
        package='simulation_worlds',
        executable='cmd_vel_timeout.py',
        name='cmd_vel_timeout',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'timeout_sec': 0.5},
            {'input_topic': 'cmd_vel'},
            {'output_topic': 'cmd_vel_gazebo'},
        ],
    )

    keyboard_teleop = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='simulation_worlds',
                executable='keyboard_teleop.py',
                name='keyboard_teleop',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'cmd_vel_topic': 'cmd_vel'},
                    {'linear_speed': 0.15},
                    {'angular_speed': 0.5},
                    {'publish_rate_hz': 10.0},
                    {'key_release_timeout_sec': 0.2},
                ],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Gazebo 仿真时钟',
        ),
        gazebo,
        robot_state_publisher,
        cmd_vel_timeout,
        spawn_robot,
        keyboard_teleop,
    ])
