#!/usr/bin/env python3
"""
阶段 3.2+3.3+3.4+3.5：Gazebo spawn robot_v0 + 差速驱动 + 激光 /scan + 相机。

启动内容：
  1. Gazebo（默认 empty.world；可改 world:=lidar_test.world）
  2. robot_state_publisher（URDF + TF，use_sim_time）
  3. spawn_entity（略高于地面落下）
  4. Gazebo 插件（写在 robot_v0.gazebo.xacro）：
     - joint_state_publisher → /joint_states
     - diff_drive → /cmd_vel_gazebo、/odom、odom→base_footprint TF
     - ray 激光 → /scan（frame_id=laser_link）
     - camera → /camera/image_raw（frame_id=camera_link）
  5. cmd_vel_timeout 节点：/cmd_vel → /cmd_vel_gazebo，超时自动刹车
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
    default_world = os.path.join(pkg_worlds, 'worlds', 'empty.world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            'world': world,
            'verbose': 'false',
            'gui': gui,
        }.items(),
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

    # 关节状态由 Gazebo 插件发布（见 robot_v0.gazebo.xacro），不再用独立 jsp 节点

    # spawn 略高于地面，让物理引擎自然落下稳定
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot_v0',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
        output='screen',
    )

    cmd_vel_timeout = Node(
        package='simulation_worlds',
        executable='cmd_vel_timeout.py',
        name='cmd_vel_timeout',
        output='screen',
        parameters=[
            # 必须用系统时钟，否则 /clock 未就绪前无法发零速度刹车（spawn 自溜）
            {'use_sim_time': False},
            {'timeout_sec': 0.5},
            {'input_topic': 'cmd_vel'},
            {'output_topic': 'cmd_vel_gazebo'},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Gazebo 仿真时钟',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Gazebo world 文件路径；激光验收可用 lidar_test.world',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='是否启动 gzclient；冒烟测试可设 false',
        ),
        DeclareLaunchArgument(
            'spawn_x',
            default_value='0.0',
            description='spawn 初始 x；test_room 建议 0.9',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='0.0',
            description='spawn 初始 y；test_room 建议 3.0',
        ),
        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.05',
            description='spawn 初始 z（略高于地面落下）',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='0.0',
            description='spawn 初始航向 yaw（rad）；test_room 起点朝 +X 为 0',
        ),
        gazebo,
        robot_state_publisher,
        cmd_vel_timeout,
        spawn_robot,
    ])
