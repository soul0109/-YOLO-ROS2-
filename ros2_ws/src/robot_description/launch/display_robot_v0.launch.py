#!/usr/bin/env python3
"""
在 RViz 中显示 robot_v0 最小差速底盘（阶段 3.1）。

启动的 3 个节点：
  1. robot_state_publisher — 读取 URDF，根据 joint_states 发布 TF
  2. joint_state_publisher_gui — 左上角滑块窗口，手动发布 /joint_states（仅调试用）
  3. rviz2 — 3D 可视化

阶段 3.3 加入 Gazebo 驱动后，joint_state_publisher_gui 可移除，
由差速插件根据 /cmd_vel 自动发布 joint_states。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('robot_description')
    default_model = os.path.join(pkg_share, 'urdf', 'robot_v0.urdf.xacro')
    default_rviz = os.path.join(pkg_share, 'rviz', 'robot_v0.rviz')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model,
        description='robot_v0 xacro 文件路径',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时钟',
    )

    # 将 xacro 展开为 URDF 字符串，交给 robot_state_publisher
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str,
    )

    # 核心：URDF → TF 树（各 link 相对位姿）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # 调试工具：左上角滑块，向 /joint_states 发布轮子转角（阶段 3.1 专用）
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # 3D 可视化，加载预设布局（Grid + TF + RobotModel）
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
    )

    return LaunchDescription([
        model_arg,
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
