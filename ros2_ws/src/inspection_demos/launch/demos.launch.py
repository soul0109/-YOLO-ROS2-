"""
阶段 2 一键启动：同时拉起 pub/sub、service、action、静态 TF 全部 demo 节点。

用法（VM 内，已 source 工作空间后）：
    ros2 launch inspection_demos demos.launch.py

说明：
    - 各节点参数来自 config/demos.yaml
    - 可用 ros2 node list / ros2 topic list 查看运行中的节点与话题
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('inspection_demos')
    config_file = os.path.join(pkg_share, 'config', 'demos.yaml')

    return LaunchDescription(
        [
            Node(
                package='inspection_demos',
                executable='log_publisher',
                name='log_publisher',
                parameters=[config_file],
                output='screen',
            ),
            Node(
                package='inspection_demos',
                executable='log_subscriber',
                name='log_subscriber',
                parameters=[config_file],
                output='screen',
            ),
            Node(
                package='inspection_demos',
                executable='robot_status_server',
                name='robot_status_server',
                parameters=[config_file],
                output='screen',
            ),
            Node(
                package='inspection_demos',
                executable='patrol_action_server',
                name='patrol_action_server',
                parameters=[config_file],
                output='screen',
            ),
            Node(
                package='inspection_demos',
                executable='static_tf_broadcaster',
                name='static_tf_broadcaster',
                parameters=[config_file],
                output='screen',
            ),
        ]
    )
