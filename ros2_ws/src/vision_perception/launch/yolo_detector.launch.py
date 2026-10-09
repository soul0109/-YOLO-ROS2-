#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('vision_perception')
    default_params = os.path.join(package_share, 'config', 'yolo_detector.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YOLO detector parameter file',
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='',
            description='Optional local Ultralytics model path',
        ),
        Node(
            package='vision_perception',
            executable='yolo_detector_node',
            name='yolo_detector',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'model_path': LaunchConfiguration('model_path')},
            ],
        ),
    ])
