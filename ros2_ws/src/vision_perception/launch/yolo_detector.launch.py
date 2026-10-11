#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *_args, **_kwargs):
    """Build the node; only override model_path when the launch arg is non-empty.

    An empty LaunchConfiguration would otherwise clobber YAML and force passthrough.
    """
    params_file = LaunchConfiguration('params_file').perform(context)
    model_path = LaunchConfiguration('model_path').perform(context).strip()
    parameters = [params_file]
    if model_path:
        parameters.append({'model_path': model_path})

    return [
        Node(
            package='vision_perception',
            executable='yolo_detector_node',
            name='yolo_detector',
            output='screen',
            parameters=parameters,
        ),
    ]


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
            description='Optional override for Ultralytics .pt/.onnx path',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
