"""Placeholder bringup launch — filled in as modules come online."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim',
                default_value='true',
                description='Launch Gazebo simulation stack when true',
            ),
            LogInfo(
                msg=[
                    'inspection_bringup: skeleton launch only. ',
                    'use_sim=',
                    LaunchConfiguration('use_sim'),
                    ' — wire simulation/navigation/perception launches in later stages.',
                ]
            ),
        ]
    )
