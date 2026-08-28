from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'inspection_demos'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soul0109',
    maintainer_email='3250168367@qq.com',
    description='ROS2 communication demos for phase 2.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'log_publisher = inspection_demos.log_publisher:main',
            'log_subscriber = inspection_demos.log_subscriber:main',
            'robot_status_server = inspection_demos.robot_status_server:main',
            'patrol_action_server = inspection_demos.patrol_action_server:main',
            'static_tf_broadcaster = inspection_demos.static_tf_broadcaster:main',
        ],
    },
)
