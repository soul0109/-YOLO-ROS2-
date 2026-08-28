from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'inspection_bringup'

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
    description='Top-level launch files for the inspection stack.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
