from setuptools import find_packages, setup

package_name = 'llm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soul0109',
    maintainer_email='3250168367@qq.com',
    description='Natural-language command parsing for patrol tasks.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # 'llm_command_node = llm_control.llm_command_node:main',
        ],
    },
)
