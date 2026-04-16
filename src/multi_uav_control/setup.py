from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multi_uav_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py') + glob('launch/*.xml') + glob('launch/*.yaml')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Hierarchical multi-UAV control system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leader_teleop = multi_uav_control.leader_teleop:main',
            'leader_offboard = multi_uav_control.leader_offboard:main',
            'formation_manager = multi_uav_control.formation_manager:main',
            'follower_controller = multi_uav_control.follower_controller:main',
            'state_monitor = multi_uav_control.state_monitor:main',
        ],
    },
)
