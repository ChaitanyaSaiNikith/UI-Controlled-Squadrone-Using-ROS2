"""
formation_control.launch.py

Launches the full formation control system.
Follower count is read from the NUM_FOLLOWERS environment variable (default 2).

Usage:
  export NUM_FOLLOWERS=4
  ros2 launch multi_uav_control formation_control.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node

NUM_FOLLOWERS = int(os.environ.get('NUM_FOLLOWERS', '2'))


def generate_launch_description():
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    leader_teleop = Node(
        package='multi_uav_control',
        executable='leader_teleop',
        name='leader_teleop',
        output='screen',
    )

    leader_offboard = Node(
        package='multi_uav_control',
        executable='leader_offboard',
        name='leader_offboard',
        output='screen',
    )

    # spawn_north_m = Gazebo Y spacing between followers (metres).
    # Must match POSE_Y step in launch_multi_sitl.sh  (default: i*3 → 3.0 m).
    SPAWN_NORTH_M = float(os.environ.get('SPAWN_NORTH_M', '3.0'))

    manager = Node(
        package='multi_uav_control',
        executable='formation_manager',
        name='formation_manager',
        output='screen',
        parameters=[{
            'num_followers': NUM_FOLLOWERS,
            'spawn_north_m': SPAWN_NORTH_M,
        }],
    )

    followers = [
        Node(
            package='multi_uav_control',
            executable='follower_controller',
            name=f'follower_controller_drone{i}',
            output='screen',
            parameters=[{'drone_id': i, 'spawn_north_m': SPAWN_NORTH_M}],
        )
        for i in range(1, NUM_FOLLOWERS + 1)
    ]

    return LaunchDescription([rosbridge, joy_node, leader_teleop, leader_offboard, manager] + followers)
