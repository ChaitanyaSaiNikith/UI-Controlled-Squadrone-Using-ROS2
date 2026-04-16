"""
multi_uav_control.launch.py

Launches all nodes for the multi-UAV hierarchical control system:
  - leader_control       (drone0 operator interface)
  - formation_manager    (computes per-follower setpoints)
  - follower_controller  (one instance per follower, drone1–drone4)
  - state_monitor        (fleet telemetry aggregator)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('multi_uav_control')
    formations_config = os.path.join(pkg_share, 'config', 'formations.yaml')

    leader_control_node = Node(
        package='multi_uav_control',
        executable='leader_control',
        name='leader_control',
        output='screen',
    )

    formation_manager_node = Node(
        package='multi_uav_control',
        executable='formation_manager',
        name='formation_manager',
        output='screen',
        parameters=[formations_config],
    )

    follower_nodes = [
        Node(
            package='multi_uav_control',
            executable='follower_controller',
            name=f'follower_controller_drone{i}',
            output='screen',
            parameters=[
                formations_config,
                {'drone_id': i},
            ],
        )
        for i in range(1, 5)
    ]

    state_monitor_node = Node(
        package='multi_uav_control',
        executable='state_monitor',
        name='state_monitor',
        output='screen',
    )

    return LaunchDescription([
        leader_control_node,
        formation_manager_node,
        *follower_nodes,
        state_monitor_node,
    ])
