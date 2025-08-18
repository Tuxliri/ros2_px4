#!/usr/bin/env python3

"""
Launch file for MAVROS offboard control node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the node'
        ),
        
        Node(
            package='vio_gap_test',
            executable='mavros_offboard.py',
            name='offboard_control',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[],
            remappings=[
                # Remap MAVROS topics if needed
                # ('mavros/state', 'mavros/state'),
                # ('mavros/setpoint_position/local', 'mavros/setpoint_position/local'),
            ]
        )
    ])
