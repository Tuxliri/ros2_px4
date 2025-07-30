#!/usr/bin/env python3
"""
OpenVINS to MAVROS bridge launch file.

This launch file bridges OpenVINS visual-inertial odometry output to MAVROS
for indoor navigation without GPS. It handles the coordinate frame transformations
between OpenVINS output and PX4's expected ENU frame.

Usage:
    ros2 launch your_package openvins_to_mavros.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    target_frame_id_arg = DeclareLaunchArgument(
        'target_frame_id',
        default_value='odom',
        description='Target frame id (world/odom frame from OpenVINS)'
    )

    source_frame_id_arg = DeclareLaunchArgument(
        'source_frame_id', 
        default_value='base_link',
        description='Source frame id (body frame)'
    )

    output_rate_arg = DeclareLaunchArgument(
        'output_rate',
        default_value='30.0',
        description='Publishing rate in Hz'
    )

    # Transform parameters for coordinate frame conversion
    roll_cam_arg = DeclareLaunchArgument(
        'roll_cam',
        default_value='0.0',
        description='Roll angle of the camera frame'
    )

    pitch_cam_arg = DeclareLaunchArgument(
        'pitch_cam',
        default_value='0.0',
        description='Pitch angle of the camera frame'  
    )

    yaw_cam_arg = DeclareLaunchArgument(
        'yaw_cam',
        default_value='0.0',
        description='Yaw angle of the camera frame'
    )

    gamma_world_arg = DeclareLaunchArgument(
        'gamma_world',
        default_value='-1.5707963',  # -π/2 for ENU to NED conversion
        description='Rotation of the world frame around Z (ENU to NED)'  
    )

    # OpenVINS to MAVROS bridge node
    openvins_to_mavros_node = Node(
        package='vision_to_mavros',
        executable='vision_to_mavros_node',
        name='openvins_to_mavros',
        output='screen',
        parameters=[{
            'target_frame_id': LaunchConfiguration('target_frame_id'),
            'source_frame_id': LaunchConfiguration('source_frame_id'),
            'output_rate': LaunchConfiguration('output_rate'),
            'roll_cam': LaunchConfiguration('roll_cam'),
            'pitch_cam': LaunchConfiguration('pitch_cam'),
            'yaw_cam': LaunchConfiguration('yaw_cam'),
            'gamma_world': LaunchConfiguration('gamma_world'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            # Input: OpenVINS pose estimate
            ('vision_pose_input', '/ov_msckf/poseimu'),
            # Output: MAVROS vision pose  
            ('vision_pose', '/mavros/vision_pose/pose'),
            # Optional: Trajectory visualization
            ('body_frame/path', '/openvins_trajectory')
        ]
    )

    # Static transform publishers for coordinate frame definitions
    # Camera to IMU/base_link transform (forward-facing camera)
    camera_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_tf',
        arguments=[
            '0.1', '0.0', '0.0',         # x, y, z translation (camera 10cm forward of base)
            '0.0', '0.0', '0.0',         # roll, pitch, yaw (camera pointing forward)
            'base_link', 'camera_link'   # parent, child frames
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # IMU to base_link transform (typically identity for PX4)
    imu_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher', 
        name='imu_to_base_tf',
        arguments=[
            '0.0', '0.0', '0.0',         # x, y, z translation
            '0.0', '0.0', '0.0',         # roll, pitch, yaw rotation  
            'base_link', 'imu_link'      # parent, child frames
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        target_frame_id_arg,
        source_frame_id_arg,
        output_rate_arg,
        roll_cam_arg,
        pitch_cam_arg,
        yaw_cam_arg,
        gamma_world_arg,
        openvins_to_mavros_node,
        camera_to_base_tf,
        imu_to_base_tf,
    ])
