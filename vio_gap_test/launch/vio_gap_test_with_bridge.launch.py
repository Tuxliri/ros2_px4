from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gap_duration_ms',
            default_value='1000',
            description='Duration of VIO gap in milliseconds'
        ),
        DeclareLaunchArgument(
            'gap_interval_ms', 
            default_value='5000',
            description='Interval between gaps in milliseconds'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='Frame ID for odometry messages'
        ),
        DeclareLaunchArgument(
            'child_frame_id',
            default_value='base_link',
            description='Child frame ID for odometry messages'
        ),
        DeclareLaunchArgument(
            'pose_index',
            default_value='1',
            description='Index in PoseArray to use (base_link entity ID)'
        ),
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/gz/pose_info',
            description='PoseArray topic to subscribe to'
        ),
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true', 
            description='Use simulated clock'
        ),

        # Gazebo-ROS2 bridge to convert Gazebo topics to ROS2 topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_pose_bridge',
            arguments=[
                # Sim time
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                # PoseArray topic from Gazebo
                "/ground_truth/odom_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance",
            ],
            # remappings=[
            #     ('/ground_truth/odom_with_covariance', '/mavros/odometry/ouht'),
            # ],
            output='screen'
        ),
        
        # VIO Gap Simulator
        Node(
            package='vio_gap_test',
            executable='vio_gap_simulator',
            name='vio_gap_simulator',
            parameters=[{
                'input_topic': '/ground_truth/odom_with_covariance',
                'output_topic': '/mavros/odometry/out',
                'gap_milliseconds': 10.0,
                'gap_start_delay': 20000.0
            }],
            output='screen'
        ),

        # Include local x500_ros_bringup.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    os.path.dirname(__file__),
                    '../../launch/x500_ros_bringup.launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'camera_direction': 'down',  # Default to downward-facing camera
                'world': 'default'  # Default world name
            }.items()
        )

    ])
