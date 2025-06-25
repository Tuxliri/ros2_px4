import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')

    px4_sitl = ExecuteProcess(
        additional_env={'HEADLESS': '1'},
        cmd=['bash', '-lc', 
             # adjust path & model as needed (iris, typhoon_h480, etc)
             'cd ~/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_x500_mono_cam'],
        output='screen',
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen'
    )
    
    # Include local mavros_px4.launch.py
    mavros_px4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(__file__),
                'launch',
                'mavros_px4.launch.py'
            )
        ),
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'camera.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # Add an AprilTag model in the Gazebo world
    apriltag_gz_spawner = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-world', 'default',
             '-file', os.path.join(
                 '/home/developer/PX4-Autopilot/Tools/simulation/gz',
                 'models',
                 'Apriltag36_11_00009',
                 'model.sdf'
             ),
             '-x', '0.5'],
        output='screen'
    )

    # AprilTag Node
    apriltag_ros_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('apriltag_ros'),
            'cfg',
            'tags_36h11.yaml'
        )],
        remappings=[
            ('image_rect',  '/camera'),
            ('camera_info', '/camera_info'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open RViz.'),
        bridge,
        rviz,
        px4_sitl,
        mavros_px4_launch,
        apriltag_ros_node,
        # spawn_after_px4
        apriltag_gz_spawner
    ])