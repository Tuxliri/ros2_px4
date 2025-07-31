import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true', 
        description='Use simulated clock'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz', 
        default_value='false',
        description='Open RViz.'
    )
    
    camera_direction_arg = DeclareLaunchArgument(
        'camera_direction',
        default_value='down',
        description="Camera direction: 'down' for downward-facing or 'forward' for forward-facing"
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='default',
        description='Gazebo world name'
    )

    def _setup(context, *args, **kwargs):
        camera_direction = LaunchConfiguration('camera_direction').perform(context)
        world_name = LaunchConfiguration('world').perform(context)
        
        # Determine camera model name based on direction
        if camera_direction == "forward":
            camera_model_name = "x500_mono_cam_0"
        elif camera_direction == "down":
            camera_model_name = "x500_mono_cam_down_0"
        else:
            raise ValueError(f"Invalid camera_direction: {camera_direction}. Must be 'forward' or 'down'")
    
        # Include local x500_ros_bringup.launch.py
        x500_ros_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    os.path.dirname(__file__),
                    'x500_ros_bringup.launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'camera_direction': LaunchConfiguration('camera_direction'),
                'world': LaunchConfiguration('world')
            }.items()
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
                 '-world', world_name,
                 '-file', os.path.join(
                     '/home/developer/ros2_ws/src/gazebo_apriltag',
                     'models',
                     'Apriltag36_11_00009',
                     'model.sdf'
                 ),
                ],
            output='screen'
        )

        # AprilTag Node with dynamic topic remapping
        apriltag_ros_node = Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('apriltag_ros'),
                    'cfg',
                    'tags_36h11.yaml'
                ),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('image_rect',  f'/world/{world_name}/model/{camera_model_name}/link/camera_link/sensor/imager/image'),
                ('camera_info', f'/world/{world_name}/model/{camera_model_name}/link/camera_link/sensor/imager/camera_info'),
            ]
        )

        # Foxglove bridge node
        foxglove_bridge_node = Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
        )
        
        return [x500_ros_bringup, rviz, apriltag_ros_node, apriltag_gz_spawner, foxglove_bridge_node]

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        camera_direction_arg,
        world_arg,
        OpaqueFunction(function=_setup),
    ])
