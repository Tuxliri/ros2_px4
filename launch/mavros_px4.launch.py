from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # -------------------------
    # Declare launch arguments
    # -------------------------
    ld.add_action(DeclareLaunchArgument(
        'fcu_url', default_value='udp://:14540@'))
    ld.add_action(DeclareLaunchArgument(
        'gcs_url', default_value=''))
    ld.add_action(DeclareLaunchArgument(
        'tgt_system', default_value='1'))
    ld.add_action(DeclareLaunchArgument(
        'tgt_component', default_value='1'))
    ld.add_action(DeclareLaunchArgument(
        'log_output', default_value='screen'))
    ld.add_action(DeclareLaunchArgument(
        'fcu_protocol', default_value='v2.0'))
    ld.add_action(DeclareLaunchArgument(
        'respawn_mavros', default_value='false'))
    ld.add_action(DeclareLaunchArgument(
        'namespace', default_value='mavros'))
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='true'))

    # -------------------------
    # Include MAVROS node file
    # -------------------------
    ld.add_action(
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mavros'),
                    'launch',
                    'node.launch'
                ])
            ),
            launch_arguments={
                'pluginlists_yaml': 'launch/px4_pluginlists.yaml',
                'config_yaml': PathJoinSubstitution([
                    FindPackageShare('mavros'),
                    'launch',
                    'px4_config.yaml'
                ]),
                'fcu_url':        LaunchConfiguration('fcu_url'),
                'gcs_url':        LaunchConfiguration('gcs_url'),
                'tgt_system':     LaunchConfiguration('tgt_system'),
                'tgt_component':  LaunchConfiguration('tgt_component'),
                'log_output':     LaunchConfiguration('log_output'),
                'fcu_protocol':   LaunchConfiguration('fcu_protocol'),
                'respawn_mavros': LaunchConfiguration('respawn_mavros'),
                'namespace':      LaunchConfiguration('namespace'),
                'use_sim_time':   LaunchConfiguration('use_sim_time'),
            }.items()
        )
    )

    return ld
