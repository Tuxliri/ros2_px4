#!/usr/bin/env python3
"""Indoor navigation launch file using OpenVINS with X500 in walls world.

This launch file demonstrates indoor navigation using:
  • PX4 SITL with X500 monocular forward-facing camera + IMU in walls world
  • OpenVINS for visual-inertial odometry
  • MAVROS for PX4 communication
  • ros_gz_bridge for Gazebo topics

Usage:
    ros2 launch your_package indoor_navigation_openvins.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="walls",
        description="Gazebo world to load (walls for indoor navigation)",
    )

    # MAVROS launch include
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(__file__),
                "mavros_px4.launch.py",
            )
        )
    )

    def _setup(context, *args, **kwargs):
        world_name = LaunchConfiguration("world").perform(context)
        
        # PX4 SITL with X500 monocular camera (forward-facing) in walls world
        px4_sitl = ExecuteProcess(
            cmd=[
                "bash",
                "-lc",
                f"cd ~/PX4-Autopilot && PX4_GZ_WORLD={world_name} make px4_sitl gz_x500_mono_cam",
            ],
            output="screen",
        )

        # Enhanced ros_gz bridge for OpenVINS requirements
        bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_openvins_bridge",
            output="screen",
            arguments=[
                # Simulation time
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                
                # Transform tree
                f"/world/{world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                
                # Camera topics for OpenVINS (forward-facing camera)
                f"/world/{world_name}/model/x500_mono_cam_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image",
                f"/world/{world_name}/model/x500_mono_cam_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                
                # IMU topics for OpenVINS  
                f"/world/{world_name}/model/x500_mono_cam_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                
                # Joint states
                f"/world/{world_name}/model/x500/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            ],
        )

        # OpenVINS node for visual-inertial odometry
        openvins_node = Node(
            package="ov_msckf",
            executable="run_subscribe_msckf",
            name="openvins_msckf",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "config_path": "/home/developer/ros2_ws/src/config/openvins_x500_mono.yaml",
            }],
            remappings=[
                # Camera remappings (forward-facing camera)
                ("/cam0/image_raw", f"/world/{world_name}/model/x500_mono_cam_0/link/camera_link/sensor/imager/image"),
                ("/cam0/camera_info", f"/world/{world_name}/model/x500_mono_cam_0/link/camera_link/sensor/imager/camera_info"),
                
                # IMU remapping
                ("/imu0", f"/world/{world_name}/model/x500_mono_cam_0/link/base_link/sensor/imu_sensor/imu"),
                
                # Output pose for MAVROS
                ("/ov_msckf/poseimu", "/vision_pose_estimate"),
            ],
        )

        # Transform OpenVINS output to MAVROS format
        vision_to_mavros = Node(
            package="vision_to_mavros",
            executable="vision_to_mavros_node",
            name="openvins_to_mavros",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "target_frame_id": "odom", 
                "source_frame_id": "base_link",
                "output_rate": 30.0,
                "roll_cam": 0.0,
                "pitch_cam": 0.0, 
                "yaw_cam": 0.0,
                "gamma_world": -1.5707963,  # -π/2 for ENU to NED conversion
            }],
            remappings=[
                ("/vision_pose", "/mavros/vision_pose/pose"),
            ],
        )

        # Static transform publisher for camera to base_link (forward-facing)
        camera_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_to_base_tf",
            arguments=[
                "0.1", "0", "0",   # x, y, z (camera 10cm forward of base)
                "0", "0", "0",     # roll, pitch, yaw (camera pointing forward)
                "base_link", "camera_link"
            ],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        )

        return [px4_sitl, bridge, openvins_node, vision_to_mavros, camera_tf, mavros_launch]

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        OpaqueFunction(function=_setup),
    ])
