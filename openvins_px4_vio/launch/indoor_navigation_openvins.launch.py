#!/usr/bin/env python3
"""Indoor navigation launch file using OpenVINS with X500 in walls world.

This launch file demonstrates indoor navigation using:
  • PX4 SITL with X500 monocular camera (configurable direction) + IMU in walls world
  • OpenVINS for visual-inertial odometry
  • MAVROS for PX4 communication
  • ros_gz_bridge for Gazebo topics

Usage:
    ros2 launch openvins_px4_vio indoor_navigation_openvins.launch.py camera_direction:=forward
    ros2 launch openvins_px4_vio indoor_navigation_openvins.launch.py camera_direction:=down
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

    camera_direction_arg = DeclareLaunchArgument(
        "camera_direction",
        default_value="forward",
        description="Camera direction: 'down' for downward-facing or 'forward' for forward-facing camera",
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
        camera_direction = LaunchConfiguration("camera_direction").perform(context)
        
        # Determine model and camera configuration based on camera direction
        if camera_direction == "forward":
            px4_model = "gz_x500_mono_cam"
            camera_model_name = "x500_mono_cam_0"
        elif camera_direction == "down":
            px4_model = "gz_x500_mono_cam_down"
            camera_model_name = "x500_mono_cam_down_0"
        else:
            raise ValueError(f"Invalid camera_direction: {camera_direction}. Must be 'forward' or 'down'")
        
        # PX4 SITL with X500 monocular camera in walls world
        px4_sitl = ExecuteProcess(
            cmd=[
                "bash",
                "-lc",
                f"cd ~/PX4-Autopilot && PX4_GZ_WORLD={world_name} make px4_sitl {px4_model}",
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
                
                # Camera topics for OpenVINS
                f"/world/{world_name}/model/{camera_model_name}/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image",
                f"/world/{world_name}/model/{camera_model_name}/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                
                # IMU topics for OpenVINS  
                f"/world/{world_name}/model/{camera_model_name}/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                
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
                "config_path": os.path.join(
                    os.path.dirname(__file__), "..", "config", "openvins_x500_mono.yaml"
                ),
            }],
            remappings=[
                # Camera remappings
                ("/cam0/image_raw", f"/world/{world_name}/model/{camera_model_name}/link/camera_link/sensor/imager/image"),
                ("/cam0/camera_info", f"/world/{world_name}/model/{camera_model_name}/link/camera_link/sensor/imager/camera_info"),
                
                # IMU remapping
                ("/imu0", f"/world/{world_name}/model/{camera_model_name}/link/base_link/sensor/imu_sensor/imu"),
                
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
                "output_rate": 50.0,  # Higher rate for better EKF fusion in PX4 1.16
                "roll_cam": 0.0,
                "pitch_cam": 0.0, 
                "yaw_cam": 0.0,
                "gamma_world": -1.5707963,  # -π/2 for ENU to NED conversion
                "position_variance": 0.01,   # Uniform covariance for PX4 1.16
                "orientation_variance": 0.01,
                "velocity_variance": 0.01,
            }],
            remappings=[
                ("odom_out", "/mavros/odometry/out"),  # Use odometry topic instead
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

        return [
            px4_sitl,
            bridge,
            openvins_node,
            # vision_to_mavros,
            camera_tf,
            mavros_launch
            ]

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        camera_direction_arg,
        OpaqueFunction(function=_setup),
    ])
