"""Unified launch file for PX4 1.15 X-500 simulation with Gazebo Harmonic.

Starts:
  • **PX4 SITL + Gazebo**          (configurable camera: `gz_x500_mono_cam` or `gz_x500_mono_cam_down`)
  • **ros_gz_bridge**              (clock, TF, joint_states, camera)
  • **robot_state_publisher**      (reads the original SDF)
  • **MAVROS**                     (included via mavros_px4.launch.py)

Usage
-----
Place this file in any ROS2 package's *launch/* folder and run:

    ros2 launch <your_pkg> x500_ros_bringup.launch.py

Optional arguments:
  model                 — Name of the PX4 model you want to use
  camera_direction      — 'down' for downward-facing camera or 'forward' for forward-facing camera
  world                 — Gazebo world to load (default, walls, aruco, etc.)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():  # noqa: D401
    # ── Where is the SDF? -----------------------------------------------------
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="gz_x500_mono_cam_down",
        description="Name of the PX4 model to use",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="default",
        description="Gazebo world to load (default, walls, aruco, etc.)",
    )

    # ── MAVROS launch include -------------------------------------------------
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(__file__),
                "mavros_px4.launch.py",
            )
        )
    )

    def _setup(context, *args, **kwargs):  # noqa: ANN001
        world_name = LaunchConfiguration("world").perform(context)
        model_px4 = LaunchConfiguration("model").perform(context)


        # PX4‑SITL (Gazebo Harmonic) with configurable world and camera direction
        px4_sitl = ExecuteProcess(
            cmd=[
                "bash",
                "-lc",
                f"cd ~/PX4-Autopilot && PX4_GZ_WORLD={world_name} make px4_sitl {model_px4}",
            ],
            output="screen",
        )

        # ros_gz parameter bridge — clock + pose TF + joint states + camera + rangefinder
        bridge_args = [
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            f"/world/{world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            f"/world/{world_name}/model/x500/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            f"/world/{world_name}/pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V",
            f"/world/{world_name}/model/x500_lidar_down_0/link/lidar_sensor_link/sensor/lidar/scan@sensor_msgs/msg/Range@gz.msgs.LaserScan",
        ]

        # If there is a camera model name add camera topics
        camera_model_map = {
            "gz_x500_mono_cam": "x500_mono_cam_0",
            "gz_x500_mono_cam_down": "x500_mono_cam_down_0",
        }

        camera_model_name = camera_model_map.get(model_px4)
        if camera_model_name is None:
            supported_models = ", ".join(camera_model_map.keys())
            print(f"Warning: No camera model found for PX4 model '{model_px4}'. No camera topics will be published."
                    f"Available models with cameras: {supported_models}")

        if camera_model_name:
            bridge_args.extend([
                f"/world/{world_name}/model/{camera_model_name}/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image",
                f"/world/{world_name}/model/{camera_model_name}/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ])

        # Remap topics (these apply regardless of camera)
        bridge_args.extend([
            '--ros-args', '--remap', f'/world/{world_name}/pose/info:=/gz/pose_info',
            '--remap', f'/world/{world_name}/model/x500_lidar_down_0/link/lidar_sensor_link/sensor/lidar/scan:=/mavros/laser_1_sub',
        ])

        # ros_gz parameter bridge — clock + pose TF + joint states + camera + rangefinder
        bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_param_bridge",
            output="screen",
            arguments=bridge_args,
        )

        return [px4_sitl, bridge, mavros_launch]

    # ── Final LD --------------------------------------------------------------
    return LaunchDescription([
        model_arg,
        use_sim_time_arg,
        world_arg,
        OpaqueFunction(function=_setup),
    ])
