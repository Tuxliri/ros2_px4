"""Unified launch file for PX4 1.15 X-500 simulation with Gazebo Harmonic.

Starts:
  • **PX4 SITL + Gazebo**          (`make px4_sitl gz_x500_mono_cam_down`)
  • **ros_gz_bridge**              (clock, TF, joint_states, camera)
  • **robot_state_publisher**      (reads the original SDF)
  • **MAVROS**                     (included via mavros_px4.launch.py)

RViz and other demo nodes (AprilTag spawner, detectors, etc.) were
intentionally left out per request; feel free to add them back later.

Usage
-----
Place this file in any ROS2 package's *launch/* folder and run:

    ros2 launch <your_pkg> x500_ros_bringup.launch.py

Optional arguments:
  model_sdf  — path to the X-500 `model.sdf` (defaults to the PX4 repo copy)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():  # noqa: D401
    # ── Where is the SDF? -----------------------------------------------------
    default_model_path = os.path.expanduser(
        "~/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf"
    )
    print(f"Using X-500 model SDF: {default_model_path}")
    model_arg = DeclareLaunchArgument(
        "model_sdf",
        default_value=TextSubstitution(text=default_model_path),
        description="Absolute path to the X-500 model.sdf file",
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

    # ── Lazy setup to load the SDF *once* ------------------------------------
    def _setup(context, *args, **kwargs):  # noqa: ANN001
        sdf_path = os.path.expanduser(LaunchConfiguration("model_sdf").perform(context))
        world_name = LaunchConfiguration("world").perform(context)
        
        if not os.path.isfile(sdf_path):
            raise RuntimeError(f"SDF file not found: {sdf_path}")
        with open(sdf_path, "r", encoding="utf-8") as sdf_file:
            sdf_xml = sdf_file.read()

        # PX4‑SITL (Gazebo Harmonic) with configurable world
        px4_sitl = ExecuteProcess(
            cmd=[
                "bash",
                "-lc",
                f"cd ~/PX4-Autopilot && PX4_GZ_WORLD={world_name} make px4_sitl gz_x500_mono_cam_down",
            ],
            output="screen",
        )

        # ros_gz parameter bridge — clock + pose TF + joint states + camera
        bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_param_bridge",
            output="screen",
            # parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            arguments=[
                # Sim time
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                # All link poses → TF tree
                "/world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                # Joint states (if any movable joints)
                "/world/default/model/x500/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
                # Camera image + info so perception nodes can subscribe
                "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
        )

        # Robot‑state publisher directly from the SDF
        rsp = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="x500_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": sdf_xml,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
        )

        return [px4_sitl, bridge, rsp, mavros_launch]

    # ── Final LD --------------------------------------------------------------
    return LaunchDescription([
        model_arg,
        use_sim_time_arg,
        world_arg,
        OpaqueFunction(function=_setup),
    ])
