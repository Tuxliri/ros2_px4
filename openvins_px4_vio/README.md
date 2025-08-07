# OpenVINS PX4 VIO Package

This package provides launch files and configuration for visual-inertial odometry (VIO) integration between OpenVINS and PX4 using MAVROS.

## Overview

The `openvins_px4_vio` package demonstrates indoor navigation using:
- PX4 SITL with X500 forward monocular camera + IMU in walls world
- OpenVINS for visual-inertial odometry
- MAVROS for PX4 communication
- ros_gz_bridge for Gazebo topics
- Pipe odometry output from OpenVINS to MAVROS

## Features

- Support for forward-facing camera configurations
- OpenVINS MSCKF estimator for robust VIO
- Direct odometry output to MAVROS
- Pre-configured for PX4 1.16 compatibility
- Launch-file-only package for easy deployment
- Future: Automated OpenVINS installation and building from source

## Usage

### Launch the complete indoor navigation system:

```bash
# Forward-facing camera (default)
ros2 launch openvins_px4_vio indoor_navigation_mono_cam_imu.launch.py

# Downward-facing camera
ros2 launch openvins_px4_vio indoor_navigation_mono_cam_imu.launch.py camera_direction:=down

# Custom world
ros2 launch openvins_px4_vio indoor_navigation_mono_cam_imu.launch.py world:=warehouse
```

### Available Launch Arguments

- `camera_direction`: Camera orientation (`forward` or `down`, default: `forward`)
- `world`: Gazebo world to load (default: `walls`)
- `use_sim_time`: Use simulation time (default: `true`)

## Configuration

The OpenVINS configuration is located in `config/openvins_x500_mono.yaml`. This file contains:
- Camera intrinsics and extrinsics
- IMU noise parameters
- MSCKF estimator settings
- Feature tracking parameters

## Dependencies

- `mavros` and `mavros_msgs`
- `ros_gz_bridge`
- `tf2_ros`
- `ov_msckf` (OpenVINS) - see [Prerequisites](#prerequisites) below.

## Installation

### Prerequisites
Make sure OpenVINS (`ov_msckf`) is installed in your ROS2 workspace. If not, you can install it manually:

```bash
# Install OpenVINS dependencies
export ROS2_DISTRO=humble # dashing=18.04, galactic=20.04, humble=22.04
sudo apt-get install ros-$ROS2_DISTRO-ros2bag ros-$ROS2_DISTRO-rosbag2* libeigen3-dev libboost-all-dev libceres-dev

# Build OpenVINS
mkdir -p ~/workspace/ws_ov/src/
cd ~/workspace/ws_ov/src/
git clone https://github.com/rpng/open_vins/
cd ..
colcon build # ROS2
```

For further information there are [OpenVINS nstallation instructions for ROS2](https://docs.openvins.com/gs-installing.html#gs-install-ros-2) available.

## Package Structure

```
openvins_px4_vio/
├── config/
│   └── openvins_x500_mono.yaml    # OpenVINS configuration
├── launch/
│   └── indoor_navigation_mono_cam_imu.launch.py  # Main launch file
├── CMakeLists.txt                  # CMake build configuration
├── package.xml                     # Package dependencies and metadata
└── README.md                       # This file
```

## Notes

- Ensure PX4-Autopilot is built and available in `~/PX4-Autopilot`
- The system outputs odometry directly to `/mavros/odometry/out` for EKF fusion
- OpenVINS odometry output (`/ov_msckf/odomimu`) is directly mapped to MAVROS
- This is a launch-file-only package for easy deployment and modularity

## Future Enhancements

- Automated OpenVINS installation and building from source
- Multiple camera configurations support
- Parameter tuning utilities
- Calibration file management
