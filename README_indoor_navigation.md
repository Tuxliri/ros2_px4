# Indoor Navigation with OpenVINS and PX4

This example demonstrates indoor navigation using OpenVINS visual-inertial odometry with PX4 SITL simulation.

## Overview

The setup includes:
- **PX4 SITL** with X500 quadcopter + monocular forward-facing camera + IMU
- **Gazebo Harmonic** with walls world for indoor environment  
- **OpenVINS** for visual-inertial odometry (VIO)
- **MAVROS** for PX4 communication
- **Vision-to-MAVROS bridge** for coordinate frame conversion

## Quick Start

### 1. Build OpenVINS (if not already built)
```bash
cd ~/workspace/catkin_ws_ov
source /opt/ros/humble/setup.bash
colcon build --event-handlers console_cohesion+ --packages-select ov_core ov_init ov_msckf ov_eval
```

### 2. Launch the complete indoor navigation system
```bash
# Terminal 1: Launch everything  
ros2 launch your_package indoor_navigation_openvins.launch.py

# Or launch with specific world
ros2 launch your_package indoor_navigation_openvins.launch.py world:=walls
```

### 3. Verify the system is working
```bash
# Check OpenVINS is publishing pose estimates
ros2 topic echo /ov_msckf/poseimu

# Check MAVROS is receiving vision pose
ros2 topic echo /mavros/vision_pose/pose

# Check transform tree
ros2 run tf2_tools view_frames
```

## Architecture

```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│   Gazebo    │───▶│ ros_gz_bridge│───▶│  OpenVINS   │
│   Camera    │    │              │    │    VIO      │
│     +       │    │   /image     │    │             │
│    IMU      │    │   /imu       │    │ /poseimu    │
└─────────────┘    └──────────────┘    └─────────────┘
                                              │
                                              ▼
┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│    PX4      │◀───│    MAVROS    │◀───│vision_to_   │
│    SITL     │    │              │    │mavros_node  │
│             │    │/vision_pose/ │    │             │
│             │    │    pose      │    │ENU→NED conv │
└─────────────┘    └──────────────┘    └─────────────┘
```

## Available Worlds

You can test different indoor environments:

### Walls World (Default for Indoor Nav)
```bash
ros2 launch your_package indoor_navigation_openvins.launch.py world:=walls
```
- Contains walls and obstacles for testing collision avoidance
- Good for testing VIO in structured environments

### Aruco World  
```bash
ros2 launch your_package indoor_navigation_openvins.launch.py world:=aruco
```
- Contains ArUco markers for precision landing tests
- Can be used alongside OpenVINS for hybrid navigation

## Configuration Files

### OpenVINS Configuration
- `config/openvins_x500_mono.yaml` - Basic configuration
- `config/openvins_x500_enhanced.yaml` - Enhanced configuration with better tuning

Key parameters to tune:
- **Camera intrinsics**: Adjust `cam_intrinsics` for your camera
- **Camera-IMU extrinsics**: Modify `T_CtoI` for camera positioning
- **IMU noise**: Tune `sigma_w`, `sigma_a` for your IMU characteristics
- **Feature tracking**: Adjust `num_pts`, `grid_x/y` for performance

### Vision-to-MAVROS Bridge
The bridge converts OpenVINS output (ENU frame) to PX4's expected NED frame:
- `gamma_world: -1.5707963` converts ENU → NED  
- Frame transformations ensure proper coordinate alignment

## Troubleshooting

### No OpenVINS Output
```bash
# Check camera/IMU topics are being published
ros2 topic list | grep -E "(image|imu)"

# Verify OpenVINS node is running
ros2 node list | grep openvins

# Check OpenVINS logs for initialization
ros2 log get openvins_msckf
```

### MAVROS Not Receiving Vision Pose
```bash
# Check vision_to_mavros node
ros2 node list | grep vision_to_mavros

# Verify coordinate transformations
ros2 run tf2_ros tf2_echo odom base_link
```

### Poor VIO Performance
1. **Increase feature count**: Set `num_pts: 400` in config
2. **Improve lighting**: Use well-lit worlds/environments  
3. **Tune IMU noise**: Reduce `sigma_w` and `sigma_a`
4. **Check camera calibration**: Verify `cam_intrinsics` are correct

## Advanced Usage

### Custom Camera Configuration
To use your own camera parameters, modify `openvins_x500_enhanced.yaml`:

```yaml
cam0:
  T_CtoI: [[...]]  # Camera-to-IMU transform (forward-facing)
  cam_intrinsics: [fx, fy, cx, cy]  # Your camera parameters
  cam_wh: [width, height]  # Your camera resolution
```

### Real Hardware Deployment
For real hardware, update:
1. Camera/IMU topic remappings in launch file
2. IMU noise parameters based on sensor specs
3. Camera intrinsics from calibration
4. Remove simulation-specific settings

## Performance Tips

- **Feature tracking**: Balance `num_pts` vs computational load
- **Update rates**: Match `track_frequency` to camera framerate  
- **Grid distribution**: Use appropriate `grid_x/y` for image resolution
- **Multi-threading**: Enable `multi_threading: true` for better performance
