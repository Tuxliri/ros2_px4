# VIO Gap Test Package

This package provides tools to investigate VIO position failure hypothesis: **"Gap in AprilTag detections causes the estimator to diverge"**.

## Overview

The package simulates VIO data gaps using Gazebo ground truth and measures estimator performance to test whether controlled gaps cause divergence.

## Architecture

1. **Gazebo Simulation**: Provides ground truth pose via `/world/default/pose/info` topic
2. **ros_gz_bridge**: Bridges Gazebo pose data to ROS2 PoseArray messages  
3. **VIO Gap Simulator**: Uses PoseArray to get ground truth, creates controlled gaps, publishes to MAVROS
4. **Estimator Analyzer**: Monitors estimator performance and detects divergence events

## Components

### Core Nodes

- **vio_gap_simulator**: Creates controlled VIO data gaps using PoseArray ground truth
- **analyze_estimator.py**: Real-time performance analysis with CSV logging

### Launch Files

- **vio_gap_test_with_bridge.launch.py**: Complete test system with PoseArray bridge

## Usage

### 1. Start PX4 SITL with Gazebo

```bash
# In PX4-Autopilot directory
make px4_sitl gz_x500_mono_cam_down_0
```

### 2. Start MAVROS 

```bash
# In another terminal
source /home/developer/ros2_ws/install/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

### 3. Run VIO Gap Test

```bash
# In another terminal
source /home/developer/ros2_ws/install/setup.bash
ros2 launch vio_gap_test vio_gap_test_with_bridge.launch.py gap_duration_ms:=2000 gap_interval_ms:=10000
```

### 4. Monitor Results

Watch for:
- VIO gap events in the logs
- Position divergence detection
- CSV output in `/tmp/vio_gap_analysis_TIMESTAMP.csv`

## Configuration Parameters

### VIO Gap Simulator

- `gap_duration_ms`: Duration of each VIO gap (default: 1000ms)
- `gap_interval_ms`: Time between gaps (default: 5000ms)  
- `pose_index`: Index in PoseArray to use (default: 11 = base_link entity)
- `pose_topic`: PoseArray topic name (default: "/gz/pose_info")
- `frame_id`: Odometry frame ID (default: "map")
- `child_frame_id`: Child frame ID (default: "base_link")

### Usage Examples

```bash
# Test with longer gaps
ros2 launch vio_gap_test vio_gap_test_with_bridge.launch.py gap_duration_ms:=3000

# Test with more frequent gaps  
ros2 launch vio_gap_test vio_gap_test_with_bridge.launch.py gap_interval_ms:=3000

# Test with different pose index (different entity)
ros2 launch vio_gap_test vio_gap_test_with_bridge.launch.py pose_index:=5
```

## Expected Behavior

1. **Normal Operation**: VIO data flows continuously to MAVROS
2. **Gap Events**: Periodic interruptions in VIO data
3. **Recovery**: VIO resumes after gap duration
4. **Analysis**: Real-time monitoring of estimator divergence

## Data Collection

The analyzer outputs CSV files with:
- Timestamp
- Ground truth position (x, y, z)
- Estimated position (x, y, z) 
- Position errors
- Gap status
- MAVROS connection status

## Troubleshooting

### No PoseArray Data

If you see "No ground truth data available from PoseArray" errors:

1. Check that Gazebo is running with the correct model
2. Verify ros_gz_bridge is running properly:
   ```bash
   ros2 topic list | grep pose
   ros2 topic echo /gz/pose_info
   ```
3. Check if pose_index matches available entities in the PoseArray

### MAVROS Connection Issues

If MAVROS shows "disconnected":

1. Verify PX4 SITL is running
2. Check MAVROS launch with correct fcu_url
3. Monitor MAVROS topics:
   ```bash
   ros2 topic echo /mavros/state
   ```

### No VIO Data

If no odometry is published:

1. Check if ground truth is available via PoseArray
2. Verify gap timing parameters
3. Monitor simulator logs for error messages

## Files Overview

```
vio_gap_test/
├── src/
│   └── vio_gap_simulator.cpp          # Main VIO gap simulator (PoseArray input)
├── scripts/
│   └── analyze_estimator.py           # Performance analyzer
├── launch/
│   ├── vio_gap_test_with_bridge.launch.py  # PoseArray-based test system
│   └── vio_gap_test_tf2.launch.py     # Legacy tf2-based system
├── package.xml                        # ROS2 package metadata
├── CMakeLists.txt                     # Build configuration
└── README.md                          # This file
```

## Development Notes

This package has been updated to use PoseArray input instead of tf2 transforms for simpler integration with Gazebo pose data. The PoseArray approach allows direct access to specific entity poses by index, making it easier to target the base_link entity (typically index 11) from Gazebo's pose information.
