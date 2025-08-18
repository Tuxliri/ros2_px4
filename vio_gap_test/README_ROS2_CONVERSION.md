# ROS2 Conversion of MAVROS Position Control Test

## Overview

The `mavros_pos_ctl.py` script has been successfully converted from ROS1 to ROS2 Humble. This script tests flying a path in offboard control by sending position setpoints via MAVROS.

## Key Changes Made

### 1. Shebang and Python Version
- Changed from `#!/usr/bin/env python2` to `#!/usr/bin/env python3`
- Removed `from __future__ import division` (Python 3 default)

### 2. Core ROS Imports
- **Old**: `import rospy`
- **New**: `import rclpy` and `from rclpy.node import Node`

### 3. Node Structure
- **Old**: Used `unittest.TestCase` with `rospy` functions
- **New**: Inherits from both `Node` and `unittest.TestCase`
- **New**: Added proper ROS2 node initialization and lifecycle management

### 4. Publishers and Subscribers
- **Old**: `rospy.Publisher()` and `rospy.Subscriber()`
- **New**: `self.create_publisher()` and `self.create_subscription()`
- **New**: Added QoS profile configuration for MAVROS compatibility

### 5. Service Clients
- **Old**: `rospy.ServiceProxy()`
- **New**: `self.create_client()` with async call patterns
- **New**: Added proper service availability waiting

### 6. Time and Rate Handling
- **Old**: `rospy.Time.now()`, `rospy.Rate()`
- **New**: `self.get_clock().now().to_msg()`, `self.create_rate()`

### 7. Logging
- **Old**: `rospy.loginfo()`, `rospy.logdebug()`, `rospy.logerror()`
- **New**: `self.get_logger().info()`, `self.get_logger().debug()`, `self.get_logger().error()`

### 8. Exception Handling
- **Old**: `rospy.ROSException`, `rospy.ROSInterruptException`
- **New**: Generic `Exception` handling with proper error propagation

### 9. Loop and Iteration
- **Old**: `xrange()` (Python 2)
- **New**: `range()` (Python 3)

### 10. Transform Utilities
- **Old**: `from tf.transformations import quaternion_from_euler`
- **New**: Try/except import with manual fallback implementation for quaternion conversion

### 11. Test Execution
- **Old**: `rostest.rosrun()`
- **New**: Direct test execution with `rclpy.init()` and proper node lifecycle

## Dependencies

Ensure the following are installed:

```bash
# ROS2 packages
sudo apt install ros-humble-mavros ros-humble-mavros-msgs ros-humble-tf2-geometry-msgs

# Python packages
pip3 install pymavlink numpy
```

## Usage

### Direct Execution
```bash
cd /home/developer/ros2_ws
source install/setup.bash
python3 src/vio_gap_test/scripts/mavros_pos_ctl.py
```

### As ROS2 Node
The script can also be launched as a standard ROS2 node:

```bash
cd /home/developer/ros2_ws
source install/setup.bash
ros2 run vio_gap_test mavros_pos_ctl.py
```

## Test Functionality

The converted script performs the following test sequence:

1. **Setup Phase**:
   - Waits for MAVROS topics to become available
   - Waits for the vehicle to be in a landed state
   - Sets RC failsafe exception parameter

2. **Flight Phase**:
   - Sets OFFBOARD mode
   - Arms the vehicle
   - Flies through a series of waypoints:
     - (0, 0, 0)
     - (50, 50, 20)
     - (50, -50, 20)
     - (-50, -50, 20)
     - (0, 0, 20)

3. **Landing Phase**:
   - Sets AUTO.LAND mode
   - Waits for landing completion
   - Disarms the vehicle

## Configuration

### Position Tolerance
The script uses a 1-meter radius tolerance for position reaching:
```python
self.radius = 1  # meters
```

### Timeouts
- Topic availability: 60 seconds
- Position reaching: 30 seconds per waypoint
- Landing: 45 seconds
- Service calls: 5 seconds

## Troubleshooting

### Common Issues

1. **MAVROS not running**: Ensure MAVROS is launched and connected to PX4
2. **Service timeouts**: Check that all MAVROS services are available
3. **Position not reached**: Verify the vehicle is in OFFBOARD mode and properly armed

### Debug Information

The script provides comprehensive logging at different levels:
- **INFO**: Major state changes and progress updates
- **DEBUG**: Detailed position information
- **ERROR**: Failures and timeout conditions

## Compatibility Notes

- **ROS2 Humble**: Primary target version
- **Python 3.8+**: Required for f-string formatting and other features
- **MAVROS 2.x**: Compatible with ROS2 MAVROS packages
- **PX4**: Compatible with PX4 autopilot systems

## Future Improvements

1. Add proper unit tests using `pytest`
2. Implement async/await patterns for better concurrency
3. Add parameter server integration for configurable waypoints
4. Add visualization support with RViz2
5. Implement proper action server pattern for long-running missions
