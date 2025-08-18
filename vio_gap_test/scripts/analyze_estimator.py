#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time
import csv
import os

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

class EstimatorAnalyzer(Node):
    def __init__(self):
        super().__init__('estimator_analyzer')
        
        # Data storage
        self.ground_truth_data = deque(maxlen=2000)
        self.estimated_data = deque(maxlen=2000)
        self.timestamps = deque(maxlen=2000)
        self.position_errors = deque(maxlen=2000)
        
        # Analysis parameters
        self.declare_parameter('log_file', '/tmp/vio_analysis.csv')
        self.declare_parameter('error_threshold', 1.0)  # meters
        
        self.log_file = self.get_parameter('log_file').get_parameter_value().string_value
        self.error_threshold = self.get_parameter('error_threshold').get_parameter_value().double_value
        
        # Subscribers - using Gazebo ground truth and MAVROS estimated position
        self.ground_truth_sub = self.create_subscription(
            PoseStamped, '/gz/base_link/pose', self.ground_truth_callback, 10)
        self.est_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', 
            self.estimated_position_callback, 10)
        self.mavros_state_sub = self.create_subscription(
            State, '/mavros/state', self.mavros_state_callback, 10)
        
        # Timer for analysis and logging
        self.analysis_timer = self.create_timer(1.0, self.analyze_performance)
        
        # Initialize CSV logging
        self.init_csv_logging()
        
        # Divergence tracking
        self.divergence_count = 0
        self.max_error = 0.0
        self.start_time = time.time()
        self.mavros_connected = False
        
        self.get_logger().info("Estimator analyzer started")
        self.get_logger().info(f"Logging to: {self.log_file}")
        self.get_logger().info(f"Error threshold: {self.error_threshold}m")
        self.get_logger().info("Monitoring topics:")
        self.get_logger().info("  - Ground truth: /gz/base_link/pose")
        self.get_logger().info("  - Estimated position: /mavros/local_position/pose")
        self.get_logger().info("  - MAVROS state: /mavros/state")
    
    def init_csv_logging(self):
        """Initialize CSV file for logging analysis data"""
        try:
            with open(self.log_file, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'timestamp', 'gt_x', 'gt_y', 'gt_z', 
                    'est_x', 'est_y', 'est_z', 
                    'error_x', 'error_y', 'error_z', 'total_error',
                    'diverged', 'mavros_connected', 'mavros_armed'
                ])
            self.get_logger().info(f"CSV logging initialized: {self.log_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CSV logging: {e}")
    
    def ground_truth_callback(self, msg):
        """Extract ground truth pose from Gazebo base_link"""
        current_time = time.time()
        
        # Use ENU coordinates from Gazebo
        gt_pos = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        
        self.ground_truth_data.append(gt_pos)
        self.timestamps.append(current_time)
    
    def estimated_position_callback(self, msg):
        """Store estimated position from MAVROS"""
        # MAVROS local_position/pose is in ENU frame
        est_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.estimated_data.append(est_pos)
    
    def mavros_state_callback(self, msg):
        """Monitor MAVROS connection state"""
        if msg.connected != self.mavros_connected:
            if msg.connected:
                self.get_logger().info("MAVROS connected to FCU")
            else:
                self.get_logger().warn("MAVROS disconnected from FCU")
            self.mavros_connected = msg.connected
        
        self.mavros_state = msg
    
    def analyze_performance(self):
        """Analyze estimator performance and detect divergence"""
        if len(self.ground_truth_data) < 10 or len(self.estimated_data) < 10:
            return
        
        # Ensure we have matching data points
        min_len = min(len(self.ground_truth_data), len(self.estimated_data))
        
        if min_len < 2:
            return
        
        # Get the latest data points
        gt_array = np.array(list(self.ground_truth_data)[-min_len:])
        est_array = np.array(list(self.estimated_data)[-min_len:])
        timestamps = np.array(list(self.timestamps)[-min_len:])
        
        # Calculate position errors
        error_vector = gt_array - est_array
        position_error = np.linalg.norm(error_vector, axis=1)
        
        # Current error metrics
        current_error = position_error[-1]
        mean_error = np.mean(position_error)
        max_error_window = np.max(position_error[-10:])  # Max error in last 10 samples
        
        # Update max error
        if current_error > self.max_error:
            self.max_error = current_error
        
        # Store current error
        self.position_errors.append(current_error)
        
        # Check for divergence
        diverged = current_error > self.error_threshold
        if diverged:
            self.divergence_count += 1
            self.get_logger().warn(f"ESTIMATOR DIVERGENCE DETECTED! Error: {current_error:.3f}m")
        
        # Log to CSV
        self.log_to_csv(
            timestamps[-1], gt_array[-1], est_array[-1], 
            error_vector[-1], current_error, diverged
        )
        
        # Console output
        runtime = time.time() - self.start_time
        connection_status = "connected" if self.mavros_connected else "DISCONNECTED"
        armed_status = "armed" if hasattr(self, 'mavros_state') and self.mavros_state.armed else "disarmed"
        
        self.get_logger().info(
            f"Runtime: {runtime:.1f}s | "
            f"Error: {current_error:.3f}m | "
            f"Mean: {mean_error:.3f}m | "
            f"Max: {self.max_error:.3f}m | "
            f"Divergences: {self.divergence_count} | "
            f"MAVROS: {connection_status}/{armed_status}"
        )
        
        # Check for sustained divergence
        if len(self.position_errors) >= 5:
            recent_errors = list(self.position_errors)[-5:]
            if all(e > self.error_threshold for e in recent_errors):
                self.get_logger().error(
                    f"SUSTAINED DIVERGENCE DETECTED! "
                    f"Error > {self.error_threshold}m for last 5 seconds"
                )
    
    def log_to_csv(self, timestamp, gt_pos, est_pos, error_vec, total_error, diverged):
        """Log current analysis data to CSV file"""
        try:
            mavros_connected = self.mavros_connected
            mavros_armed = hasattr(self, 'mavros_state') and self.mavros_state.armed
            
            with open(self.log_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    timestamp, 
                    gt_pos[0], gt_pos[1], gt_pos[2],
                    est_pos[0], est_pos[1], est_pos[2],
                    error_vec[0], error_vec[1], error_vec[2], total_error,
                    1 if diverged else 0,
                    1 if mavros_connected else 0,
                    1 if mavros_armed else 0
                ])
        except Exception as e:
            self.get_logger().error(f"Failed to log to CSV: {e}")
    
    def generate_summary_report(self):
        """Generate a summary report of the analysis"""
        runtime = time.time() - self.start_time
        
        if len(self.position_errors) > 0:
            errors = np.array(list(self.position_errors))
            mean_error = np.mean(errors)
            std_error = np.std(errors)
            max_error = np.max(errors)
            divergence_rate = self.divergence_count / len(errors) * 100
            
            report = f"""
VIO Gap Test Analysis Summary (MAVROS)
======================================
Runtime: {runtime:.1f} seconds
Samples analyzed: {len(errors)}
Mean position error: {mean_error:.3f} ± {std_error:.3f} m
Maximum error: {max_error:.3f} m
Divergence threshold: {self.error_threshold} m
Divergence events: {self.divergence_count}
Divergence rate: {divergence_rate:.1f}%
MAVROS connection: {"✓" if self.mavros_connected else "✗"}
Data logged to: {self.log_file}

Topics monitored:
- Ground truth: /gz/base_link/pose (Gazebo base_link)
- Estimated pose: /mavros/local_position/pose
- MAVROS state: /mavros/state
- VIO input: /mavros/odometry/out
"""
            self.get_logger().info(report)
            return report
        
        return "No data collected for analysis"

def main():
    rclpy.init()
    analyzer = EstimatorAnalyzer()
    
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        # Generate summary on shutdown
        summary = analyzer.generate_summary_report()
        print(summary)
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
