"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
 * Converted to ROS2 Humble
 * Modified to fly figure-8 trajectory in XY plane
"""

#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import time
import math


class OffboardControl(Node):

    def __init__(self):
        super().__init__('offb_node_py')
        
        self.current_state = State()

        # QoS profile for MAVROS compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_cb,
            qos_profile
        )

        # Publishers
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            'mavros/setpoint_position/local',
            10
        )

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services to be available
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting...')
        
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set mode service not available, waiting...')

    def state_cb(self, msg):
        self.current_state = msg

    def run_offboard_control(self):
        """Main offboard control loop with figure-8 trajectory"""
        
        # Setpoint publishing MUST be faster than 2Hz
        rate = self.create_rate(50)  # 50 Hz

        # Wait for Flight Controller connection
        while rclpy.ok() and not self.current_state.connected:
            self.get_logger().info('Waiting for FCU connection...')
            rclpy.spin_once(self, timeout_sec=0.1)

        # Initial hover position
        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 8.0

        # Send a few setpoints before starting
        self.get_logger().info('Sending initial setpoints...')
        for i in range(100):
            if not rclpy.ok():
                break
            self.local_pos_pub.publish(pose)
            rclpy.spin_once(self, timeout_sec=0.01)

        # Prepare service requests
        offb_set_mode = SetMode.Request()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBool.Request()
        arm_cmd.value = True

        last_req = self.get_clock().now()

        self.get_logger().info('Starting offboard control loop...')

        # Figure-8 trajectory parameters
        trajectory_radius = 5.0  # Radius of the figure-8 in meters
        trajectory_period = 20.0  # Time to complete one figure-8 in seconds
        start_time = self.get_clock().now()
        
        # Wait period before starting trajectory (hover for 5 seconds after arming)
        hover_duration = 5.0  # seconds

        while rclpy.ok():
            current_time = self.get_clock().now()
            
            if (self.current_state.mode != "OFFBOARD" and 
                (current_time - last_req).nanoseconds > 5.0 * 1e9):  # 5 seconds in nanoseconds
                
                future = self.set_mode_client.call_async(offb_set_mode)
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                
                result = future.result()
                if result is not None and hasattr(result, 'mode_sent') and result.mode_sent:
                    self.get_logger().info("OFFBOARD enabled")
                last_req = current_time
                
            elif (not self.current_state.armed and 
                  (current_time - last_req).nanoseconds > 5.0 * 1e9):  # 5 seconds in nanoseconds
                
                future = self.arming_client.call_async(arm_cmd)
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                
                result = future.result()
                if result is not None and hasattr(result, 'success') and result.success:
                    self.get_logger().info("Vehicle armed")
                    # Reset start time when armed
                    start_time = self.get_clock().now()
                last_req = current_time

            # Calculate time since armed/start
            elapsed_time = (current_time - start_time).nanoseconds / 1e9  # Convert to seconds
            
            if self.current_state.armed and elapsed_time > hover_duration:
                # Start figure-8 trajectory after hover period
                trajectory_time = elapsed_time - hover_duration
                
                # Parametric equations for figure-8 (lemniscate)
                # x(t) = a * sin(2πt/T)
                # y(t) = a * sin(4πt/T) / 2
                omega = 2.0 * math.pi / trajectory_period
                
                pose.pose.position.x = trajectory_radius * math.sin(omega * trajectory_time)
                pose.pose.position.y = trajectory_radius * math.sin(2.0 * omega * trajectory_time) / 2.0
                pose.pose.position.z = 8.0  # Maintain constant altitude
                
                # Log progress every 2 seconds
                if int(elapsed_time) % 2 == 0 and int(elapsed_time * 10) % 20 == 0:
                    self.get_logger().info(f'Figure-8 trajectory: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}')
            
            else:
                # Hover at initial position
                pose.pose.position.x = 0.0
                pose.pose.position.y = 0.0
                pose.pose.position.z = 8.0

            self.local_pos_pub.publish(pose)
            rclpy.spin_once(self, timeout_sec=0.01)


def main(args=None):
    rclpy.init(args=args)
    
    offboard_control = OffboardControl()
    
    try:
        offboard_control.run_offboard_control()
    except KeyboardInterrupt:
        pass
    
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()