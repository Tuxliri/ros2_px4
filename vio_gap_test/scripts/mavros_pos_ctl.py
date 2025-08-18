#!/usr/bin/env python3
#***************************************************************************
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/

#
# @author Andreas Antener <andreas@uaventure.com>
#
# Converted to ROS2 Humble

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import numpy as np
import unittest
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import ParamValue, Altitude, ExtendedState, HomePosition, State, WaypointList
from mavros_msgs.srv import CommandBool, ParamSet, SetMode, WaypointClear, WaypointPush
from rcl_interfaces.srv import GetParameters
from pymavlink import mavutil
from std_msgs.msg import Header
from threading import Thread

try:
    from tf_transformations import quaternion_from_euler
except ImportError:
    # Fallback for ROS2 humble - use scipy or manual implementation
    def quaternion_from_euler(roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0.0, 0.0, 0.0, 0.0]
        q[0] = sr * cp * cy - cr * sp * sy  # x
        q[1] = cr * sp * cy + sr * cp * sy  # y
        q[2] = cr * cp * sy - sr * sp * cy  # z
        q[3] = cr * cp * cy + sr * sp * sy  # w
        return q
        
from sensor_msgs.msg import NavSatFix, Imu
import time



class MavrosTestCommon(Node, unittest.TestCase):
    """
    Base class for MAVROS integration tests using ROS2
    """
    
    def __init__(self, node_name='mavros_test_node'):
        Node.__init__(self, node_name)
        unittest.TestCase.__init__(self)
        
        # Initialize message containers
        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.imu_data = Imu()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.mission_wp = WaypointList()
        self.state = State()
        self.mav_type = None

        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos',
                'mission_wp', 'state', 'imu'
            ]
        }

        # QoS profile for MAVROS topics
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ROS service clients
        service_timeout = 30.0
        self.get_logger().info("waiting for ROS services")
        
        self.get_param_cli = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.set_param_cli = self.create_client(ParamSet, '/mavros/param/set')
        self.set_arming_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.wp_clear_cli = self.create_client(WaypointClear, '/mavros/mission/clear')
        self.wp_push_cli = self.create_client(WaypointPush, '/mavros/mission/push')

        # Wait for services
        services = [
            self.get_param_cli, self.set_param_cli, self.set_arming_cli,
            self.set_mode_cli, self.wp_clear_cli, self.wp_push_cli
        ]
        
        for service in services:
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {service.srv_name} not available, waiting...')

        self.get_logger().info("ROS services are up")

        # ROS subscribers
        self.alt_sub = self.create_subscription(
            Altitude, 'mavros/altitude', self.altitude_callback, self.qos_profile)
        self.ext_state_sub = self.create_subscription(
            ExtendedState, 'mavros/extended_state', self.extended_state_callback, self.qos_profile)
        self.global_pos_sub = self.create_subscription(
            NavSatFix, 'mavros/global_position/global', self.global_position_callback, self.qos_profile)
        self.imu_data_sub = self.create_subscription(
            Imu, 'mavros/imu/data', self.imu_data_callback, self.qos_profile)
        self.home_pos_sub = self.create_subscription(
            HomePosition, 'mavros/home_position/home', self.home_position_callback, self.qos_profile)
        self.local_pos_sub = self.create_subscription(
            PoseStamped, 'mavros/local_position/pose', self.local_position_callback, self.qos_profile)
        self.mission_wp_sub = self.create_subscription(
            WaypointList, 'mavros/mission/waypoints', self.mission_wp_callback, self.qos_profile)
        self.state_sub = self.create_subscription(
            State, 'mavros/state', self.state_callback, self.qos_profile)

    def tearDown(self):
        self.log_topic_vars()

    # Callback functions
    def altitude_callback(self, data):
        self.altitude = data
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            self.get_logger().info(f"VTOL state changed from {self.extended_state.vtol_state} to {data.vtol_state}")
        self.extended_state = data
        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True

    def global_position_callback(self, data):
        self.global_position = data
        if not self.sub_topics_ready['global_pos']:
            self.sub_topics_ready['global_pos'] = True

    def imu_data_callback(self, data):
        self.imu_data = data
        if not self.sub_topics_ready['imu']:
            self.sub_topics_ready['imu'] = True

    def home_position_callback(self, data):
        self.home_position = data
        if not self.sub_topics_ready['home_pos']:
            self.sub_topics_ready['home_pos'] = True

    def local_position_callback(self, data):
        self.local_position = data
        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def mission_wp_callback(self, data):
        if self.mission_wp.current_seq != data.current_seq:
            self.get_logger().info(f"current mission waypoint sequence updated: {data.current_seq}")
        self.mission_wp = data
        if not self.sub_topics_ready['mission_wp']:
            self.sub_topics_ready['mission_wp'] = True

    def state_callback(self, data):
        if self.state.armed != data.armed:
            self.get_logger().info(f"armed state changed from {self.state.armed} to {data.armed}")
        if self.state.connected != data.connected:
            self.get_logger().info(f"connected state changed from {self.state.connected} to {data.connected}")
        if self.state.mode != data.mode:
            self.get_logger().info(f"mode changed from {self.state.mode} to {data.mode}")
        self.state = data
        if not self.sub_topics_ready['state']:
            self.sub_topics_ready['state'] = True

    def set_arm(self, arm, timeout):
        """arm: True to arm, False to disarm, timeout(int): seconds"""
        self.get_logger().info(f"setting FCU arm: {arm}")
        req = CommandBool.Request()
        req.value = arm
        future = self.set_arming_cli.call_async(req)
        
        timeout_time = time.time() + timeout
        while time.time() < timeout_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                try:
                    response = future.result()
                    if hasattr(response, 'success') and response.success:
                        self.get_logger().info(f"arm set to {arm}")
                        return True
                    else:
                        result_msg = getattr(response, 'result', 'unknown error')
                        self.get_logger().error(f"failed to set arm: {result_msg}")
                        return False
                except Exception as e:
                    self.get_logger().error(f"service call failed: {e}")
                    return False
        
        self.get_logger().error(f"failed to set arm within {timeout} seconds")
        return False

    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        self.get_logger().info(f"setting FCU mode: {mode}")
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_cli.call_async(req)
        
        timeout_time = time.time() + timeout
        while time.time() < timeout_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                try:
                    response = future.result()
                    if hasattr(response, 'mode_sent') and response.mode_sent:
                        self.get_logger().info(f"mode set to {mode}")
                        return True
                    else:
                        self.get_logger().error(f"failed to set mode: {mode}")
                        return False
                except Exception as e:
                    self.get_logger().error(f"service call failed: {e}")
                    return False
        
        self.get_logger().error(f"failed to set mode within {timeout} seconds")
        return False

    def set_param(self, param_id, param_value, timeout):
        """param_id: string, param_value: ParamValue, timeout(int): seconds"""
        req = ParamSet.Request()
        req.param_id = param_id
        req.value = param_value
        future = self.set_param_cli.call_async(req)
        
        timeout_time = time.time() + timeout
        while time.time() < timeout_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                try:
                    response = future.result()
                    if hasattr(response, 'success') and response.success:
                        self.get_logger().info(f"param {param_id} set")
                        return True
                    else:
                        self.get_logger().error(f"failed to set param {param_id}")
                        return False
                except Exception as e:
                    self.get_logger().error(f"service call failed: {e}")
                    return False
        
        self.get_logger().error(f"failed to set param within {timeout} seconds")
        return False

    def wait_for_topics(self, timeout):
        """timeout(int): seconds"""
        self.get_logger().info("waiting for subscribed topics to be available")
        timeout_time = time.time() + timeout
        
        while time.time() < timeout_time:
            if all(self.sub_topics_ready.values()):
                self.get_logger().info("all topics ready")
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        
        missing_topics = [k for k, v in self.sub_topics_ready.items() if not v]
        self.get_logger().error(f"failed to receive topics: {missing_topics}")
        return False

    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        """timeout(int): seconds, index(int): next waypoint index"""
        self.get_logger().info(f"waiting for landed state | state: {desired_landed_state}, timeout(s): {timeout}")
        timeout_time = time.time() + timeout
        
        while time.time() < timeout_time:
            if self.extended_state.landed_state == desired_landed_state:
                self.get_logger().info(f"landed state reached | seconds: {time.time() - (timeout_time - timeout)}")
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().error(f"landed state not reached within {timeout} seconds")
        return False

    def log_topic_vars(self):
        """Log the state of topic variables"""
        self.get_logger().info("=== Topic Variables ===")
        self.get_logger().info(f"altitude: {self.altitude}")
        self.get_logger().info(f"extended_state: {self.extended_state}")
        self.get_logger().info(f"global_position: {self.global_position}")
        self.get_logger().info(f"local_position: {self.local_position}")
        self.get_logger().info(f"state: {self.state}")


class MavrosOffboardPosctlTest(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).__init__()

        self.pos = PoseStamped()
        self.radius = 1

        self.pos_setpoint_pub = self.create_publisher(
            PoseStamped, 'mavros/setpoint_position/local', 10)

        # send setpoints in separate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def tearDown(self):
        super(MavrosOffboardPosctlTest, self).tearDown()

    #
    # Helper methods
    #
    def send_pos(self):
        rate = self.create_rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while rclpy.ok():
            self.pos.header.stamp = self.get_clock().now().to_msg()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except Exception:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        self.get_logger().debug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        self.get_logger().info(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = self.create_rate(loop_freq)
        reached = False
        timeout_time = time.time() + timeout
        
        while time.time() < timeout_time:
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                elapsed = time.time() - (timeout_time - timeout)
                self.get_logger().info("position reached | seconds: {0:.2f} of {1}".format(elapsed, timeout))
                reached = True
                break

            try:
                rclpy.spin_once(self, timeout_sec=0.1)
                rate.sleep()
            except Exception as e:
                self.fail(str(e))

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))

    #
    # Test method
    #
    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        # exempting failsafe from lost RC to allow offboard
        rcl_except = ParamValue()
        rcl_except.integer = 1<<2
        rcl_except.real = 0.0
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        self.get_logger().info("run mission")
        positions = ((0, 0, 0), (50, 50, 20), (50, -50, 20), (-50, -50, 20),
                     (0, 0, 20))

        for i in range(len(positions)):
            self.reach_position(positions[i][0], positions[i][1],
                                positions[i][2], 30)

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)


def main():
    rclpy.init()
    
    # Create the test node
    test_node = MavrosOffboardPosctlTest('mavros_offboard_posctl_test')
    
    try:
        # Run the test
        test_node.test_posctl()
        test_node.get_logger().info("Test completed successfully")
    except Exception as e:
        test_node.get_logger().error(f"Test failed: {e}")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()