#!/usr/bin/env python3

# Copyright (c) 2023, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# ROS2 module imports
import rclpy # ROS2 client library (rcl) for Python (built on rcl C API)
from rclpy.node import Node # Node class for Python nodes
from geometry_msgs.msg import Twist # Twist (linear and angular velocities) message class
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Ouality of Service (tune communication between nodes)
from rclpy.duration import Duration # Time duration class

# Python mudule imports
from math import pi # Common mathematical constant

# Node class
class RobotController(Node):

    #######################
    '''Class constructor'''
    #######################

    def __init__(self):
        # Information and debugging
        info = '\nMake the robot go in a square with open-loop control.\n'
        print(info)
        # ROS2 infrastructure
        super().__init__('robot_controller') # Create a node with name 'robot_controller'
        qos_profile = QoSProfile( # Ouality of Service profile
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, # Reliable (not best effort) communication
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, # Keep/store only up to last N samples
        depth=10 # Queue size/depth of 10 (only honored if the “history” policy was set to “keep last”)
        )
        self.robot_ctrl_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile) # Publisher which will publish Twist message to the topic '/cmd_vel' adhering to 'qos_profile' QoS profile
        timer_period = 0.001 # Node execution time period (seconds)
        self.timer = self.create_timer(timer_period, self.robot_controller_callback) # Define timer to execute 'robot_controller_callback()' every 'timer_period' seconds
        self.ctrl_msg = Twist() # Robot control commands (twist)
        self.declare_parameter('lin_vel') # Linear velocity of robot along local x-axis
        self.declare_parameter('ang_vel') # Angular velocity of robot about local z-axis
        self.start_time = self.get_clock().now() # Record current time in seconds

    #######################
    '''Callback function'''
    #######################

    def robot_controller_callback(self):
        LIN_VEL = self.get_parameter('lin_vel').value # Linear velocity parameter (m/s)
        ANG_VEL = self.get_parameter('ang_vel').value # Angular velocity parameter (rad/s)
        LIN_DIS = 2.0 # Square side length (m)
        ANG_DIS = pi/2 # Square side length (m)
        LIN_TIM = LIN_DIS/LIN_VEL # Base time duration for linear motion (s)
        ANG_TIM = ANG_DIS/ANG_VEL # Base time duration for angular motion (s)
        DELAY = 4.0 # Time delay (s)
        if self.get_clock().now() - self.start_time > Duration(seconds=DELAY):
            # Robot going to 1st vertex by moving LIN_DIS m with LIN_VEL m/s velocity
            if self.get_clock().now() - self.start_time < Duration(seconds=DELAY+LIN_TIM):
                print('Robot going to 1st vertex by moving {} m with {} m/s velocity'.format(LIN_DIS, LIN_VEL))
                self.ctrl_msg.linear.x = LIN_VEL
                self.ctrl_msg.angular.z = 0.0
            # Robot preparing for 2nd vertex by rotating 90 degrees with ANG_VEL rad/s velocity
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+LIN_TIM) and self.get_clock().now() - self.start_time <= Duration(seconds=DELAY+LIN_TIM+ANG_TIM):
                print('Robot preparing for 2nd vertex by rotating 90 degrees with {} rad/s velocity'.format(ANG_VEL))
                self.ctrl_msg.linear.x = 0.0
                self.ctrl_msg.angular.z = ANG_VEL
            # Robot going to 2nd vertex by moving LIN_DIS m with LIN_VEL m/s velocity
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+LIN_TIM+ANG_TIM) and self.get_clock().now() - self.start_time <= Duration(seconds=DELAY+2*LIN_TIM+ANG_TIM):
                print('Robot going to 2nd vertex by moving {} m with {} m/s velocity'.format(LIN_DIS, LIN_VEL))
                self.ctrl_msg.linear.x = LIN_VEL
                self.ctrl_msg.angular.z = 0.0
            # Robot preparing for 3rd vertex by rotating 90 degrees with ANG_VEL rad/s velocity
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+2*LIN_TIM+ANG_TIM) and self.get_clock().now() - self.start_time <= Duration(seconds=DELAY+2*LIN_TIM+2*ANG_TIM):
                print('Robot preparing for 3rd vertex by rotating 90 degrees with {} rad/s velocity'.format(ANG_VEL))
                self.ctrl_msg.linear.x = 0.0
                self.ctrl_msg.angular.z = ANG_VEL
            # Robot going to 3rd vertex by moving LIN_DIS m with LIN_VEL m/s velocity
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+2*LIN_TIM+2*ANG_TIM) and self.get_clock().now() - self.start_time <= Duration(seconds=DELAY+3*LIN_TIM+2*ANG_TIM):
                print('Robot going to 3rd vertex by moving {} m with {} m/s velocity'.format(LIN_DIS, LIN_VEL))
                self.ctrl_msg.linear.x = LIN_VEL
                self.ctrl_msg.angular.z = 0.0
            # Robot preparing for 4th vertex by rotating 90 degrees with ANG_VEL rad/s velocity
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+3*LIN_TIM+2*ANG_TIM) and self.get_clock().now() - self.start_time <= Duration(seconds=DELAY+3*LIN_TIM+3*ANG_TIM):
                print('Robot preparing for 4th vertex by rotating 90 degrees with {} rad/s velocity'.format(ANG_VEL))
                self.ctrl_msg.linear.x = 0.0
                self.ctrl_msg.angular.z = ANG_VEL
            # Robot going to 4th vertex by moving LIN_DIS m with LIN_VEL m/s velocity
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+3*LIN_TIM+3*ANG_TIM) and self.get_clock().now() - self.start_time <= Duration(seconds=DELAY+4*LIN_TIM+3*ANG_TIM):
                print('Robot going to 4th vertex by moving {} m with {} m/s velocity'.format(LIN_DIS, LIN_VEL))
                self.ctrl_msg.linear.x = LIN_VEL
                self.ctrl_msg.angular.z = 0.0
            # Robot resetting to initial configuration by rotating 90 degrees with ANG_VEL rad/s velocity
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+4*LIN_TIM+3*ANG_TIM) and self.get_clock().now() - self.start_time <= Duration(seconds=DELAY+4*LIN_TIM+4*ANG_TIM):
                print('Robot resetting to initial configuration by rotating 90 degrees with {} rad/s velocity'.format(ANG_VEL))
                self.ctrl_msg.linear.x = 0.0
                self.ctrl_msg.angular.z = ANG_VEL
            # Stop robot and quit controller execution
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+4*LIN_TIM+4*ANG_TIM):
                self.ctrl_msg.linear.x = 0.0
                self.ctrl_msg.angular.z = 0.0
                self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message
                quit() # Quit execution
            self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message
        else:
            print('Initializing simulation...')

def main(args=None):
    rclpy.init(args=args) # Start ROS2 communications
    node = RobotController() # Create node
    rclpy.spin(node) # Execute node
    node.destroy_node() # Destroy node explicitly (optional - otherwise it will be done automatically when garbage collector destroys the node object)
    rclpy.shutdown() # Shutdown ROS2 communications

if __name__ == "__main__":
    main()
