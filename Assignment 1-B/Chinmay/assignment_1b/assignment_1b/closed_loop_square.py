#!/usr/bin/env python

# Copyright (c) 2023, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
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

######################################################################

# Import modules
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration
from math import pow, atan2, sqrt, pi

######################################################################

# Node class
class Controller(Node):

    def __init__(self):
        super().__init__('controller') # Node
        qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, # Reliable communication
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, # Keep last N samples
        depth=10 # Queue depth (N)
        )
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.subscriber_callback, qos_profile) # Subscriber
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile) # Publisher
        self.message = Twist() # Message
        timer_period = 0.001 # Execution time
        self.timer = self.create_timer(timer_period, self.timer_callback) # Node timer
        self.pose = Pose() # Turtle pose
        self.wp_pose = Pose() # Current waypoint pose
        self.data_available = False # Flag to check if feedback data is available
        self.wp_reached = False # Flag to check if current waypoint is reached
        self.wp_idx = 0 # Current waypoint index
        self.declare_parameter('wp_pos_x') # Position of all waypoints along x-axis
        self.declare_parameter('wp_pos_y') # Position of all waypoints along y-axis
        self.declare_parameter('wp_rot_z') # Orientation of all waypoints about z-axis
    
    def pos_error(self):
        return sqrt(pow((self.wp_pose.x - self.pose.x), 2) + pow((self.wp_pose.y - self.pose.y), 2)) # Euclidean distance
    
    def rot_error(self):
        return atan2(self.wp_pose.y - self.pose.y, self.wp_pose.x - self.pose.x) - self.pose.theta # Relative orientation

    def lin_vel_ctrl(self, const=1.0):
        return const * self.pos_error() # P-controller for linear velocity

    def ang_vel_ctrl(self, const=1.0):
        return const * (self.rot_error()) # P-controller for angular velocity
    
    def subscriber_callback(self, msg):
        self.pose = msg # Capture incomming message (Pose)
        self.pose.x = msg.x # Extract Position along x-axis
        self.pose.y = msg.y # Extract Position along y-axis
        self.pose.theta = msg.theta # Extract orientation about z-axis
        self.data_available = True # Set flag to feedback data available

    def timer_callback(self):
        self.wp_pos_x = self.get_parameter('wp_pos_x').value # Position of all waypoints along x-axis
        self.wp_pos_y = self.get_parameter('wp_pos_y').value # Position of all waypoints along y-axis
        self.wp_rot_z = self.get_parameter('wp_rot_z').value # Orientation of all waypoints about z-axis
        if self.data_available:
            self.wp_pose.x = self.wp_pos_x[self.wp_idx] # Position of current waypoint along x-axis
            self.wp_pose.y = self.wp_pos_y[self.wp_idx] # Position of current waypoint along y-axis
            self.wp_pose.theta = self.wp_rot_z[self.wp_idx] # Orientation of current waypoint about z-axis
            # Go
            if self.pos_error() > 0.005:
                self.message.linear.x = self.lin_vel_ctrl(const=1.5) # Set turtle linear velocity
                self.message.angular.z = self.ang_vel_ctrl(const=6.0) # Set turtle angular velocity
            else:
                self.wp_reached = True # Set flag to waypoint reached
            # Rotate
            if self.wp_reached and abs(self.wp_pose.theta - self.pose.theta) > 0.001:
                self.message.linear.x = 0.0 # Set turtle linear velocity
                self.message.angular.z = self.wp_pose.theta - self.pose.theta # Set turtle angular velocity
                self.wp_reached = False
            self.publisher.publish(self.message) # Publish turtle twist message
            if self.wp_reached:
                self.wp_idx += 1 # Increment waypoint index
                self.wp_reached = False
            if self.wp_idx >= len(self.wp_pos_x):
                quit() # Quit

######################################################################

def main():
    rclpy.init() # Initialize communication
    info = '''
MAKE THE TURTLE GO IN A SQUARE WITH CLOSED LOOP CONTROL
-------------------------------------------------------
WP_POS_X  = [5.0, 8.0, 8.0, 5.0, 5.0]   # Position of all waypoints along x-axis (m)
WP_POS_Y  = [5.0, 5.0, 8.0, 8.0, 5.0]   # Position of all waypoints along y-axis (m)
WP_ROT_Z  = [0.0, pi/2, pi, -pi/2, 0.0] # Orientation of all waypoints about z-axis (rad)
POS_TOL   = 0.005                       # Waypoint position tolerance (m)
ROT_TOL   = 0.001                       # Waypoint orientation tolerance (rad)
LIN_VEL_K = 1.5                         # P-controller gain for linear velocity
ANG_VEL_K = 6.0                         # P-controller gain for angular velocity
    '''
    print(info) # Print info
    controller_node = Controller() # Instantiate node
    rclpy.spin(controller_node) # Spin node
    controller_node.destroy_node() # Destroy node
    rclpy.shutdown() # Shutdown communication

######################################################################

if __name__ == "__main__":
    main()