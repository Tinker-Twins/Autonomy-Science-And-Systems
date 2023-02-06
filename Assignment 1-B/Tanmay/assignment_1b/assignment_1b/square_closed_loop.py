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

# ROS2 module imports
import rclpy # ROS2 client library (rcl) for Python (built on rcl C API)
from rclpy.node import Node # Node class for Python nodes
from geometry_msgs.msg import Twist # Twist (linear and angular velocities) message class
from turtlesim.msg import Pose # Turtlesim pose (x, y, theta) message class
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Ouality of Service (tune communication between nodes)

# Python mudule imports
from math import pow, atan2, sqrt, pi # Common mathematical functions and constants

######################################################################

# Node class
class RobotController(Node):

    #######################
    '''Class constructor'''
    #######################

    def __init__(self):
        # Information and debugging
        info = '''\nMake the robot go to the specified goal.\n'''
        print(info)
        # ROS2 infrastructure
        super().__init__('robot_controller') # Create a node with name 'robot_controller'
        qos_profile = QoSProfile( # Ouality of Service profile
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, # Reliable (not best effort) communication
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, # Keep/store only up to last N samples
        depth=10 # Queue size/depth of 10 (only honored if the “history” policy was set to “keep last”)
        )
        self.robot_ctrl_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile) # Publisher which will publish Twist message to the topic '/turtle1/cmd_vel' adhering to 'qos_profile' QoS profile
        self.robot_pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.robot_feedback_callback, qos_profile) # Subscriber which will subscribe Pose message from the topic '/turtle1/pose' and execute 'robot_feedback_callback()' callback function adhering to 'qos_profile' QoS profile
        timer_period = 0.01 # Node execution time period (seconds)
        self.timer = self.create_timer(timer_period, self.robot_controller_callback) # Define timer to execute 'robot_controller_callback()' every 'timer_period' seconds
        # Initialize variables
        self.robot_pose = Pose() # Robot pose (position & rotation)
        self.robot_flag = False # Flag to check if robot feedback is available
        self.goal_pose = Pose() # Goal pose (position & rotation)
        self.goal_flag = False # Flag to check if set goal is reached
        self.ctrl_msg = Twist() # Robot control commands (twist)
        # Set goal pose
        self.wp_index = 0 # Current waypoint index
        self.goal_pose_x = [5.0, 8.0, 8.0 ,5.0 ,5.0] # Position of all waypoints along x-axis
        self.goal_pose_y = [5.0, 5.0, 8.0 ,8.0 ,5.0] # Position of all waypoints along y-axis
        self.goal_pose_theta = [0.0, pi/2, pi, -pi/2, 0.0] # Orientation of all waypoints about z-axis
    
    ########################
    '''Callback functions'''
    ########################
    
    def robot_feedback_callback(self, message):
        '''Robot feedback (pose) callback'''
        self.robot_pose = message # Capture incomming message (Pose)
        self.robot_pose.x = message.x # Extract position along x-axis
        self.robot_pose.y = message.y # Extract position along y-axis
        self.robot_pose.theta = message.theta # Extract orientation about z-axis
        self.robot_flag = True # Set robot flag to feedback available
        #print('Goal Pose  : x = {}, y = {}, theta = {}'.format(round(self.goal_pose.x, 1), round(self.goal_pose.y, 1), round(self.goal_pose.theta, 1)))
        #print('Robot Pose : x = {}, y = {}, theta = {}'.format(round(self.robot_pose.x, 1), round(self.robot_pose.y, 1), round(self.robot_pose.theta, 1)))
        #self.get_logger().info('Goal Pose  : ' + str(self.goal_pose.x) + ', ' + str(self.goal_pose.y) + ', ' + str(self.goal_pose.theta))
        #self.get_logger().info('Robot Pose : ' + str(self.robot_pose.x) + ', ' + str(self.robot_pose.y) + ', ' + str(self.robot_pose.theta))

    def robot_controller_callback(self):
        '''Robot controller (twist) callback'''
        if self.robot_flag:
            lin_vel, ang_vel = self.set_robot_controls(pos_tol=0.01, rot_tol=0.001) # Set and pubblish robot controls
            self.ctrl_msg.linear.x = lin_vel # Set robot linear velocity
            self.ctrl_msg.angular.z = ang_vel # Set robot angular velocity
            self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message
            if self.goal_flag:
                # Increment waypoint index upto total number of waypoints available
                if self.wp_index < len(self.goal_pose_x)-1:
                    self.wp_index += 1
                else:
                     quit() # Quit execution
                print('Robot reached specified waypoint pose: x = {} m, y = {} m, theta = {} rad'.format(self.goal_pose.x, self.goal_pose.y, self.goal_pose.theta))
            else:
                print('Robot going to specified waypoint pose: x = {} m, y = {} m, theta = {} rad'.format(self.goal_pose.x, self.goal_pose.y, self.goal_pose.theta))

    ######################
    '''Helper functions'''
    ######################

    def get_position_error(self):
        '''Error in position as Euclidean distance between current pose and goal pose.'''
        return sqrt(pow((self.goal_pose.x - self.robot_pose.x), 2) + pow((self.goal_pose.y - self.robot_pose.y), 2))
    
    def get_rotation_error(self):
        '''Error in rotation as relative angle between current pose and goal pose.'''
        return atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x) - self.robot_pose.theta

    def get_linear_velocity(self, gain=1.0):
        '''Compute robot linear velocity using P-controller'''
        return gain * self.get_position_error()

    def get_angular_velocity(self, gain=1.0):
        '''Compute robot angular velocity using P-controller'''
        return gain * self.get_rotation_error()

    def set_robot_controls(self, pos_tol=0.1, rot_tol=0.1):
        '''Set robot controls (twist) based on deviation from goal'''
        self.goal_pose.x = self.goal_pose_x[self.wp_index] # Position of current waypoint along x-axis
        self.goal_pose.y = self.goal_pose_y[self.wp_index] # Position of current waypoint along y-axis
        self.goal_pose.theta = self.goal_pose_theta[self.wp_index] # Orientation of current waypoint about z-axis
        if self.get_position_error() > pos_tol: # Go to goal
            self.goal_flag = False # Set goal flag to not reached
            lin_vel = self.get_linear_velocity(gain=1.5) # Set robot linear velocity
            ang_vel = self.get_angular_velocity(gain=6.0) # Set robot angular velocity
            return lin_vel, ang_vel
        if abs(self.goal_pose.theta - self.robot_pose.theta) > rot_tol: # Orient at goal
            self.goal_flag = False # Set goal flag to not reached
            lin_vel = 0.0 # Set robot linear velocity
            ang_vel = self.goal_pose.theta - self.robot_pose.theta # Set robot angular velocity
            return lin_vel, ang_vel
        # Stop robot after the goal is reached
        lin_vel = 0.0 # Set robot linear velocity
        ang_vel = 0.0 # Set robot angular velocity
        self.goal_flag = True # Set goal flag to reached
        return lin_vel, ang_vel

######################################################################

def main(args=None):
    rclpy.init(args=args) # Start ROS2 communications
    node = RobotController() # Create node
    rclpy.spin(node) # Execute node
    node.destroy_node() # Destroy node explicitly (optional - otherwise it will be done automatically when garbage collector destroys the node object)
    rclpy.shutdown() # Shutdown ROS2 communications

######################################################################

if __name__ == "__main__":
    main()
