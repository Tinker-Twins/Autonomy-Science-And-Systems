#!/usr/bin/env python3

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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration
from math import pi

######################################################################

# Constants and parameters
SQ_SIDE = 2.0  # Square side length (m)
SQ_ANG  = pi/2 # Square vertex angle (rad)
LIN_VEL = 0.2  # Linear velocity (m/s)
ANG_VEL = 0.2  # Angular velocity (rad/s)
N_LOOPS = 1    # Number of loops around square

######################################################################

# Display info and error
info = '''
MAKE THE TURTLE GO IN A SQUARE WITH OPEN LOOP CONTROL
-----------------------------------------------------
SQ_SIDE = 2.0  # Square side length (m)
SQ_ANG  = pi/2 # Square vertex angle (rad)
LIN_VEL = 0.2  # Linear velocity (m/s)
ANG_VEL = 0.2  # Angular velocity (rad/s)
N_LOOPS = 1    # Number of loops around square
'''

error = '''
Communication failed!
'''

######################################################################

# Helper function(s)
def print_status(lin_vel, ang_vel):
    print('Linear Velocity: {} m/s\t Angular Velocity: {} rad/s'.format(lin_vel, ang_vel))

######################################################################

# Main function
def main():
    rclpy.init() # Initialize communication
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, # Reliable communication
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, # Keep last N samples
        depth=10 # Queue depth (N) (N)
        )
    node = rclpy.create_node('controller') # Node
    publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile) # Publisher
    message = Twist() # Message

    try:
        print(info)
        while rclpy.ok():
            for i in range(4*N_LOOPS): # Go in a square with 4 vertices N_LOOPS times
                # Go
                LIN_DISP = 0.0 # Linear displacement
                node.start_time = node.get_clock().now() # Start time
                while(LIN_DISP <= SQ_SIDE):
                    message.linear.x = LIN_VEL # Linear velocity along x-axis
                    message.linear.y = 0.0 # Linear velocity along y-axis
                    message.linear.z = 0.0 # Linear velocity along z-axis
                    message.angular.x = 0.0 # Angular velocity along x-axis
                    message.angular.y = 0.0 # Angular velocity along y-axis
                    message.angular.z = 0.0 # Angular velocity along z-axis
                    publisher.publish(message) # Publish turtle twist message
                    # print_status(message.linear.x, message.angular.z) # Debug
                    LIN_DISP = LIN_VEL * (node.get_clock().now().nanoseconds - node.start_time.nanoseconds)/1e9 # Update linear displacement (dx = v*dt)
                # Stop
                message.linear.x = 0.0 # Linear velocity along x-axis
                message.linear.y = 0.0 # Linear velocity along y-axis
                message.linear.z = 0.0 # Linear velocity along z-axis
                message.angular.x = 0.0 # Angular velocity along x-axis
                message.angular.y = 0.0 # Angular velocity along y-axis
                message.angular.z = 0.0 # Angular velocity along z-axis
                publisher.publish(message) # Publish turtle twist message
                # Rotate
                ANG_DISP = 0.0 # Angular displacement
                node.start_time = node.get_clock().now() # Start time
                while(ANG_DISP <= SQ_ANG):
                    message.linear.x = 0.0 # Linear velocity along x-axis
                    message.linear.y = 0.0 # Linear velocity along y-axis
                    message.linear.z = 0.0 # Linear velocity along z-axis
                    message.angular.x = 0.0 # Angular velocity along x-axis
                    message.angular.y = 0.0 # Angular velocity along y-axis
                    message.angular.z = ANG_VEL # Angular velocity along z-axis
                    publisher.publish(message) # Publish turtle twist message
                    # print_status(message.linear.x, message.angular.z) # Debug
                    ANG_DISP = ANG_VEL * (node.get_clock().now().nanoseconds - node.start_time.nanoseconds)/1e9 # Update angular displacement (ds = w*dt)
                # Stop
                message.linear.x = 0.0 # Linear velocity along x-axis
                message.linear.y = 0.0 # Linear velocity along y-axis
                message.linear.z = 0.0 # Linear velocity along z-axis
                message.angular.x = 0.0 # Angular velocity along x-axis
                message.angular.y = 0.0 # Angular velocity along y-axis
                message.angular.z = 0.0 # Angular velocity along z-axis
                publisher.publish(message) # Publish turtle twist message
            quit() # Quit

    except Exception as error:
        print(error)

    finally:
        message.linear.x = 0.0 # Linear velocity along x-axis
        message.linear.y = 0.0 # Linear velocity along y-axis
        message.linear.z = 0.0 # Linear velocity along z-axis
        message.angular.x = 0.0 # Angular velocity along x-axis
        message.angular.y = 0.0 # Angular velocity along y-axis
        message.angular.z = 0.0 # Angular velocity along z-axis
        publisher.publish(message) # Publish turtle twist message
        node.destroy_node() # Destroy node
        rclpy.shutdown() # Shutdown communication
        quit() # Quit

######################################################################

if __name__ == "__main__":
    main()