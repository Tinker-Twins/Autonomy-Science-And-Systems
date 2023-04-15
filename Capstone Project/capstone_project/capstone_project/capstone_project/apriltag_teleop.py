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
from geometry_msgs.msg import Twist # Twist (linear and angular velocities) message class
from rclpy.qos import QoSProfile # Ouality of Service (tune communication between nodes)

# Python mudule imports
import os # Miscellaneous operating system interfaces
import select # Waiting for I/O completion
import sys # System-specific parameters and functions
if os.name == 'nt':
    import msvcrt # Useful routines from the MS VC++ runtime
else:
    import termios # POSIX style tty control
    import tty # Terminal control functions

# Parameters
MAX_LIN_VEL = 0.22
MAX_ANG_VEL = 2.84
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

# Information
info = """
---------------------------------------
  AprilTag Marker Teleoperation Panel
---------------------------------------
                 W   
             A   S   D
                 X
W/X: Increase/decrease linear velocity
A/D: Increase/decrease angular velocity
Space/S: Emergency brake
Press CTRL+C to quit
NOTE: Press keys within this terminal
---------------------------------------
"""

# Error
error = """
ERROR: Communication failed!
"""

# Get keyboard key
def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Print current twist commands
def print_twist(target_linear_velocity, target_angular_velocity):
    print('Linear Velocity: {0}\t Angular Velocity: {1} '.format(round(target_linear_velocity, 1),
                                                                 round(target_angular_velocity, 1),
                                                          )
    )

# Manage slop (i.e. overflow) of twist commands
def manage_slop(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output

# Constrain twist commands
def constrain(velocity, low_bound, high_bound):
    if velocity < low_bound:
        velocity = low_bound
    elif velocity > high_bound:
        velocity = high_bound
    else:
        velocity = velocity
    return velocity

# Constrain linear velocity command
def check_linear_velocity_limit(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)

# Constrain angular velocity command
def check_angular_velocity_limit(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)

def main():
    # Settings
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # ROS 2 infrastructure
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_apriltag')
    pub = node.create_publisher(Twist, 'apriltag/cmd_vel', qos)

    # Initialize
    twist = Twist()
    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        # Print information
        print(info)
        # Generate twist commands
        while(1):
            key = get_key(settings)
            if key == 'w':
                target_linear_velocity = check_linear_velocity_limit(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_twist(target_linear_velocity, target_angular_velocity)
            elif key == 'x':
                target_linear_velocity = check_linear_velocity_limit(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_twist(target_linear_velocity, target_angular_velocity)
            elif key == 'a':
                target_angular_velocity = check_angular_velocity_limit(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                print_twist(target_linear_velocity, target_angular_velocity)
            elif key == 'd':
                target_angular_velocity = check_angular_velocity_limit(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                print_twist(target_linear_velocity, target_angular_velocity)
            elif key == ' ' or key == 's':
                target_linear_velocity = 0.0
                control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                print_twist(target_linear_velocity, target_angular_velocity)
            else:
                if (key == '\x03'):
                    break
            # Recursively print information
            if status == 20:
                print(info)
                status = 0
            # Generate and publish twist
            control_linear_velocity = manage_slop(control_linear_velocity,
                                                          target_linear_velocity,
                                                          LIN_VEL_STEP_SIZE/2.0,
                                      )
            control_angular_velocity = manage_slop(control_angular_velocity,
                                                           target_angular_velocity,
                                                           ANG_VEL_STEP_SIZE/2.0,
                                       )
            twist.linear.x = control_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular_velocity
            pub.publish(twist)

    except Exception as error:
        # Print error
        print(error)

    finally:
        # Generate and publish zero twist
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
