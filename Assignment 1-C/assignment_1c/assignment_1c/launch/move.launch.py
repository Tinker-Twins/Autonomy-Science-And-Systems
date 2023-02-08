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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ld = LaunchDescription()

    turtlebot3_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("turtlebot3_gazebo"), '/launch', '/empty_world.launch.py',
            ])
        )
    
    maneuver_arg = DeclareLaunchArgument('maneuver', default_value='square', description='Maneuver (circle/square) that the robot must perform.')
    lin_vel_arg = DeclareLaunchArgument('lin_vel', default_value='0.3', description='Linear velocity of the robot (m/s).')
    ang_vel_arg = DeclareLaunchArgument('ang_vel', default_value='0.3', description='Angular velocity of the robot (rad/s).')

    controller_node = Node(
        package='assignment_1c',
        executable=LaunchConfiguration('maneuver'),
        name='move_node',
        parameters = [
            {'lin_vel': LaunchConfiguration('lin_vel')},
            {'ang_vel': LaunchConfiguration('ang_vel')},
        ],
        emulate_tty=True,
        output='screen',
        )
    
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='odometry_rviz',
            arguments=['-d', [FindPackageShare("assignment_1c"), '/rviz', '/assignment_1c.rviz',]]
        )

    ld.add_action(turtlebot3_node)
    ld.add_action(rviz_node)
    ld.add_action(maneuver_arg)
    ld.add_action(lin_vel_arg)
    ld.add_action(ang_vel_arg)
    ld.add_action(controller_node)

    return ld