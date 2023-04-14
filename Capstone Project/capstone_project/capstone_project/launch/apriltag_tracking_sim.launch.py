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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('capstone_project')

    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_share, 'models')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    turtlebot3_gazebo_launch = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(pkg_share, 'worlds', 'apriltag_tracking.sdf')],
            description='Simulation Description Format (SDFormat/SDF) for Describing Robot and Environment',
        ),
        gazebo,
        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot3_gazebo_launch, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        ExecuteProcess(
            cmd=[['ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/camera/image_raw -r camera_info:=/camera/camera_info --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml']],
            shell=True,
        ),
        # Node(
        #     package='capstone_project',
        #     executable='apriltag_tracking_sim',
        #     name='apriltag_tracking_sim_node',
        #     emulate_tty=True,
        #     output='screen',
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', [FindPackageShare("capstone_project"), '/rviz', '/capstone_project_sim.rviz',]]
        ),
        
    ])