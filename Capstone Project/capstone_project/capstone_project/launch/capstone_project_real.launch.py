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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():

    pkg_darknet_ros = get_package_share_directory('darknet_ros')
    tiny_yolo_darknet_config = pkg_darknet_ros + '/config/yolov7-tiny.yaml'
    darknet_ros_config = pkg_darknet_ros + '/config/capstone_real.yaml'

    return LaunchDescription([
        ExecuteProcess(
            cmd=[['ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=image/compressed --remap out:=image/uncompressed']],
            shell=True,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([pkg_darknet_ros + '/launch/darknet_ros.launch.py']),
            launch_arguments={'network_param_file': tiny_yolo_darknet_config,
                              'ros_param_file': darknet_ros_config}.items()
        ),
        ExecuteProcess(
            cmd=[['ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/image/uncompressed -r camera_info:=/camera_info --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml']],
            shell=True,
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera_tf',
            arguments = ['0.055', '0.000', '0.100',
                         '0.000', '1.571', '0.000',
                         'base_link', 'camera']
        ),
        Node(
            package='capstone_project',
            executable='capstone_project_real',
            name='capstone_project_real_node',
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', [FindPackageShare("capstone_project"), '/rviz', '/capstone_project_real.rviz',]]
        ),
    ])