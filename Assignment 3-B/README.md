# Assignment 3-B: TurtleBot3 Vision Processing, Tracking and Following
**Authors:** Chinmay Samak and Tanmay Samak

## Disclaimer:
I certify that all the work and writing that I contributed to here is my own and not acquired from external sources. I have cited sources appropriately and paraphrased correctly. I have not shared my writing with other students (for individual assignments) and other students outside my group (for group assignments), nor have I acquired any written portion of this document from past or present students.

## Robot Setup:
- [`turtlebot3_burger.urdf`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-B/assignment_3b/turtlebot3/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf) from [`turtlebot3_description`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%203-B/assignment_3b/turtlebot3/turtlebot3/turtlebot3_description) package was modified [to define the fixed links and joints](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/47ece12be831e78be49e4889f11b2391f7642dcc/Assignment%203-B/assignment_3b/turtlebot3/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf#L196-L223) (static transforms) for the camera module for simulating TurtleBot3 Burger with a camera in Gazebo simulator.
- [`model.sdf`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-B/assignment_3b/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf) from [`turtlebot3_gazebo`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%203-B/assignment_3b/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo) package was modified [to define links, physical properties](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/47ece12be831e78be49e4889f11b2391f7642dcc/Assignment%203-B/assignment_3b/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf#L318-L370) as well as the [fixed joints](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/47ece12be831e78be49e4889f11b2391f7642dcc/Assignment%203-B/assignment_3b/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf#L419-L435) (static transforms) for the camera module for simulating TurtleBot3 Burger with a camera in Gazebo simulator.

## Environment Setup:
- [`lane_keeping.world`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-B/assignment_3b/assignment_3b/worlds/lane_keeping.world) file was defined to setup the base environment with lane lines on a plane.
- [`lane_keeping.sdf`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-B/assignment_3b/assignment_3b/worlds/lane_keeping.sdf) was defined to spawn the [modified TurtleBot3 Burger with a camera](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-B/README.md#robot-setup) in the [`lane_keeping.world`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-B/assignment_3b/assignment_3b/worlds/lane_keeping.world) environment.


## Description:
The ROS2 package [`assignment_3b`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%203-B/assignment_3b/assignment_3b) for this assignment hosts the following [Python scripts](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%203-B/assignment_3b/assignment_3b/assignment_3b):
- [`emergency_braking.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/assignment_3a/assignment_3a/emergency_braking.py) makes the robot stop if obstacles are detected within a threshold distance. The robot is commanded to move with 1 m/s linear velocity unless any obstacle is detected. In case any obstacle is detected right in front of the robot within a threshold distance of 1 m, the robot is commanded to perform emergency braking. The script is setup to wait 4 seconds for simulation to initialize properly and then run for eternity to demonstrate dynamically variable emergency braking application.
- [`wall_following.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/assignment_3a/assignment_3a/wall_following.py) makes the robot follow walls by maintaining equal distance from them. The robot makes use of de-coupled longitudinal and lateral PID controllers (with FIFO integral anti-windup mechanism) acting on frontal distance to collision and relative distance from walls respectively for motion control. A simple finite state machine is implemented to account for `inf` scan measurements beyond LIDAR maximum range. The script is setup to wait 4 seconds for simulation to initialize properly and then run for eternity to demonstrate dynamically variable wall following application.
- [`obstacle_avoidance.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/assignment_3a/assignment_3a/obstacle_avoidance.py) makes the robot avoid obstacles by maintaining a safe distance from them. The robot makes use of de-coupled longitudinal and lateral PID controllers (with FIFO integral anti-windup mechanism) acting on frontal distance to collision and minimum distance to collision from left and right sectors spanning 30° each respectively for motion control. A finite state machine is implemented to account for turning left, turning right, going straight and cautiously rotating on the spot in case of too many obstacle in proximity. The script is setup to wait 4 seconds for simulation to initialize properly and then run for eternity to demonstrate dynamically variable obstacle avoidance application.
- [`collision_avoidance.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/assignment_3a/assignment_3a/collision_avoidance.py) makes the robot avoid collision with obstacles by maintaining a safe distance from them. The robot makes use of de-coupled longitudinal and lateral PID controllers (with FIFO integral anti-windup mechanism) acting on frontal distance to collision and minimum distance to collision from left and right sectors spanning 30° each respectively for motion control. A finite state machine is implemented to account for turning left, turning right, going straight and cautiously rotating on the spot in case of too many obstacle in proximity. The script is setup to wait 4 seconds for simulation to initialize properly and then run for eternity to demonstrate dynamically variable obstacle avoidance application. It is worth mentioning that with TurtleBot3, it is either possible to subscribe to sensor data or publish actuator commands (but NOT both) when interfaced with ROS-2 over a single [`Quality of Service (QoS)`](https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html) profile of the underlying [Data Distribution Service (DDS) or Real-Time Publish Subscribe (RTPS)](https://design.ros2.org/articles/ros_on_dds.html) implementation (also, [ROS-2 supports multiple DDS/RTPS implementations](https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html)). Hence, we have created different QoS profiles for subscribing to sensor data (using `best effort` reliability and small queue depth) and publishing actuator commands (using `reliable` reliability and relatively large queue depth).

The ROS2 package [`assignment_3a`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%203-A/assignment_3a/assignment_3a) for this assignment hosts the following [launch files](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%203-A/assignment_3a/assignment_3a/launch):
- [`emergency_braking.launch.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/assignment_3a/launch/emergency_braking.launch.py) launches [Gazebo simulator](https://gazebosim.org/home), the [`emergency_braking.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/assignment_3a/assignment_3a/emergency_braking.py) node as well as an [RViz](https://github.com/ros2/rviz) window to visualize the laserscan and odometry estimates of the robot.
- [`wall_following.launch.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/assignment_3a/launch/wall_following.launch.py) launches [Gazebo simulator](https://gazebosim.org/home), the [`wall_following.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/assignment_3a/assignment_3a/wall_following.py) node as well as an [RViz](https://github.com/ros2/rviz) window to visualize the laserscan and odometry estimates of the robot.
- [`obstacle_avoidance.launch.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/assignment_3a/launch/obstacle_avoidance.launch.py) launches [Gazebo simulator](https://gazebosim.org/home), the [`obstacle_avoidance.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/assignment_3a/assignment_3a/obstacle_avoidance.py) node as well as an [RViz](https://github.com/ros2/rviz) window to visualize the laserscan and odometry estimates of the robot.
- [`collision_avoidance.launch.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/assignment_3a/launch/collision_avoidance.launch.py) the [`collision_avoidance.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%203-A/assignment_3a/assignment_3a/assignment_3a/collision_avoidance.py) node as well as an [RViz](https://github.com/ros2/rviz) window to visualize the laserscan and odometry estimates of the robot.

## Dependencies:
- [TurtleBot3 Burger Robot Hardware](https://www.robotis.us/turtlebot-3-burger-us/) with [TurtleBot3 SBC Image](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/)
- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html) on [Ubuntu 20.04 Focal Fossa](https://releases.ubuntu.com/focal/)
- [TurtleBot3 Packages](https://github.com/ROBOTIS-GIT/turtlebot3/tree/foxy-devel) - Included with this repository
- [TurtleBot3 Simulations Packages](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/foxy-devel) - Included with this repository
- [TurtleBot3 Messages Package](https://github.com/ROBOTIS-GIT/turtlebot3_msgs/tree/foxy-devel) - Included with this repository
- [TurtleBot3 Dynamixel SDK Packages](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/foxy-devel) - Included with this repository

## Build:

1. Make a directory `ROS2_WS` to act as your ROS2 workspace.
    ```bash
    $ mkdir -p ~/ROS2_WS/src/
    ```
2. Clone this repository:
    ```bash
    $ git clone https://github.com/Tinker-Twins/Autonomy-Science-And-Systems.git
    ```
3. Move `assignment_3b` directory with required ROS2 packages to the source space (`src`) of your `ROS2_WS`.
    ```bash
    $ mv ~/Autonomy-Science-And-Systems/Assignment\ 3-B/assignment_3b/ ~/ROS2_WS/src/
    ```
4. [Optional] Remove the unnecessary files.
    ```bash
    $ sudo rm -r Autonomy-Science-And-Systems
    ```
5. Build the packages.
    ```bash
    $ cd ~/ROS2_WS
    $ colcon build
    ```
6. Source the `setup.bash` file of your `ROS2_WS`.
    ```bash
    $ echo "source ~/ROS2_WS/install/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    ```

## Execute:
### Simulation:
1. Emergency Braking:
    ```bash
    $ ros2 launch assignment_3a emergency_braking.launch.py
    ```
2. Wall Following:
    ```bash
    $ ros2 launch assignment_3a wall_following.launch.py
    ```
3. Obstacle Avoidance:
    ```bash
    $ ros2 launch assignment_3a obstacle_avoidance.launch.py
    ```
### Real World:
1. Connect to the TurtleBot3 SBC via Secure Shell Protocol (SSH):
    ```bash
    user@computer:~$ sudo ssh <username>@<ip.address.of.turtlebot3>
    user@computer:~$ sudo ssh ubuntu@192.168.43.48
    ```
2. Bringup TurtleBot3:
    ```bash
    ubuntu@ubuntu:~$ ros2 launch turtlebot3_bringup robot.launch.py
    ubuntu@ubuntu:~$ ros2 launch v4l2_camera camera.launch.py
    ```   
3. AprilTag Detection:

    **Standalone Executable:**
    ```bash
    user@computer:~$ ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=image/compressed --remap out:=image/uncompressed
    user@computer:~$ ros2 run rqt_image_view rqt_image_view
    user@computer:~$ ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/image/uncompressed -r camera_info:=/camera_info --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml
    # user@computer:~$ ros2 topic echo /camera_info
    # user@computer:~$ ros2 topic echo /detections
    # user@computer:~$ ros2 topic echo /tf
    user@computer:~$ rviz2 -d `ros2 pkg prefix assignment_3b`/share/assignment_3b/rviz/apriltag.rviz
    ```
    
    **Launch File:**

## Results:
The [`media`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%203-B/media) directory hosts pictures and videos of the implementations.

1. Simulation:

| ![Emergency Braking](media/emergency_braking.gif) | ![Wall Following](media/wall_following.gif) | ![Obstacle Avoidance](media/obstacle_avoidance.gif) |
|:-------------------------------------:|:-----------------------------------------:|:-------------------------------------:|
| Emergency Braking | Wall Following | Obstacle Avoidance |


2. Real World:

| ![Collision Avoidance Robot](media/collision_avoidance_robot.gif) | ![Collision Avoidance Terminal](media/collision_avoidance_rviz.gif) |
|:-------------------------------------:|:-----------------------------------------:|
| Collision Avoidance - TurtleBot3 | Collision Avoidance - Remote PC |
