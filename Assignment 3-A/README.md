# Assignment 3-A : TurtleBot3 Emergency Braking, Wall Following and Obstacle Avoidance
**Authors:** Chinmay Samak and Tanmay Samak

## Disclaimer:
I certify that all the work and writing that I contributed to here is my own and not acquired from external sources. I have cited sources appropriately and paraphrased correctly. I have not shared my writing with other students (for individual assignments) and other students outside my group (for group assignments), nor have I acquired any written portion of this document from past or present students.

## Description:
The ROS2 package [`assignment_3a`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%203-A/assignment_3a/assignment_3a) for this assignment hosts the following [Python scripts](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%203-A/assignment_3a/assignment_3a/assignment_3a):
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
3. Move `assignment_3a` directory with required ROS2 packages to the source space (`src`) of your `ROS2_WS`.
    ```bash
    $ mv ~/Autonomy-Science-And-Systems/Assignment\ 3-A/assignment_3a/ ~/ROS2_WS/src/
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
    $ sudo ssh <username>@<ip.address.of.turtlebot3>
    $ sudo ssh ubuntu@192.168.43.48
    ```
2. Bringup TurtleBot3:
    ```bash
    $ ros2 launch turtlebot3_bringup robot.launch.py
    ```   
6. Open-Loop Move (square or circle maneuver with twist commands specified by the user):
    ```bash
    $ ros2 launch assignment_3a collision_avoidance.launch.py
    ```
## Results:
The [`media`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%203-A/media) directory hosts pictures and videos of the implementations.

1. Simulation:

| ![Slow Square](media/square_slow_robot.gif) ![Slow Square](media/square_slow_rviz.gif) | ![Medium Square](media/square_medium_robot.gif) ![Medium Square](media/square_medium_rviz.gif) | ![Fast Square](media/square_fast_robot.gif) ![Fast Square](media/square_fast_rviz.gif) |
|:-------------------------------------:|:-----------------------------------------:|:-------------------------------------:|
| Slow (`lin_vel` = 0.05, `ang_vel` = 0.2) | Medium (`lin_vel` = 0.10, `ang_vel` = 0.4) | Fast (`lin_vel` = 0.15, `ang_vel` = 0.6) |


2. Real World:

| ![Teleop Robot](media/teleop_robot.gif) | ![Teleop Terminal](media/teleop_terminal.gif) |
|:-------------------------------------:|:-----------------------------------------:|
| Physical TurtleBot3 Burger | Remote PC Terminal Window |
