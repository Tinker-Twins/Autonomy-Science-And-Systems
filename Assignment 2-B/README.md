# Assignment 2-B : TurtleBot3 Teleop and Open-Loop Control Analysis
**Authors:** Chinmay Samak and Tanmay Samak

## Disclaimer:
I certify that all the work and writing that I contributed to here is my own and not acquired from external sources. I have cited sources appropriately and paraphrased correctly. I have not shared my writing with other students (for individual assignments) and other students outside my group (for group assignments), nor have I acquired any written portion of this document from past or present students.

## Description:
The ROS2 package [`turtlebot3_teleop`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%202-B/assignment_2b/turtlebot3/turtlebot3/turtlebot3_teleop) for this assignment hosts the following [Python script](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%202-B/assignment_2b/turtlebot3/turtlebot3/turtlebot3_teleop/turtlebot3_teleop/script):
- [`teleop_keyboard.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%202-B/assignment_2b/turtlebot3/turtlebot3/turtlebot3_teleop/turtlebot3_teleop/script/teleop_keyboard.py) accepts keyboard inputs and modifies the commanded linear and angular velocities (twist) of the robot based on the key pressed.

The ROS2 package [`assignment_2b`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%202-B/assignment_2b/assignment_2b) for this assignment hosts the following [Python scripts](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%202-B/assignment_2b/assignment_2b/assignment_2b):
- [`circle.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%202-B/assignment_2b/assignment_2b/assignment_2b/circle.py) makes the robot move in a circle with a specified constant twist command. The radius of the circle being traversed is governed by the ratio of linear and angular velocities provided to the robot. The script is setup to wait 4 seconds for simulation to initialize properly and then run for one complete traversal of the robot around the circle in an open-loop manner.
- [`square.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%202-B/assignment_2b/assignment_2b/assignment_2b/square.py) makes the robot move in a square of 2x2 m with a specified constant twist command. The script is setup to to wait 4 seconds for simulation to initialize properly and then run for one complete traversal of the robot around the square in an open-loop manner.

The ROS2 package [`turtlebot3_bringup`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%202-B/assignment_2b/turtlebot3/turtlebot3/turtlebot3_bringup) for this assignment hosts the following significant [launch file](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%202-B/assignment_2b/turtlebot3/turtlebot3/turtlebot3_bringup/launch):
- [`robot.launch.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%202-B/assignment_2b/turtlebot3/turtlebot3/turtlebot3_bringup/launch/robot.launch.py) launches the [`turtlebot3_node`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%202-B/assignment_2b/turtlebot3/turtlebot3/turtlebot3_node) with specified arguments for device types, drivers, transforms, etc. for different links, joints, sensors, actuators, etc. of the robot. This launch file also resets the odometry estimate from potentially high values (close to NaN) to zero at each run without the need to physically restart the robot with the help of modified [`odometry.cpp`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%202-B/assignment_2b/turtlebot3/turtlebot3/turtlebot3_node/src/odometry.cpp) and [`odometry.hpp`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%202-B/assignment_2b/turtlebot3/turtlebot3/turtlebot3_node/include/turtlebot3_node/odometry.hpp) source files in [`turtlebot3_node`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%202-B/assignment_2b/turtlebot3/turtlebot3/turtlebot3_node/include/turtlebot3_node) package, thereby resolving a major bug in the [original TurtleBot3 packages](https://github.com/ROBOTIS-GIT/turtlebot3/tree/foxy-devel).

The ROS2 package [`assignment_2b`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%202-B/assignment_2b/assignment_2b) for this assignment hosts the following [launch files](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%202-B/assignment_2b/assignment_2b/launch):
- [`circle.launch.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%202-B/assignment_2b/assignment_2b/launch/circle.launch.py) launches the [`circle.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%202-B/assignment_2b/assignment_2b/assignment_2b/circle.py) node with `lin_vel` and `ang_vel` parameters (specified as CLI arguments) as well as an RViz window to visualize the odometry estimates of the robot.
- [`square.launch.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%202-B/assignment_2b/assignment_2b/launch/square.launch.py) launches the [`square.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%202-B/assignment_2b/assignment_2b/assignment_2b/square.py) node with `lin_vel` and `ang_vel` parameters (specified as CLI arguments) as well as an RViz window to visualize the odometry estimates of the robot.
- [`move.launch.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%202-B/assignment_2b/assignment_2b/launch/move.launch.py) launches the [`circle.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%202-B/assignment_2b/assignment_2b/assignment_2b/circle.py) or [`square.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%202-B/assignment_2b/assignment_2b/assignment_2b/square.py) node (depending on the `maneuver` argument specified) with `lin_vel` and `ang_vel` parameters (specified as CLI arguments) as well as an RViz window to visualize the odometry estimates of the robot.

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
3. Move `assignment_2b` directory with required ROS2 packages to the source space (`src`) of your `ROS2_WS`.
    ```bash
    $ mv ~/Autonomy-Science-And-Systems/Assignment\ 2-B/assignment_2b/ ~/ROS2_WS/src/
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
1. Connect to the TurtleBot3 SBC via Secure Shell Protocol (SSH):
    ```bash
    $ sudo ssh <username>@<ip.address.of.turtlebot3>
    $ sudo ssh ubuntu@192.168.1.87
    ```
2. Bringup TurtleBot3:
    ```bash
    $ ros2 launch turtlebot3_bringup robot.launch.py
    ```
3. Teleoperation using Keyboard:
    ```bash
    $ ros2 run turtlebot3_teleop teleop_keyboard
    ```
4. Open-Loop Circle (twist commands specified by the user):
    ```bash
    $ ros2 launch assignment_2b circle.launch.py lin_vel:=0.15 ang_vel:=0.15
    ```
5. Open-Loop Square (twist commands specified by the user):
    ```bash
    $ ros2 launch assignment_2b square.launch.py lin_vel:=0.15 ang_vel:=0.15
    ```    
6. Open-Loop Move (square or circle maneuver with twist commands specified by the user):
    ```bash
    $ ros2 launch assignment_2b move.launch.py maneuver:=circle lin_vel:=0.15 ang_vel:=0.15
    $ ros2 launch assignment_2b move.launch.py maneuver:=square lin_vel:=0.15 ang_vel:=0.15
    ```
## Results:
The [`media`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%202-B/media) directory hosts pictures and videos of the implementations.

1. Open-Loop Circle:

| ![Slow Circle](media/circle_slow_robot.gif) ![Slow Circle](media/circle_slow_rviz.gif) | ![Medium Circle](media/circle_medium_robot.gif) ![Medium Circle](media/circle_medium_rviz.gif) | ![Fast Circle](media/circle_fast_robot.gif) ![Fast Circle](media/circle_fast_rviz.gif) |
|:-------------------------------------:|:-----------------------------------------:|:-------------------------------------:|
| Slow (`lin_vel` = 0.05, `ang_vel` = 0.2) | Medium (`lin_vel` = 0.10, `ang_vel` = 0.4) | Fast (`lin_vel` = 0.15, `ang_vel` = 0.6) |

2. Open-Loop Square:

| ![Slow Square](media/square_slow_robot.gif) ![Slow Square](media/square_slow_rviz.gif) | ![Medium Square](media/square_medium_robot.gif) ![Medium Square](media/square_medium_rviz.gif) | ![Fast Square](media/square_fast_robot.gif) ![Fast Square](media/square_fast_rviz.gif) |
|:-------------------------------------:|:-----------------------------------------:|:-------------------------------------:|
| Slow (`lin_vel` = 0.05, `ang_vel` = 0.2) | Medium (`lin_vel` = 0.10, `ang_vel` = 0.4) | Fast (`lin_vel` = 0.15, `ang_vel` = 0.6) |
