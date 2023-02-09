# Assignment 1-C : TurtleBot3 Open-Loop Control
**Authors:** Chinmay Samak and Tanmay Samak

## Disclaimer:
I certify that all the work and writing that I contributed to here is my own and not acquired from external sources. I have cited sources appropriately and paraphrased correctly. I have not shared my writing with other students (for individual assignments) and other students outside my group (for group assignments), nor have I acquired any written portion of this document from past or present students.

## Description:
The ROS2 package [`assignment_1c`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%201-C/assignment_1c/assignment_1c) for this assignment hosts the following [Python scripts](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%201-C/assignment_1c/assignment_1c/assignment_1c):
- [`circle.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-C/assignment_1c/assignment_1c/assignment_1c/circle.py) makes the robot move in a circle with a specified constant twist command. The radius of the circle being traversed is governed by the ratio of linear and angular velocities provided to the robot. The script is setup to wait 4 seconds for simulation to initialize properly and then run for one complete traversal of the robot around the circle in an open-loop manner.
- [`square.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-C/assignment_1c/assignment_1c/assignment_1c/square.py) makes the turtle move in a square of 2x2 m with a specified constant twist command. The script is setup to to wait 4 seconds for simulation to initialize properly and then run for one complete traversal of the robot around the square in an open-loop manner.

## Dependencies:
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
3. Move `assignment_1c` directory with required ROS2 packages to the source space (`src`) of your `ROS2_WS`.
    ```bash
    $ mv ~/Autonomy-Science-And-Systems/Assignment\ 1-C/assignment_1c/ ~/ROS2_WS/src/
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
1. Open-Loop Circle (twist commands specified by the user):
    ```bash
    $ ros2 launch assignment_1c circle.launch.py lin_vel:=0.15 ang_vel:=0.15
    ```
2. Open-Loop Square (twist commands specified by the user):
    ```bash
    $ ros2 launch assignment_1c square.launch.py lin_vel:=0.15 ang_vel:=0.15
    ```    
3. Open-Loop Move (square or circle maneuver with twist commands specified by the user):
    ```bash
    $ ros2 launch assignment_1c move.launch.py maneuver:=circle lin_vel:=0.15 ang_vel:=0.15
    $ ros2 launch assignment_1c move.launch.py maneuver:=square lin_vel:=0.15 ang_vel:=0.15
    ```
