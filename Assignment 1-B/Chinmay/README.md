# Assignment 1-B : TurtleSim Control
**Author:** Chinmay Samak

## Disclaimer:
I certify that all the work and writing that I contributed to here is my own and not acquired from external sources. I have cited sources appropriately and paraphrased correctly. I have not shared my writing with other students (for individual assignments) and other students outside my group (for group assignments), nor have I acquired any written portion of this document from past or present students.

## Description:
The ROS2 package [`assignment_1b`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%201-B/Chinmay/assignment_1b) for this assignment hosts the following [Python scripts](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%201-B/Chinmay/assignment_1b/assignment_1b):
- [`open_loop_circle.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-B/Chinmay/assignment_1b/assignment_1b/open_loop_circle.py) makes the turtle move in a circle with a constant twist velocity. The radius of the circle being traversed is governed by the linear and angular velocities provided to the turtle. The script is setup to run for one complete traversal of the circle in an open-loop manner.
- [`open_loop_square.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-B/Chinmay/assignment_1b/assignment_1b/open_loop_square.py) makes the turtle move in a square of 2x2 m with a linear velocity of 0.2 m/s and an angular velocity of 0.2 rad/s. The script is setup to run for one complete traversal of the square in an open-loop manner.
- [`closed_loop_square.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-B/Chinmay/assignment_1b/assignment_1b/closed_loop_square.py) makes the turtle move in a square of 3x3 m defined by the following vertices: (5,5) --> (8,5) --> (8,8) --> (5,8) --> (5,5). The coordinates are loaded as parameters from the [`launch`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-B/Chinmay/assignment_1b/launch/closed_loop_square.launch.py) file. The script is setup to run for one complete traversal of the square in a closed-loop manner (velocity control).

## Dependencies:
- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html) on [Ubuntu 20.04 Focal Fossa](https://releases.ubuntu.com/focal/)

## Build:

1. Make a directory `ROS2_WS` to act as your ROS2 workspace.
    ```bash
    $ mkdir -p ~/ROS2_WS/src/
    ```
2. Clone this repository:
    ```bash
    $ git clone https://github.com/Tinker-Twins/Autonomy-Science-And-Systems.git
    ```
3. Move my `assignment_1b` ROS2 package to the source space (`src`) of your `ROS2_WS`.
    ```bash
    $ mv ~/Autonomy-Science-And-Systems/Assignment\ 1-B/Chinmay/assignment_1b/ ~/ROS2_WS/src/
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

## Run:
1. Open Loop Circle:
    ```bash
    $ ros2 launch assignment_1b open_loop_circle.launch.py
    ```
    ![Open Loop Circle](media/open_loop_circle.gif)

2. Open-Loop Square:
    ```bash
    $ ros2 launch assignment_1b open_loop_square.launch.py
    ```
    ![Open-Loop Square](media/open_loop_square.gif)
    
3. Closed-Loop Square:
    ```bash
    $ ros2 launch assignment_1b closed_loop_square.launch.py
    ```
    ![Closed-Loop Square](media/closed_loop_square.gif)
