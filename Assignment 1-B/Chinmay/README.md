# Assignment 1-A : TurtleSim
**Author:** Chinmay Samak

## Disclaimer:
I certify that all the work and writing that I contributed to here is my own and not acquired from external sources. I have cited sources appropriately and paraphrased correctly.  I have not shared my writing with other students (for individual assignments) and other students outside my group (for  group assignments), nor have I acquired any written portion of this document from past or present students.

## Description:
The ROS2 package [`assignment_1b`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%201-B/Chinmay/assignment_1b) for this assignment hosts the following [Python scripts](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%201-B/Chinmay/assignment_1b/assignment_1b):
- [`open_loop_circle.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-B/Chinmay/assignment_1b/assignment_1b/open_loop_circle.py) makes the turtle move in a circle with a constant twist velocity. The radius of the circle being traversed is governed by the linear and angular velocities provided to the turtle. The script is setup to run for one complete traversal of the circle in an open-loop manner.
- [`open_loop_square.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-B/Chinmay/assignment_1b/assignment_1b/open_loop_square.py) makes the turtle move in a square of 2x2 m with a linear velocity of 0.2 m/s and an angular velocity of 0.2 rad/s. The script is setup to run for one complete traversal of the square in an open-loop manner.
- [`closed_loop_square.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-B/Chinmay/assignment_1b/assignment_1b/closed_loop_square.py) makes the turtle move in a square of 3x3 m defined by the following vertices: (5,5) --> (8,5) --> (8,8) --> (5,8) --> (5,5). The coordinates are loaded as parameters from the [`launch`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-B/Chinmay/assignment_1b/launch/closed_loop_square.launch.py) file. The script is setup to run for one complete traversal of the square in an closed-loop manner (velocity control).

## Dependencies:
- ROS2 Foxy Fitzroy on Ubuntu 20.04 Focal Fossa
