# Assignment 1-C : TurtleBot3 Open Loop Control
**Authors:** Chinmay Samak and Tanmay Samak

## Disclaimer:
I certify that all the work and writing that I contributed to here is my own and not acquired from external sources. I have cited sources appropriately and paraphrased correctly. I have not shared my writing with other students (for individual assignments) and other students outside my group (for group assignments), nor have I acquired any written portion of this document from past or present students.

## Description:
The ROS2 package [`assignment_1c`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%201-C/assignment_1c/assignment_1c) for this assignment hosts the following [Python scripts](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%201-C/assignment_1c/assignment_1c/assignment_1c):
- [`circle.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-C/assignment_1c/assignment_1c/assignment_1c/circle.py) makes the robot move in a circle with a specified constant twist velocity. The radius of the circle being traversed is governed by the ratio of linear and angular velocities provided to the robot. The script is setup to wait 4 seconds for simulation to initialize properly and then run for one complete traversal of the robot around the circle in an open-loop manner.
- [`square.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-C/assignment_1c/assignment_1c/assignment_1c/square.py) makes the turtle move in a square of 2x2 m with a specified constant twist velocity. The script is setup to to wait 4 seconds for simulation to initialize properly and then run for one complete traversal of the robot around the square in an open-loop manner.
