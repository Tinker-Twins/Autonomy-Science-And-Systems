# Assignment 1-A : TurtleSim
**Author:** Tanmay Samak

## Disclaimer:
I certify that all the work and writing that I contributed to here is my own and not acquired from external sources. I have cited sources appropriately and paraphrased correctly.  I have not shared my writing with other students (for individual assignments) and other students outside my group (for  group assignments), nor have I acquired any written portion of this document from past or present students.

## Description:
The dedicated ROS2 package [`assignment_1b`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%201-B/Tanmay/assignment_1b) for this assignment hosts [4 Python scripts](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Assignment%201-B/Tanmay/assignment_1b/assignment_1b):
- [`go_in_circle.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-B/Tanmay/assignment_1b/assignment_1b/go_in_circle.py) executes a simple open loop controller to make the robot (turtle) go in a circle whose radius is defined by the ratio of linear velocity to angular velocity, for one complete loop.
- [`go_to_goal.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-B/Tanmay/assignment_1b/assignment_1b/go_to_goal.py) executes a closed loop controller to make the robot (turtle) attain a desired 2D goal pose (1.0, 1.0, 0.0) from its initial pose ~(5.5, 5.5).
- [`square_open_loop.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-B/Tanmay/assignment_1b/assignment_1b/square_open_loop.py) executes an open loop controller to make the robot (turtle) go in a square whose side length is 2 units, for one complete loop, with a linear velocity of 0.2 m/s and an angular velocity of 0.2 rad/s.
- [`square_closed_loop.py`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Assignment%201-B/Tanmay/assignment_1b/assignment_1b/square_closed_loop.py) executes a closed loop controller to make the robot (turtle) go in a square whose vertices are defined by the coordinates (5.0, 5.0), (8.0, 5.0), (8.0, 8.0) and (5.0, 8.0) from its initial pose ~(5.5, 5.5) for one complete loop.
