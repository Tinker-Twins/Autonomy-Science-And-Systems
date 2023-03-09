# Mini Assignment 4: TurtleBot3 Camera Calibration
**Authors:** Chinmay Samak and Tanmay Samak

## Disclaimer:
I certify that all the work and writing that I contributed to here is my own and not acquired from external sources. I have cited sources appropriately and paraphrased correctly. I have not shared my writing with other students (for individual assignments) and other students outside my group (for group assignments), nor have I acquired any written portion of this document from past or present students.

## Description:

## Dependencies:
- [TurtleBot3 Burger Robot Hardware](https://www.robotis.us/turtlebot-3-burger-us/) with [TurtleBot3 SBC Image](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/)
- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html) on [Ubuntu 20.04 Focal Fossa](https://releases.ubuntu.com/focal/)
- [Camera Packages](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Mini%20Assignment%204/camera_calibration) - Included with this repository

## Build:
1. Make a directory `ROS2_WS` to act as your ROS2 workspace.
    ```bash
    $ mkdir -p ~/ROS2_WS/src/
    ```
2. Clone this repository:
    ```bash
    $ git clone https://github.com/Tinker-Twins/Autonomy-Science-And-Systems.git
    ```
3. Move `camera_calibration` directory with required ROS2 packages to the source space (`src`) of your `ROS2_WS`.
    ```bash
    $ mv ~/Autonomy-Science-And-Systems/Mini\ Assignment\ 4/camera_calibration/ ~/ROS2_WS/src/
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

## Select Camera Device
All available/connected camera devices can be listed using:
```bash
$ v4l2-ctl --list-devices
# v4l-utils can be installed via apt
# $sudo apt install v4l-utils
```
The first port of a device is generally the one over which videocapture is available. [This line](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Mini%20Assignment%204/camera_calibration/v4l2_camera/ros2_v4l2_camera/src/v4l2_camera.cpp#L52) of `v4l2_camera.cpp` can be modified to change/switch camera device (if need be). Rebuild the package to reflect any changes.

## Execute:
1. Launch camera image publisher node:
```bash
$ ros2 launch v4l2_camera camera.launch.py
```

2. Launch camera calibration node:
```bash
$ ros2 run camera_calibration cameracalibrator --camera_name arducam --size=8x6 --square=0.02 --approximate=0.3 --no-service-check --ros-args -r image:=/image
```

## Results:
The [`media`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Mini%20Assignment%204/media) directory hosts pictures and videos of the implementation.
