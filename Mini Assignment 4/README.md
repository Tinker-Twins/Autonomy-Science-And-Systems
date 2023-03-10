# Mini Assignment 4: TurtleBot3 Camera Calibration
**Authors:** Chinmay Samak and Tanmay Samak

## Disclaimer:
I certify that all the work and writing that I contributed to here is my own and not acquired from external sources. I have cited sources appropriately and paraphrased correctly. I have not shared my writing with other students (for individual assignments) and other students outside my group (for group assignments), nor have I acquired any written portion of this document from past or present students.

## Description:
Camera calibration computes the following parameters:
* Distortion Coefficients (nonlinear intrinsic parameters)
* Camera Intrinsic Parameters (linear intrinsic parameters)
    * Focal length
    * Optical centers
* Camera Extrinsic Parameters (linear exptinsic parameters)
    * Rotation matrix
    * Translation vector

#### Distortion Coefficients
<p align="justify">
Some cameras introduce significant distortion to images. Two major kinds of distortion are radial distortion and tangential distortion.

Radial distortion causes straight lines to appear curved. Radial distortion becomes larger the farther points are from the center of the image. The amount of radial distortion can be represented as follows:
</p>

<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=x_%7Bdistorted%7D%20%3D%20x(%201%20%2B%20k_1%20r%5E2%20%2B%20k_2%20r%5E4%20%2B%20k_3%20r%5E6)">
</p>
<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=y_%7Bdistorted%7D%20%3D%20y(%201%20%2B%20k_1%20r%5E2%20%2B%20k_2%20r%5E4%20%2B%20k_3%20r%5E6)">
</p>

<p align="justify">
Similarly, tangential distortion occurs because the image-taking lense is not aligned perfectly parallel to the imaging plane. So, some areas in the image may look nearer than expected. The amount of tangential distortion can be represented as below:
</p>

<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=x_%7Bdistorted%7D%20%3D%20x%20%2B%20%5B%202p_1xy%20%2B%20p_2(r%5E2%2B2x%5E2)%5D">
</p>
<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=y_%7Bdistorted%7D%20%3D%20y%20%2B%20%5B%20p_1(r%5E2%2B%202y%5E2)%2B%202p_2xy%5D">
</p>

<p align="justify">
In short, we need to find five parameters, known as distortion coefficients given by:
</p>

<p align="center">
<img src="https://latex.codecogs.com/svg.image?D&space;=&space;\begin{bmatrix}&space;k_1&space;&&space;k_2&space;&&space;p_1&space;&&space;p_2&space;&&space;k_3&space;\\&space;\end{bmatrix}">
</p>

#### Camera Intrinsic Parameters
<p align="justify">
Intrinsic parameters are specific to a camera. They include information like focal length <img src="https://render.githubusercontent.com/render/math?math=(f_x%2Cf_y)"> and optical center <img src="https://render.githubusercontent.com/render/math?math=(c_x%2Cc_y)">. The focal length and optical center can be used to create a camera matrix, which can be used to remove distortion due to the lens of a specific camera. The camera matrix is unique to a specific camera, so once calculated, it can be reused on other images captured by the same camera. It is expressed as a <img src="https://render.githubusercontent.com/render/math?math=3%20%5Ctimes%203"> matrix:
</p>

<p align="center">
<img src="https://latex.codecogs.com/svg.image?K&space;=&space;\begin{bmatrix}f_x&space;&&space;0&space;&&space;c_x&space;\\0&space;&&space;f_y&space;&&space;c_y&space;\\0&space;&&space;0&space;&&space;1&space;\\\end{bmatrix}">
</p>

#### Camera Extrinsic Parameters
<p align="justify">
Extrinsic parameters correspond to rotation matrix and translation vector, <img src="https://latex.codecogs.com/svg.image?R"> and <img src="https://latex.codecogs.com/svg.image?T"> respectively, which transform 3D coordinates of a point in world frame to camera coordinate system.
    
The resultant projective mapping <img src="https://latex.codecogs.com/svg.image?M"> from world coordinates <img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x&space;&&space;y&space;&&space;z&space;\\\end{bmatrix}^T"> to image (pixel) coordinates <img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}u&space;&&space;v&space;\\\end{bmatrix}^T"> can be ultimately described by:

<p align="center">
<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}u&space;\\v&space;\\\end{bmatrix}&space;=&space;K\begin{bmatrix}R&space;&&space;T&space;\\\end{bmatrix}\begin{bmatrix}x&space;\\y&space;\\z&space;\\\end{bmatrix}&space;=M\begin{bmatrix}x&space;\\y&space;\\z&space;\\\end{bmatrix}">
</p>
</p>

#### Resources
- https://github.com/Tinker-Twins/Camera-Calibration
- https://en.wikipedia.org/wiki/Camera_resectioning
- https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

## Dependencies:
- [TurtleBot3 Burger Robot Hardware](https://www.robotis.us/turtlebot-3-burger-us/) with [TurtleBot3 SBC Image](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/)
- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html) on [Ubuntu 20.04 Focal Fossa](https://releases.ubuntu.com/focal/)
- [Camera Packages](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Mini%20Assignment%204/camera_calibration) - Included with this repository
- [Camera Calibration Parsers](https://github.com/ros-perception/image_common/tree/foxy/camera_calibration_parsers) - Installable via apt `$ sudo apt install ros-foxy-camera-calibration-parsers`
- [Camera Info Manager](https://github.com/ros-perception/image_common/tree/foxy/camera_info_manager) - Installable via apt `$ sudo apt install ros-foxy-camera-info-manager`
- [Launch Testing](https://index.ros.org/p/launch_testing_ament_cmake/#foxy) - Installable via apt `$ sudo apt install ros-foxy-launch-testing-ament-cmake`

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
The [`results`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Mini%20Assignment%204/results) directory hosts the camera calibration data including sampled frames as well as intrinsic and extrinsic camera parameters.

The [`media`](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/tree/main/Mini%20Assignment%204/media) directory hosts pictures and videos of the implementation.

![Camera Calibration](https://github.com/Tinker-Twins/Autonomy-Science-And-Systems/blob/main/Mini%20Assignment%204/media/camera_calibration.gif)
