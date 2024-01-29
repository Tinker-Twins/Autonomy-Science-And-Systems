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
$x_{distorted} = x( 1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$
</p>
<p align="center">
$y_{distorted} = y( 1 + k_1 r^2 + k_2 r^4 + k_3 r^6)$
</p>

<p align="justify">
Similarly, tangential distortion occurs because the image-taking lense is not aligned perfectly parallel to the imaging plane. So, some areas in the image may look nearer than expected. The amount of tangential distortion can be represented as below:
</p>

<p align="center">
$x_{distorted} = x + [ 2p_1xy + p_2(r^2+2x^2)]$
</p>
<p align="center">
$y_{distorted} = y + [ p_1(r^2+ 2y^2)+ 2p_2xy]$
</p>

<p align="justify">
In short, we need to find five parameters, known as distortion coefficients given by:
</p>

<p align="center">
$D = [k_1 \hspace{10pt} k_2 \hspace{10pt} p_1 \hspace{10pt} p_2 \hspace{10pt} k_3]$
</p>

#### Camera Intrinsic Parameters

<p align="justify">
Intrinsic parameters are specific to a camera. They include information like focal length $(f_x,f_y)$ and optical center $(c_x,c_y)$. The focal length and optical center can be used to create a camera matrix, which can be used to remove distortion due to the lens of a specific camera. The camera matrix is unique to a specific camera, so once calculated, it can be reused on other images captured by the same camera. It is expressed as a $3 \times 3$ matrix:
</p>

<p align="center">
   $$K=\left[\begin{matrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{matrix}\right]$$
</p>

#### Camera Extrinsic Parameters

<p align="justify">
Extrinsic parameters correspond to rotation matrix and translation vector, $R$ and $T$ respectively, which transform 3D coordinates of a point in world frame to camera coordinate system.

The resultant projective mapping $M$ from world coordinates $[x \hspace{10pt} y \hspace{10pt} z]^T$ to image (pixel) coordinates $[u \hspace{10pt} v]^T$ can be ultimately described by:
</p>

<p align="center">
   $$\left[\begin{matrix} u \\ v \end{matrix}\right] = K [R \hspace{10pt} T] \left[\begin{matrix} x \\ y \\ z \end{matrix}\right] = M \left[\begin{matrix} x \\ y \\ z \end{matrix}\right]$$
</p>

#### Resources
- https://github.com/Tinker-Twins/Camera-Calibration
- https://en.wikipedia.org/wiki/Camera_resectioning
- https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- https://dsp.stackexchange.com/questions/6055/how-does-resizing-an-image-affect-the-intrinsic-camera-matrix

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
# $ sudo apt install v4l-utils
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
