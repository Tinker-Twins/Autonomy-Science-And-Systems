# V4L2 Camera - ROS 2 Foxy
Camera Frame Grabbing with Video4Linux2 (V4L2) for ROS 2 Foxy

## Clone

First, we will create a new workspace and clone the code for the `v4l2_camera` package and some of its dependencies, and will install any further dependencies using `rosdep`:

```bash
# Open new terminal.
$ mkdir -p ~/V4L2_Camera/src && cd ~/V4L2_Camera/src
$ git clone https://github.com/Tinker-Twins/V4L2_Camera.git
# Clone following repositories instead for default functionalities.
# $ git clone --branch foxy https://gitlab.com/boldhearts/ros2_v4l2_camera.git
# $ git clone --branch foxy https://github.com/ros-perception/vision_opencv.git
# $ git clone --branch foxy https://github.com/ros-perception/image_common.git
# $ git clone --branch foxy-devel https://github.com/ros-perception/image_transport_plugins.git
$ cd ..
$ rosdep install --from-paths src -r -y
```

## Build

Now build everything we need and source the new workspace:
```bash
$ colcon build --packages-up-to v4l2_camera image_transport_plugins
$ source install/local_setup.bash
```

## Run

And finally you can run the camera node:
```bash
$ # Open new terminal.
$ ros2 run v4l2_camera v4l2_camera_node
```
NOTE: Ignore the following errors/warnings.
```bash
[ERROR] [1676909521.403530709] [camera_calibration_parsers]: Unable to open camera calibration file [/home/user/.ros/camera_info/integrated_webcam_hd:_integrate.yaml]
[WARN] [1676909521.403548765] [v4l2_camera]: Camera calibration file /home/user/.ros/camera_info/integrated_webcam_hd:_integrate.yaml not found
```

View the camera output, for instance by running the RQT image viewer:
```bash
$ # Open new terminal.
$ ros2 run rqt_image_view rqt_image_view
```
When doing so, select the /image/compressed topic rather than the base /image so that the images are streamed much more efficiently (at a higher rate).

## Launch

**OPTION 1:** Launch the camera node and separately view the camera output, for instance by running the RQT image viewer.
```bash
$ # Open new terminal.
$ ros2 launch v4l2_camera camera.launch.py
$ # Open new terminal.
$ ros2 run rqt_image_view rqt_image_view
```
NOTE 1: Ignore the following errors/warnings.
```bash
[ERROR] [1676909521.403530709] [camera_calibration_parsers]: Unable to open camera calibration file [/home/user/.ros/camera_info/integrated_webcam_hd:_integrate.yaml]
[WARN] [1676909521.403548765] [v4l2_camera]: Camera calibration file /home/user/.ros/camera_info/integrated_webcam_hd:_integrate.yaml not found
```
NOTE 2: Select the /image/compressed topic rather than the base /image so that the images are streamed much more efficiently (at a higher rate).

**OPTION 2:** Launch the camera node and view the camera output.
```bash
# Open new terminal.
$ ros2 launch v4l2_camera camera_view.launch.py
```
NOTE: Select the /image/compressed topic rather than the base /image so that the images are streamed much more efficiently (at a higher rate).

## Parameters

The driver exposes the controls supported by the camera as ROS parameters and all of them can be listed with:
```bash
$ ros2 param list /camera_node
```

Further, the ROS 2 CLI can be used to see what these parameters are for and which values are available and what they mean, with `ros2 param describe` command. One can get and set their values dynamically with `ros2 param get` and `ros2 param set` commands.
```bash
$ # Describe parameter
$ ros2 param describe /camera_node image_size
$ # Get parameter
$ ros2 param get /camera_node image_size
$ # Set parameter
$ ros2 param set /camera_node image_size [320,240]
```

**Note:** The following command can be used to check supported resolutions i.e. `image_size` of a particular video device:
```bash
$ sudo v4l2-ctl -d /dev/video0 --list-formats-ex
```

## Calibration

The camera calibration `yaml` file can be placed in the `$HOME/.ros/camera_info` directory renamed and populated with the correct camera name.

## Reference

https://gitlab.com/boldhearts/ros2_v4l2_camera \
https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304
