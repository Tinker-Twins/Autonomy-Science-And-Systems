1. Launch camera image publisher node:
```bash
$ ros2 launch v4l2_camera camera.launch.py
```

2. Launch camera calibration node:
```bash
$ ros2 run camera_calibration cameracalibrator --camera_name arducam --size=8x6 --square=0.02 --approximate=0.3 --no-service-check --ros-args -r image:=/image
```
