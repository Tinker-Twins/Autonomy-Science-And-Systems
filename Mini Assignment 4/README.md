# Mini Assignment 4: TurtleBot3 Camera Calibration
**Authors:** Chinmay Samak and Tanmay Samak

## Disclaimer:
I certify that all the work and writing that I contributed to here is my own and not acquired from external sources. I have cited sources appropriately and paraphrased correctly. I have not shared my writing with other students (for individual assignments) and other students outside my group (for group assignments), nor have I acquired any written portion of this document from past or present students.

1. Launch camera image publisher node:
```bash
$ ros2 launch v4l2_camera camera.launch.py
```

2. Launch camera calibration node:
```bash
$ ros2 run camera_calibration cameracalibrator --camera_name arducam --size=8x6 --square=0.02 --approximate=0.3 --no-service-check --ros-args -r image:=/image
```
