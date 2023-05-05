#!/usr/bin/env python3

# Copyright (c) 2023, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# ROS2 module imports
import rclpy # ROS2 client library (rcl) for Python (built on rcl C API)
from rclpy.node import Node # Node class for Python nodes
from geometry_msgs.msg import Twist # Twist (linear and angular velocities) message class
from sensor_msgs.msg import LaserScan # LaserScan (LIDAR range measurements) message class
from sensor_msgs.msg import Image # Image (camera frame) message class
from darknet_ros_msgs.msg import BoundingBoxes # BoundingBoxes (Tiny-YOLO object detections) message class
from apriltag_msgs.msg import AprilTagDetectionArray # AprilTagDetectionArray (AprilTag detections) message class
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Ouality of Service (tune communication between nodes)
from rclpy.qos import qos_profile_sensor_data # Ouality of Service for sensor data, using best effort reliability and small queue depth
from rclpy.duration import Duration # Time duration class
from tf2_ros.transform_listener import TransformListener # Transform (tf2) listener
from tf2_ros.buffer import Buffer # Transform buffer

# Python mudule imports
import numpy as np # Numpy
import queue # FIFO queue
import time # Tracking time
from math import inf # Common mathematical constant
import cv2 # OpenCV
from cv_bridge import CvBridge, CvBridgeError # OpenCV bridge for ROS2

# PID controller class
class PIDController:
    '''
    Generates control action taking into account instantaneous error (proportional action),
    accumulated error (integral action) and rate of change of error (derivative action).
    '''
    def __init__(self, kP, kI, kD, kS):
        self.kP       = kP # Proportional gain
        self.kI       = kI # Integral gain
        self.kD       = kD # Derivative gain
        self.kS       = kS # Saturation constant (error history buffer size)
        self.err_int  = 0 # Error integral
        self.err_dif  = 0 # Error difference
        self.err_prev = 0 # Previous error
        self.err_hist = queue.Queue(self.kS) # Limited buffer of error history
        self.t_prev   = 0 # Previous time

    def control(self, err, t):
        '''
        Generate PID controller output.
        :param err: Instantaneous error in control variable w.r.t. setpoint
        :param t  : Current timestamp
        :return u: PID controller output
        '''
        dt = t - self.t_prev # Timestep
        if dt > 0.0:
            self.err_hist.put(err) # Update error history
            self.err_int += err # Integrate error
            if self.err_hist.full(): # Jacketing logic to prevent integral windup
                self.err_int -= self.err_hist.get() # Rolling FIFO buffer
            self.err_dif = (err - self.err_prev) # Error difference
            u = (self.kP * err) + (self.kI * self.err_int * dt) + (self.kD * self.err_dif / dt) # PID control law
            self.err_prev = err # Update previos error term
            self.t_prev = t # Update timestamp
            return u # Control signal

# Node class
class RobotController(Node):

    #######################
    '''Class constructor'''
    #######################

    def __init__(self):
        # Information and debugging
        info = '\nMake the robot follow wall, avoid obstacles, follow line, detect stop sign and track AprilTag marker.\n'
        print(info)
        # ROS2 infrastructure
        super().__init__('robot_controller') # Create a node with name 'robot_controller'
        qos_profile = QoSProfile( # Ouality of Service profile
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, # Reliable (not best effort) communication
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, # Keep/store only up to last N samples
        depth=10 # Queue size/depth of 10 (only honored if the “history” policy was set to “keep last”)
        )
        self.robot_lidar_sub = self.create_subscription(LaserScan, '/scan', self.robot_lidar_callback, qos_profile_sensor_data) # Subscriber which will subscribe to LaserScan message on the topic '/scan' adhering to 'qos_profile_sensor_data' QoS profile
        self.robot_lidar_sub # Prevent unused variable warning
        self.robot_camera_sub = self.create_subscription(Image, 'image/uncompressed', self.robot_camera_callback, qos_profile) # Subscriber which will subscribe to Image message on the topic '/camera/image_raw' adhering to 'qos_profile' QoS profile
        self.robot_camera_sub # Prevent unused variable warning
        self.robot_yolo_sub = self.create_subscription(BoundingBoxes, '/darknet_ros/bounding_boxes', self.robot_yolo_callback, qos_profile) # Subscriber which will subscribe to BoundingBoxes message on the topic '/darknet_ros/bounding_boxes' adhering to 'qos_profile' QoS profile
        self.robot_yolo_sub # Prevent unused variable warning
        self.robot_marker_sub = self.create_subscription(AprilTagDetectionArray, '/detections', self.robot_marker_callback, qos_profile) # Subscriber which will subscribe to AprilTagDetectionArray message on the topic '/detections' adhering to 'qos_profile' QoS profile
        self.robot_marker_sub # Prevent unused variable warning
        self.tf_buffer = Buffer() # Transform buffer
        self.tf_listener = TransformListener(self.tf_buffer, self) # Transform listener
        self.robot_ctrl_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile) # Publisher which will publish Twist message to the topic '/cmd_vel' adhering to 'qos_profile' QoS profile
        timer_period = 0.001 # Node execution time period (seconds)
        self.timer = self.create_timer(timer_period, self.robot_controller_callback) # Define timer to execute 'robot_controller_callback()' every 'timer_period' seconds
        self.pid_1_lat = PIDController(0.69, 0.01, 0.3, 10) # Lateral PID controller object initialized with kP, kI, kD, kS (for Task 1: Wall Following & Obstacle Avoidance)
        self.pid_1_lon = PIDController(0.11, 0.001, 0.01, 10) # Longitudinal PID controller object initialized with kP, kI, kD, kS (for Task 1: Wall Following & Obstacle Avoidance)
        self.pid_2     = PIDController(0.86, 0.16, 0.14, 50) # PID controller object initialized with kP, kI, kD, kS (for Task 2: Line Following & Stop Sign Detection)
        self.pid_3_lon = PIDController(0.14, 0.001, 0.05, 10) # Longitudinal PID controller object initialized with kP, kI, kD, kS (for Task 3: AprilTag Tracking)
        self.pid_3_lat = PIDController(2.5, 0.01, 0.2, 10) # Lateral PID controller object initialized with kP, kI, kD, kS (for Task 3: AprilTag Tracking)
        self.lidar_available = False # Initialize LIDAR data available flag to false
        self.camera_available = False # Initialize camera data available flag to false
        self.detection_available = False # Initialize variable to capture availability of YOLO detection
        self.marker_available = False # Initialize variable to capture availability of marker detection
        self.laserscan = None # Initialize variable to capture the laserscan
        self.cv_bridge = CvBridge() # Initialize object to capture and convert the image
        self.cv_image = None # Initialize variable to capture the image
        self.following_line = False # Flag to indicate line following operation
        self.detection = None # Initialize variable to capture the YOLO detection
        self.detected_stop_sign = False # Flag to indicate detection of stop sign
        self.obeying_stop_sign = False # Flag to indicate ongoing obedience of stop sign
        self.obeyed_stop_sign = False # Flag to indicate successful obedience of stop sign
        self.tracking_apriltag = False # Flag to indicate tracking of AprilTag marker
        self.ctrl_msg = Twist() # Initialize variable to capture the control commands (twist)
        self.stop_time = inf # Time elapsed since complete stop
        self.start_mode = 'outside' # Robot start mode (inside/outside wall arena)
        self.start_time = self.get_clock().now() # Record current time in seconds

    ########################
    '''Callback functions'''
    ########################

    def robot_lidar_callback(self, msg):
        self.laserscan = np.asarray(msg.ranges) # Capture most recent laserscan
        self.laserscan[self.laserscan == 0.0] = inf # Compensate for inf range returning 0.0
        self.laserscan[self.laserscan >= 3.5] = 3.5 # Filter laserscan data based on maximum range
        self.lidar_available = True # Set data available flag to true

    def robot_camera_callback(self, msg):
        try:
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8") # Capture and convert most recent image 'msg' to OpenCV image with 'bgr8' encoding
            self.camera_available = True
        except CvBridgeError as error:
             print(error)
    
    def robot_yolo_callback(self, msg):
        self.detection = msg.bounding_boxes[0] # Capture most recent YOLO detection
        self.detection_available = True # Set YOLO detection flag to available
    
    def robot_marker_callback(self, msg):
        self.marker = msg.detections # Capture most recent marker detection
        self.marker_available = True # Set marker detection flag to available

    def robot_controller_callback(self):
        THRESH = 1000 # Stop sign threshold area to come to a complete stop (px squared)
        DELAY = 4.0 # Time delay (s)
        if self.get_clock().now() - self.start_time > Duration(seconds=DELAY):
            if self.lidar_available and self.camera_available: # Proceed only if required data is available
                ###################
                # AprilTag Tracking
                ###################
                if not self.obeying_stop_sign and not self.following_line:
                    to_frame_rel = 'camera'
                    from_frame_rel = 'tag36h11_0'
                    if len(self.marker) != 0:
                        try:
                            tf2_msg = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
                            lon_error = tf2_msg.transform.translation.z # Calculate longitudinal error w.r.t. AprilTag marker
                            lat_error = -tf2_msg.transform.translation.x # Calculate lateral error w.r.t. AprilTag marker
                            tstamp = time.time() # Current timestamp (s)
                            if lon_error >= 0.6: # Keep tracking if the marker is far
                                LIN_VEL = self.pid_3_lon.control(lon_error, tstamp) # Linear velocity (m/s)
                            else: # Stop if the marker is close enough
                                LIN_VEL = 0.0 # Linear velocity (m/s)
                            ANG_VEL = self.pid_3_lat.control(lat_error, tstamp) # Angular velocity (rad/s)
                            self.ctrl_msg.linear.x = min(0.22, float(LIN_VEL)) # Set linear velocity
                            self.ctrl_msg.angular.z = min(2.84, float(ANG_VEL)) # Set angular velocity
                            self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message
                            self.tracking_apriltag = True
                            print('AprilTag Tracking Mode')
                        except:
                            self.tracking_apriltag = False
                            pass
                    else:
                        self.tracking_apriltag = False
                #####################
                # Stop Sign Detection
                #####################
                if self.detection_available:
                    # Perception
                    detection = self.detection.class_id # Class ID (label) of YOLO object detection
                    probability = self.detection.probability # Probability (confidence) of prediction
                    box_length = self.detection.xmax - self.detection.xmin # Length of bounding box
                    box_height = self.detection.ymax - self.detection.ymin # Height of bounding box
                    box_area = box_length*box_height # Area of bounding box
                    # Planning
                    # Stop for some time if stop sign detected within thresholded distance
                    if self.obeyed_stop_sign==False and detection=='stop sign' and probability>0.5 and box_area>THRESH:
                        print('Stop Sign Detection Mode')
                        # Control
                        self.ctrl_msg.linear.x = 0.0 # Set linear velocity
                        self.ctrl_msg.angular.z = 0.0 # Set angular velocity
                        self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message
                        # Start timer
                        if self.detected_stop_sign==False:
                            self.stop_time = self.get_clock().now() # Record current time in seconds
                            self.detected_stop_sign = True
                            self.obeying_stop_sign = True
                        # Stop at stop sign for `DELAY` amount of seconds
                        if self.get_clock().now() - self.stop_time > Duration(seconds=DELAY):
                            self.obeyed_stop_sign = True
                            self.obeying_stop_sign = False
                ################
                # LINE FOLLOWING
                ################
                # Perception
                width, height, channels = self.cv_image.shape # Get image shape (width, height, channels)
                crop = self.cv_image[int((width/2)+110):int((width/2)+120)][1:int(height)] # Crop unwanted parts of the image
                hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV) # Convert from RGB to HSV color space
                # Night
                # lower_yellow = np.array([0, 0, 160]) # Lower HSV threshold for light yellow color
                # upper_yellow = np.array([131, 255, 255]) # Upper HSV threshold for light yellow color
                # Morning (10:00 AM)
                # lower_yellow = np.array([30, 120, 120]) # Lower HSV threshold for light yellow color
                # upper_yellow = np.array([90, 255, 255]) # Upper HSV threshold for light yellow color
                # Noon (12:00 PM)
                # lower_yellow = np.array([30, 120, 100]) # Lower HSV threshold for light yellow color
                # upper_yellow = np.array([90, 255, 255]) # Upper HSV threshold for light yellow color
                # Robust
                lower_yellow = np.array([30, 100, 70]) # Lower HSV threshold for light yellow color
                upper_yellow = np.array([90, 255, 255]) # Upper HSV threshold for light yellow color
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow) # Threshold the HSV image to mask everything but yellow color
                self.following_line = False
                if mask.any() and (self.laserscan[0]>=1.0 or self.laserscan[180]>=1.0) and (self.laserscan[90]>=1.0 or self.laserscan[270]>=1.0) and not self.obeying_stop_sign:
                    self.following_line = True
                    m = cv2.moments(mask, False) # Calculate moments (weighted average of image pixel intensities) of binary image
                    try:
                        cx, cy = m['m10']/m['m00'], m['m01']/m['m00'] # Calculate centroid of the blob using moments
                    except ZeroDivisionError:
                        cx, cy = width/2, height/2 # Calculate centroid of the blob as image center
                    # cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255), -1) # Add centroid to masked frame
                    # cv2.imshow("Camera Frame", self.cv_image) # Show camera frame
                    # cv2.imshow("Cropped Frame", crop) # Show cropped frame
                    # cv2.imshow("Masked Frame", mask) # Show masked frame
                    # cv2.imshow("Line Detection", cv2.resize(mask, (int(mask.shape[1]*5.775), int(mask.shape[0]*5.775)))) # Show enlarged masked frame
                    # cv2.waitKey(1)
                    # Planning
                    error = (width/2 - cx + 10)/175 # Calculate error (deviation) from line center
                    tstamp = time.time() # Current timestamp (s)
                    # Control
                    LIN_VEL = 0.05 # Linear velocity (m/s)
                    ANG_VEL = self.pid_2.control(error, tstamp) # Angular velocity (rad/s)
                    print('Line Following Mode')
                    self.ctrl_msg.linear.x = min(0.22, float(LIN_VEL)) # Set linear velocity
                    self.ctrl_msg.angular.z = min(2.84, float(ANG_VEL)) # Set angular velocity
                    self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message
                #####################################
                # WALL FOLLOWING & OBSTACLE AVOIDANCE
                #####################################
                elif not self.following_line and not self.obeying_stop_sign and not self.tracking_apriltag:
                    # Front sector ranging
                    front_sector = 20 # Angular range (deg)
                    front = (np.mean(self.laserscan[0:front_sector])+np.mean(self.laserscan[360-front_sector:360]))/2 # Frontal distance to collision (DTC)
                    # Oblique sector ranging
                    oblique_sector = 20 # Angular range (deg)
                    oblique_left = np.mean(self.laserscan[15:oblique_sector]) # Oblique left DTC
                    oblique_right = np.mean(self.laserscan[345-oblique_sector:345])  # Oblique right DTC
                    # Side sector ranging
                    side_sector = 40 # Angular range (deg)
                    left = np.mean(self.laserscan[60:60+side_sector]) # Left DTC
                    right = np.mean(self.laserscan[300-side_sector:300]) # Right DTC
                    # Control logic
                    tstamp = time.time() # Current timestamp (s)
                    if (self.laserscan[90]>=1.0 and self.laserscan[270]>=1.0) and self.start_mode=='outside': # No walls on the sides
                        LIN_VEL = 0.1 # Linear velocity (m/s)
                        ANG_VEL = 0 # Angular velocity (rad/s)
                        print('Wall Following Mode')
                    elif oblique_left < 0.25 or oblique_right < 0.25: # Too close to obstacle(s)
                        LIN_VEL = 0.0 # Linear velocity (m/s)
                        ANG_VEL = self.pid_1_lat.control(8*(oblique_left-oblique_right)*(1/front), tstamp) # Angular velocity (rad/s) from PID controller
                        self.start_mode = 'inside'
                        print('Obstacle Avoidance Mode')
                    elif (oblique_left >= 0.25 and oblique_left < 0.8) or (oblique_right > 0.25 and oblique_right < 0.8): # Fairly away from walls/obstacles
                        LIN_VEL = self.pid_1_lon.control(front, tstamp) # Linear velocity (m/s) from PID controller
                        ANG_VEL = self.pid_1_lat.control(4*(oblique_left-oblique_right)*(1/front), tstamp) # Angular velocity (rad/s) from PID controller
                        self.start_mode = 'inside'
                        print('Obstacle Avoidance Mode')
                    else: # Safely away from walls/obstacles
                        LIN_VEL = 0.1 # Linear velocity (m/s)
                        ANG_VEL = self.pid_1_lat.control((left-right)*(1/front), tstamp) # Angular velocity (rad/s) from PID controller
                        self.start_mode = 'inside'
                        print('Wall Following Mode')
                    self.ctrl_msg.linear.x = min(0.22, float(LIN_VEL)) # Set linear velocity
                    self.ctrl_msg.angular.z = min(2.84, float(ANG_VEL)) # Set angular velocity
                    self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message
        else:
            print('Initializing...')

def main(args=None):
    rclpy.init(args=args) # Start ROS2 communications
    node = RobotController() # Create node
    rclpy.spin(node) # Execute node
    node.destroy_node() # Destroy node explicitly (optional - otherwise it will be done automatically when garbage collector destroys the node object)
    rclpy.shutdown() # Shutdown ROS2 communications

if __name__ == "__main__":
    main()
