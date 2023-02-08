#!/usr/bin/env python3

# ROS2 module imports
import rclpy # ROS2 client library (rcl) for Python (built on rcl C API)
from rclpy.node import Node # Node class for Python nodes
from geometry_msgs.msg import Twist # Twist (linear and angular velocities) message class
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Ouality of Service (tune communication between nodes)
from rclpy.duration import Duration # Time duration class

# Python mudule imports
from math import pi # Common mathematical constant

# Node class
class RobotController(Node):

    #######################
    '''Class constructor'''
    #######################

    def __init__(self):
        # Information and debugging
        info = '\nMake the robot go in a square with open-loop control.\n'
        print(info)
        # ROS2 infrastructure
        super().__init__('robot_controller') # Create a node with name 'robot_controller'
        qos_profile = QoSProfile( # Ouality of Service profile
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, # Reliable (not best effort) communication
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, # Keep/store only up to last N samples
        depth=10 # Queue size/depth of 10 (only honored if the “history” policy was set to “keep last”)
        )
        self.robot_ctrl_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile) # Publisher which will publish Twist message to the topic '/turtle1/cmd_vel' adhering to 'qos_profile' QoS profile
        timer_period = 0.001 # Node execution time period (seconds)
        self.timer = self.create_timer(timer_period, self.robot_controller_callback) # Define timer to execute 'robot_controller_callback()' every 'timer_period' seconds
        self.ctrl_msg = Twist() # Robot control commands (twist)
        self.declare_parameter('lin_vel') # Linear velocity of robot along local x-axis
        self.declare_parameter('ang_vel') # Angular velocity of robot about local z-axis
        self.start_time = self.get_clock().now() # Record current time in seconds

    #######################
    '''Callback function'''
    #######################

    def robot_controller_callback(self):
        LIN_VEL = self.get_parameter('lin_vel').value # Linear velocity parameter (m/s)
        ANG_VEL = self.get_parameter('ang_vel').value # Angular velocity parameter (rad/s)
        LIN_DIS = 2.0 # Square side length (m)
        ANG_DIS = pi/2 # Square side length (m)
        LIN_TIM = LIN_DIS/LIN_VEL # Base time duration for linear motion (s)
        ANG_TIM = ANG_DIS/ANG_VEL # Base time duration for angular motion (s)
        DELAY = 4.0 # Time delay (s)
        if self.get_clock().now() - self.start_time > Duration(seconds=DELAY):
            # Robot going to 1st vertex by moving 2 m with LIN_VEL m/s velocity
            if self.get_clock().now() - self.start_time < Duration(seconds=DELAY+LIN_TIM):
                print('Robot going to 1st vertex by moving 2 m with {} m/s velocity'.format(LIN_VEL))
                self.ctrl_msg.linear.x = LIN_VEL
                self.ctrl_msg.angular.z = 0.0
            # Robot preparing for 2nd vertex by rotating 90 degrees with ANG_VEL rad/s velocity
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+LIN_TIM) and self.get_clock().now() - self.start_time <= Duration(seconds=DELAY+LIN_TIM+ANG_TIM):
                print('Robot preparing for 2nd vertex by rotating 90 degrees with {} rad/s velocity'.format(ANG_VEL))
                self.ctrl_msg.linear.x = 0.0
                self.ctrl_msg.angular.z = ANG_VEL
            # Robot going to 2nd vertex by moving 2 m with LIN_VEL m/s velocity
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+LIN_TIM+ANG_TIM) and self.get_clock().now() - self.start_time <= Duration(seconds=DELAY+2*LIN_TIM+ANG_TIM):
                print('Robot going to 2nd vertex by moving 2 m with {} m/s velocity'.format(LIN_VEL))
                self.ctrl_msg.linear.x = LIN_VEL
                self.ctrl_msg.angular.z = 0.0
            # Robot preparing for 3rd vertex by rotating 90 degrees with ANG_VEL rad/s velocity
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+2*LIN_TIM+ANG_TIM) and self.get_clock().now() - self.start_time <= Duration(seconds=DELAY+2*LIN_TIM+2*ANG_TIM):
                print('Robot preparing for 3rd vertex by rotating 90 degrees with {} rad/s velocity'.format(ANG_VEL))
                self.ctrl_msg.linear.x = 0.0
                self.ctrl_msg.angular.z = ANG_VEL
            # Robot going to 3rd vertex by moving 2 m with LIN_VEL m/s velocity
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+2*LIN_TIM+2*ANG_TIM) and self.get_clock().now() - self.start_time <= Duration(seconds=DELAY+3*LIN_TIM+2*ANG_TIM):
                print('Robot going to 3rd vertex by moving 2 m with {} m/s velocity'.format(LIN_VEL))
                self.ctrl_msg.linear.x = LIN_VEL
                self.ctrl_msg.angular.z = 0.0
            # Robot preparing for 4th vertex by rotating 90 degrees with ANG_VEL rad/s velocity
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+3*LIN_TIM+2*ANG_TIM) and self.get_clock().now() - self.start_time <= Duration(seconds=DELAY+3*LIN_TIM+3*ANG_TIM):
                print('Robot preparing for 4th vertex by rotating 90 degrees with {} rad/s velocity'.format(ANG_VEL))
                self.ctrl_msg.linear.x = 0.0
                self.ctrl_msg.angular.z = ANG_VEL
            # Robot going to 4th vertex by moving 2 m with LIN_VEL m/s velocity
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+3*LIN_TIM+3*ANG_TIM) and self.get_clock().now() - self.start_time <= Duration(seconds=DELAY+4*LIN_TIM+3*ANG_TIM):
                print('Robot going to 4th vertex by moving 2 m with {} m/s velocity'.format(LIN_VEL))
                self.ctrl_msg.linear.x = LIN_VEL
                self.ctrl_msg.angular.z = 0.0
            # Robot resetting to initial configuration by rotating 90 degrees with ANG_VEL rad/s velocity
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+4*LIN_TIM+3*ANG_TIM) and self.get_clock().now() - self.start_time <= Duration(seconds=DELAY+4*LIN_TIM+4*ANG_TIM):
                print('Robot resetting to initial configuration by rotating 90 degrees with {} rad/s velocity'.format(ANG_VEL))
                self.ctrl_msg.linear.x = 0.0
                self.ctrl_msg.angular.z = ANG_VEL
            # Stop robot and quit controller execution
            if self.get_clock().now() - self.start_time > Duration(seconds=DELAY+4*LIN_TIM+4*ANG_TIM):
                self.ctrl_msg.linear.x = 0.0
                self.ctrl_msg.angular.z = 0.0
                self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message
                quit() # Quit execution
            self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message
        else:
            print('Initializing simulation...')

def main(args=None):
    rclpy.init(args=args) # Start ROS2 communications
    node = RobotController() # Create node
    rclpy.spin(node) # Execute node
    node.destroy_node() # Destroy node explicitly (optional - otherwise it will be done automatically when garbage collector destroys the node object)
    rclpy.shutdown() # Shutdown ROS2 communications

if __name__ == "__main__":
    main()