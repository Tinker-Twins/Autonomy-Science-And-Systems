#!/usr/bin/env python3

# ROS2 module imports
import rclpy # ROS2 client library (rcl) for Python (built on rcl C API)
from rclpy.node import Node # Node class for Python nodes
from geometry_msgs.msg import Twist # Twist (linear and angular velocities) message class
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Ouality of Service (tune communication between nodes)
from rclpy.duration import Duration # Time duration class

# Node class
class RobotController(Node):

    #######################
    '''Class constructor'''
    #######################

    def __init__(self):
        # Information and debugging
        info = '''\nMake the robot trace a square with open-loop control.\n'''
        print(info)
        # ROS2 infrastructure
        super().__init__('robot_controller') # Create a node with name 'robot_controller'
        qos_profile = QoSProfile( # Ouality of Service profile
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, # Reliable (not best effort) communication
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, # Keep/store only up to last N samples
        depth=10 # Queue size/depth of 10 (only honored if the “history” policy was set to “keep last”)
        )
        self.robot_ctrl_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile) # Publisher which will publish Twist message to the topic '/turtle1/cmd_vel' adhering to 'qos_profile' QoS profile
        timer_period = 0.01 # Node execution time period (seconds)
        self.timer = self.create_timer(timer_period, self.robot_controller_callback) # Define timer to execute 'robot_controller_callback()' every 'timer_period' seconds
        self.ctrl_msg = Twist() # Robot control commands (twist)    
        self.start_time = self.get_clock().now() # Record current time in seconds

    #######################
    '''Callback function'''
    #######################

    def robot_controller_callback(self):
        # Robot going to 1st vertex by moving 2 m with 0.2 m/s velocity
        if self.get_clock().now() - self.start_time < Duration(seconds=10):
            print('Robot going to 1st vertex by moving 2 m with 0.2 m/s velocity')
            self.ctrl_msg.linear.x = 0.2
            self.ctrl_msg.angular.z = 0.0
        # Robot preparing for 2nd vertex by rotating 90 degrees with 0.2 rad/s velocity
        if self.get_clock().now() - self.start_time > Duration(seconds=10) and self.get_clock().now() - self.start_time <= Duration(seconds=17.853981634):
            print('Robot preparing for 2nd vertex by rotating 90 degrees with 0.2 rad/s velocity')
            self.ctrl_msg.linear.x = 0.0
            self.ctrl_msg.angular.z = 0.2
        # Robot going to 2nd vertex by moving 2 m with 0.2 m/s velocity
        if self.get_clock().now() - self.start_time > Duration(seconds=17.853981634) and self.get_clock().now() - self.start_time <= Duration(seconds=27.853981634):
            print('Robot going to 2nd vertex by moving 2 m with 0.2 m/s velocity')
            self.ctrl_msg.linear.x = 0.2
            self.ctrl_msg.angular.z = 0.0
        # Robot preparing for 3rd vertex by rotating 90 degrees with 0.2 rad/s velocity
        if self.get_clock().now() - self.start_time > Duration(seconds=27.853981634) and self.get_clock().now() - self.start_time <= Duration(seconds=35.707963268):
            print('Robot preparing for 3rd vertex by rotating 90 degrees with 0.2 rad/s velocity')
            self.ctrl_msg.linear.x = 0.0
            self.ctrl_msg.angular.z = 0.2
        # Robot going to 3rd vertex by moving 2 m with 0.2 m/s velocity
        if self.get_clock().now() - self.start_time > Duration(seconds=35.707963268) and self.get_clock().now() - self.start_time <= Duration(seconds=45.707963268):
            print('Robot going to 3rd vertex by moving 2 m with 0.2 m/s velocity')
            self.ctrl_msg.linear.x = 0.2
            self.ctrl_msg.angular.z = 0.0
        # Robot preparing for 4th vertex by rotating 90 degrees with 0.2 rad/s velocity
        if self.get_clock().now() - self.start_time > Duration(seconds=45.707963268) and self.get_clock().now() - self.start_time <= Duration(seconds=53.561944902):
            print('Robot preparing for 4th vertex by rotating 90 degrees with 0.2 rad/s velocity')
            self.ctrl_msg.linear.x = 0.0
            self.ctrl_msg.angular.z = 0.2
        # Robot going to 4th vertex by moving 2 m with 0.2 m/s velocity
        if self.get_clock().now() - self.start_time > Duration(seconds=53.561944902) and self.get_clock().now() - self.start_time <= Duration(seconds=63.561944902):
            print('Robot going to 4th vertex by moving 2 m with 0.2 m/s velocity')
            self.ctrl_msg.linear.x = 0.2
            self.ctrl_msg.angular.z = 0.0
        # Robot resetting to initial configuration by rotating 90 degrees with 0.2 rad/s velocity
        if self.get_clock().now() - self.start_time > Duration(seconds=63.561944902) and self.get_clock().now() - self.start_time <= Duration(seconds=71.415926536):
            print('Robot resetting to initial configuration by rotating 90 degrees with 0.2 rad/s velocity')
            self.ctrl_msg.linear.x = 0.0
            self.ctrl_msg.angular.z = 0.2
        # Stop robot controller execution
        if self.get_clock().now() - self.start_time > Duration(seconds=71.415926536):
            quit() # Quit execution
        self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message

def main(args=None):
    rclpy.init(args=args) # Start ROS2 communications
    node = RobotController() # Create node
    rclpy.spin(node) # Execute node
    node.destroy_node() # Destroy node explicitly (optional - otherwise it will be done automatically when garbage collector destroys the node object)
    rclpy.shutdown() # Shutdown ROS2 communications

if __name__ == "__main__":
    main()