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
        info = '\nMake the robot go in a circle with open-loop control.\n'
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
        # Make the robot go in a circle
        LIN_VEL = self.get_parameter('lin_vel').value # Linear velocity parameter (m/s)
        ANG_VEL = self.get_parameter('ang_vel').value # Angular velocity parameter (rad/s)
        RADIUS = LIN_VEL/ANG_VEL # Circle radius
        DELAY = 4.0 # Time delay (s)
        if self.get_clock().now() - self.start_time > Duration(seconds=DELAY):
            if self.get_clock().now() - self.start_time < Duration(seconds=DELAY+2*pi/ANG_VEL):
                self.ctrl_msg.linear.x = LIN_VEL # Set linear velocity
                self.ctrl_msg.angular.z = ANG_VEL # Set angular velocity
                self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message
                print('Robot going in a circle of radius {} m'.format(RADIUS))
            # Stop robot controller execution
            else:
                self.ctrl_msg.linear.x = 0.0 # Set linear velocity
                self.ctrl_msg.angular.z = 0.0 # Set angular velocity
                self.robot_ctrl_pub.publish(self.ctrl_msg) # Publish robot controls message
                quit() # Quit execution
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