#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time

class JointStateSerialPublisher(Node):

    def __init__(self):
        super().__init__('joint_state_serial_publisher')
        self.subscription = self.create_subscription(
            JointState,
            '/moveit_joint_states',  # Replace with your actual topic name
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Set up serial connection
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust the port and baud rate as needed
        self.timer = self.create_timer(1.0, self.timer_callback)  # Publish every second
        self.latest_joint_state = None

    def listener_callback(self, msg):
        self.latest_joint_state = msg

    def timer_callback(self):
        if self.latest_joint_state is not None:
            # Convert joint states to a string format
            joint_positions = ','.join(map(str, self.latest_joint_state.position))
            self.ser.write(joint_positions.encode('utf-8'))
            self.get_logger().info(f'Published: {joint_positions}')
        
def main(args=None):
    rclpy.init(args=args)
    node = JointStateSerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
