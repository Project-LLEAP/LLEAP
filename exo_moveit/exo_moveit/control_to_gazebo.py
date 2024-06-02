#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration

class CmdVelToJoint(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_joint')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher_ = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)
        self.joint_names = [
            'left_hip_revolute_joint', 'left_knee_revolute_joint', 'left_ankle_revolute_joint',
            'right_hip_revolute_joint', 'right_knee_revolute_joint', 'right_ankle_revolute_joint'
        ]
        self.current_positions = {joint: 0.0 for joint in self.joint_names}
        self.last_time = self.get_clock().now()

        self.get_logger().info('CmdVelToJoint node initialized')

    def cmd_vel_callback(self, msg):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time

        # Update positions based on the velocity command and time difference
        self.current_positions['left_hip_revolute_joint'] = 0.1
        self.current_positions['right_hip_revolute_joint'] = 0.1
        self.current_positions['left_knee_revolute_joint'] = 0.1
        self.current_positions['right_knee_revolute_joint'] = 0.1
        self.current_positions['left_ankle_revolute_joint'] = 0.1
        self.current_positions['right_ankle_revolute_joint'] = 0.1

        # Log the current positions for debugging
        self.get_logger().info(f'Current positions: {self.current_positions}')

        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [
            self.current_positions['left_hip_revolute_joint'],
            self.current_positions['left_knee_revolute_joint'],
            self.current_positions['left_ankle_revolute_joint'],
            self.current_positions['right_hip_revolute_joint'],
            self.current_positions['right_knee_revolute_joint'],
            self.current_positions['right_ankle_revolute_joint']
        ]
        point.time_from_start = Duration(seconds=1).to_msg()  # Update time duration as necessary

        joint_trajectory_msg.points.append(point)
        joint_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        joint_trajectory_msg.header.frame_id = 'base_link'

        self.publisher_.publish(joint_trajectory_msg)
        self.get_logger().info(f'Published joint trajectory: {joint_trajectory_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
