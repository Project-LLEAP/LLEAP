#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        
        # Parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('topic_imu', '/imu/data')
        self.declare_parameter('left_leg_controller', '/left_leg_controller/joint_trajectory')
        self.declare_parameter('right_leg_controller', '/right_leg_controller/joint_trajectory')

        # Retrieve parameters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        topic_imu = self.get_parameter('topic_imu').value
        self.left_leg_controller = self.get_parameter('left_leg_controller').value
        self.right_leg_controller = self.get_parameter('right_leg_controller').value

        # State variables for PID
        self.last_orientation_error = 0.0
        self.integral_error = 0.0
        self.last_time = self.get_clock().now()

        # Subscribers and Publishers
        self.subscription = self.create_subscription(Imu, topic_imu, self.imu_callback, 10)
        self.left_leg_publisher = self.create_publisher(JointTrajectory, self.left_leg_controller, 10)
        self.right_leg_publisher = self.create_publisher(JointTrajectory, self.right_leg_controller, 10)

    def imu_callback(self, msg: Imu):
        # Extract pitch angle from IMU orientation (assuming quaternion)
        orientation = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        # Compute control error (desired pitch is 0 for upright)
        orientation_error = -pitch

        # Compute time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Compute PID terms
        self.integral_error += orientation_error * dt
        derivative_error = (orientation_error - self.last_orientation_error) / dt
        self.last_orientation_error = orientation_error

        # PID Control
        control_effort = self.kp * orientation_error + self.ki * self.integral_error + self.kd * derivative_error

        # Publish control command as joint trajectories
        left_leg_cmd = JointTrajectory()
        left_leg_cmd.joint_names = ['left_hip_revolute_joint', 'left_knee_revolute_joint', 'left_ankle_revolute_joint']
        point = JointTrajectoryPoint()
        point.positions = [control_effort, control_effort, control_effort]
        point.time_from_start.sec = 1
        left_leg_cmd.points.append(point)

        right_leg_cmd = JointTrajectory()
        right_leg_cmd.joint_names = ['right_hip_revolute_joint', 'right_knee_revolute_joint', 'right_ankle_revolute_joint']
        point = JointTrajectoryPoint()
        point.positions = [control_effort, control_effort, control_effort]
        point.time_from_start.sec = 1
        right_leg_cmd.points.append(point)

        self.left_leg_publisher.publish(left_leg_cmd)
        self.right_leg_publisher.publish(right_leg_cmd)

    def quaternion_to_euler(self, x, y, z, w):
        # Convert quaternion to Euler angles
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
