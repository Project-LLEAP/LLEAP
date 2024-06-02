import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint, pv):
        error = setpoint - pv
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class UprightControlNode(Node):
    def __init__(self):
        super().__init__('upright_control_node')

        # PID controllers for roll and pitch
        self.roll_pid = PIDController(2.0, 0.0, 0.5)
        self.pitch_pid = PIDController(2.0, 0.0, 0.5)

        # Subscriber to IMU data
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)

        # Publisher to left and right leg controllers
        self.left_leg_publisher = self.create_publisher(
            JointTrajectory,
            '/left_leg_controller/joint_trajectory',
            10)
        self.right_leg_publisher = self.create_publisher(
            JointTrajectory,
            '/right_leg_controller/joint_trajectory',
            10)

        self.timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

        # Placeholder for the current orientation
        self.orientation = None

    def imu_callback(self, msg):
        self.orientation = msg.orientation

    def control_loop(self):
        if self.orientation is None:
            return

        # Compute the control commands based on the orientation data
        roll = self.get_roll(self.orientation)
        pitch = self.get_pitch(self.orientation)

        # Use PID controller to compute corrections
        roll_correction = self.roll_pid.compute(0.0, roll)
        pitch_correction = self.pitch_pid.compute(0.0, pitch)

        # Create joint trajectory messages for left and right leg
        left_leg_command = JointTrajectory()
        right_leg_command = JointTrajectory()

        left_leg_command.joint_names = ['left_hip_revolute_joint', 'left_knee_revolute_joint', 'left_ankle_revolute_joint']
        right_leg_command.joint_names = ['right_hip_revolute_joint', 'right_knee_revolute_joint', 'right_ankle_revolute_joint']

        point = JointTrajectoryPoint()
        point.positions = [roll_correction, 0.0, pitch_correction]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 10000000  # 10ms

        left_leg_command.points = [point]
        right_leg_command.points = [point]

        # Publish the commands
        self.left_leg_publisher.publish(left_leg_command)
        self.right_leg_publisher.publish(right_leg_command)

    def get_roll(self, orientation):
        # Convert quaternion to roll angle
        sinr_cosp = 2 * (orientation.w * orientation.x + orientation.y * orientation.z)
        cosr_cosp = 1 - 2 * (orientation.x * orientation.x + orientation.y * orientation.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        return roll

    def get_pitch(self, orientation):
        # Convert quaternion to pitch angle
        sinp = 2 * (orientation.w * orientation.y - orientation.z * orientation.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        return pitch

def main(args=None):
    rclpy.init(args=args)
    node = UprightControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
