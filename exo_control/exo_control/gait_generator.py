#!/usr/bin/env python3
"""
gait_manager.py  –  closed-loop gait & balance publisher
Publishes hip & knee **velocity** commands at 200 Hz.

Subscriptions
-------------
/joy                   sensor_msgs/Joy         – virtual game-pad
/exo/state_estimate    sensor_msgs/Imu         – UKF pose (quat + lin/ang vel in world)
/exo/com               geometry_msgs/PointStamped – UKF CoM position (world)
/exo/stance_foot       std_msgs/UInt8          – bit mask 0b01 L-stance, 0b10 R-stance

Publications
------------
/vel_controller/joint_trajectory
                       trajectory_msgs/JointTrajectory
"""

import math, pathlib, numpy as np
from enum import Enum, auto

import rclpy
from rclpy.node           import Node
from sensor_msgs.msg      import Joy, Imu
from geometry_msgs.msg    import PointStamped
from std_msgs.msg         import UInt8
from trajectory_msgs.msg  import JointTrajectory, JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory

def pkg_file(pkg: str, rel_path: str) -> str:
    return str(pathlib.Path(get_package_share_directory(pkg)) / rel_path)


def load_csv(path: str) -> np.ndarray:
    return np.loadtxt(path, delimiter=',')


def lookup(table: np.ndarray, phase: float) -> tuple[float, float]:
    n = len(table)
    idx_f = phase * (n - 1)
    i0, i1 = int(idx_f), min(int(idx_f) + 1, n - 1)
    w = idx_f - i0
    hip  = (1 - w) * table[i0, 0] + w * table[i1, 0]
    knee = (1 - w) * table[i0, 1] + w * table[i1, 1]
    return hip, knee


class Gait(Enum):
    STAND = auto()
    WALK  = auto()
    SIT   = auto()

class GaitManager(Node):
    def __init__(self):
        super().__init__("gait_manager")

        self.declare_parameter("rate_hz", 200)
        self.declare_parameter("csv_pkg", "exo_control")
        self.declare_parameter("csv_walk", "data/walk.csv")
        self.declare_parameter("csv_stand2sit", "data/stand2sit.csv")

        pkg = self.get_parameter("csv_pkg").value
        self.csv = {
            "walk":      load_csv(pkg_file(pkg, self.get_parameter("csv_walk").value)),
            "stand2sit": load_csv(pkg_file(pkg, self.get_parameter("csv_stand2sit").value)),
        }

        # capture-point constants (edit/tune)
        self.h_com   = 0.85            # com height [m]
        self.gain_cp = 0.2

        self.state         : Gait = Gait.STAND
        self.phase         : float = 0.0          # 0…1
        self.last_stance   : int   = 0b11         # start with both feet
        self.com_pos       = np.zeros(3)
        self.com_vel       = np.zeros(3)
        self.torso_pitch   = 0.0                  # from UKF orientation
        self.last_time     = self.get_clock().now()

        self.cmd_pub = self.create_publisher(JointTrajectory,
                                             "/vel_controller/joint_trajectory", 3)

        self.create_subscription(Joy,              "/joy",               self.cb_joy,   10)
        self.create_subscription(Imu,              "/exo/state_estimate",self.cb_state, 200)
        self.create_subscription(PointStamped,     "/exo/com",           self.cb_com,   200)
        self.create_subscription(UInt8,            "/exo/stance_foot",   self.cb_stance,200)

        # timer
        rate_hz = self.get_parameter("rate_hz").value
        self.timer = self.create_timer(1.0 / rate_hz, self.update)

        self.get_logger().info(f"GaitManager ☑  {rate_hz} Hz")

    def cb_joy(self, msg: Joy):
        if msg.buttons[0]:                      # A-button: toggle walk
            self.state = Gait.WALK if self.state != Gait.WALK else Gait.STAND
            self.phase = 0.0
        if msg.buttons[1]:                      # B-button: toggle sit
            self.state = Gait.SIT if self.state != Gait.SIT else Gait.STAND
            self.phase = 0.0

    def cb_state(self, msg: Imu):
        # torso pitch (about Y)
        q = msg.orientation
        sinp = 2 * (q.w * q.y - q.z * q.x)
        self.torso_pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        # linear acceleration = COM accel (but we mostly use vel published with COM)

    def cb_com(self, msg: PointStamped):
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        if dt > 0:
            new_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
            self.com_vel = (new_pos - self.com_pos)/dt
            self.com_pos = new_pos
            self.last_time = now

    def cb_stance(self, msg: UInt8):
        # reset gait phase at touchdown of new stance foot
        if msg.data != self.last_stance:
            # if swing foot just became stance → new step
            if (msg.data ^ self.last_stance) != 0b11:
                self.phase = 0.0
        self.last_stance = msg.data

    def update(self):
        hip, knee = 0.0, 0.0

        if self.state == Gait.STAND:
            pass

        elif self.state == Gait.SIT:
            hip, knee = lookup(self.csv["stand2sit"], self.phase)
            self.phase = min(self.phase + 0.005, 1.0)

        elif self.state == Gait.WALK:
            hip, knee = lookup(self.csv["walk"], self.phase)
            self.phase = (self.phase + 0.02) % 1.0

        # Capture-point: Δθ = (ẋ / ω0) with ω0 = √(g / z0)
        omega0 = math.sqrt(9.81 / self.h_com)
        cp_offset = self.com_vel[0] / omega0
        hip  += self.gain_cp * cp_offset

        # small torso pitch feedback (optional)
        hip += 0.5 * (-self.torso_pitch)

        jt = JointTrajectory()
        jt.joint_names = ["right_hip_revolute_joint",
                          "right_knee_revolute_joint"]

        p = JointTrajectoryPoint()
        p.velocities         = [hip, knee]
        p.time_from_start.sec = 0
        p.time_from_start.nanosec = 0
        jt.points = [p]
        self.cmd_pub.publish(jt)

def main():
    rclpy.init()
    node = GaitManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()