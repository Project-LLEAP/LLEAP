#!/usr/bin/env python3
"""
Real-time balance controller for the exoskeleton
------------------------------------------------
Inputs
------
  /exo/com            geometry_msgs/PointStamped   (world-frame CoM from UKF)
  /exo/stance_foot    std_msgs/UInt8               (0b01 L, 0b10 R, 0b11 double)
  /joint_states/filtered  sensor_msgs/JointState   (actual joints for Pinocchio)

Outputs
-------
  /balance/joint_cmd  sensor_msgs/JointState       (add-on Δq [rad] for ankle-IE & hip-AA)

Strategy (step-by-step)
-----------------------
1.  On every CoM update (200 Hz) we:
    •  run Pinocchio FK to get left & right foot positions in *world*  
    •  choose the support foot centre from `/exo/stance_foot`  
    •  compute ML error e = y_com – y_support  
2.  A simple PD generates ankle-roll & hip-AA corrections:
       Δq =  Kp*e  +  Kd·(e – e_prev)/dt
    (Sign is mirrored for the two legs.)
3.  We publish **only** those four joints; downstream controller can sum them
    with the gait trajectory before sending to the low-level actuators.

Tunable ROS-params (with defaults shown)
----------------------------------------
  kp            :  3.0        # proportional gain  [rad/m]
  kd            :  0.2        # derivative gain    [rad·s/m]
  max_roll      :  0.15       # saturation  [rad]
  dt_control    :  0.005      # 200 Hz loop
  urdf_package  :  "robot_description"
  urdf_subpath  :  "urdf/lleap_exo.urdf.xacro"
"""

import os, numpy as np, pinocchio as pin
import subprocess

import rclpy
from rclpy.node        import Node
from ament_index_python import get_package_share_directory
from std_msgs.msg       import UInt8
from sensor_msgs.msg    import JointState
from geometry_msgs.msg  import PointStamped
import xacro
LEFT_FOOT  = "L_sole"
RIGHT_FOOT = "R_sole"

ANKLE_ROLL_JOINTS = ("left_ankle_ie",  "right_ankle_ie")
HIP_AA_JOINTS     = ("left_hip_aa",    "right_hip_aa")     # optional: hip ad/abduction

URDF_PARAM = 'exo_description'
URDF_SUBPATH = 'urdf/lleap_exo.urdf.xacro'

class BalanceController(Node):

    def __init__(self):
        super().__init__("balance_controller")

        # parameters 
        self.declare_parameter("kp",           3.0)
        self.declare_parameter("kd",           0.2)
        self.declare_parameter("max_roll",     0.15)     # [rad] ≈ 9°
        self.declare_parameter("dt_control",   0.005)
        self.declare_parameter("urdf_package", "robot_description")
        self.declare_parameter("urdf_subpath", "urdf/lleap_exo.urdf.xacro")
        self.declare_parameter("use_fake_hardware", False)

        self.Kp        = self.get_parameter("kp").value
        self.Kd        = self.get_parameter("kd").value
        self.max_roll  = self.get_parameter("max_roll").value
        self.dt        = self.get_parameter("dt_control").value
        self.use_fake_hardware = self.get_parameter("use_fake_hardware").value

        # Get robot description from robot_state_publisher parameter
        self.get_logger().info("Waiting for robot_description parameter...")
        
        # Wait for robot_description parameter to be available
        while not self.has_parameter("robot_description"):
            try:
                self.get_parameter_or("robot_description", "")
            except:
                pass
            rclpy.spin_once(self, timeout_sec=0.5)
            
        robot_description = self.get_parameter("robot_description").value
        if not robot_description:
            # Fallback to xacro processing if parameter not available
            self.get_logger().warn("robot_description not available, using xacro file directly")
            xacro_path = os.path.join(get_package_share_directory(URDF_PARAM), URDF_SUBPATH)
            if not os.path.exists(xacro_path):
                raise FileNotFoundError(f"URDF file not found: {xacro_path}")
            
            cmd = [
                'xacro', xacro_path,
                'use_fake_hardware:=', str(self.use_fake_hardware).lower()
            ]
            robot_description = subprocess.check_output(cmd).decode('utf-8')

        self.model = pin.buildModelFromXML(robot_description)
        self.data  = self.model.createData()
        self.id_L  = self.model.getFrameId(LEFT_FOOT)
        self.id_R  = self.model.getFrameId(RIGHT_FOOT)

        # joint-name → Pinocchio q index
        self.q      = pin.neutral(self.model)
        self.idx_q  = { j.name: j.idx_q for j in self.model.joints if j.idx >= 1 }

        # state 
        self.com_y       = 0.0
        self.e_prev      = 0.0
        self.stance_mask = 0b11             # assume double at start

        # ROS I/O 
        qos = rclpy.qos.QoSProfile(depth=200)
        self.sub_com   = self.create_subscription(PointStamped, "exo/com",           self._cb_com,   qos)
        self.sub_stance= self.create_subscription(UInt8,         "exo/stance_foot",  self._cb_stance,qos)
        self.sub_js    = self.create_subscription(JointState,    "joint_states/filtered", self._cb_js,qos)

        self.pub_cmd   = self.create_publisher(JointState, "balance/joint_cmd", 10)

        # trigger control loop every dt (even if nothing new arrives)
        self.create_timer(self.dt, self.control_loop)

    def _cb_com(self, msg: PointStamped):
        self.com_y = msg.point.y

    def _cb_stance(self, msg: UInt8):
        self.stance_mask = msg.data

    def _cb_js(self, msg: JointState):
        # store *only* positions for FK
        for name, pos in zip(msg.name, msg.position):
            if name in self.idx_q:
                self.q[self.idx_q[name]] = pos


    def control_loop(self):
        # compute foot positions in world 
        pin.updateFramePlacements(self.model, self.data)   # needs only q
        y_L = self.data.oMf[self.id_L].translation[1]
        y_R = self.data.oMf[self.id_R].translation[1]

        if   self.stance_mask == 0b01:   y_support = y_L
        elif self.stance_mask == 0b10:   y_support = y_R
        else:                            y_support = 0.5*(y_L + y_R)

        # -- error & PD -----------------------------------------------------
        e      = self.com_y - y_support
        de     = (e - self.e_prev) / self.dt
        delta  = self.Kp*e + self.Kd*de
        delta  = float(np.clip(delta, -self.max_roll, self.max_roll))
        self.e_prev = e

        # left = +delta (foot rocks outward), right = -delta
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name   = [ANKLE_ROLL_JOINTS[0], ANKLE_ROLL_JOINTS[1],
                      HIP_AA_JOINTS[0],     HIP_AA_JOINTS[1]]
        cmd.position = [ delta, -delta,
                         delta, -delta]      # tiny hip AA helps too
        self.pub_cmd.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(BalanceController())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
