#!/usr/bin/env python3
"""
State-estimation node for the exoskeleton
• Unscented Kalman Filter for floating-base pos / vel / orientation
• Joint positions & velocities are taken directly from encoders
• Pinocchio gives COM & foot kinematics every cycle
Publishes:
  /exo/com            geometry_msgs/PointStamped     (world frame)
  /exo/stance_foot    std_msgs/UInt8                 (bit-mask 0b01 L, 0b10 R)
  /exo/state_estimate sensor_msgs/Imu                (quat + lin/ang vel in world)
"""
import os, math, numpy as np, pinocchio as pin
from collections import deque
import subprocess
import rclpy                     
from ament_index_python import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg  import Imu, JointState
from geometry_msgs.msg import PointStamped
from std_msgs.msg      import UInt8
from filterpy.kalman   import UnscentedKalmanFilter, MerweScaledSigmaPoints
import xacro
DT            = 0.005        # 200 Hz UKF rate
URDF_PARAM    = 'robot_description'
URDF_SUBPATH  = 'urdf/lleap_exo.urdf.xacro'
LEFT_FOOT     = 'L_sole'
RIGHT_FOOT    = 'R_sole'

ACC_WINDOW  = 8        # IMU mean-filter length
GYRO_WINDOW = 8
JOINT_BLACKLIST = {
    'left_hip_aa', 'right_hip_aa',  # DOFs we are not actuating
}

def quat_mul(q, r):
    """Hamilton product q⊗r, scalar-first convention."""
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1])

def meanfilter(buf: deque, new_val: float) -> float:
    buf.append(new_val)
    return sum(buf) / len(buf)

class ExoStateEstimator(Node):
    def __init__(self):
        super().__init__('state_estimator_pinocchio')

        # Declare parameters
        self.declare_parameter("urdf_package", URDF_PARAM)
        self.declare_parameter("urdf_subpath", URDF_SUBPATH)
        self.declare_parameter("use_fake_hardware", False)

        # Get parameters
        urdf_pkg = self.get_parameter("urdf_package").value
        urdf_subpath = self.get_parameter("urdf_subpath").value
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
            xacro_path = os.path.join(get_package_share_directory(urdf_pkg), urdf_subpath)
            if not os.path.exists(xacro_path):
                raise FileNotFoundError(f"URDF file not found: {xacro_path}")
            
            cmd = [
                'xacro', xacro_path,
                'use_fake_hardware:=', str(self.use_fake_hardware).lower()
            ]
            robot_description = subprocess.check_output(cmd).decode('utf-8')

        self.model = pin.buildModelFromXML(robot_description)
        self.data = self.model.createData()

        # Map joint-name → configuration index (position & velocity share the index).
        self.joint_names   = [j.name for j in self.model.joints if j.idx >= 1]  # skip universe
        self.jid_from_name = {name: self.model.getJointId(name)
                              for name in self.joint_names}

        # Joint limits
        self.q_lo = self.model.lowerPositionLimit.copy()
        self.q_hi = self.model.upperPositionLimit.copy()

        # Frames for COM & feet
        self.id_l_foot  = self.model.getFrameId(LEFT_FOOT)
        self.id_r_foot  = self.model.getFrameId(RIGHT_FOOT)

        # UKF setup
        nx     = 10  # [p(3) v(3) quat(4)]
        sigmas = MerweScaledSigmaPoints(nx, alpha=.1, beta=2., kappa=1.)
        self.ukf = UnscentedKalmanFilter(nx,    # state
                                         dim_z=0,# we do "pure prediction"; JS-update below
                                         dt=DT,
                                         fx=self.fx,
                                         hx=lambda x: x[:0],   # dummy
                                         points=sigmas)
        self.ukf.x[:] = np.array([0,0,0,  0,0,0,  1,0,0,0])  # initial pos=(0,0,0), quat=(1,0,0,0)
        self.ukf.P   *= 0.01

        # IMU mean-filters
        self.acc_filt  = [deque(maxlen=ACC_WINDOW)  for _ in range(3)]
        self.gyro_filt = [deque(maxlen=GYRO_WINDOW) for _ in range(3)]

        # ROS I/O
        qos = rclpy.qos.QoSProfile(depth=200)
        self.sub_imu = self.create_subscription(Imu, 'imu/filtered', self.cb_imu, qos)
        self.sub_js = self.create_subscription(JointState, 'joint_states/filtered', self.cb_js, qos)

        self.pub_com = self.create_publisher(PointStamped, 'exo/com', 10)
        self.pub_stance = self.create_publisher(UInt8, 'exo/stance_foot', 10)
        self.pub_state = self.create_publisher(Imu, 'exo/state_estimate', 10)

        # cache of last joint array (Pinocchio wants full nq vector)
        self.q  = pin.neutral(self.model)
        self.dq = np.zeros(self.model.nv)
        self.last_js_stamp = self.get_clock().now()

    # process model
    def fx(self, x, dt, u):
        """u = [wx wy wz ax ay az] body frame"""
        p, v, q = x[0:3], x[3:6], x[6:10]
        wx,wy,wz, ax,ay,az = u

        # integrate velocity
        v = v + np.array([ax,ay,az])*dt
        p = p + v*dt

        # quaternion integration (small-angle)
        dq = np.array([0, wx, wy, wz])*0.5*dt
        q  = q + quat_mul(q, dq)
        q  = q / np.linalg.norm(q)

        return np.hstack((p, v, q))

    # ROS callbacks
    def cb_imu(self, msg: Imu):
        ax = meanfilter(self.acc_filt[0],  msg.linear_acceleration.x)
        ay = meanfilter(self.acc_filt[1],  msg.linear_acceleration.y)
        az = meanfilter(self.acc_filt[2],  msg.linear_acceleration.z)
        wx = meanfilter(self.gyro_filt[0], msg.angular_velocity.x)
        wy = meanfilter(self.gyro_filt[1], msg.angular_velocity.y)
        wz = meanfilter(self.gyro_filt[2], msg.angular_velocity.z)
        u = np.array([wx, wy, wz, ax, ay, az])
        self.ukf.predict(u=u)
        self.publish_state(msg.header.stamp)       # publish every IMU tick (fast)

    def cb_js(self, msg: JointState):
        # Pinocchio expects q & dq arrays the size of model.nq/nv.
        for i, name in enumerate(msg.name):
            if name in self.jid_from_name: 
                continue
            jid = self.jid_from_name[name]
            qidx = self.model.joints[jid].idx_q

            # position NaN guard + clamp
            pos = msg.position[i]
            if math.isnan(pos):
                pos = self.q[qidx]
            pos = np.clip(pos, self.q_lo[qidx], self.q_hi[qidx])

            # velocity 
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            if name in JOINT_BLACKLIST:
                vel = 0.0            # freeze passive joints

            self.q[qidx]  = pos
            self.dq[qidx] = vel

        self.last_js_stamp = msg.header.stamp

    # publishing
    def publish_state(self, stamp):
        # kinematics with Pinocchio
        pin.forwardKinematics(self.model, self.data, self.q, self.dq)
        pin.updateFramePlacements(self.model, self.data)

        com, com_vel = pin.centerOfMass(self.model, self.data, self.q, self.dq), \
                       pin.getCenterOfMassVelocity(self.model, self.data, self.q, self.dq)

        # stance foot detection
        z_L = self.data.oMf[self.id_l_foot].translation[2]
        z_R = self.data.oMf[self.id_r_foot].translation[2]
        stance = 3        # both
        if z_L < z_R - 0.015: stance = 0x01
        elif z_R < z_L - 0.015: stance = 0x02

        # publish COM
        pmsg = PointStamped()
        pmsg.header.stamp    = stamp
        pmsg.header.frame_id = 'world'
        pmsg.point.x, pmsg.point.y, pmsg.point.z = com
        self.pub_com.publish(pmsg)

        # publish stance flag
        smsg = UInt8(); smsg.data = stance
        self.pub_stance.publish(smsg)

        # publish full state (imu-like message)
        im = Imu()
        im.header.stamp = stamp
        im.header.frame_id = 'world'
        w, x, y, z = self.ukf.x[6:10]     # q = [w,x,y,z]
        im.orientation.w = w
        im.orientation.x = x
        im.orientation.y = y
        im.orientation.z = z
        im.linear_acceleration.x, im.linear_acceleration.y, im.linear_acceleration.z = com_vel
        # we're not estimating angular velocity here
        self.pub_state.publish(im)


def main():
    rclpy.init()
    rclpy.spin(ExoStateEstimator())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
