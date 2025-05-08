#include "exo_control/balance_controller.hpp"
using std::placeholders::_1;
using sensor_msgs::msg::JointState;
using sensor_msgs::msg::Imu;

BalanceController::BalanceController()
: Node("balance_controller")
{
  declare_parameter("hip_kp",  80.0);
  declare_parameter("hip_kd",  10.0);
  declare_parameter("knee_kp", 60.0);
  declare_parameter("knee_kd",  8.0);
  declare_parameter("target_pitch", 0.0);

  get_parameter("hip_kp",  kp_hip_);
  get_parameter("hip_kd",  kd_hip_);
  get_parameter("knee_kp", kp_knee_);
  get_parameter("knee_kd", kd_knee_);
  get_parameter("target_pitch", tgt_pitch_);

  sub_imu_ = create_subscription<Imu>(
      "/imu/data_raw", rclcpp::SensorDataQoS(),
      std::bind(&BalanceController::imu_cb, this, _1));

  sub_js_  = create_subscription<JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&BalanceController::state_cb, this, _1));

  pub_trim_ = create_publisher<JointState>("/balance/vel_trim", 10);

  timer_ = create_wall_timer(
      std::chrono::microseconds(2500),          // 400 Hz
      std::bind(&BalanceController::control_loop, this));
}

/* -------------------- callbacks -------------------------------------- */
void BalanceController::imu_cb(const Imu::SharedPtr& m)
{
  // simple: use orientation.x as small-angle pitch
  tf2::Quaternion q(m->orientation.x, m->orientation.y,
                    m->orientation.z, m->orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  rclcpp::Time t = m->header.stamp;
  double dt = (t - last_imu_).seconds();
  if (dt > 1e-4)
    dpitch_ = (pitch - pitch_) / dt;
  pitch_  = pitch;
  last_imu_ = t;
}

void BalanceController::state_cb(const JointState::SharedPtr& m)
{
  /* assumes ordering hip, knee */
  if (m->name.size() < 2) return;
  hip_pos_  = m->position[0];
  knee_pos_ = m->position[1];
  hip_vel_  = m->velocity[0];
  knee_vel_ = m->velocity[1];
}

/* -------------------- control loop ----------------------------------- */
void BalanceController::control_loop()
{
  /* very small linearised model: hip & knee velocities chosen to reduce torso pitch error */
  double ep  = tgt_pitch_ - pitch_;
  double edp = -dpitch_;

  double vh = kp_hip_  * ep + kd_hip_  * edp;
  double vk = kp_knee_ * ep + kd_knee_ * edp;

  JointState js;
  js.header.stamp = now();
  js.name        = {"right_hip_revolute_joint", "right_knee_revolute_joint"};
  js.velocity    = {vh, vk};
  pub_trim_.publish(js);
}

/* ----------- pluginlib not required – regular node ------------------- */
