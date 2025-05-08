#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Dense>

class BalanceController : public rclcpp::Node
{
public:
  BalanceController();

private:
  /* callbacks ----------------------------------------------------------- */
  void state_cb(const sensor_msgs::msg::JointState::SharedPtr&);
  void imu_cb  (const sensor_msgs::msg::Imu::SharedPtr&);

  /* helpers  ------------------------------------------------------------ */
  void control_loop();                 // 400 Hz timer

  /* params -------------------------------------------------------------- */
  double kp_hip_, kd_hip_, kp_knee_, kd_knee_;
  double tgt_pitch_;

  /* state  -------------------------------------------------------------- */
  double pitch_ = 0.0, dpitch_ = 0.0;          // torso orientation (rad, rad/s)
  double hip_pos_ = 0.0, hip_vel_ = 0.0;
  double knee_pos_= 0.0, knee_vel_= 0.0;
  rclcpp::Time last_imu_{0,0,RCL_ROS_TIME};

  /* ROS entities -------------------------------------------------------- */
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr          sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr   sub_js_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr      pub_trim_;
  rclcpp::TimerBase::SharedPtr                                     timer_;
};