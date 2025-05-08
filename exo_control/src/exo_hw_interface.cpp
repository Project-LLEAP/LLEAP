#include "exo_control/exo_hw_interface.hpp"
#include <algorithm>
#include <iterator>

namespace hi = hardware_interface;

hi::CallbackReturn ExoHWInterface::on_init(const hi::HardwareInfo &info)
{
  if (hi::SystemInterface::on_init(info) != hi::CallbackReturn::SUCCESS)
    return hi::CallbackReturn::ERROR;

  const size_t nj = info_.joints.size();
  pos_.assign(nj, 0.0);
  vel_.assign(nj, 0.0);
  cmd_vel_.assign(nj, 0.0);

  node_ = rclcpp::Node::make_shared("exo_hw_bridge");
  pub_  = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_velocity_cmd", 10);

  sub_  = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&ExoHWInterface::joint_state_cb, this, std::placeholders::_1));

  return hi::CallbackReturn::SUCCESS;
}

std::vector<hi::StateInterface> ExoHWInterface::export_state_interfaces()
{
  std::vector<hi::StateInterface> si;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    si.emplace_back(info_.joints[i].name, hi::HW_IF_POSITION, &pos_[i]);
    si.emplace_back(info_.joints[i].name, hi::HW_IF_VELOCITY, &vel_[i]);
  }
  return si;
}

std::vector<hi::CommandInterface> ExoHWInterface::export_command_interfaces()
{
  std::vector<hi::CommandInterface> ci;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    ci.emplace_back(info_.joints[i].name, hi::HW_IF_VELOCITY, &cmd_vel_[i]);
  }
  return ci;
}

hi::CallbackReturn ExoHWInterface::on_activate(const rclcpp_lifecycle::State &)
{
  std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0.0);
  return hi::CallbackReturn::SUCCESS;
}

hi::CallbackReturn ExoHWInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  return hi::CallbackReturn::SUCCESS;
}

hi::return_type ExoHWInterface::read(
    const rclcpp::Time &, const rclcpp::Duration &)
{
  rclcpp::spin_some(node_);
  return hi::return_type::OK;
}

hi::return_type ExoHWInterface::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
  sensor_msgs::msg::JointState js;
  js.velocity = cmd_vel_;                // copy full vector
  pub_->publish(js);
  return hi::return_type::OK;
}

void ExoHWInterface::joint_state_cb(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const size_t nj = std::min(msg->name.size(), info_.joints.size());

  for (size_t i = 0; i < nj; ++i) {
    // find matching index in incoming msg
    auto it = std::find(msg->name.begin(), msg->name.end(), info_.joints[i].name);
    if (it == msg->name.end()) continue;
    size_t idx = std::distance(msg->name.begin(), it);

    if (idx < msg->position.size()) pos_[i] = msg->position[idx];
    if (idx < msg->velocity.size()) vel_[i] = msg->velocity[idx];
  }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ExoHWInterface, hardware_interface::SystemInterface)
