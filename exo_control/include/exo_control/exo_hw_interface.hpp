#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class ExoHWInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo &info) override;

  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(
      const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(
      const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Node::SharedPtr                                   node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr    pub_;

  std::vector<double> pos_, vel_, cmd_vel_;
};
