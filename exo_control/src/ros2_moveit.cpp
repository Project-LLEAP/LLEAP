#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

constexpr char NODE_NAME[] = "exo_control";

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    NODE_NAME,
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger(NODE_NAME);

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "manipulator");

geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x    = 0.28;
  target_pose.position.y    = -0.20;
  target_pose.position.z    = 0.50;
  move_group_interface.setPoseTarget(target_pose);

  MoveGroupInterface::Plan my_plan;
  bool success =
      (move_group_interface.plan(my_plan) ==
       moveit::core::MoveItErrorCode::SUCCESS);

  if (!success)
  {
    RCLCPP_ERROR(logger, "Motion planning failed.");
    rclcpp::shutdown();
    return 1;
  }

  if (move_group_interface.execute(my_plan) !=
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Execution failed.");
    rclcpp::shutdown();
    return 2;
  }

  RCLCPP_INFO(logger, "Motion completed successfully.");
  rclcpp::shutdown();
  return 0;
} 