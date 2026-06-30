#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  // Create the node
  auto node = std::make_shared<rclcpp::Node>("test_moveit_sequence");
  
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("test_moveit_sequence");
  RCLCPP_INFO(logger, "Starting MoveIt 2 sequence test...");

  // Spin the node in a background thread to process callbacks (joint states)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Get current joint values
  auto current_state = move_group_interface.getCurrentState(10.0);
  if (!current_state) {
    RCLCPP_ERROR(logger, "Failed to get current joint state!");
    rclcpp::shutdown();
    spinner.join();
    return -1;
  }

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(
      current_state->getRobotModel()->getJointModelGroup("arm"),
      joint_group_positions);

  // Print current positions
  RCLCPP_INFO(logger, "Current J1: %.2f rad, J2: %.2f rad", joint_group_positions[0], joint_group_positions[1]);

  // ─────────────────────────────────────────
  // POSE 1: +5 degrees (+0.087 rad) on J1 & J2
  // ─────────────────────────────────────────
  RCLCPP_INFO(logger, "Moving to Pose 1 (+5 degrees on J1 and J2)...");
  joint_group_positions[0] += 0.0872665;
  joint_group_positions[1] += 0.0872665;
  move_group_interface.setJointValueTarget(joint_group_positions);

  // Plan
  MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    move_group_interface.execute(my_plan);
    RCLCPP_INFO(logger, "Reached Pose 1!");
  } else {
    RCLCPP_ERROR(logger, "Planning failed for Pose 1!");
  }

  // Wait 2 seconds
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // ─────────────────────────────────────────
  // POSE 2: -5 degrees (-0.087 rad) on J1 & J2 (Back to original)
  // ─────────────────────────────────────────
  RCLCPP_INFO(logger, "Moving back to original Pose 2 (-5 degrees)...");
  joint_group_positions[0] -= 0.0872665;
  joint_group_positions[1] -= 0.0872665;
  move_group_interface.setJointValueTarget(joint_group_positions);

  success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success) {
    move_group_interface.execute(my_plan);
    RCLCPP_INFO(logger, "Reached Pose 2! Sequence finished.");
  } else {
    RCLCPP_ERROR(logger, "Planning failed for Pose 2!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
