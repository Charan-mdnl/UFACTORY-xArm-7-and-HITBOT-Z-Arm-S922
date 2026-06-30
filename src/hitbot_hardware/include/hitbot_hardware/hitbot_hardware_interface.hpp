// Copyright (c) 2026, Chethan
// BSD-3-Clause License
//
// ros2_control SystemInterface plugin for the HITBOT Z-Arm S922.
// This is the bridge between MoveIt 2 / ros2_control and the physical robot.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hitbot_hardware/hitbot_tcp_driver.hpp"

namespace hitbot_hardware {

class HitbotHardwareInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HitbotHardwareInterface)

  // ── Lifecycle callbacks ────────────────────────────────────────────
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  // ── Interface export ───────────────────────────────────────────────
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  // ── Read / Write (called every control cycle) ──────────────────────
  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // TCP driver for communication with the real robot.
  HitbotTcpDriver driver_;

  // Parameters read from the URDF <ros2_control> tag.
  std::string robot_ip_;
  int robot_port_{8080};

  // Number of joints (expected: 6).
  std::size_t num_joints_{0};

  // State vectors (read from the robot).
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Command vector (sent to the robot).
  std::vector<double> hw_commands_;
};

}  // namespace hitbot_hardware
