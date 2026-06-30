#include "hitbot_hardware/hitbot_hardware_interface.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <thread>
#include <chrono>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hitbot_hardware {

hardware_interface::CallbackReturn HitbotHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  robot_ip_   = info_.hardware_parameters.count("robot_ip")
                    ? info_.hardware_parameters.at("robot_ip")
                    : "192.168.58.2";
  robot_port_ = info_.hardware_parameters.count("robot_port")
                    ? std::stoi(info_.hardware_parameters.at("robot_port"))
                    : 8080;

  num_joints_ = info_.joints.size();

  hw_positions_.resize(num_joints_, 0.0);
  hw_velocities_.resize(num_joints_, 0.0);
  hw_efforts_.resize(num_joints_, 0.0);
  hw_commands_.resize(num_joints_, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HitbotHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("HitbotHardwareInterface"),
              "Connecting to HITBOT at %s:%d ...", robot_ip_.c_str(),
              robot_port_);

  if (driver_.connect(robot_ip_, robot_port_) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HitbotHardwareInterface"),
                 "Failed to connect to HITBOT controller!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HitbotHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("HitbotHardwareInterface"),
              "Activating hardware interface...");

  driver_.resetErrors();
  driver_.switchMode(0);

  // Mandatory motor-enable delay
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Initialize joint states ONCE to avoid saturating the socket
  auto pos_deg = driver_.getJointPosDegree();
  if (pos_deg.size() >= num_joints_) {
    for (std::size_t i = 0; i < num_joints_; ++i) {
      hw_positions_[i] = pos_deg[i] * M_PI / 180.0;
      hw_commands_[i]  = hw_positions_[i];
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("HitbotHardwareInterface"), "Failed to read initial pos!");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HitbotHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  driver_.disconnect();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
HitbotHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < num_joints_; ++i) {
    state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]);
    state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]);
    state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &hw_efforts_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
HitbotHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < num_joints_; ++i) {
    command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type HitbotHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!driver_.isConnected()) {
    return hardware_interface::return_type::ERROR;
  }

  // Determine if we are actively commanding a movement
  bool moving = false;
  for (std::size_t i = 0; i < num_joints_; ++i) {
    if (std::abs(hw_commands_[i] - hw_positions_[i]) > 0.0001) {
      moving = true;
      break;
    }
  }

  if (moving) {
    // If moving, avoid querying state to prevent socket choke.
    // Just echo the commands so ros2_control thinks we are tracking perfectly.
    for (std::size_t i = 0; i < num_joints_; ++i) {
      hw_positions_[i] = hw_commands_[i];
    }
  } else {
    // If idle, query the real state. This also acts as a TCP heartbeat
    // to prevent the HITBOT controller from dropping the connection.
    auto pos_deg = driver_.getJointPosDegree();
    if (pos_deg.size() >= num_joints_) {
      for (std::size_t i = 0; i < num_joints_; ++i) {
        hw_positions_[i] = pos_deg[i] * M_PI / 180.0;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HitbotHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  if (!driver_.isConnected()) {
    return hardware_interface::return_type::ERROR;
  }

  // Only send ServoJ if there is a meaningful movement command
  bool moving = false;
  for (std::size_t i = 0; i < num_joints_; ++i) {
    if (std::abs(hw_commands_[i] - hw_positions_[i]) > 0.0001) {
      moving = true;
      break;
    }
  }

  if (!moving) {
    return hardware_interface::return_type::OK;
  }

  std::vector<float> cmd_deg(num_joints_);
  for (std::size_t i = 0; i < num_joints_; ++i) {
    cmd_deg[i] = static_cast<float>(hw_commands_[i] * 180.0 / M_PI);
  }

  float t = static_cast<float>(period.seconds());
  if (t <= 0.0f) t = 0.1f;  // fallback to 100ms
  
  RCLCPP_INFO(rclcpp::get_logger("HitbotHardwareInterface"),
              "servoJ(%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, t=%.2f)", 
              cmd_deg[0], cmd_deg[1], cmd_deg[2], cmd_deg[3], cmd_deg[4], cmd_deg[5], t);

  driver_.servoJ(cmd_deg, 50.0f, 50.0f, t, 0.0f, 0.0f);

  return hardware_interface::return_type::OK;
}

}  // namespace hitbot_hardware

PLUGINLIB_EXPORT_CLASS(hitbot_hardware::HitbotHardwareInterface,
                       hardware_interface::SystemInterface)
