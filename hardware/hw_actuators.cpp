#include "include/hw_actuators.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace quadro
{

  CallbackReturn CybergearActuator::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  simPos = 0;

  // Initialize parameters from URDF
  can_interface_ = info_.hardware_parameters["can_interface"];
  timeout_sec_ = std::stod(info_.hardware_parameters["timeout_sec"]);
  use_bus_time_ = info_.hardware_parameters["use_bus_time"] == "true";
  interval_sec_ = std::stod(info_.hardware_parameters["interval_sec"]);
  device_id_ = std::stoi(info_.hardware_parameters["device_id"]);
  primary_id_ = std::stoi(info_.hardware_parameters["primary_id"]);
  
  // Initialize limits from URDF
  max_position_ = std::stod(info_.hardware_parameters["max_position"]);
  min_position_ = std::stod(info_.hardware_parameters["min_position"]);
  max_velocity_ = std::stod(info_.hardware_parameters["max_velocity"]);
  min_velocity_ = std::stod(info_.hardware_parameters["min_velocity"]);
  max_effort_ = std::stod(info_.hardware_parameters["max_effort"]);
  min_effort_ = std::stod(info_.hardware_parameters["min_effort"]);
  
  // Initialize gains from URDF
  max_gain_kp_ = std::stod(info_.hardware_parameters["max_gain_kp"]);
  min_gain_kp_ = std::stod(info_.hardware_parameters["min_gain_kp"]);
  max_gain_kd_ = std::stod(info_.hardware_parameters["max_gain_kd"]);
  min_gain_kd_ = std::stod(info_.hardware_parameters["min_gain_kd"]);
  
  // Initialize current limits from URDF
  max_current_ = std::stod(info_.hardware_parameters["max_current"]);
  min_current_ = std::stod(info_.hardware_parameters["min_current"]);
  temperature_scale_ = std::stod(info_.hardware_parameters["temperature_scale"]);

  // Initialize state and command variables
  position_state_ = 0.0;
  velocity_state_ = 0.0;
  effort_state_ = 0.0;
  position_command_ = 0.0;

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // 
    if (joint.command_interfaces.size() != 1)
    {
    RCLCPP_FATAL(
      get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
      joint.command_interfaces.size());

      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "Loaded joint with name: %s", joint.name.c_str());
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  // Add configuration logic here
  // reset values always when configuring hardware
  // for (const auto & [name, descr] : joint_state_interfaces_)
  // {
  //   set_state(name, 0.0);
  // }
  // for (const auto & [name, descr] : joint_command_interfaces_)
  // {
  //   set_command(name, 0.0);
  // }

  RCLCPP_INFO(get_logger(), "Successfully configured!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
  // Add cleanup logic here
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // Add activation logic here
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // Add deactivation logic here
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) {
  // Add shutdown logic here
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_error(const rclcpp_lifecycle::State & /*previous_state*/) {
  // Add error handling logic here
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CybergearActuator::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &position_state_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &velocity_state_));
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CybergearActuator::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &position_command_));
    
  return command_interfaces;
}

hardware_interface::return_type CybergearActuator::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
{
  // std::stringstream ss;
  // ss << "Reading states: ";
  // for(const auto& [name, descr] : joint_state_interfaces_)
  // {
  
  //   // set_state(name, new_value);
  //   ss << std::fixed << std::setprecision(2) << std::endl
  //      << "\t" << get_state(name) << " for joint '" << name << ", interface_name: '" << descr.get_interface_name().c_str();

  //   // RCLCPP_INFO(get_logger(), "READ: %s", name.c_str());
  // }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CybergearActuator::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  position_state_ += 0.01;
  // for(const auto& [name, descr] : joint_state_interfaces_)
  // {
  //   // RCLCPP_INFO(get_logger(), "READ: %s", name.c_str());
  // }
  // Add write logic here
  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  quadro::CybergearActuator, hardware_interface::ActuatorInterface)
