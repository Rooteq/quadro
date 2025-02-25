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

  CallbackReturn CybergearActuator::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Initialize parameters from URDF
    params.can_interface_ = info_.hardware_parameters["can_interface"];
    params.timeout_sec_ = std::stod(info_.hardware_parameters["timeout_sec"]);
    params.use_bus_time_ = info_.hardware_parameters["use_bus_time"] == "true";
    params.interval_sec_ = std::stod(info_.hardware_parameters["interval_sec"]);
    params.device_id_ = std::stoi(info_.hardware_parameters["device_id"]);
    params.primary_id_ = std::stoi(info_.hardware_parameters["primary_id"]);

    // Initialize limits from URDF
    params.max_position_ = std::stod(info_.hardware_parameters["max_position"]);
    params.min_position_ = std::stod(info_.hardware_parameters["min_position"]);
    params.max_velocity_ = std::stod(info_.hardware_parameters["max_velocity"]);
    params.min_velocity_ = std::stod(info_.hardware_parameters["min_velocity"]);
    params.max_effort_ = std::stod(info_.hardware_parameters["max_effort"]);
    params.min_effort_ = std::stod(info_.hardware_parameters["min_effort"]);
    unsigned int device_id = 127;
    // Initialize state and command variables
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
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

      actuators[joint.name] = std::make_unique<Actuator>(joint.name, device_id, params.primary_id_);
      device_id_to_actuator_name_[device_id] = joint.name;
      device_id--;

      RCLCPP_INFO(get_logger(), "Loaded joint with name: %s", joint.name.c_str());
    }


    for(const auto& [name,actuator] : actuators)
    {

      RCLCPP_INFO(get_logger(), "JOINT: %s", name.c_str());
    }


    return CallbackReturn::SUCCESS;
  }

  CallbackReturn CybergearActuator::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(params.timeout_sec_));
    interval_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(params.interval_sec_));
    try
    {
      sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(
          params.can_interface_, false);
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(get_logger(), "Error opening CAN sender: %s - %s",
                   params.can_interface_.c_str(), ex.what());
      return CallbackReturn::FAILURE;
    }

    try
    {
      receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(
          params.can_interface_, false);

      // "00007F00:0000FF00" to accept only incoming messages 
      receiver_->SetCanFilters(
          drivers::socketcan::SocketCanReceiver::CanFilterList("00007F00:0000FF00"));
      RCLCPP_DEBUG(get_logger(), "applied filters: %s", can_filters_.c_str());
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(get_logger(), "Error opening CAN receiver: %s - %s",
                   params.can_interface_.c_str(), ex.what());
      return CallbackReturn::FAILURE;
    }

    is_active_ = false;
    receiver_thread_ = std::thread(&CybergearActuator::receive, this);

    RCLCPP_INFO(get_logger(), "Successfully configured!");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn CybergearActuator::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Add cleanup logic here
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn CybergearActuator::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
  {

    for(const auto& [name,actuator] : actuators)
    {
      // change mode to position
			can_msgs::msg::Frame msg_mode;
      setDefaultCanFrame(msg_mode);
      const auto can_frame = actuator->packet_->createChangeToPositionModeCommand();
      std::copy(can_frame.data.cbegin(), can_frame.data.cend(), msg_mode.data.begin());
      msg_mode.id = can_frame.id;

			if(sendFrame(msg_mode) != return_type::OK)
			{
        RCLCPP_WARN(get_logger(), "Failed to send change mode message!");
        return CallbackReturn::FAILURE;
			}

      // enable torque
			can_msgs::msg::Frame msg;
      setDefaultCanFrame(msg);
      msg.id = actuator->packet_->frameId().getEnableTorqueId();

			if(sendFrame(msg) != return_type::OK)
			{
				RCLCPP_WARN(get_logger(), "Failed to send enable torque message!");
        return CallbackReturn::FAILURE;
			}
    }

    is_active_ = true;
    // Add activation logic here
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn CybergearActuator::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    for(const auto& [name,actuator] : actuators)
    {
      can_msgs::msg::Frame msg;
      setDefaultCanFrame(msg);
      msg.id = actuator->packet_->frameId().getResetTorqueId();

      if(sendFrame(msg) != return_type::OK)
      {
        RCLCPP_WARN(get_logger(), "Failed to disable torque!");
        return CallbackReturn::FAILURE;
      }
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn CybergearActuator::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Add shutdown logic here
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn CybergearActuator::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Add error handling logic here
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> CybergearActuator::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for(const auto& state : info_.joints)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        state.name, hardware_interface::HW_IF_POSITION, &(actuators[state.name]->state_pos_)));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
        state.name, hardware_interface::HW_IF_VELOCITY, &(actuators[state.name]->state_vel_)));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> CybergearActuator::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for(const auto& command : info_.joints)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        command.name, hardware_interface::HW_IF_POSITION, &(actuators[command.name]->cmd_vel_)));
    }
    return command_interfaces;
  }

  hardware_interface::return_type CybergearActuator::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {


    for(const auto& [name,actuator] : actuators)
    {
      can_msgs::msg::Frame msg;
      setDefaultCanFrame(msg);

      msg.id = actuator->packet_->frameId().getFeedbackId();

      if(sendFrame(msg) != hardware_interface::return_type::OK)
      {
        RCLCPP_WARN(get_logger(), "FAILED TO SEND READ FRAME");
      }

    }
    

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type CybergearActuator::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    for(const auto& [name,actuator] : actuators)
    {
      if(std::isnan(actuator->cmd_vel_)) {return hardware_interface::return_type::OK;}

      // DELETE LATER
      if(actuator->cmd_vel_ == last_command_) {return hardware_interface::return_type::OK;}

      can_msgs::msg::Frame msg;
      setDefaultCanFrame(msg);
      const auto can_frame = actuator->packet_->createPositionCommand(-(actuator->cmd_vel_));
      std::copy(can_frame.data.cbegin(), can_frame.data.cend(), msg.data.begin());
      msg.id = can_frame.id;
      if(sendFrame(msg) != hardware_interface::return_type::OK)
      {
        return hardware_interface::return_type::ERROR;
      }
    }

    return hardware_interface::return_type::OK;

  }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    quadro::CybergearActuator, hardware_interface::ActuatorInterface)
