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

    simPos = 0;

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

    // Create multiple packets!!!
    cybergear_driver_core::CybergearPacketParam packet_param;
    packet_param.device_id = static_cast<int>(127);
    packet_param.primary_id = static_cast<int>(0);
    packet_param.max_position = static_cast<float>(12.56637061);
    packet_param.min_position = static_cast<float>(-12.56637061);
    packet_param.max_velocity = static_cast<float>(30.0);
    packet_param.min_velocity = static_cast<float>(-30.0);
    packet_param.max_effort = static_cast<float>(12.0);
    packet_param.min_effort = static_cast<float>(-12.0);
    packet_param.max_gain_kp = static_cast<float>(500.0);
    packet_param.min_gain_kp = static_cast<float>(0);
    packet_param.max_gain_kd = static_cast<float>(5.0);
    packet_param.min_gain_kd = static_cast<float>(0.0);
    packet_param.temperature_scale = static_cast<float>(0.1);
    packet_ = std::make_unique<cybergear_driver_core::CybergearPacket>(packet_param);


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
      RCLCPP_INFO(get_logger(), "Loaded joint with name: %s", joint.name.c_str());
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn CybergearActuator::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {

    position_state = 0;
    velocity_state = 0;
    position_cmd = std::numeric_limits<double>::quiet_NaN();

    last_command_ = std::numeric_limits<double>::quiet_NaN();

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
    is_active_ = true;
    // Add activation logic here
    return enableTorque();
  }

  CallbackReturn CybergearActuator::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Add deactivation logic here
    return disableTorque();
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

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &position_state));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &velocity_state));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> CybergearActuator::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[0].name, hardware_interface::HW_IF_POSITION, &position_cmd));

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
    
    auto msg = std::make_unique<can_msgs::msg::Frame>();
    setDefaultCanFrame(msg);
    msg->id = packet_->frameId().getFeedbackId();

    if(sendParamRequestMessage(*msg) != hardware_interface::return_type::OK)
    {
      RCLCPP_WARN(get_logger(), "FAILED TO SEND READ FRAME");
    }

    position_state = -(packet_->parsePosition(last_received_frame_.data));
    velocity_state = -(packet_->parseVelocity(last_received_frame_.data));

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type CybergearActuator::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // position_state += 0.01;
    // for(const auto& [name, descr] : joint_state_interfaces_)
    // {
    //   // RCLCPP_INFO(get_logger(), "READ: %s", name.c_str());
    // }
    // Add write logic here
    if(std::isnan(position_cmd)) {return hardware_interface::return_type::OK;}
    if(position_cmd == last_command_) {return hardware_interface::return_type::OK;}

    auto msg = std::make_unique<can_msgs::msg::Frame>();
    setDefaultCanFrame(msg);
    const auto can_frame = packet_->createPositionCommand(-position_cmd);
    std::copy(can_frame.data.cbegin(), can_frame.data.cend(), msg->data.begin());
    msg->id = can_frame.id;
    return sendParamRequestMessage(*msg);
  }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    quadro::CybergearActuator, hardware_interface::ActuatorInterface)
