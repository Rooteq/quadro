#ifndef ROBOTO_ACTUATORS
#define ROBOTO_ACTUATORS

#include <memory>
#include <string>
#include <vector>

#include <unordered_map>

#include <atomic>
#include <mutex>
#include <thread>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cybergear_driver_core/cybergear_driver_core.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "can_msgs/msg/frame.hpp"

#include "actuator.hpp"
#include <fmt/core.h>

// #include "cybergear_driver_core/cybergear_driver_core.hpp"
using namespace std::chrono_literals;

namespace quadro
{
using hardware_interface::ActuatorInterface;
using hardware_interface::CallbackReturn;
using hardware_interface::CommandInterface;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

using rclcpp_lifecycle::LifecycleNode;

class CybergearActuator : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CybergearActuator)

  // Lifecycle Callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State&) override;

  // Hardware Interface Callbacks
  CallbackReturn on_init(const hardware_interface::HardwareInfo&) override;

  std::vector<StateInterface> export_state_interfaces() override;
  std::vector<CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
  return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;
private:
  double simPos;
private:

  bool use_bus_time_;

  MotorParams params;

    void receive();
    void processFrame(const can_msgs::msg::Frame& frame);

    void setDefaultCanFrame(can_msgs::msg::Frame & msg, const std::string& frame)
    {
      constexpr uint8_t kDlc = 8;

      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = frame;
      msg.is_rtr = false;
      msg.is_extended = true;
      msg.is_error = false;
      msg.dlc = kDlc;
    }

    return_type sendFrame(const can_msgs::msg::Frame& msg) {
      using drivers::socketcan::CanId;
      using drivers::socketcan::FrameType;
      using drivers::socketcan::ExtendedFrame;

      try {
        sender_->send(msg.data.data(), msg.dlc, CanId(msg.id, 0, FrameType::DATA, ExtendedFrame), timeout_ns_);
      } catch (const std::exception& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Error sending CAN message: %s - %s",
                            params.can_interface_.c_str(), ex.what());
        return return_type::ERROR;
      }

      return return_type::OK;
    }

    std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
    std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;

    std::string can_filters_;
    std::thread receiver_thread_;

    std::chrono::nanoseconds timeout_ns_;
    std::chrono::nanoseconds interval_ns_;

    std::atomic_bool is_active_;

    std::mutex frames_mutex_;
    std::unordered_map<std::string, std::unique_ptr<Actuator>> actuators;
    std::unordered_map<unsigned int, std::string> device_id_to_actuator_name_;
};
}

#endif