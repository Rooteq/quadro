#ifndef ROBOTO_ACTUATORS
#define ROBOTO_ACTUATORS

#include <memory>
#include <string>
#include <vector>

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

#include "ros2_socketcan/socket_can_sender.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "can_msgs/msg/frame.hpp"

#include "cybergear_driver_core/cybergear_driver_core.hpp"

namespace roboto
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
  RCLCPP_SHARED_PTR_DEFINITIONS(CybergearActuator);

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

  // Create a class responsible for can bus?

  std::unique_ptr<cybergear_driver_core::CybergearPacket> packet_;

  double joint_command_;
  double last_joint_command_;
  can_msgs::msg::Frame joint_command_template_;
  std::vector<double> joint_states_;

  std::string can_filters_;
  std::string can_interface_;

  std::chrono::nanoseconds timeout_ns_;
  std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;

  bool use_bus_time_;
  std::chrono::nanoseconds interval_ns_;
  std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;
  std::thread receiver_thread_;

  std::atomic_bool is_active_;
  can_msgs::msg::Frame last_received_frame_;
  std::mutex last_frame_mutex_;
};

}

#endif