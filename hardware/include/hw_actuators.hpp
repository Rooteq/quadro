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

#include "cybergear_driver.hpp"


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
  double simPos;
private:

  bool use_bus_time_;

  MotorParams params;

    CallbackReturn enableTorque()
    {
      // change mode to position
			can_msgs::msg::Frame msg_mode;
      setDefaultCanFrame(msg_mode);
      const auto can_frame = packet_->createChangeToPositionModeCommand();
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
      msg.id = packet_->frameId().getEnableTorqueId();

			if(sendFrame(msg) != return_type::OK)
			{
				RCLCPP_WARN(get_logger(), "Failed to send enable torque message!");
        return CallbackReturn::FAILURE;
			}

      return CallbackReturn::SUCCESS;
    }

    CallbackReturn disableTorque()
    {
			can_msgs::msg::Frame msg;
      setDefaultCanFrame(msg);
      msg.id = packet_->frameId().getResetTorqueId();

			if(sendFrame(msg) != return_type::OK)
			{
				RCLCPP_WARN(get_logger(), "Failed to disable torque!");
        return CallbackReturn::FAILURE;
			}

      return CallbackReturn::SUCCESS;
    }


    void receive() {
        using drivers::socketcan::FrameType;

        drivers::socketcan::CanId receive_id{};

        can_msgs::msg::Frame frame(rosidl_runtime_cpp::MessageInitialization::ZERO);
        frame.header.frame_id = info_.joints[0].name;

        while (rclcpp::ok()) {
            if (!is_active_) {
            std::this_thread::sleep_for(100ms);
            continue;
            }

            try {
            receive_id = receiver_->receive(frame.data.data(), interval_ns_);
            } catch (const std::exception& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                "Error receiving CAN message: %s - %s",
                                params.can_interface_.c_str(), ex.what());
            // RCLCPP_INFO(get_logger(), "LAST MESSAGE ID: %x", last_received_frame_.id);
            continue;
            }

            if (params.use_bus_time_) {
            frame.header.stamp =
                rclcpp::Time(static_cast<int64_t>(receive_id.get_bus_time() * 1000U));
            } else {
            frame.header.stamp = get_clock()->now();
            }

            frame.id = receive_id.identifier();
            frame.is_rtr = (receive_id.frame_type() == FrameType::REMOTE);
            frame.is_extended = receive_id.is_extended();
            frame.is_error = (receive_id.frame_type() == FrameType::ERROR);
            frame.dlc = receive_id.length();

            // RCLCPP_INFO(get_logger(), "RECEIVED CAN FRAME");

            {
            std::lock_guard<std::mutex> guard(last_frame_mutex_);
            last_received_frame_ = frame;
            }
        }
    }

    void setDefaultCanFrame(can_msgs::msg::Frame & msg, const std::string& frame = "cybergear")
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

    std::unique_ptr<cybergear_driver_core::CybergearPacket> packet_;

    std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
    std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;

    can_msgs::msg::Frame last_joint_command_frame_;
    can_msgs::msg::Frame last_received_frame_;

    std::string can_filters_;
    std::thread receiver_thread_;

    std::chrono::nanoseconds timeout_ns_;
    std::chrono::nanoseconds interval_ns_;

    double position_state;
    double velocity_state;
    double position_cmd;

    std::atomic_bool is_active_;
    std::mutex last_frame_mutex_;

    double last_command_;
};

}

#endif