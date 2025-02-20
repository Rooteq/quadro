#ifndef CYBERGEAR_DRIVER
#define CYBERGEAR_DRIVER

#include <thread>
#include <mutex>
#include <memory>
#include <atomic>

#include "cybergear_driver_core/cybergear_driver_core.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "can_msgs/msg/frame.hpp"

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
namespace quadro
{
    using hardware_interface::return_type;
    using hardware_interface::CallbackReturn;

struct MotorParams
{
public:
    std::string can_interface_;
    double timeout_sec_;
    bool use_bus_time_;
    double interval_sec_;
    int device_id_;
    int primary_id_;

    double max_position_;
    double min_position_;
    double max_velocity_;
    double min_velocity_;
    double max_effort_;
    double min_effort_;
};

class CybergearDriver
{
public:
    CybergearDriver() = default;

    CallbackReturn on_motors_configure()
    {
        position = 0;

        try {
            sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(
                params.can_interface_, false);
        } catch (const std::exception& ex) {
            std::cout << "Error opening can sender!" << std::endl;
            // RCLCPP_INFO(logger_,"Error opening CAN sender: %s - %s",
            //             params.can_interface_.c_str(), ex.what());
            return CallbackReturn::FAILURE;
        }

        try {
            receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(
                params.can_interface_, false);
            // apply CAN filters
            receiver_->SetCanFilters(drivers::socketcan::SocketCanReceiver::CanFilterList(can_filters_));

            std::cout <<  "applied filters: " << can_filters_.c_str() << std::endl;
        } catch (const std::exception& ex) {
            std::cout << "Error opening CAN receiver: " << params.can_interface_.c_str() << " - " << ex.what() << std::endl;
            return CallbackReturn::FAILURE;
        }
        // receiver_thread_ = std::thread(&receive, this);

        std::cout << "SUCCESSSSSSS" << std::endl;
        return CallbackReturn::SUCCESS;
    }

    double position;
    MotorParams params;

    // TODO: Try to make ex. readPosition(), writePositions() for frame construction and then sendCanFrame for pure sending
    // in the original driver node, this was on a timer - here it should be called on every read loop
    // Let it read and write to all motors' parameters
    bool enableMotor()
    {

    }

private:
    bool sendCanFrame()
    {

    }

    std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
    std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;

    can_msgs::msg::Frame last_joint_command_frame_;
    can_msgs::msg::Frame last_received_frame_;

    std::string can_filters_;
    std::thread receiver_thread_;

};

}

#endif