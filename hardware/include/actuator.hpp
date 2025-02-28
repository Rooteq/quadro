#include <iostream>

#include "cybergear_driver_core/cybergear_driver_core.hpp"

#ifndef ACTUATOR_HPP
#define ACTUATOR_HPP

namespace quadro
{

// is it even needed?
enum class ACTUATOR_ID
{
    // front right joint 1    
    FR1 = 1,
    FR2 = 2,
    FR3 = 3,
    
    FL1 = 4,
};

struct MotorParams
{
public:
    std::string can_interface_;
    double timeout_sec_;
    bool use_bus_time_;
    double interval_sec_;
    int primary_id_;
};

class Actuator
{
public:
    Actuator(const std::string &frame, const int device_id, const int primary_id)
        : frame_(frame), device_id_(device_id)
    {
        cmd_vel_ = std::numeric_limits<double>::quiet_NaN();
        last_cmd_vel_ = std::numeric_limits<double>::quiet_NaN();
        state_vel_ = 0.0;
        state_pos_ = 0.0;

        cybergear_driver_core::CybergearPacketParam packet_param;
        packet_param.device_id = static_cast<int>(device_id);
        packet_param.primary_id = static_cast<int>(primary_id);

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

        setDefaultCanFrame();

    }

    // TODO: try to avoid copying

    const can_msgs::msg::Frame &getZeroingMessage()
    {
        default_frame_ = packet_->createZeroPosition();
        std::copy(default_frame_.data.cbegin(), default_frame_.data.cend(), default_msg_.data.begin());
        default_msg_.id = default_frame_.id; 
        return default_msg_;
    }
    const can_msgs::msg::Frame &getPositionModeMessage()
    {
        default_frame_ = packet_->createChangeToPositionModeCommand();
        std::copy(default_frame_.data.cbegin(), default_frame_.data.cend(), default_msg_.data.begin());
        default_msg_.id = default_frame_.id; 
        return default_msg_;
    }
    const can_msgs::msg::Frame &getEnableTorqueMessage()
    {
        default_frame_ = packet_->createEnableTorqueCommand();
        std::copy(default_frame_.data.cbegin(), default_frame_.data.cend(), default_msg_.data.begin());
        default_msg_.id = default_frame_.id; 
        return default_msg_;
    }
    const can_msgs::msg::Frame &getDisableTorqueMessage()
    {
        default_frame_ = packet_->createDisableTorqueCommand();
        std::copy(default_frame_.data.cbegin(), default_frame_.data.cend(), default_msg_.data.begin());
        default_msg_.id = default_frame_.id; 
        return default_msg_;
    }

    const can_msgs::msg::Frame &getPositionCommandMessage(double command_pos)
    {
        default_frame_ = packet_->createPositionCommand(command_pos);
        std::copy(default_frame_.data.cbegin(), default_frame_.data.cend(), default_msg_.data.begin());
        default_msg_.id = default_frame_.id; 
        return default_msg_;
    }
    const can_msgs::msg::Frame& getCreateFeedbackMessage()
    {
        default_frame_ = packet_->createGetFeedbackCommand();
        std::copy(default_frame_.data.cbegin(), default_frame_.data.cend(), default_msg_.data.begin());
        default_msg_.id = default_frame_.id; 
        return default_msg_;
    }

public:
    std::unique_ptr<cybergear_driver_core::CybergearPacket> packet_;

    const std::string frame_;
    const int device_id_;
    double cmd_vel_;
    double last_cmd_vel_;

    double state_vel_;
    double state_pos_;

private:
    can_msgs::msg::Frame default_msg_;
    cybergear_driver_core::CanFrame default_frame_;

    void setDefaultCanFrame()
    {
      constexpr uint8_t kDlc = 8;

      default_msg_.is_rtr = false;
      default_msg_.is_extended = true;
      default_msg_.is_error = false;
      default_msg_.dlc = kDlc;
    }
};

} // namespace quadro


#endif