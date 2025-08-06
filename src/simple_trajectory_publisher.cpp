// #include "../include/inverse_kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "position_controller.hpp"

using namespace std::chrono_literals;
using namespace IK;

class TrajectoryPublisher : public rclcpp::Node
{
public:
  TrajectoryPublisher()
  : Node("trajectory_publisher"), position_(0.0), total_duration(3.0), 
    current_roll_(0.0), current_pitch_(0.0), current_yaw_(0.0), rotation_enabled_(false),
    max_joint_velocity_(0.5), control_period(0.03)
  {
    // Create publisher with reliable QoS
    auto qos = rclcpp::QoS(1).reliable();
    position_pubilsher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", qos);

    // Create joystick subscriber
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&TrajectoryPublisher::joy_callback, this, std::placeholders::_1));

    // num_of_points = 60;
    // positions.resize(12);
    positions.data.resize(12);
    previous_positions_.resize(12, 0.0);

    // timer_ = this->create_wall_timer(
    // std::chrono::milliseconds(static_cast<int>(total_duration*1000) + 10), std::bind(&TrajectoryPublisher::timer_callback, this));
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<const int>(control_period * 1000)), std::bind(&TrajectoryPublisher::timer_callback, this));

    time = this->get_clock()->now();
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Button 5 (index 4) to enable/disable rotation
    if (msg->buttons.size() > 10) {
      rotation_enabled_ = msg->buttons[9] > 0;
    }

    // Only update rotation if button 5 is pressed
    if (rotation_enabled_) {
      // Left stick for yaw (axis 0) and pitch (axis 1)
      if (msg->axes.size() > 1) {
        current_yaw_ = msg->axes[0] * 0.3;    // Scale to reasonable rotation range
        current_pitch_ = msg->axes[1] * 0.3;
      }
      
      // Right stick for roll (axis 3)
      if (msg->axes.size() > 3) {
        current_roll_ = msg->axes[2] * 0.2;
      }
    } else {
      // Reset rotations when button is not pressed
      current_roll_ = 0.0;
      current_pitch_ = 0.0;
      current_yaw_ = 0.0;
    }
  }

  void timer_callback()
  {
    if(this->get_clock()->now().seconds() - time.seconds() < 2)
    {
      RCLCPP_INFO(this->get_logger(), "STARTUP");
      pos_controller.startup();
      set_joints();
      apply_velocity_limiting();
      position_pubilsher_->publish(positions);
      return;
    }
    else
    {
      pos_controller.set_rotation(current_roll_, current_pitch_, current_yaw_);
      pos_controller.apply_control();
    }
    // double angle_increment = 2.0 * M_PI / num_of_points;
    // double time_increment = total_duration / num_of_points;

    set_joints();
    // apply_velocity_limiting();
    position_pubilsher_->publish(positions);
  }

  void set_joints() // Make it more streamlined xd
  {
    unsigned int i = 0;
    for(Leg leg_enum : legIterator())
    {
      positions.data[i++] = pos_controller.get_leg_joint_positions(leg_enum).q1;
      positions.data[i++] = pos_controller.get_leg_joint_positions(leg_enum).q2;
      positions.data[i++] = pos_controller.get_leg_joint_positions(leg_enum).q3;
    }
  }

  void apply_velocity_limiting()
  {
    const double dt = control_period; // 50ms timer period
    
    for (size_t i = 0; i < positions.data.size(); ++i)
    {
      double target_position = positions.data[i];
      double current_position = previous_positions_[i];
      double position_diff = target_position - current_position;
      double max_position_change = max_joint_velocity_ * dt;
      
      // Limit the position change based on max velocity
      if (std::abs(position_diff) > max_position_change)
      {
        if (position_diff > 0)
          positions.data[i] = current_position + max_position_change;
        else
          positions.data[i] = current_position - max_position_change;
      }
      
      // Update previous position for next iteration
      previous_positions_[i] = positions.data[i];
    }
  }
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pubilsher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  double position_;
  
  // Joystick control variables
  double current_roll_;
  double current_pitch_;
  double current_yaw_;
  bool rotation_enabled_;

  std_msgs::msg::Float64MultiArray positions = std_msgs::msg::Float64MultiArray();
  // std::array<double> positions;
  int num_of_points;
  const double total_duration;
  const double max_joint_velocity_;
  std::vector<double> previous_positions_;

  // InverseKinematics ik;
  PositionController pos_controller;

  const double control_period;

  rclcpp::Time time;

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}