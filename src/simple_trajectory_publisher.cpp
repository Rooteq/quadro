// #include "../include/inverse_kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>

#include "position_controller.hpp"

using namespace std::chrono_literals;
using namespace IK;

class TrajectoryPublisher : public rclcpp::Node
{
public:
  TrajectoryPublisher()
  : Node("trajectory_publisher"), position_(0.0), total_duration(3.0), 
    current_roll_(0.0), current_pitch_(0.0), current_yaw_(0.0), rotation_enabled_(false),
    walk_speed_(0.0), yaw_speed_(0.0), walking_enabled_(false), walking_rotation_(0.0),
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
    // Button 10 (index 9) to enable/disable rotation
    if (msg->buttons.size() > 10) {
      rotation_enabled_ = msg->buttons[9] > 0;
    }

    // Only update rotation if button 5 is pressed
    if (rotation_enabled_) {
      // Left stick for yaw (axis 0) and pitch (axis 1)
      if (msg->axes.size() > 1) {
        current_yaw_ = msg->axes[0] * 0.13;    // Scale to reasonable rotation range
        current_pitch_ = msg->axes[1] * 0.13;
        current_roll_ = msg->axes[2] * 0.09;
      }
      RCLCPP_INFO(this->get_logger(), "R: %f, P: %f, Y: %f", current_roll_, current_pitch_, current_yaw_);
    } else {
      // Reset rotations when button is not pressed
      current_roll_ = 0.0;
      current_pitch_ = 0.0;
      current_yaw_ = 0.0;
    }

    // Button 11 (index 10) to enable/disable walking
    if (msg->buttons.size() > 11) {
      walking_enabled_ = msg->buttons[10] > 0;
    }

    // Only update walking parameters if button 11 is pressed
    if (walking_enabled_) {
      // Axes 0 and 1 for x,y speed control
      if (msg->axes.size() > 1) {
        double x_speed_ = msg->axes[0];  // Left stick X
        double y_speed_ = msg->axes[1];  // Left stick Y
        walk_speed_ = std::sqrt(x_speed_*x_speed_ + y_speed_*y_speed_);
        
        // Calculate walking rotation angle from x,y speeds
        // Only update rotation if we have significant movement to avoid jitter
        if (walk_speed_ > 0.1) {
          // Adjust the coordinate system: forward should be stick up (positive Y)
          // So we swap x and y in atan2 and negate to get correct orientation
          walking_rotation_ = std::atan2(x_speed_, y_speed_);
        }
        // RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f, ROT: %f, SPEED: %f", x_speed_, y_speed_, walking_rotation_, walk_speed_);
      }
      
      // Axis 2 for yaw speed control
      if (msg->axes.size() > 2) {
        yaw_speed_ = msg->axes[2];  // Right stick X or trigger
        RCLCPP_INFO(this->get_logger(), "YAW: %f", yaw_speed_);
      }
    } else {
      // Reset walking speeds when button is not pressed
      walk_speed_ = 0.0;
      yaw_speed_ = 0.0;
      walking_rotation_ = 0.0;
    }
  }

  void timer_callback()
  {
    if(this->get_clock()->now().seconds() - time.seconds() < 2)
    {
      RCLCPP_INFO(this->get_logger(), "STARTUP");
      crawl_controller->startup();
      set_joints();
      apply_velocity_limiting();
      position_pubilsher_->publish(positions);
      return;
    }
    else
    {
      crawl_controller->set_rotation(current_roll_, current_pitch_, current_yaw_);
      crawl_controller->set_walking_parameters(walk_speed_, yaw_speed_, walking_rotation_, walking_enabled_);
      crawl_controller->apply_control();
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
      // ISOLATING ONLY BR LEG
      // if(leg_enum == Leg::BL)
      // {
        positions.data[i++] = crawl_controller->get_leg_joint_positions(leg_enum).q1;
        positions.data[i++] = crawl_controller->get_leg_joint_positions(leg_enum).q2;
        positions.data[i++] = crawl_controller->get_leg_joint_positions(leg_enum).q3;
      // }
      // else if(leg_enum == Leg::BR)
      // {
      //   positions.data[i++] = crawl_controller->get_leg_joint_positions(leg_enum).q1;
      //   positions.data[i++] = crawl_controller->get_leg_joint_positions(leg_enum).q2;
      //   positions.data[i++] = crawl_controller->get_leg_joint_positions(leg_enum).q3;
      // }
      // else
      // {
      //   positions.data[i++] = 0.0;
      //   positions.data[i++] = 0.0;
      //   positions.data[i++] = 0.0;
      // }

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
  
  // Walking control variables
  double walk_speed_;
  double yaw_speed_;
  bool walking_enabled_;
  double walking_rotation_;

  std_msgs::msg::Float64MultiArray positions = std_msgs::msg::Float64MultiArray();
  // std::array<double> positions;
  int num_of_points;
  const double total_duration;
  const double max_joint_velocity_;
  std::vector<double> previous_positions_;

  // InverseKinematics ik;
  std::unique_ptr<GaitController> crawl_controller = std::make_unique<WalkController>();

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