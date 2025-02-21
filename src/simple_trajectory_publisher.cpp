#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::chrono_literals;

class TrajectoryPublisher : public rclcpp::Node
{
public:
  TrajectoryPublisher()
  : Node("trajectory_publisher"), position_(0.0), going_up_(true)
  {
    // Create publisher with reliable QoS
    auto qos = rclcpp::QoS(1).reliable();
    trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", qos);

    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
      4s, std::bind(&TrajectoryPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    trajectory_msg.joint_names = {"cybergear"}; // Replace with your joint name

    const int num_points = 20;  // Number of intermediate points
    const double max_vel = 2.0; // Maximum velocity (rad/s)
    const double max_acc = 2.0; // Maximum acceleration (rad/s^2)
    
    // Calculate start and end positions
    double start_pos = going_up_ ? 0.0 : M_PI / 2.0;
    double end_pos = going_up_ ? M_PI / 2.0 : 0.0;
    
    // Generate smooth trajectory points
    for (int i = 0; i < num_points; ++i) {
      auto point = trajectory_msgs::msg::JointTrajectoryPoint();
      
      // Normalized time (0 to 1)
      double t = static_cast<double>(i) / (num_points - 1);
      
      // Use smooth S-curve (quintic polynomial) for position
      double s = 10 * std::pow(t, 3) - 15 * std::pow(t, 4) + 6 * std::pow(t, 5);
      
      // Calculate position
      double pos = start_pos + (end_pos - start_pos) * s;
      
      // Calculate velocity using derivative of position curve
      double ds_dt = 30 * std::pow(t, 2) - 60 * std::pow(t, 3) + 30 * std::pow(t, 4);
      double vel = (end_pos - start_pos) * ds_dt / 4.0;  // Divide by duration (4s)
      
      // Calculate acceleration using second derivative
      double d2s_dt2 = 60 * t - 180 * std::pow(t, 2) + 120 * std::pow(t, 3);
      double acc = (end_pos - start_pos) * d2s_dt2 / 16.0;  // Divide by duration squared
      
      // Clamp velocity and acceleration to limits
      vel = std::clamp(vel, -max_vel, max_vel);
      acc = std::clamp(acc, -max_acc, max_acc);
      
      point.positions = {pos};
      point.velocities = {vel};
      point.accelerations = {acc};
      
      // Set time for this point (total duration = 4s)
      point.time_from_start.sec = static_cast<int>(4.0 * t);
      point.time_from_start.nanosec = 
        static_cast<uint32_t>((4.0 * t - static_cast<int>(4.0 * t)) * 1e9);
      
      trajectory_msg.points.push_back(point);
    }

    // Toggle direction for next cycle
    going_up_ = !going_up_;

    // Publish trajectory
    trajectory_publisher_->publish(trajectory_msg);
    RCLCPP_INFO(this->get_logger(), "Published smooth trajectory from %f to %f", 
                start_pos, end_pos);
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double position_;
  bool going_up_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}