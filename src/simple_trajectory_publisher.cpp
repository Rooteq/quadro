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
  : Node("trajectory_publisher"), position_(0.0), total_duration(3.0)
  {
    // Create publisher with reliable QoS
    auto qos = rclcpp::QoS(1).reliable();
    trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", qos);


    num_of_points = 60;
    positions.resize(3);

    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(total_duration*1000) + 10), std::bind(&TrajectoryPublisher::timer_callback, this));
  }

private:
  void ik_get_joints_pos(const double x, const double y)
  {
    double joint_offset_1 = 0.349;  // Offset for joint 1
    double joint_offset_2 = 0.785;  // Offset for joint 2
    double q1, q2;
    
    // Check if the target is reachable
    double distance = std::sqrt(x*x + y*y);
    if (distance > (l1 + l2) || distance < std::abs(l1 - l2)) {
      RCLCPP_INFO(get_logger(), "Target position out of reach!");
        // std::cout << "Target position out of reach!" << std::endl;
        return;
    }
    
    double cos_q2 = (x*x + y*y - l1*l1 - l2*l2) / (2 * l1 * l2);
    q2 = std::acos(cos_q2);

    double k1 = l1 + l2 * std::cos(q2);
    double k2 = l2 * std::sin(q2);
    
    q1 = std::atan2(y, x) - std::atan2(k2, k1);
    
    double robot_q1 = q1 - joint_offset_1;
    double robot_q2 = M_PI - q2 - joint_offset_2;

    if(robot_q1 < 0 || robot_q1 > 1.2)
    {
      RCLCPP_INFO(get_logger(), "Wrong joint 1 setting");
      return;
    }
    if(robot_q2 < 0 || robot_q2 > 1.2)
    {
      RCLCPP_INFO(get_logger(), "Wrong joint 2 setting");
      return;
    }

    // RCLCPP_INFO(get_logger(), "Setting pos to: %f, %f", robot_q1, robot_q2);
    
    positions[0] = 0.0;
    positions[1] = robot_q1;
    positions[2] = robot_q2;
  }

  void ik_get_3dof_joints_pos(const double x, const double y, const double z)
  {
      double q1, q2, q3;
      double alfa, beta;
      
      // Check if the target is reachable
      double distance = std::sqrt(x*x + y*y);
      if (distance > (l1 + l2) || distance < std::abs(l1 - l2)) {
        RCLCPP_INFO(get_logger(), "Target position out of reach!");
          // std::cout << "Target position out of reach!" << std::endl;
          return;
      }

      double denom = std::sqrt(y*y + z*z);
      alfa = std::acos(std::abs(z)/(denom));
      beta = std::acos(l1/denom);

      if(z > 0)
      {
          q1 = alfa-beta;
      }
      else if(z < 0)
      {
          q1 = M_PI - alfa - beta;
      }
      else
      {
          q1 = 0.0;
      }

      double x_prim = x;
      double y_prim = -std::sqrt(y*y+z*z - l1*l1);
      // -------------- 
      double cos_q3 = (x_prim*x_prim + y_prim*y_prim - l2*l2 - l3*l3) / (2 * l2 * l3);
      q3 = std::acos(cos_q3);

      double k1 = l2 + l3 * std::cos(q3);
      double k2 = l3 * std::sin(q3);
      
      q2 = std::atan2(y_prim, x_prim) - std::atan2(k2, k1) + M_PI;

      double robot_q2 = q2 - joint_offset_2;
      double robot_q3 = M_PI - q3 - joint_offset_3;


    if(q1 < -1.57 || q1 > 1.57)
    {
      RCLCPP_INFO(get_logger(), "Wrong joint 1 setting");
      return;
    }
    if(robot_q3 < 0 || robot_q3 > 1.2)
    {
      RCLCPP_INFO(get_logger(), "Wrong joint 2 setting");
      return;
    }
    if(robot_q3 < 0 || robot_q3 > 1.2)
    {
      RCLCPP_INFO(get_logger(), "Wrong joint 3 setting");
      return;
    }
    positions[0] = q1;
    positions[1] = robot_q2;
    positions[2] = robot_q3;
  }

  void timer_callback()
  {
    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    trajectory_msg.joint_names = {"BR1", "BR2", "BR3"}; // Replace with your joint name


    double radius = 0.04;
    double origin_x = 0.00;
    double origin_y = -0.20;
    double origin_z = 0.062;

    double angle_increment = 2.0 * M_PI / num_of_points;
    double time_increment = total_duration / num_of_points;

    for(int i = 0; i < num_of_points; ++i)
    {
      auto point = trajectory_msgs::msg::JointTrajectoryPoint();

      double angle = i * angle_increment;
      double point_time = i * time_increment;

      // ik_get_3dof_joints_pos(0.0, origin_y + radius * std::cos(angle), origin_z + radius*std::sin(angle));
      ik_get_3dof_joints_pos(origin_x + radius * std::cos(angle), origin_y + radius*std::sin(angle), 0.04);

      point.positions = positions;
      point.time_from_start.sec = static_cast<int>(point_time);
      point.time_from_start.nanosec = static_cast<uint32_t>((point_time - static_cast<int>(point_time)) * 1e9);

      trajectory_msg.points.push_back(point);
    }

    trajectory_publisher_->publish(trajectory_msg);
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double position_;

  std::vector<double> positions;
  int num_of_points;
  const double total_duration;

  double l1 = 0.062;
  double l2 = 0.2;   // Length of first link
  double l3 = 0.15;  // Length of second link

  double joint_offset_2 = 0.349;  // Offset for joint 1
  double joint_offset_3 = 0.785;  // Offset for joint 2

  unsigned int corner = 0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}