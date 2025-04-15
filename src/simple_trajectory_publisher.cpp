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

  void ik_get_3dof_joints_pos(const double x, const double y, const double z)
  {
    double q1, q2, q3;
    double alfa, beta;
    
    // Check if the target is reachable
    double distance = std::sqrt(x*x + y*y);
    if (distance > (l2 + l3) || distance < std::abs(l2 - l3)) {
      RCLCPP_INFO(get_logger(), "Position out of reach!");
        // std::cout << "Target position out of reach!" << std::endl;
        return;
    }

    double AG = std::sqrt(y*y + z*z - l1*l1);
    double OA = l1;
    
    q1 = M_PI - std::atan2(AG,OA) - std::atan2(-y,-z);

    double GC = x;
    double AC = std::sqrt(AG*AG + GC*GC);

    double cos_alpha = -((AC*AC - l2*l2 - l3*l3)/(2*l2*l3));

    alfa = std::acos(cos_alpha);

    q3 = M_PI - alfa;
    q2 = std::atan2(GC,AG) - std::atan2(l3*std::sin(q3), l2+l3*std::cos(q3));

    double robot_q1 = q1;
    double robot_q2 = joint_2_starting + q2 - joint_offset_2;
    double robot_q3 = M_PI - q3 - joint_offset_3;

    if(q1 < -1.57 || q1 > 1.57)
    {
      RCLCPP_INFO(get_logger(), "Wrong joint 1 setting");
      return;
    }
    if(robot_q2 < 0 || robot_q2 > 1.3)
    {
      RCLCPP_INFO(get_logger(), "Wrong joint 2 setting");
      return;
    }
    if(robot_q3 < 0 || robot_q3 > 1.3)
    {
      RCLCPP_INFO(get_logger(), "Wrong joint 3 setting");
      return;
    }
    positions[0] = robot_q1;
    positions[1] = robot_q2;
    positions[2] = robot_q3;
  }

  void timer_callback()
  {
    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    trajectory_msg.joint_names = {"BR1", "BR2", "BR3"}; // Replace with your joint name


    double radius = 0.05;
    double origin_x = 0.05;
    double origin_y = -0.20;
    double origin_z = 0.062;

    double angle_increment = 2.0 * M_PI / num_of_points;
    double time_increment = total_duration / num_of_points;

    for(int i = 0; i < num_of_points; ++i)
    {
      auto point = trajectory_msgs::msg::JointTrajectoryPoint();

      double angle = i * angle_increment;
      double point_time = i * time_increment;

      ik_get_3dof_joints_pos(0.0, origin_y + radius * std::cos(angle), origin_z + radius*std::sin(angle));
      // ik_get_3dof_joints_pos(origin_x + radius * std::cos(angle), origin_y + radius*std::sin(angle), 0.0);

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

  double l1 = 0.0;//0.062;
  double l2 = 0.2;   // Length of first link
  double l3 = 0.15;  // Length of second link

  double joint_offset_2 = 0.349;  // Offset for joint 1
  double joint_offset_3 = 0.785;  // Offset for joint 2

  double joint_2_starting = M_PI/2;

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