#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <eigen3/Eigen/Dense>

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
    positions.resize(12);

    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(total_duration*1000) + 10), std::bind(&TrajectoryPublisher::timer_callback, this));
  }

private:
  void apply_left_ik(const double x, double y, const double z, double& q1, double& q2, double&q3)
  {
    y = y+l1;
    
    double alfa;

    double AG = std::sqrt(y*y + z*z - l1*l1);
    double OA = l1;
    
    q1 = -(std::atan2(AG,OA) - std::atan2(y,z));

    double GC = x;
    // double GC = x;
    double AC = std::sqrt(AG*AG + GC*GC);

    double cos_alpha = -((AC*AC - l2*l2 - l3*l3)/(2*l2*l3));
    alfa = std::acos(cos_alpha);

    q3 = M_PI - alfa;
    q2 = std::atan2(GC,AG) - std::atan2(l3*std::sin(q3), l2+l3*std::cos(q3));

    if(q1 != q1)
    {
      RCLCPP_INFO(get_logger(), "BR: q1 is NAN");
      return;
    }
    if(q2 != q2)
    {
      RCLCPP_INFO(get_logger(), "BR: q2 is NAN");
      return;
    }
    if(q3 != q3)
    {
      RCLCPP_INFO(get_logger(), "BR: q3 is NAN");
      return;
    }
  }

  void apply_right_ik(const double x, double y, const double z, double& q1, double& q2, double&q3)
  {
    y = -y + l1;
    
    // apply_basic_ik(x,y,z,q1,q2,q3);
        double alfa;
    // Check if the target is reachable
    double distance = std::sqrt(x*x + y*y + z*z);
    if (distance > (l1 + l2 + l3) ){
      RCLCPP_INFO(get_logger(), "Position out of reach!");
        // std::cout << "Target position out of reach!" << std::endl;
        return;
    }

    double AG = std::sqrt(y*y + z*z - l1*l1);
    double OA = l1;
    
    q1 = std::atan2(AG,OA) - std::atan2(y,z);

    double GC = x;
    // double GC = x;
    double AC = std::sqrt(AG*AG + GC*GC);

    double cos_alpha = -((AC*AC - l2*l2 - l3*l3)/(2*l2*l3));
    alfa = std::acos(cos_alpha);

    q3 = M_PI - alfa;
    q2 = std::atan2(GC,AG) - std::atan2(l3*std::sin(q3), l2+l3*std::cos(q3));

    if(q1 != q1)
    {
      RCLCPP_INFO(get_logger(), "BR: q1 is NAN");
      return;
    }
    if(q2 != q2)
    {
      RCLCPP_INFO(get_logger(), "BR: q2 is NAN");
      return;
    }
    if(q3 != q3)
    {
      RCLCPP_INFO(get_logger(), "BR: q3 is NAN");
      return;
    }
  }

  void ik_br_leg_joints(const double x, const double y, const double z)
  {
    double q1, q2, q3;
    
    apply_right_ik(x,y,z,q1,q2,q3);

    double robot_q1 = q1 - joint_offset_1 + M_PI/2;
    double robot_q2 = q2 - joint_offset_2 + M_PI/2;
    double robot_q3 = q3 - joint_offset_3 - M_PI/2;
    
    positions[0] = robot_q1;
    positions[1] = robot_q2;
    positions[2] = robot_q3;
  }

  void ik_fr_leg_joints(const double x, const double y, const double z)
  {
    double q1, q2, q3;
    
    apply_right_ik(x,y,z,q1,q2,q3);

    double robot_q1 = q1 - joint_offset_1 + M_PI/2;
    double robot_q2 = q2 - joint_offset_2 + M_PI/2;
    double robot_q3 = q3 - joint_offset_3 - M_PI/2;
    
    positions[3] = robot_q1;
    positions[4] = robot_q2;
    positions[5] = robot_q3;
  }

  void ik_bl_leg_joints(const double x, const double y, const double z)
  {
    double q1, q2, q3;
    apply_left_ik(x,y,z,q1,q2,q3);
    
    double robot_q1 = q1 + joint_offset_1 - M_PI/2;// + 1.048; //30 deg :*
    double robot_q2 = q2 - joint_offset_2 + M_PI/2;
    double robot_q3 = q3 - joint_offset_3 - M_PI/2;

    // if(q1 < -1.57 || q1 > 1.57)
    // {
    //   RCLCPP_INFO(get_logger(), "Wrong joint 1 setting");
    //   return;
    // }
    // if(robot_q2 < 0 || robot_q2 > 1.3)
    // {
    //   RCLCPP_INFO(get_logger(), "Wrong joint 2 setting");
    //   return;
    // }
    // if(robot_q3 < 0 || robot_q3 > 1.3)
    // {
    //   RCLCPP_INFO(get_logger(), "Wrong joint 3 setting");
    //   return;
    // }
    
    positions[6] = robot_q1;
    positions[7] = robot_q2;
    positions[8] = robot_q3;
  }

  void ik_fl_leg_joints(const double x, const double y, const double z)
  {
    double q1, q2, q3;
    apply_left_ik(x,y,z,q1,q2,q3);

    double robot_q1 = q1 + joint_offset_1 - M_PI/2;// + 1.048; //30 deg :*
    double robot_q2 = q2 - joint_offset_2 + M_PI/2;
    double robot_q3 = q3 - joint_offset_3 - M_PI/2;
    // if(q1 < -1.57 || q1 > 1.57)
    // {
    //   RCLCPP_INFO(get_logger(), "Wrong joint 1 setting");
    //   return;
    // }
    // if(robot_q2 < 0 || robot_q2 > 1.3)
    // {
    //   RCLCPP_INFO(get_logger(), "Wrong joint 2 setting");
    //   return;
    // }
    // if(robot_q3 < 0 || robot_q3 > 1.3)
    // {
    //   RCLCPP_INFO(get_logger(), "Wrong joint 3 setting");
    //   return;
    // }
    
    positions[9] = robot_q1;
    positions[10] = robot_q2;
    positions[11] = robot_q3;
  }

  void timer_callback()
  {
    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    // trajectory_msg.joint_names = {"joint_br_1", "joint_br_2", "joint_br_3", "joint_bl_1", "joint_bl_2", "joint_bl_3"}; // Replace with your joint name

    trajectory_msg.joint_names = {"br_m1_s1","br_m2_s2","br_m3_s3","fr_m1_s1","fr_m2_s2","fr_m3_s3","bl_m1_s1","bl_m2_s2","bl_m3_s3","fl_m1_s1","fl_m2_s2","fl_m3_s3"};
    double radius = 0.05;
    double origin_x = 0.05;
    double origin_y = -0.20;
    double origin_z = 0.062;

    double angle_increment = 2.0 * M_PI / num_of_points;
    double time_increment = total_duration / num_of_points;

    double roll = 0.0f;
    double pitch = 0.2f;
    double yaw = 0.0f;

    Eigen::Vector3d default_leg_pos(0.0, 0.0, -0.25);

    // Leg origins relative to the robot's 0 pos (Z axis might be reversed)
    Eigen::Vector3d br_leg_origin(-0.185, 0.0, 0.0628);
    Eigen::Vector3d fr_leg_origin(0.185, 0.0, 0.0628);
    Eigen::Vector3d bl_leg_origin(-0.185, 0.0, -0.0628);
    Eigen::Vector3d fl_leg_origin(0.185, 0.0, -0.0628);

    // Rotation matrix
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();

    Eigen::Vector3d br_xyz = rotationMatrix.inverse() * (default_leg_pos + br_leg_origin);
    Eigen::Vector3d fr_xyz = rotationMatrix.inverse() * (default_leg_pos + fr_leg_origin);
    Eigen::Vector3d bl_xyz = rotationMatrix.inverse() * (default_leg_pos + bl_leg_origin);
    Eigen::Vector3d fl_xyz = rotationMatrix.inverse() * (default_leg_pos + fl_leg_origin);

    Eigen::Vector3d br_xyz_bis = br_xyz - br_leg_origin;
    Eigen::Vector3d bl_xyz_bis = bl_xyz - bl_leg_origin;
    Eigen::Vector3d fr_xyz_bis = fr_xyz - fr_leg_origin;
    Eigen::Vector3d fl_xyz_bis = fl_xyz - fl_leg_origin;

    for(int i = 0; i < num_of_points; ++i)
    {
      auto point = trajectory_msgs::msg::JointTrajectoryPoint();

      double angle = i * angle_increment;
      double point_time = i * time_increment;

      // ik_get_leg_joints(0.0, origin_y + radius * std::cos(angle), origin_z + radius*std::sin(angle));
      // ik_fr_leg_joints(0.0, 0.0, -0.25);
      // ik_fl_leg_joints(0.0, 0.0, -0.25);

      // ik_br_leg_joints(0.0, 0.0, -0.25);
      // ik_bl_leg_joints(0.0, 0.0, -0.25);
      // ik_get_3dof_joints_pos(origin_x + radius * std::cos(angle), origin_y + radius*std::sin(angle), 0.0);
      ik_fr_leg_joints(fr_xyz_bis.x(), fr_xyz_bis.y(), fr_xyz_bis.z());
      ik_fl_leg_joints(fl_xyz_bis.x(), fl_xyz_bis.y(), fl_xyz_bis.z());
      ik_br_leg_joints(br_xyz_bis.x(), br_xyz_bis.y(), br_xyz_bis.z());
      ik_bl_leg_joints(bl_xyz_bis.x(), bl_xyz_bis.y(), bl_xyz_bis.z());

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

  double l1 = 0.065;//0.062;
  double l2 = 0.2;   // Length of first link
  double l3 = 0.15;  // Length of second link

  double joint_offset_1 = 0.524;
  double joint_offset_2 = 0.349;
  double joint_offset_3 = 0.785;

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