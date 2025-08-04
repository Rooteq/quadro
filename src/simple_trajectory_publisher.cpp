// #include "../include/inverse_kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "inverse_kinematics.hpp"

using namespace std::chrono_literals;
using namespace IK;
typedef Iterator<Leg, Leg::FL, Leg::BR> legIterator;

class TrajectoryPublisher : public rclcpp::Node
{
public:
  TrajectoryPublisher()
  : Node("trajectory_publisher"), position_(0.0), total_duration(3.0), 
    current_roll_(0.0), current_pitch_(0.0), current_yaw_(0.0), rotation_enabled_(false)
  {
    // Create publisher with reliable QoS
    auto qos = rclcpp::QoS(1).reliable();
    trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", qos);

    // Create joystick subscriber
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&TrajectoryPublisher::joy_callback, this, std::placeholders::_1));

    num_of_points = 60;
    positions.resize(12);

    // timer_ = this->create_wall_timer(
    // std::chrono::milliseconds(static_cast<int>(total_duration*1000) + 10), std::bind(&TrajectoryPublisher::timer_callback, this));
    timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&TrajectoryPublisher::timer_callback, this));
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
        current_roll_ = msg->axes[1] * 0.3;
      }
      
      // Right stick for roll (axis 3)
      if (msg->axes.size() > 3) {
        current_pitch_ = msg->axes[3] * 0.3;
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
    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    // trajectory_msg.joint_names = {"joint_br_1", "joint_br_2", "joint_br_3", "joint_bl_1", "joint_bl_2", "joint_bl_3"}; // Replace with your joint name

    trajectory_msg.joint_names = {"fl_m1_s1","fl_m2_s2","fl_m3_s3","fr_m1_s1","fr_m2_s2","fr_m3_s3","bl_m1_s1","bl_m2_s2","bl_m3_s3","br_m1_s1","br_m2_s2","br_m3_s3"};

    double angle_increment = 2.0 * M_PI / num_of_points;
    double time_increment = total_duration / num_of_points;

    double roll = current_roll_;
    double pitch = current_pitch_;
    double yaw = current_yaw_;

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

    auto point = trajectory_msgs::msg::JointTrajectoryPoint();

    ik.calcJointPositions(Leg::FR, fr_xyz_bis.x(), fr_xyz_bis.y(), fr_xyz_bis.z());
    ik.calcJointPositions(Leg::FL, fl_xyz_bis.x(), fl_xyz_bis.y(), fl_xyz_bis.z());
    ik.calcJointPositions(Leg::BR, br_xyz_bis.x(), br_xyz_bis.y(), br_xyz_bis.z());
    ik.calcJointPositions(Leg::BL, bl_xyz_bis.x(), bl_xyz_bis.y(), bl_xyz_bis.z());
    set_joints();

    point.positions = positions;
    double point_time = 0.1; // Small delay for trajectory execution
    point.time_from_start.sec = static_cast<int>(point_time);
    point.time_from_start.nanosec = static_cast<uint32_t>((point_time - static_cast<int>(point_time)) * 1e9);

    trajectory_msg.points.push_back(point);
    trajectory_publisher_->publish(trajectory_msg);
  }

  void set_joints() // Make it more streamlined xd
  {
    unsigned int i = 0;
    for(Leg leg_enum : legIterator())
    {
      positions[i++] = ik.legs[leg_enum].q1;
      positions[i++] = ik.legs[leg_enum].q2;
      positions[i++] = ik.legs[leg_enum].q3;
    }
  }
  //   for(int i = 0; i < num_of_points; ++i)
  //   {
  //     auto point = trajectory_msgs::msg::JointTrajectoryPoint();

  //     double angle = i * angle_increment;
  //     double point_time = i * time_increment;

  //     // ik_get_leg_joints(0.0, origin_y + radius * std::cos(angle), origin_z + radius*std::sin(angle));
  //     // ik_fr_leg_joints(0.0, 0.0, -0.25);
  //     // ik_fl_leg_joints(0.0, 0.0, -0.25);

  //     // ik_br_leg_joints(0.0, 0.0, -0.25);
  //     // ik_bl_leg_joints(0.0, 0.0, -0.25);
  //     // ik_get_3dof_joints_pos(origin_x + radius * std::cos(angle), origin_y + radius*std::sin(angle), 0.0);
  //     ik_fr_leg_joints(fr_xyz_bis.x(), fr_xyz_bis.y(), fr_xyz_bis.z());
  //     ik_fl_leg_joints(fl_xyz_bis.x(), fl_xyz_bis.y(), fl_xyz_bis.z());
  //     ik_br_leg_joints(br_xyz_bis.x(), br_xyz_bis.y(), br_xyz_bis.z());
  //     ik_bl_leg_joints(bl_xyz_bis.x(), bl_xyz_bis.y(), bl_xyz_bis.z());

  //     point.positions = positions;
  //     point.time_from_start.sec = static_cast<int>(point_time);
  //     point.time_from_start.nanosec = static_cast<uint32_t>((point_time - static_cast<int>(point_time)) * 1e9);

  //     trajectory_msg.points.push_back(point);
  //   }

  //   trajectory_publisher_->publish(trajectory_msg);
  // }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  double position_;
  
  // Joystick control variables
  double current_roll_;
  double current_pitch_;
  double current_yaw_;
  bool rotation_enabled_;

  std::vector<double> positions;
  int num_of_points;
  const double total_duration;

  InverseKinematics ik;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}