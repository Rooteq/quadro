// #include "../include/inverse_kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <filesystem>
#include "sensor_msgs/msg/joint_state.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>

#include "position_controller.hpp"
#include "inverse_kinematics.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

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

    const auto package_share_path = ament_index_cpp::get_package_share_directory("quadro");
    const auto urdf_path = std::filesystem::path(package_share_path) / "description" / "quadro.xacro";

    try
    {
      pinocchio::urdf::buildModel(urdf_path, model_, true, true);
      data_ = pinocchio::Data(model_);

      RCLCPP_INFO(this->get_logger(), "Loaded robot model with %d joints (nq=%d, nv=%d)", 
                  model_.njoints, model_.nq, model_.nv);

      // Print joint names for debugging
      for (int i = 1; i < model_.njoints; ++i) {
          RCLCPP_INFO(this->get_logger(), "Joint %d: %s", i, model_.names[i].c_str());
      }

      num_joints_ = model_.nq;

      q_desired_ = Eigen::VectorXd::Zero(model_.nq);
      v_desired_ = Eigen::VectorXd::Zero(model_.nv);

      q_ = Eigen::VectorXd::Zero(model_.nq);
      dq_ = Eigen::VectorXd::Zero(model_.nv);
      q_ref_ = Eigen::VectorXd::Zero(model_.nq);
      dq_ref_ = Eigen::VectorXd::Zero(model_.nv);
      q_ref_prev_ = Eigen::VectorXd::Zero(model_.nq);

      kp_ = Eigen::VectorXd::Constant(model_.nq, 0.9);
      kd_ = Eigen::VectorXd::Constant(model_.nv, 0.05);


    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load robot model: %s", e.what());
      return;
    }

    // Create publisher with reliable QoS
    auto qos = rclcpp::QoS(1).reliable();
    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_effort_controller/commands", qos);

    // Create joystick subscriber
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&TrajectoryPublisher::joy_callback, this, std::placeholders::_1));

    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TrajectoryPublisher::jointStateCallback, this, std::placeholders::_1));

    // num_of_points = 60;
    // positions.resize(12);
    positions_.resize(num_joints_);
    previous_positions_.resize(num_joints_, 0.0);

    // control_timer_ = this->create_wall_timer(
    // std::chrono::milliseconds(static_cast<int>(total_duration*1000) + 10), std::bind(&TrajectoryPublisher::control_loop, this));
    control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<const int>(control_period * 1000)), std::bind(&TrajectoryPublisher::control_loop, this));

    // controller_control_timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&TrajectoryPublisher::apply_control, this));


    startup_time_ = this->get_clock()->now();
  }

private:

  void apply_control()
  {

      // ======   DYNAMIC CONTROLLER ======
      
      Eigen::VectorXd q_error = q_ref_ - q_;
      Eigen::VectorXd v_error = dq_ref_ - dq_;

      Eigen::VectorXd tau_pd = kp_.cwiseProduct(q_error) + kd_.cwiseProduct(v_error);
      
      // This computes inverse dynamics tau = M(q)*q_dd + C(q,q_d) + G(q)
      // When inputting q_d = 0 and q_dd = 0, we get just gravity term
      Eigen::VectorXd tau_gravity = pinocchio::rnea(model_, data_, q_, 
                                                      Eigen::VectorXd::Zero(num_joints_), 
                                                      Eigen::VectorXd::Zero(num_joints_));

      // Eigen::VectorXd friction_compensation = k_damp * dq;

      Eigen::VectorXd tau = tau_pd;// + tau_gravity;// + friction_compensation;
      
      // Apply torque saturation - reduced for stability
      const double max_torque = 5.0;  // Reduced from 8.0 to prevent wild oscillations
      for (int i = 0; i < tau.size(); ++i) {
          if (std::abs(tau[i]) > max_torque) {
              tau[i] = std::copysign(max_torque, tau[i]);
          }
      }
      
      auto torque_msg = std_msgs::msg::Float64MultiArray();
      torque_msg.data.resize(num_joints_);
      for(unsigned int i = 0; i < num_joints_; ++i)
      {
          torque_msg.data[i] = tau[i];
      }
      torque_pub_->publish(torque_msg);
  }


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

  void control_loop()
  {
    if (!got_state) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Waiting for joint states...");
      return;
    }

    if(this->get_clock()->now().seconds() - startup_time_.seconds() < 2)
    {
      RCLCPP_INFO(this->get_logger(), "STARTUP");
      crawl_controller->startup();
      set_joints();
      apply_velocity_limiting();
      // position_pubilsher_->publish(positions);
      apply_control();
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
    // position_pubilsher_->publish(positions);
    apply_control();
  }

  void set_joints() // Make it more streamlined xd
  {
    unsigned int i = 0;
    for(Leg leg_enum : legIterator())
    {
        positions_[i++] = crawl_controller->get_leg_joint_positions(leg_enum).q1;
        positions_[i++] = crawl_controller->get_leg_joint_positions(leg_enum).q2;
        positions_[i++] = crawl_controller->get_leg_joint_positions(leg_enum).q3;
    }

    // Copy to Eigen vector for control
    for (int j = 0; j < num_joints_; ++j) {
      q_ref_[j] = positions_[j];
    }

    // Calculate reference velocity by differentiation
    dq_ref_ = (q_ref_ - q_ref_prev_) / control_period;
    q_ref_prev_ = q_ref_;
  }

  void apply_velocity_limiting()
  {
    const double dt = control_period; // 50ms timer period
    
    for (size_t i = 0; i < positions_.size(); ++i)
    {
      double target_position = positions_[i];
      double current_position = previous_positions_[i];
      double position_diff = target_position - current_position;
      double max_position_change = max_joint_velocity_ * dt;
      
      // Limit the position change based on max velocity
      if (std::abs(position_diff) > max_position_change)
      {
        if (position_diff > 0)
          positions_[i] = current_position + max_position_change;
        else
          positions_[i] = current_position - max_position_change;
      }
      
      // Update previous position for next iteration
      previous_positions_[i] = positions_[i];
    }
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
      if (msg->position.size() != model_.nq) {
          RCLCPP_WARN(this->get_logger(), "Incomplete joint state message");
          return;
      }

      q_ = Eigen::VectorXd::Map(msg->position.data(), model_.nq);
      dq_ = Eigen::VectorXd::Map(msg->position.data(), model_.nv);

      if (!got_state) 
      {
          q_ref_ = q_;
          got_state = true;
          RCLCPP_INFO(this->get_logger(), "Got initial joint states");
      }
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pubilsher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

  rclcpp::TimerBase::SharedPtr control_timer_;
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

  // std_msgs::msg::Float64MultiArray positions = std_msgs::msg::Float64MultiArray();
  std::vector<double> positions_;
  std::vector<double> previous_positions_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;

  // std::array<double> positions;
  int num_of_points;
  const double total_duration;
  const double max_joint_velocity_;

  // InverseKinematics ik;
  std::unique_ptr<GaitController> crawl_controller = std::make_unique<WalkController>();

  const double control_period;

  rclcpp::Time startup_time_;

  int num_joints_;

  // ======== DYNAMICS VARIABLES ========
  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex ee_frame_id_;

  rclcpp::TimerBase::SharedPtr controller_control_timer_;

  Eigen::VectorXd q_, dq_;
  Eigen::VectorXd q_ref_, dq_ref_, q_ref_prev_;


  Eigen::VectorXd kp_, kd_;
  Eigen::Matrix3d Kx_;

  Eigen::VectorXd q_desired_, v_desired_;

  double k_damp = 0.01;
  double k_stiff = 0.1;

  bool got_state = false;

  const double dt_ = 0.005;
  double t_ = 0.0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}