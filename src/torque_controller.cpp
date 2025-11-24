#pragma optimize("", off)

#include <memory>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
//#include "pinocchio/algorithm/rnea.hpp"
//#include "pinocchio/parsers/urdf.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class ForwardKinematicsNode : public rclcpp::Node
{
public:
    ForwardKinematicsNode() : Node("forward_kinematics_node")
    {
        
        const auto package_share_path = ament_index_cpp::get_package_share_directory("quadro");
        const auto urdf_path = std::filesystem::path(package_share_path) / "description" / "robot.xacro";
        
        // Load robot model
        try {
            
            pinocchio::urdf::buildModel(urdf_path, model_, true, true);
            data_ = std::make_unique<pinocchio::Data>(model_);
            RCLCPP_INFO(this->get_logger(), "joints: %d", model_.njoints);

            RCLCPP_INFO(this->get_logger(), "nq (positions): %d", model_.nq);
            RCLCPP_INFO(this->get_logger(), "nv (velocities): %d", model_.nv);
            
            q = Eigen::VectorXd::Zero(model_.nq);
            q_d = Eigen::VectorXd::Zero(model_.nv);

            ref_q = Eigen::VectorXd::Zero(model_.nq);
            ref_q_d = Eigen::VectorXd::Zero(model_.nv);
            ref_q_dd = Eigen::VectorXd::Zero(model_.nv);

            v = Eigen::VectorXd::Zero(model_.nv);
            tau = Eigen::VectorXd::Zero(model_.nv);

            e = Eigen::VectorXd::Zero(model_.nq);
            e_d = Eigen::VectorXd::Zero(model_.nv);

            Kp = Eigen::MatrixXd::Zero(model_.nq,model_.nq);
            Kd = Eigen::MatrixXd::Zero(model_.nv,model_.nv);

            Kp.fill(200.0f);
            Kd.fill(50.0f);

            // Kp(0,1) = 0.0f;
            // Kp(1,1) = 0.0f;

            // Kd(0,1) = 0.0f;
            
            RCLCPP_INFO(this->get_logger(), "Robot model loaded from: %s", urdf_path.c_str());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model: %s", e.what());
            return;
        }
        
        // Create subscriber for joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ForwardKinematicsNode::jointStateCallback, this, std::placeholders::_1));
        
        // Create publisher for torque commands
        torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/effort_controller/commands", 10);
        RCLCPP_INFO(this->get_logger(), "Forward kinematics node initialized");
    }

private:
    void get_trajectory_setpoint() // Replace fill!!!!
    {
        double t = this->get_clock()->now().seconds();

        // RCLCPP_INFO(this->get_logger(), "time %f", t);
        double coeff = t*0.20;
        ref_q[0] = 0.5*std::sin(coeff) + 1.0;
        ref_q_d[0] = 0.5*0.2*std::cos(coeff);
        ref_q_dd[0] = -0.04*0.5*std::sin(coeff);

        ref_q[1] = -0.5*std::sin(coeff) - 1.0;
        ref_q_d[1] = -0.2*0.5*std::cos(coeff);
        ref_q_dd[1] = 0.04*0.5*std::sin(coeff);

        ref_q[2] = 0.5*std::sin(coeff) + 1.0;
        ref_q_d[2] = 0.2*0.5*std::cos(coeff);
        ref_q_dd[2] = -0.04*0.5*std::sin(coeff);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Check if we have the right number of joint positions
        // if (msg->position.size() != static_cast<size_t>(model_.nq)) {
        //     RCLCPP_WARN(this->get_logger(), 
        //                "Expected %d joint positions, got %zu", 
        //                model_.nq, msg->position.size());
        //     return;
        // }
        
        // Convert joint positions to Eigen vector
        // Eigen::VectorXd q(model_.nq);
        // Eigen::VectorXd q_d(model_.nv);
        // for (size_t i = 0; i < msg->position.size(); ++i) {
        //     q[i] = msg->position[i];
        //     q_d[i] = msg->velocity[i];

        //     RCLCPP_INFO(this->get_logger(), "velocity %ld: %f", i, q_d[i]);
        // }
        //     RCLCPP_INFO(this->get_logger(), "\n");


            // Fill position vector (size 2)

        // for (size_t i = 0; i < msg->position.size() && i < static_cast<size_t>(model_.nq); ++i) {
        //     q[i] = msg->position[i];
        // }
        
        // // Fill velocity vector (size 1) 
        // for (size_t i = 0; i < msg->velocity.size() && i < static_cast<size_t>(model_.nv); ++i) {
        //     q_d[i] = msg->velocity[i];
        // }

        // // get_trajectory_setpoint();
        // get_trajectory_setpoint();

        // e = ref_q - q;
        // e_d = ref_q_d - q_d;

        // Eigen::VectorXd pos_control = Kp * e;  // Size 2
        
        // // Extract only the velocity-space component (usually the last nv elements)
        // v = ref_q_dd + pos_control.head(3) + Kd * e_d; 

        // // v = ref_q_dd + Kp * e + Kd * e_d; 
        
        // // RCLCPP_INFO(this->get_logger(), "v size: %ld", v.size());
        // // RCLCPP_INFO(this->get_logger(), "ref_q_dd dim: %ld", ref_q_dd.size());
        // // RCLCPP_INFO(this->get_logger(), "Kp rows: %ld", ref_q_dd.size());

        // tau = pinocchio::rnea(model_, *data_, q, q_d, v);
        // // Publish torque (size 1)
        // auto torque_msg = std_msgs::msg::Float64MultiArray();
        // torque_msg.data.resize(3);
        // for (int i = 0; i < 3; ++i) {
        //     torque_msg.data[i] = tau[i];
        // }
        // torque_pub_->publish(torque_msg);


    }
    
    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    pinocchio::FrameIndex ee_frame_id_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;

    Eigen::VectorXd q;
    Eigen::VectorXd q_d;

    Eigen::VectorXd v;
    Eigen::VectorXd tau;

    Eigen::VectorXd ref_q;
    Eigen::VectorXd ref_q_d;
    Eigen::VectorXd ref_q_dd;

    Eigen::MatrixXd Kp;
    Eigen::MatrixXd Kd;

    Eigen::VectorXd e;
    Eigen::VectorXd e_d;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForwardKinematicsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
