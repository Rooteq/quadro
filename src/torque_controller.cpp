#pragma optimize("", off)

#include <memory>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class TorqueController : public rclcpp::Node
{
public:
    TorqueController() : Node("torque_controller")
    {
        
        const auto package_share_path = ament_index_cpp::get_package_share_directory("quadro");
        const auto urdf_path = std::filesystem::path(package_share_path) / "description" / "leg.xacro";
        
        // Load robot model
        try {
            
            pinocchio::urdf::buildModel(urdf_path, model_, true, true);
            data_ = std::make_unique<pinocchio::Data>(model_);
            // RCLCPP_INFO(this->get_logger(), "num joints: %d", model_.njoints);

            // RCLCPP_INFO(this->get_logger(), "nq (positions): %d", model_.nq);
            // RCLCPP_INFO(this->get_logger(), "nv (velocities): %d", model_.nv);

            // Set desired position to zero (middle of joint range)
            q_desired = Eigen::VectorXd::Zero(model_.nq);
            v_desired = Eigen::VectorXd::Zero(model_.nv);

            kp = Eigen::VectorXd(model_.nq);
            kd = Eigen::VectorXd(model_.nv);

            // Control all 3 joints with gravity compensation
            kp << 0.2, 0.2, 0.2;      // Moderate position gains
            kd << 0.1, 0.1, 0.1;      // Damping gains
            
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model: %s", e.what());
            return;
        }
        
        // Create subscriber for joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TorqueController::jointStateCallback, this, std::placeholders::_1));
        
        // Create publisher for torque commands
        torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_effort_controller/commands", 10);
        RCLCPP_INFO(this->get_logger(), "Torque controller initialized");
    }

private:
    void get_trajectory_setpoint() // Replace fill!!!!
    {
        double t = this->get_clock()->now().seconds();

        // // RCLCPP_INFO(this->get_logger(), "time %f", t);
        // double coeff = t*0.20;
        // ref_q[0] = 0.5*std::sin(coeff) + 1.0;
        // ref_q_d[0] = 0.5*0.2*std::cos(coeff);
        // ref_q_dd[0] = -0.04*0.5*std::sin(coeff);

        // ref_q[1] = -0.5*std::sin(coeff) - 1.0;
        // ref_q_d[1] = -0.2*0.5*std::cos(coeff);
        // ref_q_dd[1] = 0.04*0.5*std::sin(coeff);

        // ref_q[2] = 0.5*std::sin(coeff) + 1.0;
        // ref_q_d[2] = 0.2*0.5*std::cos(coeff);
        // ref_q_dd[2] = -0.04*0.5*std::sin(coeff);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < model_.nq || msg->velocity.size() < model_.nv) {
            RCLCPP_WARN(this->get_logger(), "Incomplete joint state message");
            return;
        }
        
        Eigen::VectorXd q(model_.nq);
        Eigen::VectorXd v(model_.nv);
        
        // Reorder joints (joint state data is not in order!!!)
        // q[0] = msg->position[1];  // joint1 (bl_m1_s1)
        // q[1] = msg->position[2];  // joint2 (bl_m2_s2)
        // q[2] = msg->position[0];  // joint3 (bl_l4_l3)
        
        // v[0] = msg->velocity[1];
        // v[1] = msg->velocity[2];
        // v[2] = msg->velocity[0];

        q[0] = msg->position[0];  // joint1 (bl_m1_s1)
        q[1] = msg->position[1];  // joint2 (bl_m2_s2)
        q[2] = msg->position[2];  // joint3 (bl_l4_l3)
        
        v[0] = msg->velocity[0];
        v[1] = msg->velocity[1];
        v[2] = msg->velocity[2];

        Eigen::VectorXd q_error = q_desired - q;
        Eigen::VectorXd v_error = v_desired - v;

        Eigen::VectorXd tau_pd = kp.cwiseProduct(q_error) + kd.cwiseProduct(v_error);
        
        // This computes inverse dynamics tau = M(q)*q_dd + C(q,q_d) + G(q)
        // When inputting q_d = 0 and q_dd = 0, we get just gravity term
        Eigen::VectorXd tau_gravity = pinocchio::rnea(model_, *data_, q, 
                                                       Eigen::VectorXd::Zero(model_.nv), 
                                                       Eigen::VectorXd::Zero(model_.nv));


        
        Eigen::VectorXd tau = tau_pd + tau_gravity;
        
        // Apply torque saturation
        const double max_torque = 3.0;
        for (int i = 0; i < tau.size(); ++i) {
            if (std::abs(tau[i]) > max_torque) {
                tau[i] = std::copysign(max_torque, tau[i]);
            }
        }
        
        auto torque_msg = std_msgs::msg::Float64MultiArray();
        torque_msg.data.resize(model_.nq);
        for(unsigned int i = 0; i < model_.nq; ++i)
        {
            torque_msg.data[i] = tau[i];
        }
        torque_pub_->publish(torque_msg);
    }
    
    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    pinocchio::FrameIndex ee_frame_id_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;

    Eigen::VectorXd kp;
    Eigen::VectorXd kd;
    Eigen::VectorXd q_desired;
    Eigen::VectorXd v_desired;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TorqueController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
