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

#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "inverse_kinematics.hpp"

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
            data_ = pinocchio::Data(model_);
            // RCLCPP_INFO(this->get_logger(), "num joints: %d", model_.njoints);

            // RCLCPP_INFO(this->get_logger(), "nq (positions): %d", model_.nq);
            // RCLCPP_INFO(this->get_logger(), "nv (velocities): %d", model_.nv);

            // Set desired position to zero (middle of joint range)
            q_desired = Eigen::VectorXd::Zero(model_.nq);
            v_desired = Eigen::VectorXd::Zero(model_.nv);

            q = Eigen::VectorXd::Zero(model_.nq);
            dq = Eigen::VectorXd::Zero(model_.nv);
            q_ref = Eigen::VectorXd::Zero(model_.nq);
            dq_ref = Eigen::VectorXd::Zero(model_.nv);

            kp = Eigen::VectorXd(model_.nq);
            kd = Eigen::VectorXd(model_.nv);

            q_ref_prev = Eigen::VectorXd::Zero(model_.nq);

            // Control all 3 joints with gravity compensation - BALANCED for stability
            kp << 0.9, 0.9, 0.9;      // Moderate position gains
            kd << 0.05, 0.05, 0.05;      // Higher damping to prevent oscillations (overdamped)
        
            Kx_ = 5.0 * Eigen::Matrix3d::Identity();
            
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model: %s", e.what());
            return;
        }
        
        // Create subscriber for joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TorqueController::jointStateCallback, this, std::placeholders::_1));

        controller_timer = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&TorqueController::update, this));
        
        // Create publisher for torque commands
        torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_effort_controller/commands", 10);
        RCLCPP_INFO(this->get_logger(), "Torque controller initialized");
    }

private:

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() != model_.nq) {
            RCLCPP_WARN(this->get_logger(), "Incomplete joint state message");
            return;
        }

        // q[0] = msg->position[1];  // joint1 (bl_m1_s1)
        // q[1] = msg->position[2];  // joint2 (bl_m2_s2)
        // q[2] = msg->position[0];  // joint3 (bl_l4_l3)
        
        // v[0] = msg->velocity[1];
        // v[1] = msg->velocity[2];
        // v[2] = msg->velocity[0];

        // q <<  msg->position[1], msg->position[2], msg->position[0];  // joint1 (bl_m1_s1)
        // dq <<  msg->velocity[1], msg->velocity[2], msg->velocity[0];  // joint1 (bl_m1_s1)
        // q[1] = msg->position[2];  // joint2 (bl_m2_s2)
        // q[2] = msg->position[0];  // joint3 (bl_l4_l3)
        
        // v[0] = msg->velocity[1];
        // v[1] = msg->velocity[2];
        // v[2] = msg->velocity[0];

        q = Eigen::VectorXd::Map(msg->position.data(), model_.nq);
        dq = Eigen::VectorXd::Map(msg->position.data(), model_.nv);

        if (!got_state) 
        {
            q_ref = q;
            got_state = true;
        }
    }

    void update()
    {
        if (!got_state) return;

        constexpr int EE_ID = 3;
        constexpr double lambda = 1e-6;

        // ====== TRAJECTORY GENERATION =======

        // Eigen::VectorXd ref_traj = generate_trajectory();

        // Eigen::VectorXd p_ref = ref_traj.head(3); 
        // Eigen::VectorXd dp_ref = ref_traj.tail(3); 
    //     Eigen::Vector3d p_ref;
    //     p_ref << 0.1, 0.1, 0.2; 
    //     Eigen::Vector3d dp_ref = Eigen::Vector3d::Zero();

    //     // ===== Inverse jacobian method ======

    //     pinocchio::forwardKinematics(model_, data_, q);
    //     Eigen::Vector3d p = data_.oMi[EE_ID].translation();

    //     Eigen::Vector3d err = p_ref - p;

    //     pinocchio::Data::Matrix6x J6(6, model_.nv);
    //     pinocchio::computeJointJacobian(model_, data_, q, EE_ID, J6);

    //     Eigen::MatrixXd J = J6.topRows<3>();

    //     Eigen::Matrix3d JJt = J * J.transpose();
    //     JJt.diagonal().array() += lambda;

    //     dq_ref = J.transpose() * JJt.ldlt().solve(dp_ref + Kx_ * err);

    // // 4. Integrate to get q_ref
    //     q_ref = pinocchio::integrate(model_, q_ref, dq_ref * dt_);


        // ======   DYNAMIC CONTROLLER ======
        Eigen::Vector3d p_desired(0.3, 0.0, 0.0);

        q_ref = inverse_kinematics(p_desired);

        RCLCPP_INFO(this->get_logger(),"q1: %f, q2: %f, q3: %f", q_ref[0], q_ref[1], q_ref[2]);

        dq_ref = (q_ref - q_ref_prev) / dt_;
        q_ref_prev = q_ref;  // Store for next iteration

        
        Eigen::VectorXd q_error = q_ref - q;
        Eigen::VectorXd v_error = dq_ref - dq;

        Eigen::VectorXd tau_pd = kp.cwiseProduct(q_error) + kd.cwiseProduct(v_error);
        
        // This computes inverse dynamics tau = M(q)*q_dd + C(q,q_d) + G(q)
        // When inputting q_d = 0 and q_dd = 0, we get just gravity term
        Eigen::VectorXd tau_gravity = pinocchio::rnea(model_, data_, q, 
                                                       Eigen::VectorXd::Zero(model_.nv), 
                                                       Eigen::VectorXd::Zero(model_.nv));


        Eigen::VectorXd friction_compensation = k_damp * dq;


        
        Eigen::VectorXd tau = tau_pd + tau_gravity;// + friction_compensation;
        
        // Apply torque saturation - reduced for stability
        const double max_torque = 5.0;  // Reduced from 8.0 to prevent wild oscillations
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

    Eigen::VectorXd inverse_kinematics(const Eigen::Vector3d& position)
    {
        ik.calcJointPositions(IK::Leg::BL, position[0], position[1], position[2]);
        IK::LegJointPositions q = ik.legs[2];
        Eigen::Vector3d pos;
        pos << q.q1, q.q2, q.q3;
        return pos;
        // pinocchio::Jacobian
    }

    Eigen::VectorXd generate_trajectory() // Replace fill!!!!
    {
        t_ += dt_;

        // Circle parameters
        double radius = 0.1;
        double omega = 1.0;   // rad/s
        double xc = 0.5;
        double zc = 0.4;

        // Cartesian reference
        Eigen::VectorXd ref_vec(6);

        ref_vec <<
        xc + radius * std::cos(omega * t_),
        0.0,
        zc + radius * std::sin(omega * t_);

        // Cartesian velocity reference
        Eigen::Vector3d dp_ref;
        ref_vec <<
        -radius * omega * std::sin(omega * t_),
        0.0,
        radius * omega * std::cos(omega * t_);


        return ref_vec;
    }

    
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_frame_id_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;

    rclcpp::TimerBase::SharedPtr controller_timer;

    Eigen::VectorXd q, dq;
    Eigen::VectorXd q_ref, dq_ref;


    Eigen::VectorXd kp, kd;
    Eigen::Matrix3d Kx_;

    Eigen::VectorXd q_desired, v_desired;

    double k_damp = 0.01;
    double k_stiff = 0.1;

    bool got_state = false;


    const double dt_ = 0.005;
    double t_ = 0.0;

    // JUST FOR INVERSE KINEMATICS

    IK::InverseKinematics ik;
    Eigen::VectorXd q_ref_prev;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TorqueController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
