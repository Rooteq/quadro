#include <memory>
#include <filesystem>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/parsers/urdf.hpp"

class ForwardKinematicsNode : public rclcpp::Node
{
public:
    ForwardKinematicsNode() : Node("forward_kinematics_node")
    {
        // Get URDF file path from launch arguments
        std::string ee_frame_name = "link";  // Default frame name
        
        const auto package_share_path = ament_index_cpp::get_package_share_directory("quadro");
        const auto urdf_path = std::filesystem::path(package_share_path) / "description" / "robot.xacro";
        
        // Load robot model
        try {
            
            pinocchio::urdf::buildModel(urdf_path, model_);
            data_ = std::make_unique<pinocchio::Data>(model_);
            
            // Get end-effector frame ID
            ee_frame_id_ = model_.getFrameId(ee_frame_name);
            
            RCLCPP_INFO(this->get_logger(), "Robot model loaded from: %s", urdf_path.c_str());
            RCLCPP_INFO(this->get_logger(), "End-effector frame: %s", ee_frame_name.c_str());
            RCLCPP_INFO(this->get_logger(), "Number of joints: %d", model_.nq);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model: %s", e.what());
            return;
        }
        
        // Create subscriber for joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ForwardKinematicsNode::jointStateCallback, this, std::placeholders::_1));
        
        // No publisher needed - just print to terminal
        
        RCLCPP_INFO(this->get_logger(), "Forward kinematics node initialized");
    }

private:
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
        Eigen::VectorXd q(model_.nq);
        for (size_t i = 0; i < msg->position.size(); ++i) {
            q[i] = msg->position[i];
        }
        
        // Compute forward kinematics
        pinocchio::framesForwardKinematics(model_, *data_, q);
        
        // Get end-effector transform
        const auto& ee_transform = data_->oMf[ee_frame_id_];
        
        // Print position and orientation to terminal
        std::cout << "\n--- End-Effector Pose ---" << std::endl;
        std::cout << "Position: [" 
                  << ee_transform.translation().x() << ", "
                  << ee_transform.translation().y() << ", "
                  << ee_transform.translation().z() << "]" << std::endl;
        
        // Convert rotation matrix to Euler angles for easier reading
        Eigen::Vector3d euler_angles = ee_transform.rotation().eulerAngles(0, 1, 2); // XYZ order
        std::cout << "Orientation (Euler XYZ): [" 
                  << euler_angles.x() << ", "
                  << euler_angles.y() << ", "
                  << euler_angles.z() << "]" << std::endl;
        
        // Also print rotation matrix
        std::cout << "Rotation matrix:" << std::endl 
                  << ee_transform.rotation() << std::endl;
        
        std::cout << "Joint positions: [";
        for (int i = 0; i < q.size(); ++i) {
            std::cout << q[i];
            if (i < q.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        std::cout << "------------------------\n" << std::endl;
    }
    
    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    pinocchio::FrameIndex ee_frame_id_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForwardKinematicsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}