#include "serial_manipulator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>
#include <memory>
#include <signal.h>

namespace timr {

namespace kinematics {

volatile sig_atomic_t kill_this_node = 0;

void signal_handler([[maybe_unused]] int signum) {
    kill_this_node = 1;
}

class KinematicsNode : public rclcpp::Node
{

protected:
    std::unique_ptr<kinematics::SerialManipulator> _kinematics;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _sub_joint_state;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _sub_target_pose;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr _pub_pose;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _pub_target_joint_state;

    sensor_msgs::msg::JointState _cached_target_joint_state_msg;
    geometry_msgs::msg::Pose _cached_current_pose_msg;
    rclcpp::TimerBase::SharedPtr _timer;

    void _callback_target_pose(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (!_kinematics->is_initialized()) {
            return;
        }
        if (!_pub_target_joint_state) {
            return;
        }
        try {
            const dqpose::Translation<scalar_t> target_translation(msg->position.x, msg->position.y, msg->position.z);
            const dqpose::Rotation<scalar_t> target_rotation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
            const dqpose::Pose<scalar_t> target_pose(target_rotation, target_translation);

            if (_kinematics->update(target_pose) != 0) {
                return;
            }
            _cached_target_joint_state_msg.position = _kinematics->get_target_joint_position();
            _cached_target_joint_state_msg.velocity = _kinematics->get_target_joint_velocity();
            _cached_target_joint_state_msg.effort = _kinematics->get_target_joint_effort();
            _cached_target_joint_state_msg.header.stamp = this->now();
            _pub_target_joint_state->publish(_cached_target_joint_state_msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in target pose callback: %s", e.what());
        }
    }

    void _callback_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!_kinematics->is_initialized()) {
            _kinematics->initialize(msg->position);
        }
        if (!_pub_pose) {
            return;
        }
        try {   
            if (_kinematics->set_joint_position(msg->position) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set joint position");
                return;
            }

            if (_kinematics->set_joint_velocity(msg->velocity) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set joint velocity");
                return;
            }

            if (_kinematics->set_joint_effort(msg->effort) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set joint effort");
                return;
            }

            const auto& current_pose = _kinematics->get_end_pose();
            const auto& current_translation = current_pose.translation();
            const auto& current_rotation = current_pose.rotation();
            _cached_current_pose_msg.position.x = current_translation.x();
            _cached_current_pose_msg.position.y = current_translation.y();
            _cached_current_pose_msg.position.z = current_translation.z();
            _cached_current_pose_msg.orientation.w = current_rotation.w();
            _cached_current_pose_msg.orientation.x = current_rotation.x();
            _cached_current_pose_msg.orientation.y = current_rotation.y();
            _cached_current_pose_msg.orientation.z = current_rotation.z();
            _pub_pose->publish(_cached_current_pose_msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in joint state callback: %s", e.what());
            return;
        }

    }

    void _callback_timer()
    {

    }
public:
    KinematicsNode() 
    : Node("kinematics_node")
    {
        RCLCPP_INFO(this->get_logger(), "Kinematics node Created");
    }

    bool initialize() {
        try {
            // Create subscriptions
            RCLCPP_INFO(this->get_logger(), "Initializing kinematics node...");
            this->declare_parameter("robotKinematicsDescriptionPath", "");
            this->declare_parameter("samplingTimeSec", 0.001);
            this->declare_parameter("initializationTimeoutSec", 60);
            const auto robot_kinematics_description_path = this->get_parameter("robotKinematicsDescriptionPath").as_string();
            const auto sampling_time_sec = this->get_parameter("samplingTimeSec").as_double();
            const auto initialization_timeout_sec = this->get_parameter("initializationTimeoutSec").as_int();
            
            if (!_kinematics) {
                RCLCPP_INFO(get_logger(), "Attempting to create kinematics instance...");
                RCLCPP_INFO(get_logger(), "Using robot description: %s", robot_kinematics_description_path.c_str());
                
                _kinematics = std::make_unique<kinematics::SerialManipulator>(robot_kinematics_description_path);
                    
                if (!_kinematics) {
                    RCLCPP_ERROR(get_logger(), "Failed to create kinematics instance");
                    return false;
                }
            }
            const auto DOF = _kinematics->get_dof();
            RCLCPP_INFO(this->get_logger(), "Robot name: %s, DOF: %d created.", _kinematics->get_name().c_str(), DOF);

            // Set joint names (adjust these to match your robot's joint names)
            _cached_target_joint_state_msg.name.resize(DOF);
            _cached_target_joint_state_msg.position.resize(DOF);
            _cached_target_joint_state_msg.velocity.resize(DOF);
            _cached_target_joint_state_msg.effort.resize(DOF);
            for (dof_size_t i=0; i<DOF; ++i) {
                _cached_target_joint_state_msg.name[i] = "joint" + std::to_string(i+1);
            }
            
            _sub_joint_state = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_state", 10,
                std::bind(&KinematicsNode::_callback_joint_state, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Waiting for the first joint state message...");
            
            const auto start_time = this->now();
            while (!_kinematics->is_initialized() && !kill_this_node) {
                if ((this->now() - start_time).seconds() > initialization_timeout_sec) {
                    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for joint state message");
                    return false;
                }
                rclcpp::spin_some(shared_from_this());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }    
            RCLCPP_INFO(this->get_logger(), "First joint state message received");
            
            const auto& current_pose = _kinematics->get_end_pose();
            const auto& current_translation = current_pose.translation();
            const auto& current_rotation = current_pose.rotation();
            RCLCPP_INFO(this->get_logger(), "Current pose: %f, %f, %f", current_translation.x(), current_translation.y(), current_translation.z());
            RCLCPP_INFO(this->get_logger(), "Current rotation: %f, %f, %f, %f", current_rotation.w(), current_rotation.x(), current_rotation.y(), current_rotation.z());
            RCLCPP_INFO(this->get_logger(), "Current translation: %f, %f, %f", current_translation.x(), current_translation.y(), current_translation.z());
            if (kill_this_node) {
                RCLCPP_INFO(this->get_logger(), "Received interrupt, shutting down...");
                return false;
            }

            // Create remaining subscriptions and publishers
            _sub_target_pose = this->create_subscription<geometry_msgs::msg::Pose>(
                "target_pose", 10,
                std::bind(&KinematicsNode::_callback_target_pose, this, std::placeholders::_1));

            _pub_target_joint_state = this->create_publisher<sensor_msgs::msg::JointState>(
                "target_joint_state", 10);

            _pub_pose = this->create_publisher<geometry_msgs::msg::Pose>(
                "pose", 10);

            _timer = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(sampling_time_sec)),
                std::bind(&KinematicsNode::_callback_timer, this));

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error during initialization: %s", e.what());
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Kinematics node initialized");

        
        return true;
    }

};

int run_kinematics_node(int argc, char** argv)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGQUIT, signal_handler);
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KinematicsNode>();
    if (!node->initialize()) {
        RCLCPP_INFO(node->get_logger(), "Failed to initialize kinematics node.");
        rclcpp::shutdown();
        return 1;
    }
    while (rclcpp::ok() && !kill_this_node) {
        rclcpp::spin_some(node);
    }
    
    RCLCPP_INFO(node->get_logger(), "Shutting down kinematics node...");
    rclcpp::shutdown();
    return 0;
}

} // namespace kinematics

} // namespace timr
