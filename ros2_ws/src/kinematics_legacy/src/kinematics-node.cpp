#include "kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>
#include <memory>
#include <signal.h>

volatile sig_atomic_t kill_this_node = 0;

void signal_handler(int signum) {
    kill_this_node = 1;
}

class KinematicsNode : public rclcpp::Node
{

private:
    std::string _robot_description_path;
    std::unique_ptr<kinematics::SerialManipulator> _serialmanipulator_kinematics;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _sub_target_pose;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _sub_joint_states;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _pub_target_joint_positions;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr _pub_current_pose;
    sensor_msgs::msg::JointState _cached_target_joint_state_msg;
    geometry_msgs::msg::Pose _cached_current_pose_msg;

    rclcpp::TimerBase::SharedPtr _timer;


    void _callback_target_pose(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (!_serialmanipulator_kinematics) {
            return;
        }
        const kinematics::Translation<scalar_t> translation(msg->position.x, msg->position.y, msg->position.z);
        const kinematics::Rotation<scalar_t> rotation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        const kinematics::Pose<scalar_t> target_pose(rotation, translation);
        _serialmanipulator_kinematics->update(target_pose);
    }

    void _callback_joint_positions(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::array<scalar_t, DOF> joint_positions;
        std::array<scalar_t, DOF> joint_velocities;
        std::array<scalar_t, DOF> joint_accelerations;
        for (dof_size_t i = 0; i < DOF; ++i) {
            joint_positions[i] = msg->position[i];
            joint_velocities[i] = msg->velocity[i];
            joint_accelerations[i] = msg->effort[i];
        }   

        if (!_serialmanipulator_kinematics) {
            _serialmanipulator_kinematics = std::make_unique<kinematics::SerialManipulator>(_robot_description_path, joint_positions);
        }
        _serialmanipulator_kinematics->set_joint_positions(joint_positions);
        _serialmanipulator_kinematics->set_joint_velocities(joint_velocities);
        _serialmanipulator_kinematics->set_joint_accelerations(joint_accelerations);
    }

    void _callback_timer()
    {
        // Add safety check
        if (!_serialmanipulator_kinematics) {
            return;
        }

        // Create and publish joint state message
        _cached_target_joint_state_msg.header.stamp = this->now();
        
        try {
            // Set joint positions with safety checks
            const auto& positions = _serialmanipulator_kinematics->get_target_joint_positions();
            const auto& velocities = _serialmanipulator_kinematics->get_target_joint_velocities();
            const auto& accelerations = _serialmanipulator_kinematics->get_target_joint_accelerations();

            _cached_target_joint_state_msg.position.assign(positions.begin(), positions.end());
            _cached_target_joint_state_msg.velocity.assign(velocities.begin(), velocities.end());
            _cached_target_joint_state_msg.effort.assign(accelerations.begin(), accelerations.end());

            // Publish the message
            _pub_target_joint_positions->publish(_cached_target_joint_state_msg);

            // Add end pose publishing
            const auto& current_pose = _serialmanipulator_kinematics->get_end_pose();
            const auto& translation = current_pose.translation();
            const auto& rotation = current_pose.rotation();
            _cached_current_pose_msg.position.x = translation.x();
            _cached_current_pose_msg.position.y = translation.y();
            _cached_current_pose_msg.position.z = translation.z();
            _cached_current_pose_msg.orientation.w = rotation.w();
            _cached_current_pose_msg.orientation.x = rotation.x();
            _cached_current_pose_msg.orientation.y = rotation.y();
            _cached_current_pose_msg.orientation.z = rotation.z();
            
            _pub_current_pose->publish(_cached_current_pose_msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in timer callback: %s", e.what());
        }
    }
public:
    KinematicsNode(std::string robot_description_path) 
    : Node("kinematics_node"), _robot_description_path(robot_description_path)
    {
        RCLCPP_INFO(this->get_logger(), "Kinematics node Created");
    }

    bool initialize() {
        // Create subscriptions
        RCLCPP_INFO(this->get_logger(), "Starting kinematics node initialization...");
        _sub_joint_states = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&KinematicsNode::_callback_joint_positions, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Waiting for the first joint states message...");
        while (!_serialmanipulator_kinematics && !kill_this_node) {
            rclcpp::spin_some(shared_from_this());
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (kill_this_node) {
                RCLCPP_INFO(this->get_logger(), "Received interrupt, shutting down...");
                return false;
            }
        }    
        
        // Create remaining subscriptions and publishers
        _sub_target_pose = this->create_subscription<geometry_msgs::msg::Pose>(
            "target_poses", 10,
            std::bind(&KinematicsNode::_callback_target_pose, this, std::placeholders::_1));

        _pub_target_joint_positions = this->create_publisher<sensor_msgs::msg::JointState>(
            "target_joint_states", 10);

        _pub_current_pose = this->create_publisher<geometry_msgs::msg::Pose>(
            "poses", 10);

        // Set joint names (adjust these to match your robot's joint names)
        for (int i=0; i<DOF; ++i) {
            _cached_target_joint_state_msg.name.push_back("joint" + std::to_string(i+1));
        }

        _timer = this->create_wall_timer(
            std::chrono::milliseconds(4),
            std::bind(&KinematicsNode::_callback_timer, this));

        RCLCPP_INFO(this->get_logger(), "Kinematics node initialized");
        return true;
    }

};

int main(int argc, char** argv)
{
    signal(SIGINT, signal_handler);
    
    rclcpp::init(argc, argv);
    std::string robot_description_path = "/home/kai/Projects/timr/ros2_ws/src/kinematics_legacy/config/robot.json";
    auto node = std::make_shared<KinematicsNode>(robot_description_path);
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
