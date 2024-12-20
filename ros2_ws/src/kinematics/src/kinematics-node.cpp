#include "kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>
#include <memory>
#include <signal.h>

volatile sig_atomic_t g_interrupt_flag = 0;

void kill_this_node(int signum) {
    g_interrupt_flag = 1;
}

using scalar_t = double;
using kinematics::dof_size_t;
constexpr dof_size_t DOF = 6;

struct KinematicsNodeConfig
{
    std::string robot_description_path;
};
class KinematicsNode : public rclcpp::Node
{

private:
    KinematicsNodeConfig _cfg;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _sub_target_pose;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _sub_joint_states;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _pub_target_joint_positions;

    std::unique_ptr<kinematics::SerialManipulator<scalar_t, DOF>> _manipulator_kinematics;
    rclcpp::TimerBase::SharedPtr _timer;

    void _callback_target_pose(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        const kinematics::Translation<scalar_t> translation(msg->position.x, msg->position.y, msg->position.z);
        const kinematics::Rotation<scalar_t> rotation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        const kinematics::Pose<scalar_t> target_pose(rotation, translation);
        _manipulator_kinematics->update(target_pose);
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

        if (!_manipulator_kinematics) {
            _manipulator_kinematics = std::make_unique<kinematics::SerialManipulator<scalar_t, DOF>>(_cfg.robot_description_path, joint_positions);
        }
        _manipulator_kinematics->set_joint_positions(joint_positions);
        _manipulator_kinematics->set_joint_velocities(joint_velocities);
        _manipulator_kinematics->set_joint_accelerations(joint_accelerations);
    }

    void _timer_callback()
    {
        // Add safety check
        if (!_manipulator_kinematics) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), 
                *this->get_clock(), 
                1000,  // Warn every 1 second
                "Manipulator kinematics not initialized yet");
            return;
        }

        // Create and publish joint state message
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();
        
        // Set joint names (adjust these to match your robot's joint names)
        for (int i=0; i<DOF; ++i) {
            joint_state_msg.name.push_back("joint" + std::to_string(i+1));
        }

        try {
            // Set joint positions with safety checks
            const auto& positions = _manipulator_kinematics->get_target_joint_positions();
            const auto& velocities = _manipulator_kinematics->get_target_joint_velocities();
            const auto& accelerations = _manipulator_kinematics->get_target_joint_accelerations();

            joint_state_msg.position.assign(positions.begin(), positions.end());
            joint_state_msg.velocity.assign(velocities.begin(), velocities.end());
            joint_state_msg.effort.assign(accelerations.begin(), accelerations.end());

            // Publish the message
            _pub_target_joint_positions->publish(joint_state_msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in timer callback: %s", e.what());
        }
    }
public:
    KinematicsNode(std::string robot_description_path) 
    : Node("kinematics_node")
    {
        _cfg.robot_description_path = robot_description_path;
        RCLCPP_INFO(this->get_logger(), "Kinematics node Created");
    }

    bool initialize() {
        // Create subscriptions
        RCLCPP_INFO(this->get_logger(), "Starting kinematics node initialization...");
        _sub_joint_states = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&KinematicsNode::_callback_joint_positions, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Waiting for the first joint states message...");
        while (!_manipulator_kinematics && !g_interrupt_flag) {
            rclcpp::spin_some(shared_from_this());
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (g_interrupt_flag) {
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

        _timer = this->create_wall_timer(
            std::chrono::milliseconds(4),
            std::bind(&KinematicsNode::_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Kinematics node initialized");
        return true;
    }

};

int main(int argc, char** argv)
{
    signal(SIGINT, kill_this_node);
    
    rclcpp::init(argc, argv);
    std::string robot_description_path = "/home/kai/Projects/timr/ros2_ws/src/kinematics/config/robot.json";
    auto node = std::make_shared<KinematicsNode>(robot_description_path);
    if (!node->initialize()) {
        RCLCPP_INFO(node->get_logger(), "Failed to initialize kinematics node.");
        rclcpp::shutdown();
        return 1;
    }
    while (rclcpp::ok() && !g_interrupt_flag) {
        rclcpp::spin_some(node);
    }
    
    RCLCPP_INFO(node->get_logger(), "Shutting down kinematics node...");
    rclcpp::shutdown();
    return 0;
}
