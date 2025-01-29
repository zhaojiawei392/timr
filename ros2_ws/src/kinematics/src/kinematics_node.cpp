#include "serial_manipulator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <timr_msgs/msg/joint_bounds.hpp>
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

private:
    std::string _robot_kinematics_description_path;
    std::unique_ptr<kinematics::SerialManipulator> _serialmanipulator_kinematics;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _sub_target_poses;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _sub_joint_states;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _pub_target_joint_states;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr _pub_poses;
    rclcpp::Publisher<timr_msgs::msg::JointBounds>::SharedPtr _pub_joint_bounds;

    sensor_msgs::msg::JointState _cached_target_joint_state_msg;
    geometry_msgs::msg::Pose _cached_current_pose_msg;
    rclcpp::TimerBase::SharedPtr _timer;


    void _callback_target_pose(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (!_serialmanipulator_kinematics) {
            return;
        }
        try {
            const kinematics::Translation<scalar_t> target_translation(msg->position.x, msg->position.y, msg->position.z);
            const kinematics::Rotation<scalar_t> target_rotation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
            const kinematics::Pose<scalar_t> target_pose(target_rotation, target_translation);
            _serialmanipulator_kinematics->update(target_pose);

            const auto& target_joint_positions = _serialmanipulator_kinematics->get_target_joint_positions();
            const auto& target_joint_velocities = _serialmanipulator_kinematics->get_target_joint_velocities();
            const auto& target_joint_accelerations = _serialmanipulator_kinematics->get_target_joint_accelerations();
            _cached_target_joint_state_msg.position.assign(target_joint_positions.begin(), target_joint_positions.end());
            _cached_target_joint_state_msg.velocity.assign(target_joint_velocities.begin(), target_joint_velocities.end());
            _cached_target_joint_state_msg.effort.assign(target_joint_accelerations.begin(), target_joint_accelerations.end()); 
            _cached_target_joint_state_msg.header.stamp = this->now();
            _pub_target_joint_states->publish(_cached_target_joint_state_msg);
            
            const auto& current_pose = _serialmanipulator_kinematics->get_end_pose();
            const auto& current_translation = current_pose.translation();
            const auto& current_rotation = current_pose.rotation();
            _cached_current_pose_msg.position.x = current_translation.x();
            _cached_current_pose_msg.position.y = current_translation.y();
            _cached_current_pose_msg.position.z = current_translation.z();
            _cached_current_pose_msg.orientation.w = current_rotation.w();
            _cached_current_pose_msg.orientation.x = current_rotation.x();
            _cached_current_pose_msg.orientation.y = current_rotation.y();
            _cached_current_pose_msg.orientation.z = current_rotation.z();
            _pub_poses->publish(_cached_current_pose_msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in target pose callback: %s", e.what());
        }
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
        _serialmanipulator_kinematics->set_joint_positions(joint_positions);
        _serialmanipulator_kinematics->set_joint_velocities(joint_velocities);
        _serialmanipulator_kinematics->set_joint_accelerations(joint_accelerations);

        if (!_serialmanipulator_kinematics) {
            _serialmanipulator_kinematics = std::make_unique<kinematics::SerialManipulator>(_robot_kinematics_description_path, joint_positions);
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
        // Create subscriptions
        RCLCPP_INFO(this->get_logger(), "Initializing kinematics node...");
        this->declare_parameter("robotKinematicsDescriptionPath", "");
        _robot_kinematics_description_path = this->get_parameter("robotKinematicsDescriptionPath").as_string();
        RCLCPP_INFO(this->get_logger(), "Robot kinematics description path: %s", _robot_kinematics_description_path.c_str());
        std::ifstream file(_robot_kinematics_description_path);
        if (!file.good()) {
            RCLCPP_ERROR(get_logger(), "Robot description file not found: %s", _robot_kinematics_description_path.c_str());
            return false;
        }
        file.close();
        // Debug here
        RCLCPP_INFO(this->get_logger(), "Publishing joint states upper and lower bounds");
        // Create publishers
        rclcpp::QoS qos_profile(1);
        qos_profile.transient_local();
        _pub_joint_bounds = this->create_publisher<timr_msgs::msg::JointBounds>(
            "joint_bounds", qos_profile);
        timr_msgs::msg::JointBounds msg_bounds;

        RCLCPP_INFO(this->get_logger(), "Publishing joint bounds");
        _cached_target_joint_state_msg.name.resize(DOF);    
        _cached_target_joint_state_msg.position.resize(DOF);
        _cached_target_joint_state_msg.velocity.resize(DOF);
        _cached_target_joint_state_msg.effort.resize(DOF);
        msg_bounds.name.resize(DOF);
        msg_bounds.position_lower_bound.resize(DOF);
        msg_bounds.position_upper_bound.resize(DOF);
        msg_bounds.velocity_lower_bound.resize(DOF);
        msg_bounds.velocity_upper_bound.resize(DOF);
        msg_bounds.acceleration_lower_bound.resize(DOF);
        msg_bounds.acceleration_upper_bound.resize(DOF);

        RCLCPP_INFO(this->get_logger(), "Publishing joint bounds");
        // Set joint names (adjust these to match your robot's joint names)
        for (int i=0; i<DOF; ++i) {
            _cached_target_joint_state_msg.name[i] = "joint" + std::to_string(i+1);
            msg_bounds.name[i] = "joint" + std::to_string(i+1);
        }
        RCLCPP_INFO(this->get_logger(), "Publishing joint bounds");
        const auto& joint_positions_lower_bound = _serialmanipulator_kinematics->get_joint_positions_lower_bound();
        const auto& joint_positions_upper_bound = _serialmanipulator_kinematics->get_joint_positions_upper_bound();
        const auto& joint_velocities_lower_bound = _serialmanipulator_kinematics->get_joint_velocities_lower_bound();
        const auto& joint_velocities_upper_bound = _serialmanipulator_kinematics->get_joint_velocities_upper_bound();
        const auto& joint_accelerations_lower_bound = _serialmanipulator_kinematics->get_joint_accelerations_lower_bound();
        const auto& joint_accelerations_upper_bound = _serialmanipulator_kinematics->get_joint_accelerations_upper_bound();
        msg_bounds.position_lower_bound.assign(joint_positions_lower_bound.begin(), joint_positions_lower_bound.end());
        msg_bounds.position_upper_bound.assign(joint_positions_upper_bound.begin(), joint_positions_upper_bound.end());
        msg_bounds.velocity_lower_bound.assign(joint_velocities_lower_bound.begin(), joint_velocities_lower_bound.end());
        msg_bounds.velocity_upper_bound.assign(joint_velocities_upper_bound.begin(), joint_velocities_upper_bound.end());
        msg_bounds.acceleration_lower_bound.assign(joint_accelerations_lower_bound.begin(), joint_accelerations_lower_bound.end());
        msg_bounds.acceleration_upper_bound.assign(joint_accelerations_upper_bound.begin(), joint_accelerations_upper_bound.end());

        RCLCPP_INFO(this->get_logger(), "Publishing joint bounds");
        _pub_joint_bounds->publish(msg_bounds);
        
        RCLCPP_INFO(this->get_logger(), "Publishing joint bounds");
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
        _sub_target_poses = this->create_subscription<geometry_msgs::msg::Pose>(
            "target_poses", 10,
            std::bind(&KinematicsNode::_callback_target_pose, this, std::placeholders::_1));

        _pub_target_joint_states = this->create_publisher<sensor_msgs::msg::JointState>(
            "target_joint_states", 10);

        _pub_poses = this->create_publisher<geometry_msgs::msg::Pose>(
            "poses", 10);

        _timer = this->create_wall_timer(
            std::chrono::milliseconds(4),
            std::bind(&KinematicsNode::_callback_timer, this));

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

int main(int argc, char** argv)
{
    return timr::kinematics::run_kinematics_node(argc, argv);
}
