#include "/home/kai/Projects/timr/ros2_ws/src/kinematics/include/kinematics/kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>
#include <memory>

using scalar_t = double;
constexpr kinematics::dof_size_t DOF = 6;

class KinematicsNode : public rclcpp::Node
{
protected:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _sub_desired_pose;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _pub_joint_positions;

    std::unique_ptr<kinematics::SerialManipulator<scalar_t, DOF>> _manipulator_kinematics;
    rclcpp::TimerBase::SharedPtr _timer;

public:
    KinematicsNode() 
    : Node("kinematics_node")
    {
        // Create subscription to pose topic
        _sub_desired_pose = this->create_subscription<geometry_msgs::msg::Pose>(
            "desired_pose", 10,
            std::bind(&KinematicsNode::_callback_desired_pose, this, std::placeholders::_1));

        // Create publisher for joint states
        _pub_joint_positions = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 10);

        // Create timer for publishing joint states (e.g., at 10Hz)
        _timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&KinematicsNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Kinematics node initialized");
    }

private:
    void _callback_desired_pose(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received new desired pose");
        const kinematics::Translation<scalar_t> translation(msg->position.x, msg->position.y, msg->position.z);
        const kinematics::Rotation<scalar_t> rotation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        const kinematics::Pose<scalar_t> desired_pose(rotation, translation);
        _manipulator_kinematics->update(desired_pose);
    }

    void timer_callback()
    {
        // Create and publish joint state message
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();
        std::array<scalar_t, DOF> joint_positions = _manipulator_kinematics->data().joint_positions;
        std::array<scalar_t, DOF> joint_velocities = _manipulator_kinematics->data().joint_velocities;
        // Set joint names (adjust these to match your robot's joint names)
        for (int i=0; i<DOF; ++i) {
            joint_state_msg.name.push_back("joint" + std::to_string(i+1));
        }

        // Set joint positions
        joint_state_msg.position.assign(
            joint_positions.begin(),
            joint_positions.end()
        );
        joint_state_msg.velocity.assign(
            joint_velocities.begin(),
            joint_velocities.end()
        );
        // Publish the message
        _pub_joint_positions->publish(joint_state_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KinematicsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
