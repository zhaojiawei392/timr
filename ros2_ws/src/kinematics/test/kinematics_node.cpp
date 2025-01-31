#include <gtest/gtest.h>
#include "kinematics_node.hpp"
#include "serial_manipulator.hpp"
#include <rclcpp/rclcpp.hpp>

namespace timr {
namespace kinematics {

class KinematicsNodeTest : public ::testing::Test {
protected:
    std::string yaml_file_path = "/home/kai/Projects/timr/ros2_ws/src/kinematics/config/robot1.yaml";
    std::unique_ptr<SerialManipulator> kinematics;
public:
    void SetUp() override {
        kinematics = std::make_unique<SerialManipulator>(yaml_file_path);
    }

    void TearDown() override {
    }
};

TEST_F(KinematicsNodeTest, TEST_KINEMATICS_JOINT_STATE_MSG) {
    sensor_msgs::msg::JointState::SharedPtr joint_state = std::make_shared<sensor_msgs::msg::JointState>();
    joint_state->name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    joint_state->position = {0, 0, 0, 0, 0, 0};
    joint_state->velocity = {0, 0, 0, 0, 0, 0};
    joint_state->effort = {0, 0, 0, 0, 0, 0};
    std::vector<double> joint_position = joint_state->position;
    std::vector<double> joint_velocity = joint_state->velocity;
    std::vector<double> joint_effort = joint_state->effort;
    if (!kinematics->is_initialized()) {
        kinematics->set_joint_position(joint_position);
    }
    kinematics->set_joint_position(joint_position);
    kinematics->set_joint_velocity(joint_velocity);
    kinematics->set_joint_effort(joint_effort);
}

TEST_F(KinematicsNodeTest, TEST_KINEMATICS_TARGET_POSE_MSG) {
    sensor_msgs::msg::JointState::SharedPtr joint_state = std::make_shared<sensor_msgs::msg::JointState>();
    geometry_msgs::msg::Pose::SharedPtr msg = std::make_shared<geometry_msgs::msg::Pose>();
    msg->position.x = 1;
    msg->position.y = 2;
    msg->position.z = 3;
    msg->orientation.x = 0;
    msg->orientation.y = 0;
    msg->orientation.z = 0;
    msg->orientation.w = 1;
    if (!kinematics->is_initialized()) {
        return;
    }
    try {
        const dqpose::Translation<scalar_t> target_translation(msg->position.x, msg->position.y, msg->position.z);
        const dqpose::Rotation<scalar_t> target_rotation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        const dqpose::Pose<scalar_t> target_pose(target_rotation, target_translation);
        if (kinematics->update(target_pose) != 0) {
            return;
        }
        joint_state->position = kinematics->get_target_joint_position();
        joint_state->velocity = kinematics->get_target_joint_velocity();
        joint_state->effort = kinematics->get_target_joint_effort();
    } catch (const std::exception& e) {
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

} // namespace kinematics
} // namespace timr
