#include "serial_manipulator.hpp"

#include <gtest/gtest.h>

namespace timr {

namespace kinematics {

class SerialManipulatorTest : public ::testing::Test {
protected:
    std::string yaml_file_path = "/home/kai/Projects/timr/ros2_ws/src/kinematics/config/robot1.yaml";
public:

    void SetUp() override {


    }    

    void TearDown() override {

    }
};

TEST_F(SerialManipulatorTest, TEST_SERIAL_MANIPULATOR_CONSTRUCTOR) {
    SerialManipulator manipulator(yaml_file_path);
    SerialManipulatorConfig config;
    config.translation_priority = 1;
    config.joint_damping = 0.0001;
    config.error_gain = 50;
    config.pos_gain = 1.0;
    config.vel_gain = 1.0;
    config.DOF = 6;
    config.name = "robot1";
    config.api_version = "serialmanipulator.timr.io/v1alpha1";
    config.kind = "SerialManipulator";
    config.flag = 0;
    config.is_open_loop = false;
    config.is_initialized = false;

    SerialManipulatorData data;
    data.base = Pose<scalar_t>();
    data.effector = Pose<scalar_t>();
    data.target_joint_position = {0, 0, 0, 0, 0, 0};
    data.target_joint_velocity = {0, 0, 0, 0, 0, 0};
    data.target_joint_effort = {0, 0, 0, 0, 0, 0};
    data.joint_position = {0, 0, 0, 0, 0, 0};
    data.joint_velocity = {0, 0, 0, 0, 0, 0};
    data.joint_effort = {0, 0, 0, 0, 0, 0};
    data.joint_position_lower_bound = {-180 * deg2rad_factor, 0 * deg2rad_factor, 0 * deg2rad_factor, -175 * deg2rad_factor, -120 * deg2rad_factor, -175 * deg2rad_factor};
    data.joint_position_upper_bound = {180 * deg2rad_factor, 175 * deg2rad_factor, 150 * deg2rad_factor, 175 * deg2rad_factor, 120 * deg2rad_factor, 175 * deg2rad_factor};
    data.joint_velocity_lower_bound = {-1 * deg2rad_factor, -2 * deg2rad_factor, -3 * deg2rad_factor, -4 * deg2rad_factor, -5 * deg2rad_factor, -6 * deg2rad_factor};
    data.joint_velocity_upper_bound = {1 * deg2rad_factor, 2 * deg2rad_factor, 3 * deg2rad_factor, 4 * deg2rad_factor, 5 * deg2rad_factor, 6 * deg2rad_factor};
    data.joint_effort_lower_bound = {0, 0, 0, 0, 0, 0};
    data.joint_effort_upper_bound = {1, 1, 1, 1, 1, 1};
    data.joint_pose = {Pose<scalar_t>(), Pose<scalar_t>(), Pose<scalar_t>(), Pose<scalar_t>(), Pose<scalar_t>(), Pose<scalar_t>()};
    data.joint_pose_derivative = {DualQuat<scalar_t>(), DualQuat<scalar_t>(), DualQuat<scalar_t>(), DualQuat<scalar_t>(), DualQuat<scalar_t>(), DualQuat<scalar_t>()};

    auto actual_config = manipulator.get_config();
    auto actual_data = manipulator.get_data();

    EXPECT_EQ(actual_config.translation_priority, config.translation_priority);
    EXPECT_EQ(actual_config.joint_damping, config.joint_damping);
    EXPECT_EQ(actual_config.error_gain, config.error_gain);
    EXPECT_EQ(actual_config.pos_gain, config.pos_gain);
    EXPECT_EQ(actual_config.vel_gain, config.vel_gain);
    EXPECT_EQ(actual_config.DOF, config.DOF);
    EXPECT_EQ(actual_config.name, config.name);
    EXPECT_EQ(actual_config.api_version, config.api_version);
    EXPECT_EQ(actual_config.kind, config.kind);
    EXPECT_EQ(actual_config.flag, config.flag);
    EXPECT_EQ(actual_config.is_open_loop, config.is_open_loop);
    EXPECT_EQ(actual_config.is_initialized, config.is_initialized);
    EXPECT_EQ(actual_data.base, data.base);
    EXPECT_EQ(actual_data.effector, data.effector);
    EXPECT_EQ(actual_data.target_joint_position, data.target_joint_position);
    EXPECT_EQ(actual_data.target_joint_velocity, data.target_joint_velocity);
    EXPECT_EQ(actual_data.target_joint_effort, data.target_joint_effort);
    EXPECT_EQ(actual_data.joint_position, data.joint_position);
    EXPECT_EQ(actual_data.joint_velocity, data.joint_velocity);
    EXPECT_EQ(actual_data.joint_effort, data.joint_effort);
    EXPECT_EQ(actual_data.joint_position_lower_bound, data.joint_position_lower_bound);
    EXPECT_EQ(actual_data.joint_position_upper_bound, data.joint_position_upper_bound);
    EXPECT_EQ(actual_data.joint_velocity_lower_bound, data.joint_velocity_lower_bound);
    EXPECT_EQ(actual_data.joint_velocity_upper_bound, data.joint_velocity_upper_bound);
    EXPECT_EQ(actual_data.joint_effort_lower_bound, data.joint_effort_lower_bound);
    EXPECT_EQ(actual_data.joint_effort_upper_bound, data.joint_effort_upper_bound);
    EXPECT_EQ(actual_data.joint_pose, data.joint_pose);
    EXPECT_EQ(actual_data.joint_pose_derivative, data.joint_pose_derivative);

    EXPECT_EQ(actual_data.target_joint_position.size(), 6);
    EXPECT_EQ(actual_data.target_joint_velocity.size(), 6);
    EXPECT_EQ(actual_data.target_joint_effort.size(), 6);
    EXPECT_EQ(actual_data.joint_position.size(), 6);
    EXPECT_EQ(actual_data.joint_velocity.size(), 6);
    EXPECT_EQ(actual_data.joint_effort.size(), 6);
    EXPECT_EQ(actual_data.joint_position_lower_bound.size(), 6);
    EXPECT_EQ(actual_data.joint_position_upper_bound.size(), 6);
    EXPECT_EQ(actual_data.joint_velocity_lower_bound.size(), 6);
    EXPECT_EQ(actual_data.joint_velocity_upper_bound.size(), 6);
    EXPECT_EQ(actual_data.joint_effort_lower_bound.size(), 6);
    EXPECT_EQ(actual_data.joint_effort_upper_bound.size(), 6);
    EXPECT_EQ(actual_data.joint_pose.size(), 6);
    EXPECT_EQ(actual_data.joint_pose_derivative.size(), 6);

    EXPECT_EQ(manipulator.get_dof(), 6);
}

TEST_F(SerialManipulatorTest, TEST_SERIAL_MANIPULATOR_INITIALIZE) {
    SerialManipulator manipulator(yaml_file_path);
    EXPECT_TRUE(!manipulator.is_initialized());
    std::vector<scalar_t> joint_position = {0, 0, 0, 0, 0, 0};
    manipulator.initialize(joint_position);

    EXPECT_NO_THROW(manipulator.initialize(joint_position));
    EXPECT_TRUE(manipulator.is_initialized());
    EXPECT_EQ(manipulator.get_config().flag, 1);
    dqpose::Pose<scalar_t> pose;
    EXPECT_EQ(manipulator.update(pose), -1);

    // Test initialization with wrong size
    std::vector<scalar_t> wrong_size_position = {0, 0, 0, 0, 0};
    EXPECT_THROW(manipulator.initialize(wrong_size_position), std::runtime_error);

    // Test re-initialization
    EXPECT_NO_THROW(manipulator.initialize(joint_position));
    EXPECT_EQ(manipulator.get_config().flag, 1);

    std::vector<scalar_t> velocity = {0, 0, 0, 0, 0, 0};
    std::vector<scalar_t> effort = {0, 0, 0, 0, 0, 0};
    manipulator.set_joint_velocity(velocity);
    EXPECT_EQ(manipulator.get_config().flag, 3);
    EXPECT_EQ(manipulator.update(pose), -1);
    manipulator.set_joint_effort(effort);
    EXPECT_EQ(manipulator.get_config().flag, 7);
    manipulator.update(pose);
    EXPECT_EQ(manipulator.get_config().flag, 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

} // namespace kinematics

} // namespace timr