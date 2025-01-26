#include "robot_driver.hpp"

#include <gtest/gtest.h>

namespace timr {

namespace driver {

class RobotDriverTest : public ::testing::Test {
protected:
    std::string robot_description_path = "/home/kai/Projects/timr/ros2_ws/src/driver/config/robot_driver.yaml";
public:

    void SetUp() override {

    }    

    void TearDown() override {

    }
};


TEST_F(RobotDriverTest, TEST_CONSTRUCTOR_FROM_YAML) {
    if (TEST_WITH_REAL_DRIVER) {
        timr::driver::SerialManipulatorDriver robot_driver(robot_description_path);
        EXPECT_NO_THROW();
        sleep(2);
    }
}


TEST_F(RobotDriverTest, TEST_POS_CONTROL) {
    if (TEST_WITH_REAL_DRIVER) {
        timr::driver::SerialManipulatorDriver robot_driver(robot_description_path);
        EXPECT_NO_THROW(robot_driver.position_control({10, 10, 10, 10, 10, 10}, {0.5, 0.5, 0.5, 0.5, 0.5, 0.5}, {0.1, 0.1, 0.1, 0.1, 0.1, 0.1}));
        sleep(5);
    }
}   


TEST_F(RobotDriverTest, TEST_EMERGENCY_STOP) {
    if (TEST_WITH_REAL_DRIVER) {
        timr::driver::SerialManipulatorDriver robot_driver(robot_description_path);
        EXPECT_NO_THROW(robot_driver.emergency_stop());
        sleep(2);
    }
}   

TEST_F(RobotDriverTest, TEST_HOMING) {
    if (TEST_WITH_REAL_DRIVER) {
        timr::driver::SerialManipulatorDriver robot_driver(robot_description_path);
        EXPECT_NO_THROW(robot_driver.homing());
        sleep(2);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

} // namespace driver

} // namespace timr