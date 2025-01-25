#include "joint_driver.hpp"

#include <gtest/gtest.h>

namespace timr {

namespace driver {

constexpr bool TEST_WITH_REAL_DRIVER = true;

class JointDriverTest : public ::testing::Test {
protected:
    timr::driver::JointDriver::Config joint_driver_config;
    int can_socket;
    std::string joint_description_path = "/home/kai/Projects/timr/ros2_ws/src/driver/config/joint6.yaml";

public:

    void SetUp() override {

        joint_driver_config.addr = 0x06;
        joint_driver_config.reducer_ratio = 51;
        joint_driver_config.motor_step_angle = timr::driver::JointDriver::MotorStepAngle::MOTOR_1_8_DEG;
        joint_driver_config.pulse_mode = timr::driver::JointDriver::PulseMode::PUL_FOC;
        joint_driver_config.comm_mode = timr::driver::JointDriver::CommMode::CAN1_MAP;
        joint_driver_config.en_pin_mode = timr::driver::JointDriver::EnPinMode::HOLD;
        joint_driver_config.dir_pin_mode = timr::driver::JointDriver::DirPinMode::CW;
        joint_driver_config.microstep = 8;
        joint_driver_config.interpolation = timr::driver::JointDriver::EnabledState::ENABLED;
        joint_driver_config.auto_screen_off = timr::driver::JointDriver::EnabledState::DISABLED;
        joint_driver_config.open_loop_current_ma = 500;
        joint_driver_config.closed_loop_stall_current_ma = 3000;
        joint_driver_config.closed_loop_max_voltage_mv = 4000;
        joint_driver_config.uart_baudrate = timr::driver::JointDriver::BaudRate::BAUD_115200;
        joint_driver_config.can_baudrate = timr::driver::JointDriver::CanBaudRate::CAN_1M;
        joint_driver_config.response_type = timr::driver::JointDriver::ResponseType::RECEIVE;
        joint_driver_config.checksum_type = timr::driver::JointDriver::ChecksumType::CRC8;
        joint_driver_config.stall_protection = timr::driver::JointDriver::EnabledState::ENABLED;
        joint_driver_config.stall_threshold_rpm = 40;
        joint_driver_config.stall_current_threshold_ma = 2400;
        joint_driver_config.stall_detection_ms = 4000;
        joint_driver_config.position_tolerance_deg = 0.1;

        can_socket = bind_can_socket("can0");
        ASSERT_NE(can_socket, -1) << "Failed to create CAN socket";
    }    

    void TearDown() override {
        if (can_socket >= 0) {
            close(can_socket);
        }
    }
};

TEST_F(JointDriverTest, TEST_CONFIG) {
    auto config = JointDriver::config_from_yaml(joint_description_path);
    EXPECT_EQ(config.addr, joint_driver_config.addr);
    EXPECT_EQ(config.reducer_ratio, joint_driver_config.reducer_ratio);
    EXPECT_EQ(config.motor_step_angle, joint_driver_config.motor_step_angle);
    EXPECT_EQ(config.pulse_mode, joint_driver_config.pulse_mode);
    EXPECT_EQ(config.comm_mode, joint_driver_config.comm_mode);
    EXPECT_EQ(config.en_pin_mode, joint_driver_config.en_pin_mode);
    EXPECT_EQ(config.dir_pin_mode, joint_driver_config.dir_pin_mode);
    EXPECT_EQ(config.microstep, joint_driver_config.microstep);
    EXPECT_EQ(config.interpolation, joint_driver_config.interpolation);
    EXPECT_EQ(config.auto_screen_off, joint_driver_config.auto_screen_off);
    EXPECT_EQ(config.open_loop_current_ma, joint_driver_config.open_loop_current_ma);
    EXPECT_EQ(config.closed_loop_stall_current_ma, joint_driver_config.closed_loop_stall_current_ma);
    EXPECT_EQ(config.closed_loop_max_voltage_mv, joint_driver_config.closed_loop_max_voltage_mv);
    EXPECT_EQ(config.uart_baudrate, joint_driver_config.uart_baudrate);
    EXPECT_EQ(config.can_baudrate, joint_driver_config.can_baudrate);
    EXPECT_EQ(config.response_type, joint_driver_config.response_type);
    EXPECT_EQ(config.checksum_type, joint_driver_config.checksum_type);
    EXPECT_EQ(config.stall_protection, joint_driver_config.stall_protection);
    EXPECT_EQ(config.stall_threshold_rpm, joint_driver_config.stall_threshold_rpm);
    EXPECT_EQ(config.stall_current_threshold_ma, joint_driver_config.stall_current_threshold_ma);
    EXPECT_EQ(config.stall_detection_ms, joint_driver_config.stall_detection_ms);
    EXPECT_EQ(config.position_tolerance_deg, joint_driver_config.position_tolerance_deg);
}

TEST_F(JointDriverTest, TEST_CONSTRUCTOR) {
    if (TEST_WITH_REAL_DRIVER) {
        timr::driver::JointDriver joint_driver1(can_socket, joint_description_path);
        timr::driver::JointDriver joint_driver2(can_socket, joint_driver_config);
        EXPECT_NO_THROW();
    }
}

TEST_F(JointDriverTest, TEST_CONTROL) {
    if (TEST_WITH_REAL_DRIVER) {
        timr::driver::JointDriver joint_driver(can_socket, joint_description_path);

        EXPECT_NO_THROW(joint_driver.vel_control(10, 0.1));
        EXPECT_NO_THROW(joint_driver.pos_control(3.14, 10, 0.1));
        EXPECT_NO_THROW(joint_driver.emergency_stop());
        EXPECT_NO_THROW(joint_driver.pos_control(0, 10, 0.1));
    }
}   

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

} // namespace driver

} // namespace timr