#include "joint_driver.hpp"

#include <gtest/gtest.h>

namespace timr {

namespace driver {


class JointDriverTest : public ::testing::Test {
protected:
    int can_socket;
    timr::driver::JointDriver::Config joint1_driver_config;
    std::string joint1_description_path = "/home/kai/Projects/timr/ros2_ws/src/driver/config/joint1.yaml";
    std::unique_ptr<timr::driver::JointDriver> joint_driver;

public:

    void SetUp() override {

        // Joint 1 configuration
        joint1_driver_config.addr = 0x01;
        joint1_driver_config.reducer_ratio = 50;
        joint1_driver_config.motor_step_angle = timr::driver::JointDriver::MotorStepAngle::MOTOR_1_8_DEG;
        joint1_driver_config.pulse_mode = timr::driver::JointDriver::PulseMode::PUL_FOC;
        joint1_driver_config.comm_mode = timr::driver::JointDriver::CommMode::CAN1_MAP;
        joint1_driver_config.en_pin_mode = timr::driver::JointDriver::EnPinMode::HOLD;
        joint1_driver_config.motor_plus_dir = timr::driver::JointDriver::MotorPlusDir::CCW;
        joint1_driver_config.microstep = 8;
        joint1_driver_config.interpolation = timr::driver::JointDriver::EnabledState::ENABLED;
        joint1_driver_config.auto_screen_off = timr::driver::JointDriver::EnabledState::DISABLED;
        joint1_driver_config.open_loop_current_ma = 1000;
        joint1_driver_config.closed_loop_stall_current_ma = 3000;
        joint1_driver_config.closed_loop_max_voltage_mv = 4000;
        joint1_driver_config.uart_baudrate = timr::driver::JointDriver::BaudRate::BAUD_115200;
        joint1_driver_config.can_baudrate = timr::driver::JointDriver::CanBaudRate::CAN_1M;
        joint1_driver_config.response_type = timr::driver::JointDriver::ResponseType::RECEIVE;
        joint1_driver_config.checksum_type = timr::driver::JointDriver::ChecksumType::CRC8;
        joint1_driver_config.stall_protection = timr::driver::JointDriver::EnabledState::ENABLED;
        joint1_driver_config.stall_threshold_rpm = 40;
        joint1_driver_config.stall_current_threshold_ma = 2400;
        joint1_driver_config.stall_detection_ms = 4000;
        joint1_driver_config.position_tolerance_deg = 0.1;

        can_socket = bind_can_socket("can0");
        ASSERT_NE(can_socket, -1) << "Failed to create CAN socket";
        joint_driver = std::make_unique<timr::driver::JointDriver>(can_socket, joint1_description_path);
    }    

    void TearDown() override {
        if (can_socket >= 0) {
            close(can_socket);
        }
    }
};

TEST_F(JointDriverTest, TEST_CONFIG) {
    auto config1 = JointDriver::config_from_yaml(joint1_description_path);
    EXPECT_EQ(config1.addr, joint1_driver_config.addr);
    EXPECT_EQ(config1.reducer_ratio, joint1_driver_config.reducer_ratio);
    EXPECT_EQ(config1.motor_step_angle, joint1_driver_config.motor_step_angle);
    EXPECT_EQ(config1.pulse_mode, joint1_driver_config.pulse_mode);
    EXPECT_EQ(config1.comm_mode, joint1_driver_config.comm_mode);
    EXPECT_EQ(config1.en_pin_mode, joint1_driver_config.en_pin_mode);
    EXPECT_EQ(config1.motor_plus_dir, joint1_driver_config.motor_plus_dir);
    EXPECT_EQ(config1.microstep, joint1_driver_config.microstep);
    EXPECT_EQ(config1.interpolation, joint1_driver_config.interpolation);
    EXPECT_EQ(config1.auto_screen_off, joint1_driver_config.auto_screen_off);
    EXPECT_EQ(config1.open_loop_current_ma, joint1_driver_config.open_loop_current_ma);
    EXPECT_EQ(config1.closed_loop_stall_current_ma, joint1_driver_config.closed_loop_stall_current_ma);
    EXPECT_EQ(config1.closed_loop_max_voltage_mv, joint1_driver_config.closed_loop_max_voltage_mv);
    EXPECT_EQ(config1.uart_baudrate, joint1_driver_config.uart_baudrate);
    EXPECT_EQ(config1.can_baudrate, joint1_driver_config.can_baudrate);
    EXPECT_EQ(config1.response_type, joint1_driver_config.response_type);
    EXPECT_EQ(config1.checksum_type, joint1_driver_config.checksum_type);
    EXPECT_EQ(config1.stall_protection, joint1_driver_config.stall_protection);
    EXPECT_EQ(config1.stall_threshold_rpm, joint1_driver_config.stall_threshold_rpm);
    EXPECT_EQ(config1.stall_current_threshold_ma, joint1_driver_config.stall_current_threshold_ma);
    EXPECT_EQ(config1.stall_detection_ms, joint1_driver_config.stall_detection_ms);
    EXPECT_EQ(config1.position_tolerance_deg, joint1_driver_config.position_tolerance_deg);
}

TEST_F(JointDriverTest, TEST_CONSTRUCTOR_FROM_CONFIG) {
    if (TEST_WITH_REAL_DRIVER) {
        auto config1 = JointDriver::config_from_yaml(joint1_description_path);
        timr::driver::JointDriver joint_driver(can_socket, config1);
        EXPECT_NO_THROW();
        sleep(2);
    }
}

TEST_F(JointDriverTest, TEST_CONSTRUCTOR_FROM_YAML) {
    if (TEST_WITH_REAL_DRIVER) {
        timr::driver::JointDriver joint_driver(can_socket, joint1_description_path);
        EXPECT_NO_THROW();
        sleep(2);
    }
}

TEST_F(JointDriverTest, TEST_CONFIGURATION) {
    if (TEST_WITH_REAL_DRIVER) {
        auto config = joint_driver->get_config();

        EXPECT_NO_THROW(joint_driver->send_config_to_driver(config));
        sleep(2);
    }
}

TEST_F(JointDriverTest, TEST_VEL_CONTROL) {
    if (TEST_WITH_REAL_DRIVER) {
        EXPECT_NO_THROW(joint_driver->velocity_control(0.5, 0.1));
        sleep(5);
    }
}   

TEST_F(JointDriverTest, TEST_EMERGENCY_STOP_VEL_CONTROL) {
    if (TEST_WITH_REAL_DRIVER) {
        EXPECT_NO_THROW(joint_driver->emergency_stop());
        sleep(2);
    }
}   

TEST_F(JointDriverTest, TEST_POS_CONTROL) {
    if (TEST_WITH_REAL_DRIVER) {
        EXPECT_NO_THROW(joint_driver->position_control(60, 0.5, 0.1));
        sleep(5);
    }
}   

TEST_F(JointDriverTest, TEST_EMERGENCY_STOP_POS_CONTROL) {
    if (TEST_WITH_REAL_DRIVER) {
        EXPECT_NO_THROW(joint_driver->emergency_stop());
        sleep(2);
    }
}   

TEST_F(JointDriverTest, TEST_GET_REALTIME_POSITION_BEFORE_HOMING) {
    if (TEST_WITH_REAL_DRIVER) {
        auto pos = joint_driver->read_realtime_position();
        std::cout << "Position: " << pos << std::endl;
        sleep(2);
    }
}

TEST_F(JointDriverTest, TEST_GET_REALTIME_VELOCITY_BEFORE_HOMING) {
    if (TEST_WITH_REAL_DRIVER) {
        auto vel = joint_driver->read_realtime_velocity();
        std::cout << "Velocity: " << vel << std::endl;
        sleep(2);
    }
}

TEST_F(JointDriverTest, TEST_HOMMING) {
    if (TEST_WITH_REAL_DRIVER) {
        EXPECT_NO_THROW(joint_driver->homing());
        sleep(2);
    }
}

TEST_F(JointDriverTest, TEST_GET_REALTIME_POSITION_AFTER_HOMING) {
    if (TEST_WITH_REAL_DRIVER) {
        auto pos = joint_driver->read_realtime_position();
        std::cout << "Position: " << pos << std::endl;
        sleep(2);
    }
}


TEST_F(JointDriverTest, TEST_GET_REALTIME_VELOCITY_AFTER_HOMING) {
    if (TEST_WITH_REAL_DRIVER) {
        auto vel = joint_driver->read_realtime_velocity();
        std::cout << "Velocity: " << vel << std::endl;
        sleep(2);
    }
}

TEST_F(JointDriverTest, TEST_CLEAR_POSITION) {
    if (TEST_WITH_REAL_DRIVER) {
        EXPECT_NO_THROW(joint_driver->clear_position());
        sleep(2);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

} // namespace driver

} // namespace timr