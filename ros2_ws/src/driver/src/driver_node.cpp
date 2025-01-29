// ROS2 Headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Project Headers
#include "robot_driver.hpp"

// Third-party Libraries
#include <yaml-cpp/yaml.h>

// C++ Standard Library
#include <array>
#include <filesystem>
#include <fstream>
#include <memory>
#include <signal.h>
#include <thread>

namespace timr {

namespace driver {

volatile sig_atomic_t kill_this_node = 0;

void signal_handler([[maybe_unused]] int signum) {
    kill_this_node = 1;
}

class DriverNode : public rclcpp::Node
{

private:
    std::unique_ptr<timr::driver::SerialManipulatorDriver> _driver;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _pub_joint_state;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _sub_target_joint_state;
    rclcpp::TimerBase::SharedPtr _timer;
    sensor_msgs::msg::JointState _cached_joint_state_msg;

    void _callback_timer() {
        _cached_joint_state_msg.header.stamp = now();
        
        try {
            // Get current joint positions and velocities with safety checks
            const auto& positions = _driver->get_joint_positions();
            const auto& velocities = _driver->get_joint_velocities();

            // Copy directly into existing vectors
            std::copy(positions.begin(), positions.end(), _cached_joint_state_msg.position.begin());
            std::copy(velocities.begin(), velocities.end(), _cached_joint_state_msg.velocity.begin());
            
            _pub_joint_state->publish(_cached_joint_state_msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error in publishing joint states: %s", e.what());
        }
    }

    void _callback_target_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Convert vectors to arrays
        std::array<scalar_t, DOF> positions;
        std::array<scalar_t, DOF> velocities;
        std::array<scalar_t, DOF> accelerations;
        
        std::copy(msg->position.begin(), msg->position.end(), positions.begin());
        std::copy(msg->velocity.begin(), msg->velocity.end(), velocities.begin());
        std::copy(msg->effort.begin(), msg->effort.end(), accelerations.begin());

        try {
            _driver->position_control(positions, velocities, accelerations);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error in position control: %s", e.what());
        }
    }
public:
    DriverNode()
    : Node("driver_node")
    {
        RCLCPP_INFO(this->get_logger(), "Driver node Created");
    }
    
    bool initialize() {        
        RCLCPP_INFO(this->get_logger(), "Initializing driver node...");
        this->declare_parameter("robotDriverDescriptionPath", "");
        this->declare_parameter("samplingTimeSec", 0.001);
        const auto robot_driver_description_path = this->get_parameter("robotDriverDescriptionPath").as_string();
        const auto sampling_time_sec = this->get_parameter("samplingTimeSec").as_double();

        try {
            if (!_driver) {
                RCLCPP_INFO(get_logger(), "Attempting to create driver instance...");
                RCLCPP_INFO(get_logger(), "Using robot description: %s", robot_driver_description_path.c_str());
                
                _driver = std::make_unique<timr::driver::SerialManipulatorDriver>(robot_driver_description_path);
                
                if (!_driver) {
                    RCLCPP_ERROR(get_logger(), "Failed to create driver instance");
                    return false;
                }
                RCLCPP_INFO(get_logger(), "Driver instance created successfully");
            }

            RCLCPP_INFO(get_logger(), "Initializing driver...");
            std::array<scalar_t, DOF> positions = _driver->get_joint_positions();
            
            RCLCPP_INFO(get_logger(), "Initial joint positions: %f, %f, %f, %f, %f, %f", 
                        positions[0], positions[1], positions[2], positions[3], positions[4], positions[5]);

            _cached_joint_state_msg.name.resize(DOF);
            _cached_joint_state_msg.position.resize(DOF);
            _cached_joint_state_msg.velocity.resize(DOF);
            _cached_joint_state_msg.effort.resize(DOF);
            
            // Set joint names
            for (dof_size_t i = 0; i < DOF; ++i) {
                _cached_joint_state_msg.name[i] = "joint" + std::to_string(i + 1);
            }

            // Create publisher for current joint states
            _pub_joint_state = create_publisher<sensor_msgs::msg::JointState>(
                "/joint_state", 10);

            // Create subscriber for target joint states
            _sub_target_joint_state = create_subscription<sensor_msgs::msg::JointState>(
                "/target_joint_state", 10,
                std::bind(&DriverNode::_callback_target_joint_states, this, std::placeholders::_1));

            // Create timer for publishing current joint states
            _timer = create_wall_timer(
                std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(sampling_time_sec)),
                std::bind(&DriverNode::_callback_timer, this));
            RCLCPP_INFO(get_logger(), "Driver node Initialized");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize driver: %s", e.what());
            return false;
        }
    }

    // Add destructor to ensure clean driver shutdown
    ~DriverNode() {
        if (_driver) {
            try {
                _driver.reset();
                _timer->cancel();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Error during driver cleanup: %s", e.what());
            }
        }
    }

    void stop_timer() {
        _timer->cancel();
    }

    void cleanup_driver() {
        _driver.reset();
    }

};

int run_driver_node(int argc, char** argv)
{
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGQUIT, signal_handler);
    
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DriverNode>();

    if (!node->initialize()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize driver node.");
        rclcpp::shutdown();
        return 1;
    }
    while (rclcpp::ok() && !kill_this_node) {
        rclcpp::spin_some(node);
        // Add small sleep to prevent CPU hogging
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down driver node...");
    rclcpp::shutdown();
    return 0;
}

} // namespace driver

} // namespace timr

int main(int argc, char** argv)
{
    return timr::driver::run_driver_node(argc, argv);
}
