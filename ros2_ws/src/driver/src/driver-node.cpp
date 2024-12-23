#include "driver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>
#include <memory>
#include <signal.h>
#include <thread>
#include <fstream>

volatile sig_atomic_t kill_this_node = 0;

void signal_handler(int signum) {
    kill_this_node = 1;
}

extern "C" {
    #define NON_MATLAB_PARSING
    #define MAX_EXT_API_CONNECTIONS 255
    #define NO_NOT_USE_SHARED_MEMORY
    #include "extApi.h"
}

class DriverNode : public rclcpp::Node
{

private:
    std::string _robot_description_path;
    std::unique_ptr<driver::SerialManipulatorDriver> _driver;
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
            const auto& accelerations = _driver->get_joint_accelerations();

            // Copy directly into existing vectors
            std::copy(positions.begin(), positions.end(), _cached_joint_state_msg.position.begin());
            std::copy(velocities.begin(), velocities.end(), _cached_joint_state_msg.velocity.begin());
            std::copy(accelerations.begin(), accelerations.end(), _cached_joint_state_msg.effort.begin());
            
            _pub_joint_state->publish(_cached_joint_state_msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error in publishing joint states: %s", e.what());
        }
    }

    void _callback_target_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Convert vectors to arrays
        std::array<scalar_t, DOF> positions;
        std::array<scalar_t, DOF> velocities;
        std::array<scalar_t, DOF> accelerations;  // Default acceleration values
        
        std::copy(msg->position.begin(), msg->position.end(), positions.begin());
        std::copy(msg->velocity.begin(), msg->velocity.end(), velocities.begin());
        std::copy(msg->effort.begin(), msg->effort.end(), accelerations.begin());

        _driver->set_target_joint_positions(positions);
        _driver->set_target_joint_velocities(velocities);
        _driver->set_target_joint_accelerations(accelerations);

        try {
            _driver->update();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error in position control: %s", e.what());
        }
    }
public:
    DriverNode(const std::string& robot_description_path)
    : Node("driver_node"), _robot_description_path(robot_description_path)
    {
        RCLCPP_INFO(this->get_logger(), "Driver node Created");
    }
    
    bool initialize() {        
        // Initialize message vectors first
        _cached_joint_state_msg.name.resize(DOF);
        _cached_joint_state_msg.position.resize(DOF);
        _cached_joint_state_msg.velocity.resize(DOF);
        _cached_joint_state_msg.effort.resize(DOF);  // Make sure effort is also resized
        
        // Set joint names
        for (dof_size_t i = 0; i < DOF; ++i) {
            _cached_joint_state_msg.name[i] = "joint" + std::to_string(i + 1);
        }

        // Check if file exists first
        std::ifstream file(_robot_description_path);
        if (!file.good()) {
            RCLCPP_ERROR(get_logger(), "Robot description file not found: %s", _robot_description_path.c_str());
            return false;
        }
        file.close();

        try {
            if (!_driver) {
                RCLCPP_INFO(get_logger(), "Attempting to create driver instance...");
                RCLCPP_INFO(get_logger(), "Using robot description: %s", _robot_description_path.c_str());
                
                _driver = std::make_unique<driver::SerialManipulatorDriver>(_robot_description_path);
                
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

            // Create publisher for current joint states
            _pub_joint_state = create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states", 10);

            // Create subscriber for target joint states
            _sub_target_joint_state = create_subscription<sensor_msgs::msg::JointState>(
                "/target_joint_states", 10,
                std::bind(&DriverNode::_callback_target_joint_state, this, std::placeholders::_1));

            // Create timer for publishing current joint states
            _timer = create_wall_timer(
                std::chrono::milliseconds(10),  // 100Hz
                std::bind(&DriverNode::_callback_timer, this));
            RCLCPP_INFO(get_logger(), "Driver node Initialized");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize driver: %s", e.what());
            return false;
        }
    }

};

int main(int argc, char** argv)
{
    signal(SIGINT, signal_handler);
    
    rclcpp::init(argc, argv);
    std::string robot_description_path = "/home/kai/Projects/timr/ros2_ws/src/driver/config/robot.json";
    auto node = std::make_shared<DriverNode>(robot_description_path);

    if (!node->initialize()) {
        RCLCPP_INFO(node->get_logger(), "Failed to initialize driver node.");
        rclcpp::shutdown();
        return 1;
    }
    
    while (rclcpp::ok() && !kill_this_node) {
        rclcpp::spin_some(node);
    }
    
    RCLCPP_INFO(node->get_logger(), "Shutting down driver node...");
    rclcpp::shutdown();
    return 0;
}
