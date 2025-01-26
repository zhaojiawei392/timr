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
    
    bool initialize(const std::string& robot_driver_description_path) {        
        RCLCPP_INFO(this->get_logger(), "Initializing driver node...");
        // Check if file exists first
        std::ifstream file(robot_driver_description_path);
        if (!file.good()) {
            RCLCPP_ERROR(get_logger(), "Robot description file not found: %s", robot_driver_description_path.c_str());
            return false;
        }
        file.close();

        // Initialize message vectors first
        _cached_joint_state_msg.name.resize(DOF);
        _cached_joint_state_msg.position.resize(DOF);
        _cached_joint_state_msg.velocity.resize(DOF);
        _cached_joint_state_msg.effort.resize(DOF);  // Make sure effort is also resized
        
        // Set joint names
        for (dof_size_t i = 0; i < DOF; ++i) {
            _cached_joint_state_msg.name[i] = "joint" + std::to_string(i + 1);
        }

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

            // Create publisher for current joint states
            _pub_joint_state = create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states", 10);

            // Create subscriber for target joint states
            _sub_target_joint_state = create_subscription<sensor_msgs::msg::JointState>(
                "/target_joint_states", 10,
                std::bind(&DriverNode::_callback_target_joint_states, this, std::placeholders::_1));

            // Create timer for publishing current joint states
            _timer = create_wall_timer(
                std::chrono::milliseconds(4),  // 250Hz
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

std::shared_ptr<DriverNode> global_node = nullptr;

void signal_handler([[maybe_unused]] int signum) {
    // Log which signal we received
    if (global_node) {
        try {
            RCLCPP_INFO(global_node->get_logger(), "Received signal %d, initiating shutdown...", signum);
        } catch (...) {}
    }

    // Set kill flag
    kill_this_node = 1;

    if (global_node) {
        try {
            global_node->stop_timer();
            global_node->cleanup_driver();
        } catch (...) {}
    }

    // For SIGTERM or SIGQUIT, exit directly after cleanup
    if (signum == SIGTERM || signum == SIGQUIT) {
        exit(0);
    }
}

int run_driver_node(int argc, char** argv)
{
    // Setup signal handlers for multiple signals
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    
    sigaction(SIGINT, &sa, NULL);   // Ctrl+C
    sigaction(SIGTERM, &sa, NULL);  // kill <pid>
    sigaction(SIGQUIT, &sa, NULL);  // Ctrl+backslash
    
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DriverNode>();
    global_node = node;

    // Load parameters from yaml file
    std::string param_file = "/home/kai/Projects/timr/ros2_ws/src/driver/config/driver_node.yaml";
    if (!std::filesystem::exists(param_file)) {
        RCLCPP_ERROR(rclcpp::get_logger("driver_node"), "Parameter file not found: %s", param_file.c_str());
        rclcpp::shutdown();
        return 1;
    }

    YAML::Node params = YAML::LoadFile(param_file);
    std::string robot_driver_description_path = params["robotDriverDescriptionPath"].as<std::string>();

    if (robot_driver_description_path.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("driver_node"), "robotDriverDescriptionPath not found in parameter file");
        rclcpp::shutdown();
        return 1;
    }

    if (!node->initialize(robot_driver_description_path)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize driver node.");
        rclcpp::shutdown();
        return 1;
    }
    try {
        while (rclcpp::ok() && !kill_this_node) {
            rclcpp::spin_some(node);
            // Add small sleep to prevent CPU hogging
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error during node execution: %s", e.what());
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down driver node...");
    
    // Cleanup in correct order with timeout protection
    if (node) {
        std::promise<void> cleanup_promise;
        auto cleanup_future = cleanup_promise.get_future();
        
        std::thread cleanup_thread([&node, &cleanup_promise]() {
            node->stop_timer();
            node->cleanup_driver();
            cleanup_promise.set_value();
        });

        // Wait for cleanup with timeout
        if (cleanup_future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
            RCLCPP_WARN(node->get_logger(), "Cleanup timed out after 3 seconds");
        }

        if (cleanup_thread.joinable()) {
            cleanup_thread.join();
        }
    }

    global_node.reset();
    node.reset();
    
    rclcpp::shutdown();
    
    return 0;
}

} // namespace driver

} // namespace timr

int main(int argc, char** argv)
{
    return timr::driver::run_driver_node(argc, argv);
}
