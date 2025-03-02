#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <boost/asio.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class Gripper : public rclcpp::Node 
{
public:
    struct Config
    {
        int baud_rate{115200};
        double filter_alpha{0.2};
        double sampling_time_sec{0.004};
        std::string port{"/dev/ttyACM0"};
    };
    Gripper(const Config& config) : Node("gripper"), 
                     _config(config),
                     _io_service(),
                     _serial(_io_service)
    {
        RCLCPP_INFO(this->get_logger(), "Creating Serial Reader Node");
    }

    bool initialize()
    {
        _sub_gripper = this->create_subscription<sensor_msgs::msg::JointState>("target_gripper_state", 10,
                             std::bind(&Gripper::_callback_gripper, this, std::placeholders::_1));
        _cached_gripper_state.name.push_back("gripper");
        _cached_gripper_state.position.push_back(90.0);
        _cached_gripper_state.velocity.push_back(0.0);
        _cached_gripper_state.effort.push_back(0.0);
        
        // Configure serial port
        try {
            _serial.open(_config.port);  // Change this to match your Arduino port
            _serial.set_option(boost::asio::serial_port_base::baud_rate(_config.baud_rate));
            _serial.set_option(boost::asio::serial_port_base::character_size(8));
            _serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            _serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            _serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port: %s", e.what());
            return false;
        }

        // Create timer for reading serial data
        _timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(_config.sampling_time_sec * 1000)),
            std::bind(&Gripper::_timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Serial Reader Node Initialized");
        return true;
    }

    ~Gripper() {
        if (_serial.is_open()) {
            _serial.close();
        }
    }

private:
    void _callback_gripper(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        try {
            // Convert integer to string and add newline character
            std::string data = std::to_string(msg->position[0]) + "\n";
            
            boost::system::error_code error_write;
            size_t bytes_written = boost::asio::write(_serial, boost::asio::buffer(data), error_write);
            if (error_write) {
                RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port: %s", error_write.message().c_str());
                return;
            }
            
            if (bytes_written != data.length()) {
                RCLCPP_WARN(this->get_logger(), "Incomplete write: wrote %zu bytes out of %zu", bytes_written, data.length());
            }
            _cached_gripper_state = *msg;
            _cached_gripper_state.header.stamp = this->now();

        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial write error: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Unexpected error during serial write: %s", e.what());
        }
    }
    void _timer_callback()
    {
        // do nothing
    }
    Config _config;
    boost::asio::io_service _io_service;
    boost::asio::serial_port _serial;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _sub_gripper;
    sensor_msgs::msg::JointState _cached_gripper_state;
    rclcpp::TimerBase::SharedPtr _timer;
};

Gripper::Config convert_yaml_to_config(const YAML::Node& config)
{
    Gripper::Config cfg;
    cfg.port = config["port"].as<std::string>();
    cfg.baud_rate = config["baud_rate"].as<int>();
    cfg.filter_alpha = config["filter_alpha"].as<double>();
    cfg.sampling_time_sec = config["sampling_time_sec"].as<double>();
    return cfg;
}

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    std::string config_path = ament_index_cpp::get_package_share_directory("gripper") + "/config/config.yaml";
    RCLCPP_INFO(rclcpp::get_logger("gripper"), "Loading config from %s", config_path.c_str());
    YAML::Node config = YAML::LoadFile(config_path);
    auto node = std::make_shared<Gripper>(convert_yaml_to_config(config));
    if (!node->initialize()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize node");
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}