#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <sensor_msgs/msg/joint_state.hpp>
#include <boost/asio.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/int32.hpp>

using dof_size_t = uint8_t;
constexpr dof_size_t DOF = 6;
constexpr dof_size_t SERIAL_SIZE=8;

class Controller : public rclcpp::Node 
{
public:
    struct Config
    {
        int baud_rate{115200};
        double filter_alpha{0.2};
        double sampling_time_sec{0.004};
        double pos_gain{1.0};
        double vel_gain{10.0};
        double effort{1.0};
        std::string port{"/dev/ttyACM0"};
    };
    Controller(const Config& config) : Node("controller"), 
                     _config(config),
                     _io_service(),
                     _serial(_io_service)
    {        
        RCLCPP_INFO(this->get_logger(), "Creating Serial Reader Node");
    }

    bool initialize()
    {
        // Create publisher
        _pub_target_joint_state = this->create_publisher<sensor_msgs::msg::JointState>("target_joint_state", 10);
        _pub_target_gripper_state = this->create_publisher<sensor_msgs::msg::JointState>("target_gripper_state", 10);
        _pub_gripper_state = this->create_publisher<sensor_msgs::msg::JointState>("gripper_state", 10);
            
        _target_joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        _target_joint_state.position.resize(DOF);
        _target_joint_state.velocity.resize(DOF);
        _target_joint_state.effort.resize(DOF);

        _previous_target_joint_state.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        _previous_target_joint_state.position.resize(DOF);
        _previous_target_joint_state.velocity.resize(DOF);
        _previous_target_joint_state.effort.resize(DOF);

        _target_gripper_state.name = {"gripper"};
        _target_gripper_state.position.resize(1);
        _target_gripper_state.velocity.resize(1);
        _target_gripper_state.effort.resize(1);
        _target_gripper_state.position[0] = 90.0;
        _target_gripper_state.velocity[0] = 0.0;
        _target_gripper_state.effort[0] = 0.0;

        _gripper_state = _target_gripper_state;
        
        // Initialize joint states with zeros
        for (size_t i = 0; i < DOF; ++i) {
            _target_joint_state.position[i] = 0.0;
            _target_joint_state.velocity[i] = 0.0;
            _target_joint_state.effort[i] = 0.0;

            _previous_target_joint_state.position[i] = 0.0;
            _previous_target_joint_state.velocity[i] = 0.0;
            _previous_target_joint_state.effort[i] = 0.0;
        }
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
            std::bind(&Controller::send_target_joint_state, this)
        );

        RCLCPP_INFO(this->get_logger(), "Serial Reader Node Initialized");
        return true;
    }

    ~Controller() {
        if (_serial.is_open()) {
            _serial.close();
        }
    }

private:
    Config _config;
    sensor_msgs::msg::JointState _gripper_state;
    sensor_msgs::msg::JointState _target_gripper_state;
    sensor_msgs::msg::JointState _target_joint_state;
    sensor_msgs::msg::JointState _previous_target_joint_state;
    boost::asio::io_service _io_service;
    boost::asio::serial_port _serial;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _pub_target_joint_state;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _pub_target_gripper_state;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _pub_gripper_state;
    rclcpp::TimerBase::SharedPtr _timer;

    std::vector<int> parse_serial(const std::string& line) {
        std::vector<int> values;
        values.reserve(SERIAL_SIZE);  // Reserve space for efficiency
        
        std::stringstream ss(line);
        std::string token;
        
        while (std::getline(ss, token, ',') && values.size() < SERIAL_SIZE) {
            try {
                values.push_back(std::clamp(std::stoi(token), 0, 1023));
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse value: %s", token.c_str());
            }
        }
        return values;
    }

    void send_target_joint_state() 
    {
        boost::asio::streambuf buffer;
        try {
            buffer.consume(buffer.size());
            boost::asio::read_until(_serial, buffer, '\n');
            std::string line{boost::asio::buffers_begin(buffer.data()),
                           boost::asio::buffers_end(buffer.data())};
            // Validate line length before parsing
            if (line.empty() || line.length() > 1024) {  // reasonable max length
                RCLCPP_WARN(this->get_logger(), "Invalid line length: %zu", line.length());
                return;
            }
            std::vector<int> values = parse_serial(line);
            
            // Check array size before accessing
            if (values.size() != SERIAL_SIZE) {
                RCLCPP_ERROR(this->get_logger(), "Received wrong number of values: %zu", values.size());
                return;
            }
            
            // Apply offsets safely
            std::vector<int> adjusted_values = values;
            adjusted_values[0] = values[0] - 512;
            adjusted_values[1] = 870 - values[1];
            adjusted_values[2] = 820 - values[2];
            adjusted_values[3] = 512 - values[3];
            adjusted_values[4] = 512 - values[4];
            adjusted_values[5] = values[5] - 512;
            
            // Map to radians with bounds checking
            std::vector<double> angles(DOF, 0.0);
            for (size_t i = 0; i < adjusted_values.size(); i++) {
                constexpr double factor = 1.0 / 1023.0 * 300.0 * M_PI / 180.0;
                angles[i] = static_cast<double>(adjusted_values[i]) * factor;
            }
            
            for (size_t i = 0; i < DOF; i++) {
                _target_joint_state.position[i] = _config.filter_alpha * angles[i] + (1.0 - _config.filter_alpha) * _target_joint_state.position[i];
                _target_joint_state.position[i] *= _config.pos_gain;
            }

            // Calculate velocity safely
            std::vector<double> velocity(DOF, 0.0);
            for (size_t i = 0; i < DOF; i++) {
                velocity[i] = (_target_joint_state.position[i] - _previous_target_joint_state.position[i]) * _config.vel_gain;
            }

            for (size_t i = 0; i < DOF; i++) {
                _target_joint_state.velocity[i] = _config.filter_alpha * velocity[i] + (1.0 - _config.filter_alpha) * _target_joint_state.velocity[i];
            }

            // Prepare message with safety checks
            _target_joint_state.header.stamp = this->get_clock()->now();
            _target_joint_state.effort = std::vector<double>(DOF, _config.effort);
            _previous_target_joint_state = _target_joint_state;
            _pub_target_joint_state->publish(_target_joint_state);

            int target_gripper_position = static_cast<int>(values[SERIAL_SIZE-2] * 180.0 / 1023.0);
            _target_gripper_state.velocity[0] = target_gripper_position - _target_gripper_state.position[0];
            _target_gripper_state.position[0] = target_gripper_position;
            _target_gripper_state.header.stamp = this->get_clock()->now();
            _pub_target_gripper_state->publish(_target_gripper_state);

            int gripper_position = values[SERIAL_SIZE-1];
            _gripper_state.velocity[0] = gripper_position - _gripper_state.position[0];
            _gripper_state.position[0] = gripper_position;
            _gripper_state.header.stamp = this->get_clock()->now();
            _pub_gripper_state->publish(_gripper_state);
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing data: %s", e.what());
        }
    }

};

Controller::Config convert_yaml_to_config(const YAML::Node& config)
{
    Controller::Config cfg;
    cfg.port = config["port"].as<std::string>();
    cfg.baud_rate = config["baud_rate"].as<int>();
    cfg.filter_alpha = config["filter_alpha"].as<double>();
    cfg.sampling_time_sec = config["sampling_time_sec"].as<double>();
    cfg.pos_gain = config["pos_gain"].as<double>();
    cfg.vel_gain = config["vel_gain"].as<double>();
    cfg.effort = config["effort"].as<double>();
    return cfg;
}

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    std::string config_path = ament_index_cpp::get_package_share_directory("controller") + "/config/controller.yaml";
    RCLCPP_INFO(rclcpp::get_logger("controller"), "Loading config from %s", config_path.c_str());
    YAML::Node config = YAML::LoadFile(config_path);
    auto node = std::make_shared<Controller>(convert_yaml_to_config(config));
    if (!node->initialize()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize node");
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}