#pragma once
#include <boost/asio.hpp>
#include <iomanip>
#include <iostream>
#include <array>
#include <csignal>
#include <vector>
#include <thread>
#include <chrono>
#include <sstream>
#include <nlohmann/json.hpp>
#include <fstream>
#include <sensor_msgs/msg/joint_state.hpp>

using dof_size_t = uint8_t;
using scalar_t = double;
constexpr dof_size_t DOF = 6;

namespace driver {

class JointDriver {
protected:
    boost::asio::serial_port* _port;
    uint8_t _addr;
    uint8_t _microsteps;
    uint8_t _reducer_ratio;
    bool _is_clockwise_positive;

    template<std::size_t size>
    std::string hex_to_str(const std::array<uint8_t, size>& hex){
        std::ostringstream oss;
        for (size_t i = 0; i < hex.size(); ++i) {
            oss << std::hex << std::setw(2) << std::setfill('0') << (int)hex[i];
            if (i < hex.size() - 1) {
                oss << " ";  // Add space between numbers
            }
        }
        return oss.str();
    }
    inline bool set_microsteps(uint8_t microsteps) {
        std::array<uint8_t, 6> code = {_addr, 0x84, 0x8A, 0x01, microsteps, 0x6B};
        std::array<uint8_t, 4> response;
        boost::asio::write(*_port, boost::asio::buffer(code, code.size()));
        boost::asio::read(*_port, boost::asio::buffer(response, response.size()));
        std::array<uint8_t, 4> success = {_addr, 0x84, 0x02, 0x6B};
        uint32_t* response_hex = reinterpret_cast<uint32_t*>(response.data());
        uint32_t* success_hex = reinterpret_cast<uint32_t*>(success.data());
        
        return (*response_hex == *success_hex);
    }
public:
    explicit JointDriver(boost::asio::serial_port* port, uint8_t addr, uint8_t microsteps, uint8_t reducer_ratio, bool reverse_direction)
    : _port(port), _addr(addr), _microsteps(microsteps), _reducer_ratio(reducer_ratio), _is_clockwise_positive(reverse_direction) {
        set_microsteps( _microsteps );
    }
    inline scalar_t get_position() {
        std::array<uint8_t, 3> code = {_addr, 0x36, 0x6B};
        std::array<uint8_t, 8> response;
        boost::asio::write(*_port, boost::asio::buffer(code, code.size()));
        boost::asio::read(*_port, boost::asio::buffer(response, response.size()));
        
        // Properly reconstruct 32-bit value from bytes
        uint32_t pos_value = (static_cast<uint32_t>(response[3]) << 24) |
                            (static_cast<uint32_t>(response[4]) << 16) |
                            (static_cast<uint32_t>(response[5]) << 8)  |
                            (static_cast<uint32_t>(response[6]));
                            
        return pos_value / _reducer_ratio / 65536. * 2. * M_PI * ((response[2] == 0x00) == _is_clockwise_positive? 1 : -1);
    }
    inline scalar_t get_velocity() {
        std::array<uint8_t, 3> code = {_addr, 0x35, 0x6B};
        std::array<uint8_t, 6> response;
        boost::asio::write(*_port, boost::asio::buffer(code, code.size()));
        boost::asio::read(*_port, boost::asio::buffer(response, response.size()));

        uint16_t vel_value = (static_cast<uint16_t>(response[3]) << 8)  |
                            (static_cast<uint16_t>(response[4]));
                            
        return vel_value / _reducer_ratio / 60. * 2. * M_PI * ((response[0] == 0x00) == _is_clockwise_positive? 1 : -1);
    }
    inline bool set_open_loop(bool open_loop) {
        std::array<uint8_t, 6> code = {_addr, 0x46, 0x69, 0x01, static_cast<uint8_t>(!open_loop)+0x01, 0x6B};
        std::array<uint8_t, 4> response;
        boost::asio::write(*_port, boost::asio::buffer(code, code.size()));
        boost::asio::read(*_port, boost::asio::buffer(response, response.size()));
        std::array<uint8_t, 4> success = {_addr, 0x46, 0x02, 0x6B};
        uint32_t* response_hex = reinterpret_cast<uint32_t*>(response.data());
        uint32_t* success_hex = reinterpret_cast<uint32_t*>(success.data());

        return (*response_hex == *success_hex);
    }
    inline static void sync_move(boost::asio::serial_port* port) {
        std::array<uint8_t, 4> code = {0x00, 0xFF, 0x66, 0x6B};
        boost::asio::write(*port, boost::asio::buffer(code, code.size()));
    }
    inline bool emergency_stop(bool wait_sync=false) {
        std::array<uint8_t, 5> code = {_addr, 0xFE, 0x98, static_cast<uint8_t>(wait_sync), 0x6B};
        std::array<uint8_t, 4> response;
        boost::asio::write(*_port, boost::asio::buffer(code, code.size()));
        boost::asio::read(*_port, boost::asio::buffer(response, response.size()));
        std::array<uint8_t, 4> success = {_addr, 0xFE, 0x02, 0x6B};
        uint32_t* response_hex = reinterpret_cast<uint32_t*>(response.data());
        uint32_t* success_hex = reinterpret_cast<uint32_t*>(success.data());

        return (*response_hex == *success_hex); 
    }
    inline bool enable(bool enable) {
        std::array<uint8_t, 6> code = {_addr, 0xF3, 0xAB, static_cast<uint8_t>(enable), 0x00, 0x6B};
        std::array<uint8_t, 4> response;
        boost::asio::write(*_port, boost::asio::buffer(code, code.size()));
        boost::asio::read(*_port, boost::asio::buffer(response, response.size()));
        std::array<uint8_t, 4> success = {_addr, 0xF3, 0x02, 0x6B};
        uint32_t* response_hex = reinterpret_cast<uint32_t*>(response.data());
        uint32_t* success_hex = reinterpret_cast<uint32_t*>(success.data());

        return (*response_hex == *success_hex);
    }
    inline void vel_control(scalar_t vel, scalar_t acc, bool wait_sync=false) {
        uint16_t rpm_hex = static_cast<uint16_t>(std::abs(vel) / M_PI / 2 * 60 * _reducer_ratio);
        uint8_t acc_hex = static_cast<uint8_t>(std::abs(acc) * 0xFF);
        if (acc_hex == 0) acc_hex = 0x01;
        bool is_position_negative = (vel < 0);
        const uint8_t* p_rpm = reinterpret_cast<const uint8_t*>(&rpm_hex);
        std::array<uint8_t, 8> code = {_addr, 0xF6,
                                        static_cast<uint8_t>((is_position_negative == _is_clockwise_positive)), 
                                        p_rpm[1], p_rpm[0],
                                        acc_hex,
                                        static_cast<uint8_t>(wait_sync), 
                                        0x6B};
        boost::asio::write(*_port, boost::asio::buffer(code, code.size()));
    }
    inline void pos_control(scalar_t pos, scalar_t vel, scalar_t acc, bool wait_sync=false) {
        uint16_t rpm_hex = static_cast<uint16_t>(std::abs(vel) / M_PI / 2. * 60. * _reducer_ratio);
        uint8_t acc_hex = static_cast<uint8_t>(std::abs(acc) * 0xFF);
        if (acc_hex == 0) acc_hex = 0x01;
        const uint8_t* p_rpm = reinterpret_cast<const uint8_t*>(&rpm_hex);
        const scalar_t step_size = 1.8 / _microsteps / 180 * M_PI;
        const uint32_t impulse_number = std::abs(pos) / step_size * _reducer_ratio;
        const uint8_t* p_pos = reinterpret_cast<const uint8_t*>(&impulse_number);
        bool is_position_negative = (pos < 0);
        std::array<uint8_t, 13> code = {_addr, 0xFD,
                                        static_cast<uint8_t>(is_position_negative == _is_clockwise_positive), 
                                        p_rpm[1], p_rpm[0],
                                        acc_hex,
                                        p_pos[3], p_pos[2], p_pos[1], p_pos[0],
                                        0x01,
                                        static_cast<uint8_t>(wait_sync), 
                                        0x6B};
        boost::asio::write(*_port, boost::asio::buffer(code, code.size()));
    }
};

class SerialManipulatorDriver {
protected:
    boost::asio::io_service _io;
    std::vector<std::unique_ptr<boost::asio::serial_port>> _port_objects;
    std::array<boost::asio::serial_port*, DOF> _ports;
    std::array<std::unique_ptr<JointDriver>, DOF> _joints;
    std::array<uint8_t, DOF> _microsteps {8,8,8,8,8,8};
    std::array<uint8_t, DOF> _reducer_ratios {1,1,1,1,1,1};
    std::array<bool, DOF> _reverse_directions {true,true,true,true,true,true};

    std::array<scalar_t, DOF> _joint_positions {};
    std::array<scalar_t, DOF> _joint_velocities {};
    std::array<scalar_t, DOF> _joint_accelerations {};
    std::array<scalar_t, DOF> _target_joint_positions {};
    std::array<scalar_t, DOF> _target_joint_velocities {};
    std::array<scalar_t, DOF> _target_joint_accelerations {};

    void _construct(const std::vector<std::string>& port_names,
                    const std::array<uint8_t, DOF>& microsteps,
                    const std::array<uint8_t, DOF>& reducer_ratios,
                    const std::array<bool, DOF>& reverse_directions) {
        _microsteps = microsteps;
        _reducer_ratios = reducer_ratios;
        _reverse_directions = reverse_directions;
        
        // Create serial ports
        for (const auto& port_name : port_names) {
            _port_objects.push_back(
                std::make_unique<boost::asio::serial_port>(_io, port_name)
            );
        }

        // If only one port is provided, use it for all joints
        boost::asio::serial_port* default_port = _port_objects[0].get();
        _ports.fill(default_port);

        // Configure all ports
        for (auto& port : _port_objects) {
            port->set_option(boost::asio::serial_port_base::baud_rate(115200));
            port->set_option(boost::asio::serial_port_base::character_size(8));
            port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        }

        // Initialize joints
        for (dof_size_t i = 0; i < DOF; ++i) {
            _joints[i] = std::make_unique<JointDriver>(_ports[i], i+1, _microsteps[i], _reducer_ratios[i], _reverse_directions[i]);
        }
    }

    inline void _get_joint_positions() {
        for (dof_size_t i = 0; i < DOF; ++i) {
            _joint_positions[i] = _joints[i]->get_position();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    inline void _get_joint_velocities() {
        for (dof_size_t i = 0; i < DOF; ++i) {
            _joint_velocities[i] = _joints[i]->get_velocity();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    inline void _get_joint_accelerations() {
        for (dof_size_t i = 0; i < DOF; ++i) {
            _joint_accelerations[i] = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
public:
    explicit SerialManipulatorDriver(const std::string& robot_description_path) {
        using json = nlohmann::json;
        // Open the JSON file
        std::ifstream file(robot_description_path);

        // Check if the file opened successfully
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open the file: " + robot_description_path);
        }

        // Parse the JSON file
        json data;
        try {
            file >> data;
        } catch (json::parse_error& e) {
            throw std::runtime_error("Parse error: " + std::string(e.what()));
        }

        // Extract driver configuration
        try {
            const auto& driver_config = data["spec"]["driver_config"];
            
            std::vector<std::string> port_names = driver_config["port_names"].get<std::vector<std::string>>();
            
            // Add validation
            if (port_names.empty()) {
                throw std::runtime_error("No port names provided in configuration");
            }
            
            std::array<uint8_t, DOF> microsteps = driver_config["microsteps"].get<std::array<uint8_t, DOF>>();
            std::array<uint8_t, DOF> reducer_ratios = driver_config["reducer_ratios"].get<std::array<uint8_t, DOF>>();
            std::array<bool, DOF> reverse_directions = driver_config["reverse_directions"].get<std::array<bool, DOF>>();

            _construct(port_names, microsteps, reducer_ratios, reverse_directions);
            
        } catch (json::exception& e) {
            throw std::runtime_error("Error parsing driver configuration: " + std::string(e.what()));
        }
        file.close();
    }
    void update() {
        position_control(_target_joint_positions, _target_joint_velocities, _target_joint_accelerations);
        // _get_joint_positions();
        // _get_joint_velocities();
        // _get_joint_accelerations();
    }
    ~SerialManipulatorDriver() {
        homing();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        for (auto& port : _port_objects) {
            if (port->is_open()) {
                try {
                    boost::system::error_code ec;
                    port->close(ec);
                    if (!ec) {
                        std::cout << "Port closed successfully" << std::endl;
                    }
                } catch (...) {
                    std::cerr << "Error closing port" << std::endl;
                }
            }
        }
    }

    inline void emergency_stop() {
        for (dof_size_t i = 0; i < DOF; ++i) {
            _joints[i]->emergency_stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        std::cout<< "Emergency stop triggered!" << std::endl;
    }
    inline void position_control(std::array<scalar_t, DOF> positions, std::array<scalar_t, DOF> velocities, std::array<scalar_t, DOF> accelerations) {
        for (dof_size_t i = 0; i < DOF; ++i) {
            _joints[i]->pos_control(positions[i], velocities[i], 1);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    inline void homing(std::array<scalar_t, DOF> velocities = {10,10,10,10,10,10}, 
                        std::array<scalar_t, DOF> acc = {0.1,0.1,0.1,0.1,0.1,0.1}) {
        for (int i = DOF-1; i >= 0; --i) {
            _joints[i]->pos_control(0, velocities[i], acc[i]);
            std::cout << "Homing joint " << i+1 << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
    inline void set_target_joint_positions(std::array<scalar_t, DOF> positions) noexcept {
        _target_joint_positions = positions;
    }
    inline void set_target_joint_velocities(std::array<scalar_t, DOF> velocities) noexcept {
        _target_joint_velocities = velocities;
    }
    inline void set_target_joint_accelerations(std::array<scalar_t, DOF> accelerations) noexcept {
        _target_joint_accelerations = accelerations;
    }
    inline const std::array<scalar_t, DOF>& get_joint_positions() noexcept { return _joint_positions; }
    inline const std::array<scalar_t, DOF>& get_joint_velocities() noexcept { return _joint_velocities; }
    inline const std::array<scalar_t, DOF>& get_joint_accelerations() noexcept { return _joint_accelerations; }
};

}