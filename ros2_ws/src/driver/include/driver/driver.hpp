#pragma once
#include <boost/asio.hpp>
#include <iomanip>
#include <iostream>
#include <array>
#include <csignal>

namespace driver {

template<typename dScalar, typename = std::enable_if_t<std::is_floating_point_v<dScalar>>>
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
    explicit JointDriver(boost::asio::serial_port* port, uint8_t addr, uint8_t microsteps, uint8_t reducer_ratio, bool CW_direction)
    : _port(port), _addr(addr), _microsteps(microsteps), _reducer_ratio(reducer_ratio), _is_clockwise_positive(CW_direction) {
        set_microsteps( _microsteps );
    }
    inline dScalar get_position() {
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
    inline dScalar get_velocity() {
        std::array<uint8_t, 3> code = {_addr, 0x35, 0x6B};
        std::array<uint8_t, 6> response;
        boost::asio::write(*_port, boost::asio::buffer(code, code.size()));
        boost::asio::read(*_port, boost::asio::buffer(response, response.size()));

        uint16_t vel_value = (static_cast<uint16_t>(response[3]) << 8)  |
                            (static_cast<uint16_t>(response[4]));
                            
        return vel_value / _reducer_ratio / 60. * 2. * M_PI * ((response[0] == 0x00) == _is_clockwise_positive? 1 : -1);
    }
    inline bool set_open_loop(bool open_loop) {
        std::array<uint8_t, 5> code = {_addr, 0x46, 0x69, 0x01, static_cast<uint8_t>(!open_loop)+0x01, 0x6B};
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
    inline void vel_control(dScalar vel, uint8_t acc, bool wait_sync=false) {
        uint16_t rpm_hex = static_cast<uint16_t>(std::abs(vel) / M_PI / 2 * 60 * _reducer_ratio);
        bool is_position_negative = (vel < 0);
        const uint8_t* p_rpm = reinterpret_cast<const uint8_t*>(&rpm_hex);
        std::array<uint8_t, 8> code = {_addr, 0xF6,
                                        static_cast<uint8_t>((is_position_negative == _is_clockwise_positive)), 
                                        p_rpm[1], p_rpm[0],
                                        acc,
                                        static_cast<uint8_t>(wait_sync), 
                                        0x6B};
        boost::asio::write(*_port, boost::asio::buffer(code, code.size()));
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    inline void pos_control(dScalar pos, dScalar vel, uint8_t acc, bool wait_sync=false) {
        uint16_t rpm_hex = static_cast<uint16_t>(std::abs(vel) / M_PI / 2. * 60. * _reducer_ratio);
        const uint8_t* p_rpm = reinterpret_cast<const uint8_t*>(&rpm_hex);
        const dScalar step_size = 1.8 / _microsteps / 180 * M_PI;
        const uint32_t impulse_number = std::abs(pos) / step_size * _reducer_ratio;
        const uint8_t* p_pos = reinterpret_cast<const uint8_t*>(&impulse_number);
        bool is_position_negative = (pos < 0);
        std::array<uint8_t, 13> code = {_addr, 0xFD,
                                        static_cast<uint8_t>(is_position_negative == _is_clockwise_positive), 
                                        p_rpm[1], p_rpm[0],
                                        acc,
                                        p_pos[3], p_pos[2], p_pos[1], p_pos[0],
                                        0x01,
                                        static_cast<uint8_t>(wait_sync), 
                                        0x6B};
        boost::asio::write(*_port, boost::asio::buffer(code, code.size()));
        // std::cout << "sent: " << hex_to_str(code) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
};

template<typename dScalar, size_t dof, typename = std::enable_if_t<std::is_floating_point_v<dScalar>>>
class RobotDriver {
protected:
    std::array<boost::asio::serial_port*, dof> _ports;
    std::array<std::unique_ptr<JointDriver<dScalar>>, dof> _joints;
public:
    explicit RobotDriver(std::array<boost::asio::serial_port*, dof> ports, std::array<uint8_t, dof> microsteps, std::array<uint8_t, dof> reducer_ratios, std::array<bool, dof> CW_directions)
    : _ports(ports), _joints() {
        for (auto port: _ports) {
            port->set_option(boost::asio::serial_port_base::baud_rate(115200));
            port->set_option(boost::asio::serial_port_base::character_size(8));
            port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        }
        for (size_t i = 0; i < dof; ++i) {
            _joints[i] = std::make_unique<JointDriver<dScalar>>(_ports[i], i+1, microsteps[i], reducer_ratios[i], CW_directions[i]);
        }
    }
    inline void emergency_stop() {
        for (size_t i = 0; i < dof; ++i) {
            _joints[i]->emergency_stop();
        }
        std::cout<< "Emergency stop triggered!" << std::endl;
    }
    template<typename Scalar>
    inline void position_control(std::array<Scalar, dof> positions, std::array<Scalar, dof> velocities, std::array<uint8_t, dof> acc) {
        for (size_t i = 0; i < dof; ++i) {
            _joints[i]->pos_control(positions[i], velocities[i], acc[i]);
        }
    }
    inline void homing(std::array<dScalar, dof> velocities = {10,10,10,10,10,10}, 
                        std::array<uint8_t, dof> acc = {1,1,1,1,1,1}) {
        for (int i = dof-1; i >= 0; --i) {
            _joints[i]->pos_control(0, velocities[i], acc[i]);
            std::cout << "Homing joint " << i+1 << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
    inline std::array<dScalar, dof> get_joint_positions() {
        std::array<dScalar, dof> joint_positions;
        for (size_t i = 0; i < dof; ++i) {
            joint_positions[i] = _joints[i]->get_position();
        }
        return joint_positions; 
    }
    inline std::array<dScalar, dof> get_joint_velocities() {
        std::array<dScalar, dof> joint_velocities;
        for (size_t i = 0; i < dof; ++i) {
            joint_velocities[i] = _joints[i]->get_velocity();
        }
        return joint_velocities;
    }
};

boost::asio::io_service io;
boost::asio::serial_port port0(io, "/dev/ttyUSB0");
// boost::asio::serial_port port1(io, "/dev/ttyUSB1");
// boost::asio::serial_port port2(io, "/dev/ttyUSB2");
std::array<boost::asio::serial_port*, 6> ports = {&port0, &port0, &port0, &port0, &port0, &port0};

const std::array<uint8_t, 6> microsteps = {8, 8, 8, 8, 8, 8};
const std::array<uint8_t, 6> reducer_ratios = {50, 27, 36, 27, 36, 51};
const std::array<bool, 6> is_clockwise_positive = {true, false, false, true, true, false};

RobotDriver<double, 6> robotdriver0(ports, microsteps, reducer_ratios, is_clockwise_positive);

void closePorts() {
    // Cleanup code here
    if (port0.is_open()) {
        try {
            boost::system::error_code ec;
            port0.close(ec);
            if (!ec) {
                std::cout << "Port /dev/ttyUSB0 closed successfully" << std::endl;
            }
        } catch (...) {
            std::cerr << "Error closing port" << std::endl;
        }
    }
    // if (port1.is_open()) {
    //     try {
    //         boost::system::error_code ec;
    //         port0.close(ec);
    //         if (!ec) {
    //             std::cout << "Port /dev/ttyUSB1 closed successfully" << std::endl;
    //         }
    //     } catch (...) {
    //         std::cerr << "Error closing port" << std::endl;
    //     }
    // }
    // if (port2.is_open()) {
    //     try {
    //         boost::system::error_code ec;
    //         port0.close(ec);
    //         if (!ec) {
    //             std::cout << "Port /dev/ttyUSB2 closed successfully" << std::endl;
    //         }
    //     } catch (...) {
    //         std::cerr << "Error closing port" << std::endl;
    //     }
    // }
}

}