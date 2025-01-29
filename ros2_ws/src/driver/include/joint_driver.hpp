#pragma once
#include "helper.hpp"
// C++ Standard Library
#include <array>
#include <filesystem>
#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>
// Third-party Libraries
#include <yaml-cpp/yaml.h>

// Linux System Headers
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

// CAN Bus Headers
#include <linux/can.h>
#include <linux/can/bcm.h>
#include <linux/can/error.h>
#include <linux/can/j1939.h>
#include <linux/can/netlink.h>
#include <linux/can/raw.h>

namespace timr {

namespace driver {

class JointDriver {
public:
    struct Config;

    ~JointDriver()=default;

    explicit JointDriver(int can_socket, const std::string& yaml_path)
    : _can_socket(can_socket) {
        _config = config_from_yaml(yaml_path);
        send_config_to_driver(_config);
        enable(false);
    }

    explicit JointDriver(int can_socket, Config config)
    : _can_socket(can_socket), _config(config) {
        send_config_to_driver(_config);
        enable(false);
    }

    enum class MotorStepAngle : uint8_t {
        MOTOR_0_9_DEG = 0x32,  // 0.9° motor
        MOTOR_1_8_DEG = 0x19   // 1.8° motor
    };

    enum class PulseMode : uint8_t {
        PUL_OFF = 0x00,    // Disable pulse input port, pulse control invalid
        PUL_OPEN = 0x01,   // Open-loop mode, no encoder needed, max speed 200-400 RPM
        PUL_FOC = 0x02,    // FOC vector closed-loop mode, default mode, real-time current control, max speed ~3000+ RPM
        ESI_RCO = 0x03     // Multiplexed as limit input and position output
    };

    enum class CommMode : uint8_t {
        RXTX_OFF = 0x00,   // Disable communication ports, communication control invalid
        ESI_ALO = 0x01,    // Multiplex R/A/H as limit switch inputs, T/B/L as alarm outputs
        UART_FUN = 0x02,   // Multiplex R/A/H and T/B/L as UART/RS232/RS485 communication ports
        CAN1_MAP = 0x03    // Multiplex R/A/H and T/B/L as CAN communication ports
    };

    enum class EnPinMode : uint8_t {
        LOW_ACTIVE = 0x00,  // Enable when EN pin is low, disable when high
        HIGH_ACTIVE = 0x01, // Enable when EN pin is high, disable when low  
        HOLD = 0x02        // Always enabled regardless of EN pin state
    };

    enum class MotorPlusDir : uint8_t {
        CW = 0x00,   // Clockwise - positive direction when Dir pin is active
        CCW = 0x01   // Counter-clockwise - positive direction when Dir pin is active
    };

    enum class EnabledState : uint8_t {
        DISABLED = 0x00,
        ENABLED = 0x01
    };

    enum class BaudRate : uint8_t {
        BAUD_9600 = 0x00,
        BAUD_19200 = 0x01,
        BAUD_25000 = 0x02,
        BAUD_38400 = 0x03,
        BAUD_57600 = 0x04,
        BAUD_115200 = 0x05,
        BAUD_256000 = 0x06,
        BAUD_512000 = 0x07,
        BAUD_921600 = 0x08
    };

    enum class CanBaudRate : uint8_t {
        CAN_10K = 0x00,   // 10000 baud
        CAN_20K = 0x01,   // 20000 baud
        CAN_50K = 0x02,   // 50000 baud
        CAN_83_3K = 0x03, // 83333 baud
        CAN_100K = 0x04,  // 100000 baud
        CAN_125K = 0x05,  // 125000 baud
        CAN_250K = 0x06,  // 250000 baud
        CAN_500K = 0x07,  // 500000 baud (default)
        CAN_800K = 0x08,  // 800000 baud
        CAN_1M = 0x09     // 1000000 baud
    };

    enum class ChecksumType : uint8_t {
        FIXED_6B = 0x00,  // Default checksum 0x6B
        XOR = 0x01,       // XOR checksum
        CRC8 = 0x02,     // CRC-8 checksum
        MODBUS = 0x03     // Modbus RTU checksum
    };

    enum class ResponseType : uint8_t {
        NONE = 0x00,    // No response
        RECEIVE = 0x01,  // Only respond with command received confirmation
        REACHED = 0x02,  // Only respond with position reached confirmation
        BOTH = 0x03,    // Respond with both received and reached confirmations
        OTHER = 0x04     // Position mode: reached only, other commands: received only
    };

    struct Config {
        std::string api_version;
        std::string name;
        std::string kind;
        uint8_t addr;                                     // Motor CAN address
        uint8_t reducer_ratio;                           // Gear reduction ratio
        MotorStepAngle motor_step_angle;                  // Motor step angle
        PulseMode pulse_mode;                             // Control mode
        CommMode comm_mode;                               // Communication interface mode
        EnPinMode en_pin_mode;                           // Enable pin configuration
        MotorPlusDir motor_plus_dir;                         // Direction pin configuration
        uint8_t microstep;                               // Microstepping setting (1-256)
        EnabledState interpolation;                       // Trajectory smoothing
        EnabledState auto_screen_off;                     // Display power saving
        uint16_t open_loop_current_ma;                   // Open loop current limit
        uint16_t closed_loop_stall_current_ma;           // Closed loop stall current
        uint16_t closed_loop_max_voltage_mv;             // Maximum voltage in closed loop
        BaudRate uart_baudrate;                          // UART communication speed
        CanBaudRate can_baudrate;                        // CAN bus speed
        ResponseType response_type;                       // Command response behavior
        ChecksumType checksum_type;                      // Communication checksum method
        EnabledState stall_protection;                    // Motor stall detection
        uint16_t stall_threshold_rpm;                    // Speed threshold for stall
        uint16_t stall_current_threshold_ma;             // Current threshold for stall
        uint16_t stall_detection_ms;                     // Stall detection time
        double position_tolerance_deg;                    // Position control tolerance
        scalar_t position_upper_bound;
        scalar_t position_lower_bound;
        scalar_t velocity_upper_bound;
        scalar_t velocity_lower_bound;
        scalar_t acceleration_upper_bound;
        scalar_t acceleration_lower_bound;
    };

    static inline Config config_from_yaml(const std::string& yaml_path) {
        // Validate file exists
        if (!std::filesystem::exists(yaml_path)) {
            throw std::runtime_error("JointDriver: Motor description file not found: " + yaml_path);
        }

        // Load and parse YAML
        YAML::Node yaml;
        try {
            yaml = YAML::LoadFile(yaml_path);
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("JointDriver: Failed to parse motor description YAML: " + std::string(e.what()));
        }

        Config config;

        config.api_version = yaml["apiVersion"].as<std::string>();
        config.name = yaml["metadata"]["name"].as<std::string>();
        config.kind = yaml["kind"].as<std::string>();

        // Validate required fields
        if (!yaml["spec"]) {
            throw std::runtime_error("JointDriver: Missing required 'spec' field in motor description");
        }
        auto spec = yaml["spec"];
        
        // Basic motor configuration
        config.addr = spec["addr"].as<uint8_t>();
        if (spec["motorStepAngle"].as<double>() == 1.8) {
            config.motor_step_angle = MotorStepAngle::MOTOR_1_8_DEG;
        } else if (spec["motorStepAngle"].as<double>() == 0.9) {
            config.motor_step_angle = MotorStepAngle::MOTOR_0_9_DEG;
        } else {
            throw std::runtime_error("JointDriver: Invalid motor step angle: " + spec["motorStepAngle"].as<std::string>() + ", valid options are: 0.9, 1.8");
        }
        config.microstep = spec["microstep"].as<uint8_t>();
        config.reducer_ratio = spec["reducerRatio"].as<uint8_t>();
        if (spec["jointPositiveDirection"].as<std::string>() == "ccw") {
            config.motor_plus_dir = MotorPlusDir::CCW;
        } else if (spec["jointPositiveDirection"].as<std::string>() == "cw") {
            config.motor_plus_dir = MotorPlusDir::CW;
        } else {
            throw std::runtime_error("JointDriver: Invalid positive direction: " + spec["jointPositiveDirection"].as<std::string>() + ", valid options are: ccw, cw");
        }
        if (spec["reducerChangeDirection"].as<bool>()) {
            config.motor_plus_dir = (config.motor_plus_dir == MotorPlusDir::CCW) ? MotorPlusDir::CW : MotorPlusDir::CCW;
        }
        std::string plus_mode = spec["pulseMode"].as<std::string>();
        if (plus_mode == "foc") {
            config.pulse_mode = PulseMode::PUL_FOC;
        } else if (plus_mode == "open") {
            config.pulse_mode = PulseMode::PUL_OPEN;
        } else if (plus_mode == "off") {
            config.pulse_mode = PulseMode::PUL_OFF;
        } else if (plus_mode == "esi") {
            config.pulse_mode = PulseMode::ESI_RCO;
        } else {
            throw std::runtime_error("JointDriver: Invalid pulse mode: " + plus_mode + ", valid options are: foc, open, off, esi");
        }

        std::string comm_mode = spec["communicationMode"].as<std::string>();
        if (comm_mode == "can") {
            config.comm_mode = CommMode::CAN1_MAP;
        } else if (comm_mode == "uart") {
            config.comm_mode = CommMode::UART_FUN;
        } else if (comm_mode == "off") {
            config.comm_mode = CommMode::RXTX_OFF;
        } else if (comm_mode == "esi") {
            config.comm_mode = CommMode::ESI_ALO;
        } else {
            throw std::runtime_error("JointDriver: Invalid communication mode: " + comm_mode + ", valid options are: can, uart, off, esi");
        }

        std::string en_pin_mode = spec["enPinMode"].as<std::string>();
        if (en_pin_mode == "hold") {
            config.en_pin_mode = EnPinMode::HOLD;
        } else if (en_pin_mode == "high") {
            config.en_pin_mode = EnPinMode::HIGH_ACTIVE;
        } else if (en_pin_mode == "low") {
            config.en_pin_mode = EnPinMode::LOW_ACTIVE;
        } else {
            throw std::runtime_error("JointDriver: Invalid en_pin_mode: " + en_pin_mode + ", valid options are: hold, high, low");
        }
        config.interpolation = spec["interpolationEnabled"].as<bool>() ? EnabledState::ENABLED : EnabledState::DISABLED;
        config.auto_screen_off = spec["autoScreenOff"].as<bool>() ? EnabledState::ENABLED : EnabledState::DISABLED;
        
        // Current and voltage limits
        config.open_loop_current_ma = spec["openLoopCurrentMa"].as<uint16_t>();
        config.closed_loop_stall_current_ma = spec["closedLoopStallCurrentMa"].as<uint16_t>();
        config.closed_loop_max_voltage_mv = spec["closedLoopMaxVoltageMv"].as<uint16_t>();
        
        // Communication settings
        uint32_t uart_baud = spec["uartBaudrate"].as<uint32_t>();
        switch (uart_baud) {
            case 9600: config.uart_baudrate = BaudRate::BAUD_9600; break;
            case 19200: config.uart_baudrate = BaudRate::BAUD_19200; break;
            case 25000: config.uart_baudrate = BaudRate::BAUD_25000; break;
            case 38400: config.uart_baudrate = BaudRate::BAUD_38400; break;
            case 57600: config.uart_baudrate = BaudRate::BAUD_57600; break;
            case 115200: config.uart_baudrate = BaudRate::BAUD_115200; break;
            case 256000: config.uart_baudrate = BaudRate::BAUD_256000; break;
            case 512000: config.uart_baudrate = BaudRate::BAUD_512000; break;
            case 921600: config.uart_baudrate = BaudRate::BAUD_921600; break;
            default: throw std::runtime_error("JointDriver: Invalid UART baud rate: " + std::to_string(uart_baud));
        }

        uint32_t can_baud = spec["canBaudrate"].as<uint32_t>();
        switch (can_baud) {
            case 10000: config.can_baudrate = CanBaudRate::CAN_10K; break;
            case 20000: config.can_baudrate = CanBaudRate::CAN_20K; break;
            case 50000: config.can_baudrate = CanBaudRate::CAN_50K; break;
            case 83333: config.can_baudrate = CanBaudRate::CAN_83_3K; break;
            case 100000: config.can_baudrate = CanBaudRate::CAN_100K; break;
            case 125000: config.can_baudrate = CanBaudRate::CAN_125K; break;
            case 250000: config.can_baudrate = CanBaudRate::CAN_250K; break;
            case 500000: config.can_baudrate = CanBaudRate::CAN_500K; break;
            case 800000: config.can_baudrate = CanBaudRate::CAN_800K; break;
            case 1000000: config.can_baudrate = CanBaudRate::CAN_1M; break;
            default: throw std::runtime_error("JointDriver: Invalid CAN baud rate: " + std::to_string(can_baud));
        }

        std::string checksum_type = spec["checksumType"].as<std::string>();
        if (checksum_type == "fixed_6b") {
            config.checksum_type = ChecksumType::FIXED_6B;
        } else if (checksum_type == "xor") {
            config.checksum_type = ChecksumType::XOR;
        } else if (checksum_type == "crc8") {
            config.checksum_type = ChecksumType::CRC8;
        } else if (checksum_type == "modbus") {
            config.checksum_type = ChecksumType::MODBUS;
        } else {
            throw std::runtime_error("JointDriver: Invalid checksum_type: " + checksum_type + ", valid options are: fixed_6b, xor, crc_8, modbus");
        }

        std::string response_type = spec["responseType"].as<std::string>();
        if (response_type == "none") {
            config.response_type = ResponseType::NONE;
        } else if (response_type == "receive") {
            config.response_type = ResponseType::RECEIVE;
        } else if (response_type == "reached") {
            config.response_type = ResponseType::REACHED;
        } else if (response_type == "both") {
            config.response_type = ResponseType::BOTH;
        } else if (response_type == "other") {
            config.response_type = ResponseType::OTHER;
        } else {
            throw std::runtime_error("JointDriver: Invalid response_type: " + response_type + ", valid options are: none, receive, reached, both, other");
        }
        // Protection settings
        config.stall_protection = spec["stallProtectionEnabled"].as<bool>() ? EnabledState::ENABLED : EnabledState::DISABLED;
        config.stall_threshold_rpm = spec["stallThresholdRpm"].as<uint16_t>();
        config.stall_current_threshold_ma = spec["stallCurrentThresholdMa"].as<uint16_t>();
        config.stall_detection_ms = spec["stallDetectionMs"].as<uint16_t>();
        config.position_tolerance_deg = spec["positionToleranceDeg"].as<double>();
        return config;
    }

    inline Config get_config() const noexcept {
        return _config;
    }

    inline void send_config_to_driver(const Config& config, bool store_flag=true) {
        std::array<uint8_t, 33> code = {
            config.addr,                                  // Address
            0x48,                                        // Command code
            0xD1,                                        // Subcommand
            static_cast<uint8_t>(store_flag),            // Store flag
            
            // Basic motor configuration
            static_cast<uint8_t>(config.motor_step_angle),
            static_cast<uint8_t>(config.pulse_mode),
            static_cast<uint8_t>(config.comm_mode),
            static_cast<uint8_t>(config.en_pin_mode),
            static_cast<uint8_t>(MotorPlusDir::CCW),   // Fixed ccw basic + direction
            config.microstep,
            
            // Feature settings
            static_cast<uint8_t>(config.interpolation),
            static_cast<uint8_t>(config.auto_screen_off),
            
            // Current and voltage limits (16-bit, little endian)
            static_cast<uint8_t>(config.open_loop_current_ma >> 8),
            static_cast<uint8_t>(config.open_loop_current_ma & 0xFF),
            static_cast<uint8_t>(config.closed_loop_stall_current_ma >> 8),
            static_cast<uint8_t>(config.closed_loop_stall_current_ma & 0xFF),
            static_cast<uint8_t>(config.closed_loop_max_voltage_mv >> 8),
            static_cast<uint8_t>(config.closed_loop_max_voltage_mv & 0xFF),
            
            // Communication settings
            static_cast<uint8_t>(config.uart_baudrate),
            static_cast<uint8_t>(config.can_baudrate),
            0x01,                                        // Fixed ID value
            static_cast<uint8_t>(config.checksum_type),
            static_cast<uint8_t>(config.response_type),
            
            // Protection settings
            static_cast<uint8_t>(config.stall_protection),
            static_cast<uint8_t>(config.stall_threshold_rpm >> 8),
            static_cast<uint8_t>(config.stall_threshold_rpm & 0xFF),
            static_cast<uint8_t>(config.stall_current_threshold_ma >> 8),
            static_cast<uint8_t>(config.stall_current_threshold_ma & 0xFF),
            static_cast<uint8_t>(config.stall_detection_ms >> 8),
            static_cast<uint8_t>(config.stall_detection_ms & 0xFF),
            
            // Position control (little endian)
            static_cast<uint8_t>(static_cast<uint16_t>(config.position_tolerance_deg * 10) >> 8),
            static_cast<uint8_t>(static_cast<uint16_t>(config.position_tolerance_deg * 10) & 0xFF),
            
            0x00                                         // Checksum placeholder
        };

        _command_transaction(code);
    }

    inline void set_open_loop_current(uint16_t current_ma, bool store_flag=true) {
        // Command format: [addr][0x44][0x33][store_flag][current_low][current_high][checksum]
        std::array<uint8_t, 7> code = {
            _config.addr,                          // Address
            0x44,                          // Command code
            0x33,                          // Subcommand
            static_cast<uint8_t>(store_flag),// Store flag (1=store, 0=don't store)
            static_cast<uint8_t>(current_ma >> 8),   // Current high byte
            static_cast<uint8_t>(current_ma & 0xFF), // Current low byte
            0x00                           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _command_transaction(code);
    }

    inline void set_control_mode(bool open_loop, bool store_flag=true) {
        // Command format: [addr][0x46][0x69][store_flag][mode][checksum]
        std::array<uint8_t, 6> code = {
            _config.addr,                          // Address
            0x46,                          // Command code
            0x69,                          // Subcommand
            static_cast<uint8_t>(store_flag),// Store flag (1=store, 0=don't store)
            static_cast<uint8_t>(open_loop ? 0x01 : 0x02), // 0x01=open loop, 0x02=closed loop
            0x00                           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _command_transaction(code);
    }

    inline void set_motor_address(uint8_t new_addr, bool store_flag=true) {
        // Command format: [addr][0xAE][0x4B][store_flag][new_id][checksum]
        std::array<uint8_t, 6> code = {
            _config.addr,                          // Address
            0xAE,                          // Command code
            0x4B,                          // Subcommand
            static_cast<uint8_t>(store_flag),// Store flag (1=store, 0=don't store)
            new_addr,                        // New ID address
            0x00                           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _command_transaction(code);
        _config.addr = new_addr;
    }

    inline void set_microstep(uint8_t microstep, bool store_flag=true) {
        // Command format: [addr][0x84][0x8A][store_flag][microstep][checksum]
        std::array<uint8_t, 6> code = {
            _config.addr,                          // Address
            0x84,                          // Command code
            0x8A,                          // Subcommand
            static_cast<uint8_t>(store_flag),// Store flag (1=store, 0=don't store)
            microstep,                    // Microstep value (0=256, 1-255 as is)
            0x00                           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _command_transaction(code);
        _config.microstep = microstep;
    }

    struct MotorStatus {
        bool is_enabled;              // Motor enable status
        bool is_position_reached;     // Position reached flag
        bool is_stalled;             // Motor stall flag
        bool is_stall_protected;      // Stall protection triggered flag
        
        uint8_t raw_status;          // Raw status byte for additional checking if needed
    };

    inline MotorStatus read_motor_status() {
        // Command format: [addr][0x3A][checksum]
        std::array<uint8_t, 3> code = {
            _config.addr,          // Address
            0x3A,          // Command code
            0x00           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        code.back() = crc8_checksum(code.data(), code.size() - 1);
        auto frames = serial_data_to_can_frames(code);
        _send_can_frames(frames);
        auto frames_response = _read_can_frames<1>();
        auto response = can_frames_to_serial_data<4>(frames_response);

        // Extract status byte
        uint8_t status = response[2];

        return MotorStatus{
            static_cast<bool>(status & 0x01),  // is_enabled
            static_cast<bool>(status & 0x02),  // is_position_reached
            static_cast<bool>(status & 0x04),  // is_stalled
            static_cast<bool>(status & 0x08),  // is_stall_protected
            status                             // raw_status
        };
    }

    inline scalar_t read_realtime_velocity() {
        // Command format: [addr][0x35][checksum]
        std::array<uint8_t, 3> code = {
            _config.addr,          // Address
            0x35,          // Command code
            0x00           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        code.back() = crc8_checksum(code.data(), code.size() - 1);
        auto frames = serial_data_to_can_frames(code);
        _send_can_frames(frames);
        auto frames_response = _read_can_frames<1>();
        auto response = can_frames_to_serial_data<6>(frames_response);

        // Extract sign and velocity from response
        bool is_negative = (_config.motor_plus_dir == MotorPlusDir::CCW) ? 
            (response[2] == 0x01) : (response[2] == 0x00);
        
        // Combine 2 bytes into velocity value (RPM) - little endian
        uint16_t rpm = static_cast<uint16_t>(response[3]) |
                      (static_cast<uint16_t>(response[4]) << 8);

        // Convert RPM to rad/s
        scalar_t velocity = static_cast<scalar_t>(rpm) * 2.0 * M_PI / 60.0 / _config.reducer_ratio;
        
        return is_negative ? -velocity : velocity;
    }

    inline scalar_t read_realtime_position() {
        // Command format: [addr][0x36][checksum]
        std::array<uint8_t, 3> code = {
            _config.addr,          // Address
            0x36,          // Command code
            0x00           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        code.back() = crc8_checksum(code.data(), code.size() - 1);
        auto frames = serial_data_to_can_frames(code);
        _send_can_frames(frames);
        auto frames_response = _read_can_frames<1>();
        auto response = can_frames_to_serial_data<6>(frames_response);

        // Extract sign and position from response
        bool is_negative = (_config.motor_plus_dir == MotorPlusDir::CCW) ? 
            (response[2] == 0x01) : (response[2] == 0x00);
        
        // Combine 4 bytes into position value (little endian)
        uint32_t position = static_cast<uint32_t>(response[6]) |
                           (static_cast<uint32_t>(response[5]) << 8) |
                           (static_cast<uint32_t>(response[4]) << 16) |
                           (static_cast<uint32_t>(response[3]) << 24);

        // Convert to radians (2π per rotation)
        scalar_t angle = static_cast<scalar_t>(position) * 2.0 * M_PI / 65536.0 / _config.reducer_ratio;
        
        return is_negative ? -angle : angle;
    }

    inline scalar_t read_position_error() {
        // Command format: [addr][0x37][checksum]
        std::array<uint8_t, 3> code = {
            _config.addr,          // Address
            0x37,          // Command code
            0x00           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        code.back() = crc8_checksum(code.data(), code.size() - 1);
        auto frames = serial_data_to_can_frames(code);
        _send_can_frames(frames);
        auto frames_response = _read_can_frames<1>();
        auto response = can_frames_to_serial_data<8>(frames_response);

        // Extract sign and error from response
        bool is_negative = (_config.motor_plus_dir == MotorPlusDir::CCW) ? 
            (response[2] == 0x01) : (response[2] == 0x00);
        
        // Combine 4 bytes into error value (little endian)
        uint32_t error = static_cast<uint32_t>(response[6]) |
                        (static_cast<uint32_t>(response[5]) << 8) |
                        (static_cast<uint32_t>(response[4]) << 16) |
                        (static_cast<uint32_t>(response[3]) << 24);

        // Convert to radians (2π per rotation)
        scalar_t angle_error = static_cast<scalar_t>(error) * 2.0 * M_PI / 65536.0 / _config.reducer_ratio;
        
        return is_negative ? -angle_error : angle_error;
    }

    inline scalar_t read_realtime_target_position() {
        // Command format: [addr][0x34][checksum]
        std::array<uint8_t, 3> code = {
            _config.addr,          // Address
            0x34,          // Command code
            0x00           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        code.back() = crc8_checksum(code.data(), code.size() - 1);
        auto frames = serial_data_to_can_frames(code);
        _send_can_frames(frames);
        auto frames_response = _read_can_frames<1>();
        auto response = can_frames_to_serial_data<8>(frames_response);

        // Extract sign and position from response
        bool is_negative = (_config.motor_plus_dir == MotorPlusDir::CCW) ? 
            (response[2] == 0x01) : (response[2] == 0x00);
        
        // Combine 4 bytes into position value (little endian)
        uint32_t position = static_cast<uint32_t>(response[6]) |
                           (static_cast<uint32_t>(response[5]) << 8) |
                           (static_cast<uint32_t>(response[4]) << 16) |
                           (static_cast<uint32_t>(response[3]) << 24);

        // Convert to radians (2π per rotation)
        scalar_t angle = static_cast<scalar_t>(position) * 2.0 * M_PI / 65536.0 / _config.reducer_ratio;
        
        return is_negative ? -angle : angle;
    }

    inline scalar_t read_target_position() {
        // Command format: [addr][0x33][checksum]
        std::array<uint8_t, 3> code = {
            _config.addr,          // Address
            0x33,          // Command code
            0x00           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        code.back() = crc8_checksum(code.data(), code.size() - 1);
        auto frames = serial_data_to_can_frames(code);
        _send_can_frames(frames);
        auto frames_response = _read_can_frames<1>();
        auto response = can_frames_to_serial_data<8>(frames_response);

        // Extract sign and position from response, reversing if direction is CW
        bool is_negative = (_config.motor_plus_dir == MotorPlusDir::CCW) ? 
            (response[2] == 0x01) : (response[2] == 0x00);
        // Combine 4 bytes into position value (little endian)
        uint32_t position = static_cast<uint32_t>(response[3]) |
                           (static_cast<uint32_t>(response[4]) << 8) |
                           (static_cast<uint32_t>(response[5]) << 16) |
                           (static_cast<uint32_t>(response[6]) << 24);

        // Convert to radians (2π per rotation)
        scalar_t angle = static_cast<scalar_t>(position) * 2.0 * M_PI / 65536.0 / _config.reducer_ratio;
        
        return is_negative ? -angle : angle;
    }

    inline void clear_stall_protection() {
        // Command format: [addr][0x0E][0x52][checksum]
        std::array<uint8_t, 4> code = {
            _config.addr,          // Address
            0x0E,          // Command code
            0x52,          // Clear stall protection command
            0x00           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _command_transaction(code);
    }

    inline void calibration() {
        // Command format: [addr][0x06][0x45][checksum]
        std::array<uint8_t, 4> code = {
            _config.addr,          // Address
            0x06,          // Command code
            0x45,          // Calibrate encoder command
            0x00           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _command_transaction(code);
    }

    inline void clear_position() {
        // Command format: [addr][0x0A][0x6D][checksum]
        std::array<uint8_t, 4> code = {
            _config.addr,          // Address
            0x0A,          // Command code
            0x6D,          // Clear position command
            0x00           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _command_transaction(code);
    }

    inline void interrupt_homing() {
        // Command format: [addr][0x9C][0x48][checksum]
        std::array<uint8_t, 4> code = {
            _config.addr,          // Address
            0x9C,          // Command code
            0x48,          // Interrupt homing command
            0x00           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _command_transaction(code);
    }

    enum class HomingMode : uint8_t {
        SINGLE_TURN_NEAREST = 0x00,    // Single-turn nearest direction
        SINGLE_TURN_DIRECTED = 0x01,   // Single-turn specified direction
        MULTI_TURN_COLLISION = 0x02,   // Multi-turn unlimited collision
        MULTI_TURN_LIMIT_SWITCH = 0x03 // Multi-turn with limit switch
    };

    inline void driver_homing(HomingMode mode = HomingMode::SINGLE_TURN_NEAREST, bool sync_flag = false) {
        // Command format: [addr][0x9A][mode][sync_flag][checksum]
        std::array<uint8_t, 5> code = {
            _config.addr,                          // Address
            0x9A,                          // Command code
            static_cast<uint8_t>(mode),     // Homing mode
            static_cast<uint8_t>(sync_flag),// Sync flag
            0x00                           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _command_transaction(code);
    }

    inline void set_zero_position(bool store_flag=true) {
        // Command format: [addr][0x93][0x88][store_flag][checksum]
        std::array<uint8_t, 5> code = {
            _config.addr,                          // Address
            0x93,                          // Command code
            0x88,                          // Set zero position command
            static_cast<uint8_t>(store_flag),// Store flag (1=store, 0=don't store)
            0x00                           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _command_transaction(code);
    }

    inline void sync_move() {
        // Command format: [addr][0xFF][0x66][checksum]
        std::array<uint8_t, 4> code = {
            _config.addr,          // Address
            0xFF,          // Command code
            0x66,          // Sync move command
            0x00           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _command_transaction(code);
    }
    
    inline void emergency_stop(bool sync_flag=false) {
        // Command format: [addr][0xFE][0x98][sync_flag][checksum]
        std::array<uint8_t, 5> code = {
            _config.addr,                          // Address
            0xFE,                          // Command code
            0x98,                          // Stop command
            static_cast<uint8_t>(sync_flag),// Sync flag
            0x00                           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _command_transaction(code);
    }

    inline void enable(bool enable, bool sync_flag=false) {
        // Command format: [addr][0xF3][0xAB][enable_state][sync_flag][checksum]
        std::array<uint8_t, 6> code = {
            _config.addr, 
            0xF3, 
            0xAB, 
            static_cast<uint8_t>(enable), 
            static_cast<uint8_t>(sync_flag), 
            0x00
        };

        _command_transaction(code);
    }
    
    inline void velocity_control(scalar_t vel, scalar_t acc, bool sync_flag=false) {
        // Clamp velocity and acceleration to configured bounds
        vel = std::clamp(vel, _config.velocity_lower_bound, _config.velocity_upper_bound); 
        acc = std::clamp(acc, _config.acceleration_lower_bound, _config.acceleration_upper_bound);

        // Convert velocity from rad/s to RPM, clamped to uint16 range
        const scalar_t rpm = std::abs(vel) * 60.0 / (2.0 * M_PI) * _config.reducer_ratio;
        const uint16_t rpm_hex = static_cast<uint16_t>(std::min(rpm, static_cast<scalar_t>(UINT16_MAX)));
        
        // Convert acceleration to control level (0-255), ensuring full range utilization
        const scalar_t normalized_acc = std::min(std::abs(acc), 1.0); 
        const uint8_t acc_hex = static_cast<uint8_t>(normalized_acc * 255);
        // Command format: [addr][0xF6][direction][speed_h][speed_l][acc][sync][checksum]

        // Extract sign for direction, defaulting to configured direction for zero
        const bool is_negative = (vel < 0);
        const bool use_ccw = (is_negative == (_config.motor_plus_dir == MotorPlusDir::CW));
        std::array<uint8_t, 8> code = {
            _config.addr,                          // Address
            0xF6,                          // Command code
            static_cast<uint8_t>(use_ccw ? MotorPlusDir::CCW : MotorPlusDir::CW),    // Direction (0=CW, 1=CCW)
            static_cast<uint8_t>(rpm_hex >> 8),  // Speed high byte
            static_cast<uint8_t>(rpm_hex & 0xFF),// Speed low byte
            acc_hex,                       // Acceleration
            static_cast<uint8_t>(sync_flag),// Sync flag
            0x00                           // Checksum placeholder
        };

        _command_transaction(code);
    }

    inline void position_control(scalar_t pos, scalar_t vel, scalar_t acc, bool sync_flag=false) {
        // Clamp position, velocity and acceleration to configured bounds
        pos = std::clamp(pos, _config.position_lower_bound, _config.position_upper_bound);
        vel = std::clamp(vel, _config.velocity_lower_bound, _config.velocity_upper_bound); 
        acc = std::clamp(acc, _config.acceleration_lower_bound, _config.acceleration_upper_bound);

        // Convert velocity from rad/s to RPM, clamped to uint16 range
        const scalar_t rpm = std::abs(vel) * 60.0 / (2.0 * M_PI) * _config.reducer_ratio;
        const uint16_t rpm_hex = static_cast<uint16_t>(std::min(rpm, static_cast<scalar_t>(UINT16_MAX)));
        
        // Convert acceleration to control level (0-255), ensuring full range utilization
        const scalar_t normalized_acc = std::min(std::abs(acc), 1.0);
        const uint8_t acc_hex = static_cast<uint8_t>(normalized_acc * 255);

        // Calculate step size based on motor configuration
        const scalar_t base_step = (_config.motor_step_angle == MotorStepAngle::MOTOR_0_9_DEG) ? 0.9 : 1.8;
        const scalar_t step_size = (base_step * M_PI) / (180.0 * _config.microstep);  // rad per step
        
        // Calculate required pulses for position, accounting for gear ratio
        const uint32_t pulse_count = static_cast<uint32_t>(std::abs(pos) / step_size * _config.reducer_ratio);

        // Extract sign for direction, defaulting to configured direction for zero
        const bool is_negative = (pos < 0);
        // Determine direction based on sign and configured direction
        // If both negative and CW, or both positive and CCW, use CCW
        // If signs differ (negative and CCW, or positive and CW), use CW
        const bool use_ccw = (is_negative == (_config.motor_plus_dir == MotorPlusDir::CW));

        // Command format: [addr][0xFD][direction][speed_l][speed_h][acc][pulse0][pulse1][pulse2][pulse3][mode][sync][checksum]
        std::array<uint8_t, 13> code = {
            _config.addr,                              // Address
            0xFD,                              // Command code
            static_cast<uint8_t>(use_ccw ? MotorPlusDir::CCW : MotorPlusDir::CW),    // Direction (0=CW, 1=CCW)
            static_cast<uint8_t>(rpm_hex >> 8),     // Speed high byte
            static_cast<uint8_t>(rpm_hex & 0xFF),    // Speed low byte
            acc_hex,                           // Acceleration
            static_cast<uint8_t>((pulse_count >> 24) & 0xFF), // Pulse count byte 3 (MSB)
            static_cast<uint8_t>((pulse_count >> 16) & 0xFF), // Pulse count byte 2
            static_cast<uint8_t>((pulse_count >> 8) & 0xFF),  // Pulse count byte 1
            static_cast<uint8_t>(pulse_count & 0xFF),       // Pulse count byte 0 (LSB)
            0x01,                              // Mode (0=relative, 1=absolute)
            static_cast<uint8_t>(sync_flag),   // Sync flag
            0x00                               // Checksum placeholder
        };

        _command_transaction(code);
    }

    inline void homing() {
        position_control(0, HOMING_VEL, HOMING_ACC);
    }

    /**
     * @brief Set the bounds for the joint driver
     * 
     * @param position_upper 
     * @param velocity_upper 
     * @param acceleration_upper 
     * @param position_lower 
     * @param velocity_lower 
     * @param acceleration_lower 
     */
    inline void set_bounds(scalar_t position_upper, scalar_t velocity_upper, scalar_t acceleration_upper,
                         scalar_t position_lower, scalar_t velocity_lower, scalar_t acceleration_lower) {
        _config.position_upper_bound = position_upper;
        _config.velocity_upper_bound = velocity_upper; 
        _config.acceleration_upper_bound = acceleration_upper;
        _config.position_lower_bound = position_lower;
        _config.velocity_lower_bound = velocity_lower;
        _config.acceleration_lower_bound = acceleration_lower;
        enable(true);
        std::cout << _config.name << ": Joint bounds set, enabling driver.\n";
    }

protected:
    int _can_socket;
    Config _config;

    template<std::size_t frameSize>
    inline void _send_can_frames(std::array<can_frame, frameSize>& frames) {
#if DEBUG
            std::cout << "----------------SENDING FRAMES----------------\n";
            print_can_frames(frames);
#endif
        // Send frames
        for (size_t i = 0; i < frames.size(); i++) {
            ssize_t nbytes = write(_can_socket, &frames[i], CAN_MTU);
            if (nbytes != CAN_MTU) {
                throw std::runtime_error(_config.name + ": Failed to write CAN frame: " + std::string(strerror(errno)));
            }
        }
    }

    template<std::size_t frameSize>
    inline std::array<can_frame, frameSize> _read_can_frames() {
        struct can_frame frame;
        struct timeval timeout = {1, 0};  // 1 second timeout
        std::array<can_frame, frameSize> frames;
        if (setsockopt(_can_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            throw std::runtime_error(_config.name + ": setsockopt failed: " + std::string(strerror(errno)));
        }   
        size_t frame_count = 0;
        while (frame_count < frameSize) {
            ssize_t nbytes = ::read(_can_socket, &frame, sizeof(frame));
            if (nbytes == -1) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // No more frames to read
                    break;
                }
                throw std::runtime_error(_config.name + ": read failed: " + std::string(strerror(errno)));
            }
            frames[frame_count++] = frame;
        }

        if (frame_count == 0) {
            throw std::runtime_error(_config.name + ": No frames received");
        }

#if DEBUG
            std::cout << "----------------READING FRAMES----------------\n";
            print_can_frames(frames);
#endif

        return frames;
    }

    inline uint8_t _checksum(const uint8_t* data, size_t size) {
        if (_config.checksum_type == ChecksumType::FIXED_6B) {
            return 0x6B;
        } else if (_config.checksum_type == ChecksumType::CRC8) {
            return crc8_checksum(data, size);
        } else if (_config.checksum_type == ChecksumType::XOR) {
            uint8_t checksum = 0;
            for (size_t i = 0; i < size; i++) {
                checksum ^= data[i];
            }
            return checksum;
        } else {
            throw std::runtime_error(_config.name + ": Unknown checksum type");
        }
    }

    template<std::size_t size>
    inline void _command_transaction(std::array<uint8_t, size>& code) {
        code.back() = _checksum(code.data(), size - 1);
        auto send_frames = serial_data_to_can_frames(code);
        _send_can_frames(send_frames);
        auto read_frames = _read_can_frames<1>();
        // command response is expected to be 4 bytes long [addr][common_byte][response_type][checksum]
        auto read_serial_data = can_frames_to_serial_data<4>(read_frames);
        if (read_serial_data.back() != crc8_checksum(read_serial_data.data(), read_serial_data.size() - 1)) {
#if DEBUG
                std::cout << "Checksum error: ";
                for (const auto& byte : read_serial_data) {
                    std::cout << std::hex << (int)byte << " ";
                }
                std::cout << "\n";
#endif
            throw std::runtime_error(_config.name + ": Checksum error");
        }

        // Check response type and handle accordingly
        if (read_serial_data[0] == code[0] && read_serial_data[1] == code[1] && read_serial_data[2] == 0x02) {
#if DEBUG
                std::cout << "Command successful\n";
#endif
            return;
        }

        // Helper function to print frame details
        auto print_frame_details = [&read_serial_data, &code]() {
            std::cout << "Command sent: ";
            for (const auto& byte : code) {
                std::cout << std::hex << (int)byte << " ";
            }
            std::cout << "Command response: ";
            for (const auto& byte : read_serial_data) {
                std::cout << std::hex << (int)byte << " ";
            }
        };

        // Handle error cases
        print_frame_details();
        
        if (read_serial_data[0] == code[0] && read_serial_data[1] == code[1] && read_serial_data[2] == 0xE2) {
            throw std::runtime_error(_config.name + ": Condition not met.");
        } else if (read_serial_data[0] == code[0] && read_serial_data[1] == 0x00 && read_serial_data[2] == 0xEE) {
            throw std::runtime_error(_config.name + ": Wrong command.");
        } else {
            throw std::runtime_error(_config.name + ": unknown error.");
        }
    }

};

} // namespace driver

} // namespace timr