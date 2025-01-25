#pragma once
#include "helper.hpp"
// C++ Standard Library
#include <array>
#include <filesystem>
#include <string>
#include <stdexcept>
#include <iostream>

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
    : _can_socket(can_socket), _config(config_from_yaml(yaml_path)) {
        // configure_driver(_config);
    }

    explicit JointDriver(int can_socket, Config config)
    : _can_socket(can_socket), _config(config) {
        // configure_driver(_config);
    }

    enum class MotorStepAngle : uint8_t {
        MOTOR_0_9_DEG = 0x00,  // 0.9° motor
        MOTOR_1_8_DEG = 0x01   // 1.8° motor
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

    enum class DirPinMode : uint8_t {
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
        DirPinMode dir_pin_mode;                         // Direction pin configuration
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
    };

    static inline Config config_from_yaml(const std::string& yaml_path) {
        // Validate file exists
        if (!std::filesystem::exists(yaml_path)) {
            throw std::runtime_error("Motor description file not found: " + yaml_path);
        }

        // Load and parse YAML
        YAML::Node yaml;
        try {
            yaml = YAML::LoadFile(yaml_path);
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("Failed to parse motor description YAML: " + std::string(e.what()));
        }

        Config config;

        config.api_version = yaml["apiVersion"].as<std::string>();
        config.name = yaml["metadata"]["name"].as<std::string>();
        config.kind = yaml["kind"].as<std::string>();

        // Validate required fields
        if (!yaml["spec"]) {
            throw std::runtime_error("Missing required 'spec' field in motor description");
        }
        auto spec = yaml["spec"];
        
        // Basic motor configuration
        config.addr = spec["addr"].as<uint8_t>();
        config.motor_step_angle = spec["motorStepAngle"].as<double>() == 1.8 ? MotorStepAngle::MOTOR_1_8_DEG : MotorStepAngle::MOTOR_0_9_DEG;
        config.microstep = spec["microstep"].as<uint8_t>();
        config.reducer_ratio = spec["reducerRatio"].as<uint8_t>();
        config.dir_pin_mode = spec["positiveDirection"].as<std::string>() == "ccw" ? DirPinMode::CCW : DirPinMode::CW;
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
            throw std::runtime_error("Invalid pulse mode: " + plus_mode + ", valid options are: foc, open, off, esi");
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
            throw std::runtime_error("Invalid communication mode: " + comm_mode + ", valid options are: can, uart, off, esi");
        }

        std::string en_pin_mode = spec["enPinMode"].as<std::string>();
        if (en_pin_mode == "hold") {
            config.en_pin_mode = EnPinMode::HOLD;
        } else if (en_pin_mode == "high") {
            config.en_pin_mode = EnPinMode::HIGH_ACTIVE;
        } else if (en_pin_mode == "low") {
            config.en_pin_mode = EnPinMode::LOW_ACTIVE;
        } else {
            throw std::runtime_error("Invalid en_pin_mode: " + en_pin_mode + ", valid options are: hold, high, low");
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
            default: throw std::runtime_error("Invalid UART baud rate: " + std::to_string(uart_baud));
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
            default: throw std::runtime_error("Invalid CAN baud rate: " + std::to_string(can_baud));
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
            throw std::runtime_error("Invalid checksum_type: " + checksum_type + ", valid options are: fixed_6b, xor, crc_8, modbus");
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
            throw std::runtime_error("Invalid response_type: " + response_type + ", valid options are: none, receive, reached, both, other");
        }
        // Protection settings
        config.stall_protection = spec["stallProtectionEnabled"].as<bool>() ? EnabledState::ENABLED : EnabledState::DISABLED;
        config.stall_threshold_rpm = spec["stallThresholdRpm"].as<uint16_t>();
        config.stall_current_threshold_ma = spec["stallCurrentThresholdMa"].as<uint16_t>();
        config.stall_detection_ms = spec["stallDetectionMs"].as<uint16_t>();
        config.position_tolerance_deg = spec["positionToleranceDeg"].as<double>();
        return config;
    }

    inline void configure_driver(const Config& config, bool store_flag=true) {
        std::array<uint8_t, 33> code;
        
        // Header
        code[0] = config.addr;                          // Address
        code[1] = 0x48;                                  // Command code
        code[2] = 0xD1;                                  // Subcommand
        code[3] = static_cast<uint8_t>(store_flag);      // Store flag
        
        // Basic motor configuration
        code[4] = static_cast<uint8_t>(config.motor_step_angle);
        code[5] = static_cast<uint8_t>(config.pulse_mode);
        code[6] = static_cast<uint8_t>(config.comm_mode);
        code[7] = static_cast<uint8_t>(config.en_pin_mode);
        code[8] = static_cast<uint8_t>(config.dir_pin_mode);
        code[9] = config.microstep;
        
        // Feature settings
        code[10] = static_cast<uint8_t>(config.interpolation);
        code[11] = static_cast<uint8_t>(config.auto_screen_off);
        
        // Current and voltage limits (16-bit)
        code[12] = static_cast<uint8_t>(config.open_loop_current_ma & 0xFF);
        code[13] = static_cast<uint8_t>(config.open_loop_current_ma >> 8);
        code[14] = static_cast<uint8_t>(config.closed_loop_stall_current_ma & 0xFF);
        code[15] = static_cast<uint8_t>(config.closed_loop_stall_current_ma >> 8);
        code[16] = static_cast<uint8_t>(config.closed_loop_max_voltage_mv & 0xFF);
        code[17] = static_cast<uint8_t>(config.closed_loop_max_voltage_mv >> 8);
        
        // Communication settings
        code[18] = static_cast<uint8_t>(config.uart_baudrate);
        code[19] = static_cast<uint8_t>(config.can_baudrate);
        code[20] = 0x01;                                 // Fixed ID value
        code[21] = static_cast<uint8_t>(config.checksum_type);
        code[22] = static_cast<uint8_t>(config.response_type);
        
        // Protection settings
        code[23] = static_cast<uint8_t>(config.stall_protection);
        code[24] = static_cast<uint8_t>(config.stall_threshold_rpm & 0xFF);
        code[25] = static_cast<uint8_t>(config.stall_threshold_rpm >> 8);
        code[26] = static_cast<uint8_t>(config.stall_current_threshold_ma & 0xFF);
        code[27] = static_cast<uint8_t>(config.stall_current_threshold_ma >> 8);
        code[28] = static_cast<uint8_t>(config.stall_detection_ms & 0xFF);
        code[29] = static_cast<uint8_t>(config.stall_detection_ms >> 8);
        
        // Position control
        uint16_t tolerance = static_cast<uint16_t>(config.position_tolerance_deg * 10);
        code[30] = static_cast<uint8_t>(tolerance & 0xFF);
        code[31] = static_cast<uint8_t>(tolerance >> 8);
        
        code[32] = 0x00;                                 // Checksum placeholder

        _can_transaction(code);
    }

    inline void set_open_loop_current(uint16_t current_ma, bool store_flag=true) {
        // Command format: [addr][0x44][0x33][store_flag][current_low][current_high][checksum]
        std::array<uint8_t, 7> code = {
            _config.addr,                          // Address
            0x44,                          // Command code
            0x33,                          // Subcommand
            static_cast<uint8_t>(store_flag),// Store flag (1=store, 0=don't store)
            static_cast<uint8_t>(current_ma & 0xFF), // Current low byte
            static_cast<uint8_t>(current_ma >> 8),   // Current high byte
            0x00                           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _can_transaction(code);
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

        _can_transaction(code);
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

        _can_transaction(code);
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

        _can_transaction(code);
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
        auto frames_response = _read_can_frames();
        auto response = can_frames_to_serial_data(frames_response);

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
        auto frames_response = _read_can_frames();
        auto response = can_frames_to_serial_data(frames_response);

        // Extract sign and velocity from response
        bool is_negative = (response[2] == 0x01);
        
        // Combine 2 bytes into velocity value (RPM)
        uint16_t rpm = (static_cast<uint16_t>(response[3]) << 8) |
                      static_cast<uint16_t>(response[4]);

        // Convert RPM to rad/s
        scalar_t velocity = static_cast<scalar_t>(rpm) * 2.0 * M_PI / 60.0;
        
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
        auto frames_response = _read_can_frames();
        auto response = can_frames_to_serial_data(frames_response);

        // Extract sign and position from response
        bool is_negative = (response[2] == 0x01);
        
        // Combine 4 bytes into position value
        uint32_t position = (static_cast<uint32_t>(response[3]) << 24) |
                           (static_cast<uint32_t>(response[4]) << 16) |
                           (static_cast<uint32_t>(response[5]) << 8)  |
                           static_cast<uint32_t>(response[6]);

        // Convert to radians (2π per rotation)
        scalar_t angle = static_cast<scalar_t>(position) * 2.0 * M_PI / 65536.0;
        
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
        auto frames_response = _read_can_frames();
        auto response = can_frames_to_serial_data(frames_response);

        // Extract sign and error from response
        bool is_negative = (response[2] == 0x01);
        
        // Combine 4 bytes into error value
        uint32_t error = (static_cast<uint32_t>(response[3]) << 24) |
                        (static_cast<uint32_t>(response[4]) << 16) |
                        (static_cast<uint32_t>(response[5]) << 8)  |
                        static_cast<uint32_t>(response[6]);

        // Convert to radians (2π per rotation)
        scalar_t angle_error = static_cast<scalar_t>(error) * 2.0 * M_PI / 65536.0;
        
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
        auto frames_response = _read_can_frames();
        auto response = can_frames_to_serial_data(frames_response);

        // Extract sign and position from response
        bool is_negative = (response[2] == 0x01);
        
        // Combine 4 bytes into position value
        uint32_t position = (static_cast<uint32_t>(response[3]) << 24) |
                           (static_cast<uint32_t>(response[4]) << 16) |
                           (static_cast<uint32_t>(response[5]) << 8)  |
                           static_cast<uint32_t>(response[6]);

        // Convert to radians (2π per rotation)
        scalar_t angle = static_cast<scalar_t>(position) * 2.0 * M_PI / 65536.0;
        
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
        auto frames_response = _read_can_frames();
        auto response = can_frames_to_serial_data(frames_response);

        // Extract sign and position from response
        bool is_negative = (response[2] == 0x01);
        
        // Combine 4 bytes into position value
        uint32_t position = (static_cast<uint32_t>(response[3]) << 24) |
                           (static_cast<uint32_t>(response[4]) << 16) |
                           (static_cast<uint32_t>(response[5]) << 8)  |
                           static_cast<uint32_t>(response[6]);

        // Convert to radians (2π per rotation)
        scalar_t angle = static_cast<scalar_t>(position) * 2.0 * M_PI / 65536.0;
        
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

        _can_transaction(code);
    }

    inline void calibration() {
        // Command format: [addr][0x06][0x45][checksum]
        std::array<uint8_t, 4> code = {
            _config.addr,          // Address
            0x06,          // Command code
            0x45,          // Calibrate encoder command
            0x00           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _can_transaction(code);
    }

    inline void clear_position() {
        // Command format: [addr][0x0A][0x6D][checksum]
        std::array<uint8_t, 4> code = {
            _config.addr,          // Address
            0x0A,          // Command code
            0x6D,          // Clear position command
            0x00           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _can_transaction(code);
    }

    inline void interrupt_homing() {
        // Command format: [addr][0x9C][0x48][checksum]
        std::array<uint8_t, 4> code = {
            _config.addr,          // Address
            0x9C,          // Command code
            0x48,          // Interrupt homing command
            0x00           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _can_transaction(code);
    }

    enum class HomingMode : uint8_t {
        SINGLE_TURN_NEAREST = 0x00,    // Single-turn nearest direction
        SINGLE_TURN_DIRECTED = 0x01,   // Single-turn specified direction
        MULTI_TURN_COLLISION = 0x02,   // Multi-turn unlimited collision
        MULTI_TURN_LIMIT_SWITCH = 0x03 // Multi-turn with limit switch
    };

    inline void trigger_homing(HomingMode mode = HomingMode::SINGLE_TURN_NEAREST, bool sync_flag = false) {
        // Command format: [addr][0x9A][mode][sync_flag][checksum]
        std::array<uint8_t, 5> code = {
            _config.addr,                          // Address
            0x9A,                          // Command code
            static_cast<uint8_t>(mode),     // Homing mode
            static_cast<uint8_t>(sync_flag),// Sync flag
            0x00                           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _can_transaction(code);
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

        _can_transaction(code);
    }

    inline void sync_move() {
        // Command format: [addr][0xFF][0x66][checksum]
        std::array<uint8_t, 4> code = {
            _config.addr,          // Address
            0xFF,          // Command code
            0x66,          // Sync move command
            0x00           // Checksum placeholder (will be calculated by _crc8_checksum)
        };

        _can_transaction(code);
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

        _can_transaction(code);
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

        _can_transaction(code);
    }
    
    inline void vel_control(scalar_t vel, scalar_t acc, bool sync_flag=false) {
        // Convert velocity from rad/s to RPM, clamped to uint16 range
        const scalar_t rpm = std::abs(vel) * 60.0 / (2.0 * M_PI) * _config.reducer_ratio;
        const uint16_t rpm_hex = static_cast<uint16_t>(std::min(rpm, static_cast<scalar_t>(UINT16_MAX)));
        
        // Convert acceleration to control level (0-255), ensuring full range utilization
        const scalar_t normalized_acc = std::min(std::abs(acc), 1.0); 
        const uint8_t acc_hex = static_cast<uint8_t>(normalized_acc * 255);
        // Command format: [addr][0xF6][direction][speed_l][speed_h][acc][sync][checksum]
        std::array<uint8_t, 8> code = {
            _config.addr,                          // Address
            0xF6,                          // Command code
            static_cast<uint8_t>(_config.dir_pin_mode),// Direction (0=CW, 1=CCW)
            static_cast<uint8_t>(rpm_hex & 0xFF),// Speed low byte
            static_cast<uint8_t>(rpm_hex >> 8),  // Speed high byte
            acc_hex,                       // Acceleration
            static_cast<uint8_t>(sync_flag),// Sync flag
            0x00                           // Checksum placeholder
        };

        _can_transaction(code);
    }

    inline void pos_control(scalar_t pos, scalar_t vel, scalar_t acc, bool sync_flag=false) {
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

        // Command format: [addr][0xFD][direction][speed_l][speed_h][acc][pulse0][pulse1][pulse2][pulse3][mode][sync][checksum]
        std::array<uint8_t, 13> code = {
            _config.addr,                              // Address
            0xFD,                              // Command code
            static_cast<uint8_t>(_config.dir_pin_mode),    // Direction (0=CW, 1=CCW)
            static_cast<uint8_t>(rpm_hex & 0xFF),// Speed low byte
            static_cast<uint8_t>(rpm_hex >> 8), // Speed high byte
            acc_hex,                           // Acceleration
            static_cast<uint8_t>(pulse_count & 0xFF),   // Pulse count byte 0 (LSB)
            static_cast<uint8_t>(pulse_count >> 8),     // Pulse count byte 1
            static_cast<uint8_t>(pulse_count >> 16),    // Pulse count byte 2
            static_cast<uint8_t>(pulse_count >> 24),    // Pulse count byte 3 (MSB)
            0x01,                              // Mode (0=relative, 1=absolute)
            static_cast<uint8_t>(sync_flag),   // Sync flag
            0x00                               // Checksum placeholder
        };

        _can_transaction(code);
    }

protected:
    int _can_socket;
    Config _config;

    template<std::size_t frameSize>
    inline void _send_can_frames(std::array<can_frame, frameSize>& frames) {
        // Send frames
        for (size_t i = 0; i < frames.size(); i++) {
            ssize_t nbytes = write(_can_socket, &frames[i], CAN_MTU);
            if (DEBUG) {
                std::cout << "Send Frame " << i + 1 << "/" << frames.size() << std::endl;
                std::cout << "Frame ID: 0x" << std::hex << (frames[i].can_id & ~CAN_EFF_FLAG) << std::endl;
                std::cout << "Frame Length: " << std::dec << (int)frames[i].can_dlc << std::endl;
                std::cout << "Frame data: ";
                for (int j = 0; j < frames[i].can_dlc; j++) {
                    std::cout << std::hex << (int)frames[i].data[j] << " ";
                }
                std::cout << std::endl;
            }
            if (nbytes != CAN_MTU) {
                throw std::runtime_error("Failed to write CAN frame: " + std::string(strerror(errno)));
            }
        }
    }

    inline std::array<can_frame, MAX_FRAMES> _read_can_frames() {
        struct can_frame frame;
        struct timeval timeout = {1, 0};  // 1 second timeout
        std::array<can_frame, MAX_FRAMES> frames;
        size_t frame_count = 0;
        
        if (setsockopt(_can_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            throw std::runtime_error("setsockopt failed: " + std::string(strerror(errno)));
        }   

        while (frame_count < MAX_FRAMES) {
            ssize_t nbytes = ::read(_can_socket, &frame, sizeof(frame));
            if (nbytes == -1) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // No more frames to read
                    break;
                }
                throw std::runtime_error("read failed: " + std::string(strerror(errno)));
            }
            if (DEBUG) {
                std::cout << "Read Frame " << frame_count + 1 << "/" << frames.size() << std::endl;
                std::cout << "Frame ID: 0x" << std::hex << (frames[frame_count].can_id & ~CAN_EFF_FLAG) << std::endl;
                std::cout << "Frame Length: " << std::dec << (int)frames[frame_count].can_dlc << std::endl;
                std::cout << "Frame data: ";
                for (int j = 0; j < frames[frame_count].can_dlc; j++) {
                    std::cout << std::hex << (int)frames[frame_count].data[j] << " ";
                }
                std::cout << std::endl;
            }
            frames[frame_count++] = frame;
        }

        if (frame_count == 0) {
            throw std::runtime_error("No frames received");
        }

        return frames;
    }

    template<std::size_t size>
    inline void _can_transaction(std::array<uint8_t, size>& code) {
        code.back() = crc8_checksum(code.data(), size - 1);
        auto frames = serial_data_to_can_frames(code);
        _send_can_frames(frames);
        auto frame_response = _read_can_frames();
        auto response = can_frames_to_serial_data(frame_response);

        // Check response type and handle accordingly
        if (response[0] == code[0] && response[1] == code[1] && response[2] == 0x02) {
            return;
        }

        // Helper function to print frame details
        auto print_frame_details = [&response, &code]() {
            std::cout << "Command sent: ";
            for (const auto& byte : code) {
                std::cout << std::hex << (int)byte << " ";
            }
            std::cout << "Command response: ";
            for (const auto& byte : response) {
                std::cout << std::hex << (int)byte << " ";
            }
        };

        // Handle error cases
        print_frame_details();
        
        if (response[0] == code[0] && response[1] == code[1] && response[2] == 0xE2) {
            throw std::runtime_error("Condition not met.");
        } else if (response[0] == code[0] && response[1] == 0x00 && response[2] == 0xEE) {
            throw std::runtime_error("Wrong command.");
        } else {
            throw std::runtime_error("Unknown error.");
        }
    }

};

} // namespace driver

} // namespace timr