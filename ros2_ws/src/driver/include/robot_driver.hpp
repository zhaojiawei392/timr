#pragma once

#include "helper.hpp"
#include "joint_driver.hpp"

// C++ Standard Library
#include <thread>
#include <iostream>
#include <vector>

namespace timr {

namespace driver {

class SerialManipulatorDriver {
public:    
    explicit SerialManipulatorDriver(const std::string& robot_description_path) {
        // Validate file exists
        if (!std::filesystem::exists(robot_description_path)) {
            throw std::runtime_error(_config.name + ": Robot description file not found: " + robot_description_path);
        }

        // Load and parse YAML
        YAML::Node robot_description;
        try {
            robot_description = YAML::LoadFile(robot_description_path);
        } catch (const YAML::Exception& e) {
            throw std::runtime_error(_config.name + ": Failed to parse robot description YAML: " + std::string(e.what()));
        }

        // Validate required fields
        if (!robot_description["spec"]["jointDescriptionPaths"] || !robot_description["spec"]["canInterface"]) {
            throw std::runtime_error(_config.name + ": Missing required fields in robot description");
        }
        
        _config.api_version = robot_description["apiVersion"].as<std::string>();
        _config.name = robot_description["metadata"]["name"].as<std::string>();
        _config.kind = robot_description["kind"].as<std::string>();

        // Get CAN interface config
        std::string can_interface_name = robot_description["spec"]["canInterface"]["name"].as<std::string>();
        _can_socket = bind_can_socket(can_interface_name);

        const auto& joint_description_paths = robot_description["spec"]["jointDescriptionPaths"];
        
        if (joint_description_paths.size() != DOF) {
            throw std::runtime_error(_config.name + ": Number of motor descriptions (" + 
                std::to_string(joint_description_paths.size()) + ") does not match DOF (" + 
                std::to_string(DOF) + ")");
        }

        // Initialize joints
        for (std::size_t i = 0; i < DOF; i++) {
            try {
                _joints[i] = std::make_unique<JointDriver>(_can_socket, joint_description_paths[i].as<std::string>());
            }
            catch (const std::exception& e) {
                close(_can_socket);
                throw std::runtime_error(_config.name + ": Failed to initialize joint " + std::to_string(i+1) + ": " + e.what());
            }
        }
    }
    
    ~SerialManipulatorDriver() {
        if (_can_socket >= 0) {
            close(_can_socket);
        }
    }

    struct Config {
        std::string api_version;
        std::string name;
        std::string kind;
    };

    inline void emergency_stop() {
        for (dof_size_t i = 0; i < DOF; ++i) {
            _joints[i]->emergency_stop();
        }
        std::cout<< "Emergency stop triggered!\n";
    }
    inline void position_control(std::array<scalar_t, DOF> positions, std::array<scalar_t, DOF> velocities, std::array<scalar_t, DOF> accelerations) {
        for (dof_size_t i = 0; i < DOF; ++i) {
            _joints[i]->position_control(positions[i], velocities[i], accelerations[i]);
        }
    }
    inline void homing() {
        for (int i = DOF-1; i >= 0; --i) {
            _joints[i]->homing();
            std::cout << "Homing joint " << i+1 << "\n";
        }
    }

    inline const std::array<scalar_t, DOF> get_joint_positions() noexcept { 
        std::array<scalar_t, DOF> positions;
        for (dof_size_t i = 0; i < DOF; ++i) {
            positions[i] = _joints[i]->read_realtime_position();
        }
        return positions; 
    }
    inline const std::array<scalar_t, DOF> get_joint_velocities() noexcept { 
        std::array<scalar_t, DOF> velocities;
        for (dof_size_t i = 0; i < DOF; ++i) {
            velocities[i] = _joints[i]->read_realtime_velocity();
        }
        return velocities; 
    } 
    inline const std::array<scalar_t, DOF> get_joint_efforts() noexcept { 
        std::array<scalar_t, DOF> efforts;
        for (dof_size_t i = 0; i < DOF; ++i) {
            efforts[i] = _joints[i]->read_realtime_effort();
        }
        return efforts; 
    }
protected:
    int _can_socket;
    Config _config;
    std::array<std::unique_ptr<JointDriver>, DOF> _joints;

};

} // namespace driver

} // namespace timr