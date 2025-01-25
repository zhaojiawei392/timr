#pragma once

#include "helper.hpp"
#include "joint_driver.hpp"

// C++ Standard Library
#include <thread>
#include <iostream>
namespace timr {

namespace driver {

class SerialManipulatorDriver {
protected:
    int _can_socket;
    std::array<std::unique_ptr<JointDriver>, DOF> _joints;
public:    
    explicit SerialManipulatorDriver(const std::string& robot_description_path) {
        // Validate file exists
        if (!std::filesystem::exists(robot_description_path)) {
            throw std::runtime_error("Robot description file not found: " + robot_description_path);
        }

        // Load and parse YAML
        YAML::Node robot_description;
        try {
            robot_description = YAML::LoadFile(robot_description_path);
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("Failed to parse robot description YAML: " + std::string(e.what()));
        }

        // Validate required fields
        if (!robot_description["spec"]["jointDescriptionPaths"] || !robot_description["spec"]["canConfig"]) {
            throw std::runtime_error("Missing required fields in robot description");
        }

        // Get CAN interface config
        const auto& can_config = robot_description["spec"]["canConfig"];
        std::string can_interface = can_config["canInterface"].as<std::string>();
        _can_socket = bind_can_socket(can_interface);

        const auto& joint_description_paths = robot_description["spec"]["jointDescriptionPaths"];
        
        if (joint_description_paths.size() != DOF) {
            throw std::runtime_error("Number of motor descriptions (" + 
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
                throw std::runtime_error("Failed to initialize joint " + std::to_string(i) + ": " + e.what());
            }
        }
    }
    
    ~SerialManipulatorDriver() {
        homing();
        if (_can_socket >= 0) {
            close(_can_socket);
        }
    }

    inline void emergency_stop() {
        for (dof_size_t i = 0; i < DOF; ++i) {
            _joints[i]->emergency_stop();
        }
        std::cout<< "Emergency stop triggered!" << std::endl;
    }
    inline void position_control(std::array<scalar_t, DOF> positions, std::array<scalar_t, DOF> velocities, std::array<scalar_t, DOF> accelerations) {
        for (dof_size_t i = 0; i < DOF; ++i) {
            _joints[i]->pos_control(positions[i], velocities[i], accelerations[i]);
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

};

} // namespace driver

} // namespace timr