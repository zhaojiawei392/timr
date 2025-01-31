/** 
 *     This file is part of timr.
 *  
 *     timr is free software: you can redistribute it and/or modify 
 *     it under the terms of the GNU General Public License as published 
 *     by the Free Software Foundation, either version 3 of the License, 
 *     or (at your option) any later version.
 *  
 *     timr is distributed in the hope that it will be useful, 
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *     See the GNU General Public License for more details.
 *  
 *     You should have received a copy of the GNU General Public License
 *     along with timr. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once
#include "dqpose.hpp"
#include <qpOASES.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <vector>
#include <fstream>
#include <signal.h>
#include <filesystem>

namespace timr {
    
namespace kinematics {

using scalar_t = double;
using dof_size_t = unsigned int;
constexpr scalar_t deg2rad_factor = M_PI / 180.;
constexpr scalar_t rad2deg_factor = 180. / M_PI;

using namespace dqpose;

class Joint;
class DHJoint;
class RevoluteJoint;
class PrismaticJoint;
class SerialManipulator;

inline Quat<scalar_t> __closest_invariant_rotation_error(const Rotation<scalar_t>& r, const Rotation<scalar_t>& rd) noexcept {
    Quat<scalar_t> er_plus = r.conj() * rd - Quat(1);
    Quat<scalar_t> er_minus = r.conj() * rd + Quat(1);

    if (er_plus.norm() < er_minus.norm()) {
        return er_plus;
    } else {
        return er_minus;
    }
}

class Joint {
public:
    Joint() noexcept {}
    virtual inline Pose<scalar_t> fkm(const scalar_t position) const noexcept = 0;
    virtual inline DualQuat<scalar_t> derivative(const scalar_t position) const noexcept = 0;

    virtual ~Joint() = default;
            Joint(const Joint&) = default;
            Joint(Joint&&) = default;
    Joint& operator=(const Joint&) = default;
    Joint& operator=(Joint&&) = default;

};

class RevoluteJoint : public Joint {
protected:
    std::array<scalar_t, 4> _dhparams; 
public:
    explicit RevoluteJoint(const scalar_t theta, const scalar_t d, const scalar_t a, const scalar_t alpha) noexcept
        : _dhparams({theta, d, a, alpha}) {}
    
    virtual inline Pose<scalar_t> fkm(const scalar_t position) const noexcept override {
        const scalar_t cos_theta = cos( 0.5 * ( theta() + position) );
        const scalar_t sin_theta = sin( 0.5 * ( theta() + position) );
        const scalar_t cos_alpha = cos( 0.5 * alpha() );
        const scalar_t sin_alpha = sin( 0.5 * alpha() );
        return Pose<scalar_t> (
            cos_theta * cos_alpha,
            cos_theta * sin_alpha,
            sin_theta * sin_alpha,
            sin_theta * cos_alpha,
            0.5 * ( -sin_theta * d() * cos_alpha - cos_theta * a() * sin_alpha ),
            0.5 * ( -sin_theta * d() * sin_alpha + cos_theta * a() * cos_alpha ),
            0.5 * ( sin_theta * a() * cos_alpha + cos_theta * d() * sin_alpha ),
            0.5 * ( cos_theta * d() * cos_alpha - sin_theta * a() * sin_alpha )
        );
    }
    virtual inline DualQuat<scalar_t> derivative(const scalar_t position) const noexcept override {
        const scalar_t cos_theta_dot = -0.5 * sin( 0.5 * ( theta() + position));
        const scalar_t sin_theta_dot = 0.5 * cos( 0.5 * ( theta() + position));
        const scalar_t cos_alpha = cos( 0.5 * alpha() );
        const scalar_t sin_alpha = sin( 0.5 * alpha() );
        return DualQuat<scalar_t> (
            cos_theta_dot * cos_alpha,
            cos_theta_dot * sin_alpha,
            sin_theta_dot * sin_alpha,
            sin_theta_dot * cos_alpha,
            0.5 * (-sin_theta_dot * d() * cos_alpha - cos_theta_dot * a() * sin_alpha),
            0.5 * (-sin_theta_dot * d() * sin_alpha + cos_theta_dot * a() * cos_alpha),
            0.5 * (sin_theta_dot * a() * cos_alpha + cos_theta_dot * d() * sin_alpha),
            0.5 * (cos_theta_dot * d() * cos_alpha - sin_theta_dot * a() * sin_alpha)
        );
    }

    inline scalar_t theta() const noexcept { return _dhparams[0];}
    inline scalar_t d() const noexcept { return _dhparams[1];}
    inline scalar_t a() const noexcept { return _dhparams[2];}
    inline scalar_t alpha() const noexcept { return _dhparams[3];}

    RevoluteJoint() = delete;
    virtual ~RevoluteJoint() = default;
            RevoluteJoint(const RevoluteJoint&) = default;
            RevoluteJoint(RevoluteJoint&&) = default;
    RevoluteJoint& operator=(const RevoluteJoint&) = default;
    RevoluteJoint& operator=(RevoluteJoint&&) = default;
};

class PrismaticJoint : public Joint {
protected:
    std::array<scalar_t, 4> _dhparams; 
public:
    explicit PrismaticJoint(const scalar_t theta, const scalar_t d, const scalar_t a, const scalar_t alpha) noexcept
        : _dhparams({theta, d, a, alpha}) {}
    
    virtual inline Pose<scalar_t> fkm(const scalar_t position) const noexcept override {
        scalar_t cos_theta = cos( 0.5 * theta() );
        scalar_t sin_theta = sin( 0.5 * theta() );
        scalar_t cos_alpha = cos( 0.5 * alpha() );
        scalar_t sin_alpha = sin( 0.5 * alpha() );
        scalar_t d_ = d() + position;
        scalar_t a_ = a();
        return Pose<scalar_t> (
            cos_theta * cos_alpha,
            cos_theta * sin_alpha,
            sin_theta * sin_alpha,
            sin_theta * cos_alpha,
            0.5 * ( -sin_theta * d_ * cos_alpha - cos_theta * a_ * sin_alpha ),
            0.5 * ( -sin_theta * d_ * sin_alpha + cos_theta * a_ * cos_alpha ),
            0.5 * ( sin_theta * a_ * cos_alpha + cos_theta * d_ * sin_alpha ),
            0.5 * ( cos_theta * d_ * cos_alpha - sin_theta * a_ * sin_alpha )
        );
    }
    virtual inline DualQuat<scalar_t> derivative([[maybe_unused]] const scalar_t position) const noexcept override {
        scalar_t cos_theta = sin(0.5 * theta());
        scalar_t sin_theta = cos(0.5 * theta());
        scalar_t cos_alpha = cos(0.5 * alpha());
        scalar_t sin_alpha = sin(0.5 * alpha());
        return DualQuat<scalar_t> (
            0,
            0,
            0,
            0,
            0.5 * -sin_theta * cos_alpha,
            0.5 * -sin_theta * sin_alpha,
            0.5 * cos_theta * sin_alpha,
            0.5 * cos_theta * cos_alpha
        );
    }

    inline scalar_t theta() const noexcept { return _dhparams[0];}
    inline scalar_t d() const noexcept { return _dhparams[1];}
    inline scalar_t a() const noexcept { return _dhparams[2];}
    inline scalar_t alpha() const noexcept { return _dhparams[3];}

    PrismaticJoint() = delete;
    virtual ~PrismaticJoint() = default;
            PrismaticJoint(const PrismaticJoint&) = default;
            PrismaticJoint(PrismaticJoint&&) = default;
    PrismaticJoint& operator=(const PrismaticJoint&) = default;
    PrismaticJoint& operator=(PrismaticJoint&&) = default;
};

struct SerialManipulatorConfig {
    scalar_t translation_priority{0.9999};
    scalar_t joint_damping{0.0001};
    scalar_t error_gain{50};
    scalar_t pos_gain{1.0};
    scalar_t vel_gain{1.0};
    dof_size_t DOF{0};
    std::string name;
    std::string api_version;
    std::string kind;
    uint8_t flag{0};
    bool is_open_loop{false};
    bool is_initialized{false};
};

struct SerialManipulatorData {
    Pose<scalar_t> base;
    Pose<scalar_t> effector;
    std::vector<scalar_t> target_joint_position{};
    std::vector<scalar_t> target_joint_velocity{};
    std::vector<scalar_t> target_joint_effort{};
    std::vector<scalar_t> joint_position{};
    std::vector<scalar_t> joint_velocity{};
    std::vector<scalar_t> joint_effort{};

    std::vector<scalar_t> joint_position_lower_bound{};
    std::vector<scalar_t> joint_position_upper_bound{};
    std::vector<scalar_t> joint_velocity_lower_bound{};
    std::vector<scalar_t> joint_velocity_upper_bound{};
    std::vector<scalar_t> joint_effort_lower_bound{};
    std::vector<scalar_t> joint_effort_upper_bound{};

    std::vector<Pose<scalar_t>> joint_pose; // {joint1->joint2, joint2->joint3, joint3->joint4, ... , joint?->end}
    std::vector<DualQuat<scalar_t>> joint_pose_derivative; // {joint1->joint2, joint2->joint3, joint3->joint4, ... , joint?->end}
};

class Solver {
private:
    qpOASES::SQProblem qp;
    qpOASES::Options options;
    int nWSR{500};
    bool first_time{true};
    size_t problem_size{0};

public:
    Solver() = default;
    ~Solver() = default;

    void initialize(size_t size) {
        problem_size = size;
        qp = qpOASES::SQProblem(size, size); // Initialize with proper dimensions
        options.printLevel = qpOASES::PL_LOW;
        qp.setOptions(options);
        first_time = true;
    }

    std::vector<double> solve(const double* H_raw, const double* g_raw, 
                            const double* A_raw, const double* lb_raw, 
                            const double* ub_raw, const double* lb_A_raw, 
                            const double* ub_A_raw) {
        if (problem_size == 0) {
            throw std::runtime_error("Solver not initialized with proper dimensions");
        }

        auto nWSR_in_use = nWSR;
        qpOASES::returnValue status;

        if (first_time) {
            status = qp.init(H_raw, g_raw, A_raw, lb_raw, ub_raw, 
                           lb_A_raw, ub_A_raw, nWSR_in_use);
            first_time = false;
        } else {
            status = qp.hotstart(H_raw, g_raw, A_raw, lb_raw, ub_raw, 
                               lb_A_raw, ub_A_raw, nWSR_in_use);
        }

        if (status != qpOASES::SUCCESSFUL_RETURN) {
            throw std::runtime_error("Failed to solve QP problem");
        }

        std::vector<double> xOpt(problem_size);
        qp.getPrimalSolution(xOpt.data());
        return xOpt;
    }

    void reset() {
        first_time = true;
    }
};

class SerialManipulator {
protected:
    SerialManipulatorData _data;
    std::vector<std::unique_ptr<Joint>> _pjoints;
    SerialManipulatorConfig _cfg;
    Solver _solver;
public:
    SerialManipulator() = delete;
    ~SerialManipulator() = default;
    SerialManipulator(const SerialManipulator& other) = default;
    SerialManipulator(SerialManipulator&& other) = default;
    SerialManipulator& operator=(const SerialManipulator& other) = default;
    SerialManipulator& operator=(SerialManipulator&& other) = default;

    explicit SerialManipulator(const std::string& yaml_path)
    {
        // Check if the YAML file exists
        if (!std::filesystem::exists(yaml_path)) {
            throw std::runtime_error("YAML file does not exist: " + yaml_path);
        }

        // Open and parse the YAML file
        YAML::Node yaml;
        try {
            yaml = YAML::LoadFile(yaml_path);
        } catch (const YAML::Exception& e) {
            throw std::runtime_error("Failed to load YAML file: " + std::string(e.what()));
        }

        try {
            _cfg.api_version = yaml["apiVersion"].as<std::string>();
            _cfg.name = yaml["metadata"]["name"].as<std::string>();
            _cfg.kind = yaml["kind"].as<std::string>();
            const auto& spec = yaml["spec"];
            _cfg.DOF = spec["DOF"].as<dof_size_t>();

            auto theta = spec["dhParams"]["theta"].as<std::vector<scalar_t>>();
            if (theta.size() != _cfg.DOF) {
                throw std::runtime_error("SerialManipulator yaml spec.dhParams.theta.size() != spec.DOF");
            }
            auto d = spec["dhParams"]["d"].as<std::vector<scalar_t>>();
            if (d.size() != _cfg.DOF) {
                throw std::runtime_error("SerialManipulator yaml spec.dhParams.d.size() != spec.DOF");
            }
            auto a = spec["dhParams"]["a"].as<std::vector<scalar_t>>();
            if (a.size() != _cfg.DOF) {
                throw std::runtime_error("SerialManipulator yaml spec.dhParams.a.size() != spec.DOF");
            }
            auto alpha = spec["dhParams"]["alpha"].as<std::vector<scalar_t>>();
            if (alpha.size() != _cfg.DOF) {
                throw std::runtime_error("SerialManipulator yaml spec.dhParams.alpha.size() != spec.DOF");
            }
            auto joint_type = spec["dhParams"]["jointType"].as<std::vector<scalar_t>>();
            if (joint_type.size() != _cfg.DOF) {
                throw std::runtime_error("SerialManipulator yaml spec.dhParams.jointType.size() != spec.DOF");
            }

            _data.joint_position_lower_bound = spec["jointBounds"]["positionLowerBound"].as<std::vector<scalar_t>>();
            if (_data.joint_position_lower_bound.size() != _cfg.DOF) {
                throw std::runtime_error("SerialManipulator yaml spec.jointBounds.positionLowerBound.size() != spec.DOF");
            }
            _data.joint_velocity_lower_bound = spec["jointBounds"]["velocityLowerBound"].as<std::vector<scalar_t>>();
            if (_data.joint_velocity_lower_bound.size() != _cfg.DOF) {
                throw std::runtime_error("SerialManipulator yaml spec.jointBounds.velocityLowerBound.size() != spec.DOF");
            }
            _data.joint_effort_lower_bound = spec["jointBounds"]["effortLowerBound"].as<std::vector<scalar_t>>();
            if (_data.joint_effort_lower_bound.size() != _cfg.DOF) {
                throw std::runtime_error("SerialManipulator yaml spec.jointBounds.effortLowerBound.size() != spec.DOF");
            }
            _data.joint_position_upper_bound = spec["jointBounds"]["positionUpperBound"].as<std::vector<scalar_t>>();
            if (_data.joint_position_upper_bound.size() != _cfg.DOF) {
                throw std::runtime_error("SerialManipulator yaml spec.jointBounds.positionUpperBound.size() != spec.DOF");
            }
            _data.joint_velocity_upper_bound = spec["jointBounds"]["velocityUpperBound"].as<std::vector<scalar_t>>();
            if (_data.joint_velocity_upper_bound.size() != _cfg.DOF) {
                throw std::runtime_error("SerialManipulator yaml spec.jointBounds.velocityUpperBound.size() != spec.DOF");
            }
            _data.joint_effort_upper_bound = spec["jointBounds"]["effortUpperBound"].as<std::vector<scalar_t>>();
            if (_data.joint_effort_upper_bound.size() != _cfg.DOF) {
                throw std::runtime_error("SerialManipulator yaml spec.jointBounds.effortUpperBound.size() != spec.DOF");
            }

            // Convert angles from degrees to radians
            for (dof_size_t i=0; i<_cfg.DOF; ++i) {
                theta[i] *= deg2rad_factor;
                alpha[i] *= deg2rad_factor;
                _data.joint_position_lower_bound[i] *= deg2rad_factor;
                _data.joint_position_upper_bound[i] *= deg2rad_factor;
                _data.joint_velocity_lower_bound[i] *= deg2rad_factor;
                _data.joint_velocity_upper_bound[i] *= deg2rad_factor;
            }

            // Configure solver parameters
            _cfg.translation_priority = spec["solverConfig"]["translationPriority"].as<scalar_t>();
            _cfg.error_gain = spec["solverConfig"]["errorGain"].as<scalar_t>();
            _cfg.joint_damping = spec["solverConfig"]["jointDamping"].as<scalar_t>();
            _cfg.pos_gain = spec["solverConfig"]["posGain"].as<scalar_t>();
            _cfg.vel_gain = spec["solverConfig"]["velGain"].as<scalar_t>();
            _cfg.is_open_loop = spec["solverConfig"]["isOpenLoop"].as<bool>();
            // Initialize joints
            for (dof_size_t i=0; i<_cfg.DOF; ++i){
                if (joint_type[i] == 0){
                    _pjoints.push_back( std::make_unique<RevoluteJoint>(theta[i], d[i], a[i], alpha[i]) );
                } else if (joint_type[i] == 1){
                    _pjoints.push_back( std::make_unique<PrismaticJoint>(theta[i], d[i], a[i], alpha[i]) );
                } else {
                    throw std::runtime_error("Error: SerialManipulator Constructor arg0 jointType[" + std::to_string(i) + "] = " + std::to_string(joint_type[i]) + ", this value should be 0 or 1.(0: Revolute joint, 1: Prismatic joint)\n");
                }
            }

            // Resize vectors
            _data.joint_position.resize(_cfg.DOF);
            _data.joint_velocity.resize(_cfg.DOF);
            _data.joint_effort.resize(_cfg.DOF);
            _data.target_joint_position.resize(_cfg.DOF);
            _data.target_joint_velocity.resize(_cfg.DOF);
            _data.target_joint_effort.resize(_cfg.DOF);
            _data.joint_pose.resize(_cfg.DOF);
            _data.joint_pose_derivative.resize(_cfg.DOF);

        } catch (const YAML::Exception& e) {
            throw std::runtime_error("Error parsing YAML data: " + std::string(e.what()));
        }
    }

    int update(const Pose<scalar_t>& desired_pose) {
        if (_cfg.flag != 7) {
            return -1;
        }
        // SOLVE QP PROBLEM
        USING_NAMESPACE_QPOASES;
        const Rotation<scalar_t>& r_end = _data.joint_pose.back().rotation();
        const Translation<scalar_t>& t_end = _data.joint_pose.back().translation();
        const Rotation<scalar_t>& rd = desired_pose.rotation();
        const Translation<scalar_t>& td = desired_pose.translation();

        std::vector<Quat<scalar_t>> r_rd_derivative(_cfg.DOF);
        std::vector<Quat<scalar_t>> t_derivative(_cfg.DOF);
        for (dof_size_t i=0; i<_cfg.DOF; ++i) {
            r_rd_derivative[i] = _data.joint_pose_derivative[i].real().conj() * rd;
            t_derivative[i] = 2 * (_data.joint_pose_derivative[i].dual() * _data.joint_pose.back().real().conj() 
                                    + _data.joint_pose.back().dual() * _data.joint_pose_derivative[i].real().conj() );
        }

        // Initialize a zero vector
        std::vector<double> H(_cfg.DOF * _cfg.DOF);
        for (dof_size_t i=0; i<_cfg.DOF; ++i) {
            for (dof_size_t j=0; j<_cfg.DOF; ++j) {
                const double Ht = t_derivative[i].dot( t_derivative[j] );
                const double Hr = r_rd_derivative[i].dot( r_rd_derivative[j] );
                H[i*_cfg.DOF+j] = _cfg.translation_priority * Ht + (1-_cfg.translation_priority) * Hr;
                if (i==j)
                    H[i*_cfg.DOF+j] += _cfg.joint_damping;
                
            }
        }

        const Quat<scalar_t> t_err = t_end - td;
        const Quat<scalar_t> r_rd_err = __closest_invariant_rotation_error(r_end, rd);
        // Initialize a zero vector
        std::vector<double> g(_cfg.DOF);
        for (dof_size_t i=0; i<_cfg.DOF; ++i) {
            const double ct = _cfg.error_gain * t_err.dot(t_derivative[i]);
            const double cr = _cfg.error_gain * r_rd_err.dot(r_rd_derivative[i]);
            g[i] = _cfg.translation_priority * ct + (1-_cfg.translation_priority) * cr;
        }

        // Initialize a zero vector
        std::vector<double> constraint(_cfg.DOF * _cfg.DOF);
        for (dof_size_t i=0; i<_cfg.DOF; ++i) {
            for (dof_size_t j=0; j<_cfg.DOF; ++j) {
                constraint[i*_cfg.DOF+j] = i==j ? 1 : 0;
            }
        }
        // SET CONSTRAINTS
        std::vector<double> lower_constraint_bound(_cfg.DOF);
        std::vector<double> upper_constraint_bound(_cfg.DOF);
        std::vector<double> lower_bound(_cfg.DOF);
        std::vector<double> upper_bound(_cfg.DOF);
        for (dof_size_t i=0; i<_cfg.DOF; ++i) {
            lower_constraint_bound[i] = _data.joint_position_lower_bound[i] - _data.joint_position[i];
            upper_constraint_bound[i] = _data.joint_position_upper_bound[i] - _data.joint_position[i];
            lower_bound[i] = _data.joint_velocity_lower_bound[i];
            upper_bound[i] = _data.joint_velocity_upper_bound[i];
        }

        const double* H_raw = H.data();
        const double* g_raw = g.data();
        const double* A_raw = constraint.data();
        const double* lb_A_raw = lower_constraint_bound.data();
        const double* ub_A_raw = upper_constraint_bound.data();
        const double* lb_raw = lower_bound.data();
        const double* ub_raw = upper_bound.data();

        std::vector<double> xOpt = _solver.solve(H_raw, g_raw, A_raw, lb_raw, ub_raw, lb_A_raw, ub_A_raw);

        // UPDATE JOINT POSITIONS, VELOCITIES, AND ACCELERATIONS
        for (dof_size_t i=0; i<_cfg.DOF; ++i) {
            // Update target velocity to QP solution
            _data.target_joint_velocity[i] = std::clamp(xOpt[i] * _cfg.vel_gain, 
                _data.joint_velocity_lower_bound[i], _data.joint_velocity_upper_bound[i]);

            // Calculate velocity gap between QP solution and current target velocity
            scalar_t vel_gap = std::abs(xOpt[i] - _data.target_joint_velocity[i]);
            
            // Calculate effort scaling factor:
            // - For small velocity changes (< 1e-6), use factor of 1.0
            // - For larger changes, scale inversely with velocity gap to smooth effort
            scalar_t acc_factor = vel_gap > 1e-6 ? 1.0 / vel_gap : 1.0;
            
            // Clamp effort between 0 and 1 to limit maximum effort
            _data.target_joint_effort[i] = std::clamp(acc_factor, 
                _data.joint_effort_lower_bound[i], _data.joint_effort_upper_bound[i]);
            
            // Update target position using velocity * time;
            _data.target_joint_position[i] = std::clamp(xOpt[i] * _cfg.pos_gain, 
                _data.joint_position_lower_bound[i], _data.joint_position_upper_bound[i]);
        }
        // RESET UPDATE CHECKLIST
        _cfg.flag = 0;
        return 0;
    }

    inline void initialize(const std::vector<scalar_t>& joint_position) {
        if (joint_position.size() != _cfg.DOF) {
            throw std::runtime_error("SerialManipulator::initialize() joint_position size != DOF");
        }
        if (!_cfg.is_initialized) {
            set_joint_position(_data.joint_position);
            _solver.initialize(_cfg.DOF);
            _cfg.is_initialized = true;
        }
    }

    // setters
    inline void set_base(const Pose<scalar_t>& base) noexcept { _data.base = base; }
    inline void set_effector(const Pose<scalar_t>& effector) noexcept { _data.effector = effector; }
    inline void set_config(const SerialManipulatorConfig& config) noexcept { _cfg = config; }
    inline void set_open_loop(const bool is_open_loop) noexcept { _cfg.is_open_loop = is_open_loop; }
    inline int set_joint_position(const std::vector<scalar_t>& joint_position) noexcept {
        if (joint_position.size() != _cfg.DOF) {
            return -1;
        }
        _data.joint_position = joint_position;
        _update_forward_kinematics();
        _cfg.flag |= 1;
        return 0;
    }
    inline int set_joint_velocity(const std::vector<scalar_t>& joint_velocity) noexcept {
        if (joint_velocity.size() != _cfg.DOF) {
            return -1;
        }
        _data.joint_velocity = joint_velocity;
        _cfg.flag |= 2;
        return 0;
    }
    inline int set_joint_effort(const std::vector<scalar_t>& joint_effort) noexcept {
        if (joint_effort.size() != _cfg.DOF) {
            return -1;
        }
        _data.joint_effort = joint_effort;
        _cfg.flag |= 4;
        return 0;
    }

    // getters
    inline std::vector<scalar_t> get_joint_position_lower_bound() const noexcept {return _data.joint_position_lower_bound;}
    inline std::vector<scalar_t> get_joint_position_upper_bound() const noexcept {return _data.joint_position_upper_bound;}
    inline std::vector<scalar_t> get_joint_velocity_lower_bound() const noexcept {return _data.joint_velocity_lower_bound;}
    inline std::vector<scalar_t> get_joint_velocity_upper_bound() const noexcept {return _data.joint_velocity_upper_bound;}
    inline std::vector<scalar_t> get_joint_effort_lower_bound() const noexcept {return _data.joint_effort_lower_bound;}
    inline std::vector<scalar_t> get_joint_effort_upper_bound() const noexcept {return _data.joint_effort_upper_bound;}
    inline std::vector<scalar_t> get_target_joint_position() const noexcept {return _data.target_joint_position;}
    inline std::vector<scalar_t> get_target_joint_velocity() const noexcept {return _data.target_joint_velocity;}
    inline std::vector<scalar_t> get_target_joint_effort() const noexcept {return _data.target_joint_effort;}
    inline std::vector<scalar_t> get_joint_position() const noexcept {return _data.joint_position;}
    inline std::vector<scalar_t> get_joint_velocity() const noexcept {return _data.joint_velocity;}
    inline std::vector<scalar_t> get_joint_effort() const noexcept {return _data.joint_effort;}
    inline Pose<scalar_t>            get_end_pose() const noexcept {return _data.joint_pose.back();}
    inline dof_size_t                get_dof() const noexcept {return _cfg.DOF;}
    inline SerialManipulatorConfig   get_config() const noexcept {return _cfg;}
    inline SerialManipulatorData   get_data() const noexcept {return _data;}
    inline std::string get_name() const noexcept {return _cfg.name;}
    inline std::string get_api_version() const noexcept {return _cfg.api_version;}
    inline std::string get_kind() const noexcept {return _cfg.kind;}
    inline bool is_initialized() const noexcept {return _cfg.is_initialized;}
private:
    void _update_forward_kinematics() noexcept {     
        if (_cfg.DOF == 0) return;
        if (_cfg.DOF == 1) {
            _data.joint_pose.front() = _data.base * _pjoints.front()->fkm(_data.joint_position.front()) * _data.effector;
            _data.joint_pose_derivative.front() = _data.base * _pjoints.front()->derivative(_data.joint_position.front()) 
                                                    * _data.joint_pose.front().conj() * _data.joint_pose.back() * _data.effector;
            return;
        }
        // UPDATE FORWARD KINEMATICS
        _data.joint_pose.front() = _data.base * _pjoints.front()->fkm(_data.joint_position.front());
        _data.joint_pose_derivative.front() = _data.base * _pjoints.front()->derivative(_data.joint_position.front()) 
                                                * _data.joint_pose.front().conj() * _data.joint_pose.back();

        for (dof_size_t i=1; i<_cfg.DOF-1; ++i) {
            _data.joint_pose[i] = _data.joint_pose[i-1] * _pjoints[i]->fkm(_data.joint_position[i]);
        }
        _data.joint_pose.back() = _data.joint_pose[_cfg.DOF-2] * _pjoints.back()->fkm(_data.joint_position.back()) * _data.effector;

        for (dof_size_t i=1; i<_cfg.DOF-1; ++i){
            _data.joint_pose_derivative[i] = _data.joint_pose[i-1] * _pjoints[i]->derivative(_data.joint_position[i]) 
                                                * _data.joint_pose[i].conj() * _data.joint_pose.back();
        }
        _data.joint_pose_derivative.back() = _data.joint_pose[_cfg.DOF-2] * _pjoints.back()->derivative(_data.joint_position.back()) * _data.effector;
    }
};

} // namespace kinematics

} // namespace timr