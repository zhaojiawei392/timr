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
#include <nlohmann/json.hpp>
#include <memory>
#include <vector>
#include <array>
#include <fstream>

using dof_size_t = uint8_t;
using scalar_t = double;
constexpr dof_size_t DOF = 6;

namespace kinematics
{
using namespace timr::dqpose;

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
    explicit RevoluteJoint(const std::array<scalar_t, 4>& DH_parameters) noexcept
        : _dhparams(DH_parameters) {}
    
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
    explicit PrismaticJoint(const std::array<scalar_t, 4>& DH_parameters) noexcept
        : _dhparams(DH_parameters) {}
    
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
    scalar_t error_gain{50};
    scalar_t sampling_time_sec{0.0004};
    scalar_t joint_damping{0.0001};
    bool is_open_loop{false};
};

struct SerialManipulatorData {
    Pose<scalar_t> base;
    Pose<scalar_t> effector;
    std::array<scalar_t, DOF> target_joint_position{};
    std::array<scalar_t, DOF> target_joint_velocity{};
    std::array<scalar_t, DOF> target_joint_effort{};
    std::array<scalar_t, DOF> joint_position{};
    std::array<scalar_t, DOF> joint_velocity{};
    std::array<scalar_t, DOF> joint_effort{};

    std::array<scalar_t, DOF> joint_position_limit_lower{};
    std::array<scalar_t, DOF> joint_position_limit_upper{};
    std::array<scalar_t, DOF> joint_velocity_limit_lower{};
    std::array<scalar_t, DOF> joint_velocity_limit_upper{};
    std::array<scalar_t, DOF> joint_effort_limit_lower{};
    std::array<scalar_t, DOF> joint_effort_limit_upper{};

    std::array<Pose<scalar_t>, DOF> joint_poses; // {joint1->joint2, joint2->joint3, joint3->joint4, ... , joint?->end}
    std::array<DualQuat<scalar_t>, DOF> joint_pose_derivatives; // {joint1->joint2, joint2->joint3, joint3->joint4, ... , joint?->end}

};

class SerialManipulator {
protected:
    SerialManipulatorConfig _cfg;
    std::array<std::unique_ptr<Joint>, DOF> _pjoints;
    SerialManipulatorData _data;

    void _construct(const std::array<std::array<scalar_t, DOF>, 5>& dhparams, 
                        const std::array<std::array<scalar_t, DOF>, 6>& limits, 
                        const std::array<scalar_t, DOF>& joint_position)
    {
        // Initialize joints
        for (dof_size_t i=0; i<DOF; ++i){
            if (dhparams.back()[i] == 0){
                const std::array<scalar_t, 4> dharray {dhparams[0][i], dhparams[1][i], dhparams[2][i], dhparams[3][i]};
                _pjoints[i].reset( static_cast<Joint*>(new RevoluteJoint(dharray)) );
            } else if (dhparams.back()[i] == 1){
                const std::array<scalar_t, 4> dharray {dhparams[0][i], dhparams[1][i], dhparams[2][i], dhparams[3][i]};
                _pjoints[i].reset( static_cast<Joint*>(new PrismaticJoint(dharray)) );
            } else {
                throw std::runtime_error("Error: SerialManipulator Constructor arg0 dhparams[4][" + std::to_string(i) + "] = " + std::to_string(dhparams[4][i]) + ", this value should be 0 or 1.(0: Revolute joint, 1: Prismatic joint)\n");
            }
        }

        _data.joint_position_limit_lower = limits[0];
        _data.joint_position_limit_upper = limits[1];
        _data.joint_velocity_limit_lower = limits[2];
        _data.joint_velocity_limit_upper = limits[3];
        _data.joint_effort_limit_lower = limits[4];
        _data.joint_effort_limit_upper = limits[5];
        _data.joint_position = joint_position;
        _update_kinematics();

        std::cout << "Constructed a " + std::to_string(joint_position.size()) + "-DoF timr::SerialManipulator.\n" ;
    }
public:
    SerialManipulator() = delete;
    ~SerialManipulator() = default;
    SerialManipulator(const SerialManipulator& other) = default;
    SerialManipulator(SerialManipulator&& other) = default;
    SerialManipulator& operator=(const SerialManipulator& other) = default;
    SerialManipulator& operator=(SerialManipulator&& other) = default;

    explicit SerialManipulator(const std::string& json_path, const std::array<scalar_t, DOF>& joint_position)
    {
        using json = nlohmann::json;
        // Open the JSON file
        std::ifstream file(json_path);

        // Check if the file opened successfully
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open the file: " + json_path);
        }

        // Parse the JSON file
        json data;
        try {
            file >> data;
        } catch (json::parse_error& e) {
            throw std::runtime_error("Parse error: " + std::string(e.what()));
        }
        const auto& spec = data["spec"];
        // Accessing JSON data
        try {
            std::array<std::array<scalar_t, DOF>, 5> dh_params {
                spec["DH_params"]["theta"],
                spec["DH_params"]["d"],
                spec["DH_params"]["a"],
                spec["DH_params"]["alpha"],
                spec["DH_params"]["joint_types"]
            };

            std::array<std::array<scalar_t, DOF>, 6> limits {
                spec["joint_limits"]["min_joint_position"],
                spec["joint_limits"]["max_joint_position"],
                spec["joint_limits"]["min_joint_velocity"],
                spec["joint_limits"]["max_joint_velocity"],
                spec["joint_limits"]["min_joint_effort"],
                spec["joint_limits"]["max_joint_effort"]
            };

            for (dof_size_t i=0; i<DOF; ++i) {{
                dh_params[0][i] = dh_params[0][i] / 180. * M_PI;
                dh_params[3][i] = dh_params[3][i] / 180. * M_PI;
                limits[0][i] = limits[0][i] / 180. * M_PI;
                limits[1][i] = limits[1][i] / 180. * M_PI;
                limits[2][i] = limits[2][i] / 180. * M_PI;
                limits[3][i] = limits[3][i] / 180. * M_PI;
                limits[4][i] = limits[4][i] / 180. * M_PI;
                limits[5][i] = limits[5][i] / 180. * M_PI;
            }}

            SerialManipulatorConfig cfg;
            cfg.translation_priority = spec["solver_config"]["translation_priority"];
            cfg.error_gain = spec["solver_config"]["error_gain"];
            cfg.joint_damping = spec["solver_config"]["joint_damping"];
            cfg.sampling_time_sec = spec["solver_config"]["sampling_time_sec"];
            cfg.is_open_loop = spec["solver_config"]["is_open_loop"];
            _construct(dh_params, limits, joint_position);
            _cfg = cfg;
        } catch (json::exception& e) {
            std::cerr << "Error accessing JSON data: " << e.what() << std::endl;
        }
    }

    explicit SerialManipulator(const std::array<std::array<scalar_t, DOF>, 5>& dhparams, const std::array<std::array<scalar_t, DOF>, 6>& limits, const std::array<scalar_t, DOF>& joint_position)
    {
        _construct(dhparams, limits, joint_position);
    }

    void update(const Pose<scalar_t>& desired_pose) {
        _update_kinematics();
        USING_NAMESPACE_QPOASES;
        const Rotation<scalar_t>& r_end = _data.joint_poses.back().rotation();
        const Translation<scalar_t>& t_end = _data.joint_poses.back().translation();
        const Rotation<scalar_t>& rd = desired_pose.rotation();
        const Translation<scalar_t>& td = desired_pose.translation();

        std::array<Quat<scalar_t>, DOF> r_rd_derivatives;
        std::array<Quat<scalar_t>, DOF> t_derivatives;
        for (dof_size_t i=0; i<DOF; ++i) {
            r_rd_derivatives[i] = _data.joint_pose_derivatives[i].real().conj() * rd;
            t_derivatives[i] = 2 * (_data.joint_pose_derivatives[i].dual() * _data.joint_poses.back().real().conj() 
                                    + _data.joint_poses.back().dual() * _data.joint_pose_derivatives[i].real().conj() );
        }

        constexpr dof_size_t dof2 = DOF * DOF;
        // Initialize a zero array
        std::array<double, dof2> H {};
        for (dof_size_t i=0; i<DOF; ++i) {
            for (dof_size_t j=0; j<DOF; ++j) {
                const double Ht = t_derivatives[i].dot( t_derivatives[j] );
                const double Hr = r_rd_derivatives[i].dot( r_rd_derivatives[j] );
                H[i*DOF+j] = _cfg.translation_priority * Ht + (1-_cfg.translation_priority) * Hr;
                if (i==j)
                    H[i*DOF+j] += _cfg.joint_damping;
                
            }
        }

        const Quat<scalar_t> t_err = t_end - td;
        const Quat<scalar_t> r_rd_err = __closest_invariant_rotation_error(r_end, rd);
        // Initialize a zero array
        std::array<double, DOF> g {};
        for (dof_size_t i=0; i<DOF; ++i) {
            const double ct = _cfg.error_gain * t_err.dot(t_derivatives[i]);
            const double cr = _cfg.error_gain * r_rd_err.dot(r_rd_derivatives[i]);
            g[i] = _cfg.translation_priority * ct + (1-_cfg.translation_priority) * cr;
        }

        // Initialize a zero array
        std::array<double, dof2> constraints {};
        for (dof_size_t i=0; i<DOF; ++i) {
            for (dof_size_t j=0; j<DOF; ++j) {
                if (i==j)
                    constraints[i*DOF+j] = 1;
            }
        }

        std::array<double, DOF> lower_constraint_bound;
        std::array<double, DOF> upper_constraint_bound;
        std::array<double, DOF> lower_bound;
        std::array<double, DOF> upper_bound;
        for (dof_size_t i=0; i<DOF; ++i) {
            lower_constraint_bound[i] = _data.joint_position_limit_lower[i] - _data.joint_position[i];
            upper_constraint_bound[i] = _data.joint_position_limit_upper[i] - _data.joint_position[i];
            lower_bound[i] = _data.joint_velocity_limit_lower[i];
            upper_bound[i] = _data.joint_velocity_limit_upper[i];
        }

        const double* H_raw = H.data();
        const double* g_raw = g.data();
        const double* A_raw = constraints.data();
        const double* lb_A_raw = lower_constraint_bound.data();
        const double* ub_A_raw = upper_constraint_bound.data();
        const double* lb_raw = lower_bound.data();
        const double* ub_raw = upper_bound.data();

        static SQProblem qp(DOF, DOF);
        static Options options;
        static int_t nWSR = 500;

        bool first_time(true);
        if (first_time){
            options.printLevel = PL_LOW;
            qp.setOptions(options);
            auto nWSR_in_use = nWSR;
            returnValue status = qp.init(H_raw, g_raw, A_raw, lb_raw, ub_raw, lb_A_raw, ub_A_raw, nWSR_in_use); 
            if (status != SUCCESSFUL_RETURN){
                throw std::runtime_error("Failed to solve QP problem.\n");
            }
            first_time = false;
        }else{
            auto nWSR_in_use = nWSR;
            returnValue status = qp.hotstart(H_raw, g_raw, A_raw, lb_raw, ub_raw, lb_A_raw, ub_A_raw, nWSR_in_use);
            if (status != SUCCESSFUL_RETURN){
                throw std::runtime_error("Failed to solve QP problem.\n");
            }
        }

        double xOpt[DOF];
        qp.getPrimalSolution(xOpt);
        // update joint position
        for (dof_size_t i=0; i<DOF; ++i) {
            _data.target_joint_effort[i] = 1;
            _data.target_joint_velocity[i] = xOpt[i];
            _data.target_joint_velocity[i] = std::clamp(_data.target_joint_velocity[i],
                _data.joint_velocity_limit_lower[i],
                _data.joint_velocity_limit_upper[i]);
            _data.target_joint_position[i] += xOpt[i] * _cfg.sampling_time_sec;
            _data.target_joint_position[i] = std::clamp(_data.target_joint_position[i],
                _data.joint_position_limit_lower[i],
                _data.joint_position_limit_upper[i]);
        }
        if (_cfg.is_open_loop) {
            set_joint_position(_data.target_joint_position);
            set_joint_velocity(_data.target_joint_velocity);
            set_joint_effort(_data.target_joint_effort);
        }
    }

    // setters
    inline void set_base(const Pose<scalar_t>& base) noexcept { _data.base = base; }
    inline void set_effector(const Pose<scalar_t>& effector) noexcept { _data.effector = effector; }
    inline void set_config(const SerialManipulatorConfig& config) noexcept { _cfg = config; }
    inline void set_open_loop(const bool is_open_loop) noexcept { _cfg.is_open_loop = is_open_loop; }
    inline void set_joint_position(const std::array<scalar_t, DOF>& joint_position) noexcept {_data.joint_position = joint_position;}
    inline void set_joint_velocity(const std::array<scalar_t, DOF>& joint_velocity) noexcept {_data.joint_velocity = joint_velocity;}
    inline void set_joint_effort(const std::array<scalar_t, DOF>& joint_effort) noexcept {_data.joint_effort = joint_effort;}

    // getters
    inline const std::array<scalar_t, DOF>&              get_target_joint_position() const noexcept {return _data.target_joint_position;}
    inline const std::array<scalar_t, DOF>&              get_target_joint_velocity() const noexcept {return _data.target_joint_velocity;}
    inline const std::array<scalar_t, DOF>&              get_target_joint_effort() const noexcept {return _data.target_joint_effort;}
    inline const std::array<scalar_t, DOF>&              get_joint_position() const noexcept {return _data.joint_position;}
    inline const std::array<scalar_t, DOF>&              get_joint_velocity() const noexcept {return _data.joint_velocity;}
    inline const std::array<scalar_t, DOF>&              get_joint_effort() const noexcept {return _data.joint_effort;}
    inline const Pose<scalar_t>&                         get_end_pose() const noexcept {return _data.joint_poses.back();}
    inline dof_size_t                                           get_dof() const noexcept {return DOF;}
    inline const SerialManipulatorConfig&      get_config() const noexcept {return _cfg;}
    inline const SerialManipulatorData&   get_data() const noexcept {return _data;}

private:
    void _update_kinematics() {
        _data.joint_poses.front() = _data.base * _pjoints.front()->fkm(_data.joint_position.front());
        for (dof_size_t i=1; i<DOF-1; ++i) {
            _data.joint_poses[i] = _data.joint_poses[i-1] * _pjoints[i]->fkm(_data.joint_position[i]);
        }
        _data.joint_poses.back() = _data.joint_poses[DOF-2] * _pjoints.back()->fkm(_data.joint_position.back()) * _data.effector;

        _data.joint_pose_derivatives.front() = _data.base * _pjoints.front()->derivative(_data.joint_position.front()) 
                                                * _data.joint_poses.front().conj() * _data.joint_poses.back();
        for (dof_size_t i=1; i<DOF-1; ++i){
            _data.joint_pose_derivatives[i] = _data.joint_poses[i-1] * _pjoints[i]->derivative(_data.joint_position[i]) 
                                                * _data.joint_poses[i].conj() * _data.joint_poses.back();
        }
        _data.joint_pose_derivatives.back() = _data.joint_poses[DOF-2] * _pjoints.back()->derivative(_data.joint_position.back()) * _data.effector;
    }
};

}
