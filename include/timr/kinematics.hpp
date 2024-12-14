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

/**
 *     \file include/timr/kinematics.hpp
 *	   \author Jiawei ZHAO
 *	   \version 1.0
 *	   \date 2023-2024
 */

#pragma once
#include "qpOASES.hpp"
#include "dqpose.hpp"
#include "nlohmann/json.hpp"
#include <memory>
#include <vector>
#include <array>
#include <fstream>

namespace timr
{

template<typename jScalar, typename = std::enable_if_t<std::is_floating_point_v<jScalar>>>
class Joint;
template<typename jScalar, typename = std::enable_if_t<std::is_floating_point_v<jScalar>>>
class DHJoint;
template<typename jScalar, typename = std::enable_if_t<std::is_floating_point_v<jScalar>>>
class RevoluteJoint;
template<typename jScalar, typename = std::enable_if_t<std::is_floating_point_v<jScalar>>>
class PrimaticJoint;
template<typename jScalar, std::size_t dof, typename = std::enable_if_t<std::is_floating_point_v<jScalar>>>
class SerialManipulator;

template<typename Scalar>
inline Quat<Scalar> __closest_invariant_rotation_error(const Rotation<Scalar>& r, const Rotation<Scalar>& rd) noexcept {
    Quat<Scalar> er_plus = r.conj() * rd - Quat(1);
    Quat<Scalar> er_minus = r.conj() * rd + Quat(1);

    if (er_plus.norm() < er_minus.norm()) {
        return er_plus;
    } else {
        return er_minus;
    }
}

template<typename jScalar, typename>
class Joint {
public:
    Joint() noexcept {}
    virtual inline Pose<jScalar> fkm(const jScalar position) const noexcept = 0;
    virtual inline DualQuat<jScalar> derivative(const jScalar position) const noexcept = 0;

    virtual ~Joint() = default;
            Joint(const Joint&) = default;
            Joint(Joint&&) = default;
    Joint& operator=(const Joint&) = default;
    Joint& operator=(Joint&&) = default;

};


template<typename jScalar, typename>
class RevoluteJoint : public Joint<jScalar> {
protected:
    std::array<jScalar, 4> _dhparams; 
public:
    explicit RevoluteJoint(const std::array<jScalar, 4>& DH_parameters) noexcept
        : _dhparams(DH_parameters) {}
    
    virtual inline Pose<jScalar> fkm(const jScalar position) const noexcept override {
        const jScalar cos_theta = cos( 0.5 * ( theta() + position) );
        const jScalar sin_theta = sin( 0.5 * ( theta() + position) );
        const jScalar cos_alpha = cos( 0.5 * alpha() );
        const jScalar sin_alpha = sin( 0.5 * alpha() );
        return Pose<jScalar> (
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
    virtual inline DualQuat<jScalar> derivative(const jScalar position) const noexcept override {
        const jScalar cos_theta_dot = -0.5 * sin( 0.5 * ( theta() + position));
        const jScalar sin_theta_dot = 0.5 * cos( 0.5 * ( theta() + position));
        const jScalar cos_alpha = cos( 0.5 * alpha() );
        const jScalar sin_alpha = sin( 0.5 * alpha() );
        return DualQuat<jScalar> (
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

    inline jScalar theta() const noexcept { return _dhparams[0];}
    inline jScalar d() const noexcept { return _dhparams[1];}
    inline jScalar a() const noexcept { return _dhparams[2];}
    inline jScalar alpha() const noexcept { return _dhparams[3];}

    RevoluteJoint() = delete;
    virtual ~RevoluteJoint() = default;
            RevoluteJoint(const RevoluteJoint&) = default;
            RevoluteJoint(RevoluteJoint&&) = default;
    RevoluteJoint& operator=(const RevoluteJoint&) = default;
    RevoluteJoint& operator=(RevoluteJoint&&) = default;
};

template<typename jScalar, typename>
class PrismaticJoint : public Joint<jScalar> {
protected:
    std::array<jScalar, 4> _dhparams; 
public:
    explicit PrismaticJoint(const std::array<jScalar, 4>& DH_parameters) noexcept
        : _dhparams(DH_parameters) {}
    
    virtual inline Pose<jScalar> fkm(const jScalar position) const noexcept override {
        jScalar cos_theta = cos( 0.5 * theta() );
        jScalar sin_theta = sin( 0.5 * theta() );
        jScalar cos_alpha = cos( 0.5 * alpha() );
        jScalar sin_alpha = sin( 0.5 * alpha() );
        jScalar d_ = d() + position;
        jScalar a_ = a();
        return Pose<jScalar> (
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
    virtual inline DualQuat<jScalar> derivative(const jScalar position) const noexcept override {
        jScalar cos_theta = sin(0.5 * theta());
        jScalar sin_theta = cos(0.5 * theta());
        jScalar cos_alpha = cos(0.5 * alpha());
        jScalar sin_alpha = sin(0.5 * alpha());
        return DualQuat<jScalar> (
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

    inline jScalar theta() const noexcept { return _dhparams[0];}
    inline jScalar d() const noexcept { return _dhparams[1];}
    inline jScalar a() const noexcept { return _dhparams[2];}
    inline jScalar alpha() const noexcept { return _dhparams[3];}

    PrismaticJoint() = delete;
    virtual ~PrismaticJoint() = default;
            PrismaticJoint(const PrismaticJoint&) = default;
            PrismaticJoint(PrismaticJoint&&) = default;
    PrismaticJoint& operator=(const PrismaticJoint&) = default;
    PrismaticJoint& operator=(PrismaticJoint&&) = default;
};

template<typename jScalar>
struct SerialManipulatorConfig {
    jScalar translation_priority{0.9999};
    jScalar error_gain{50};
    jScalar sampling_time_sec{0.0004};
    jScalar joint_damping{0.0001};
};

template<typename jScalar, std::size_t dof>
struct SerialManipulatorData {
    Pose<jScalar> base;
    Pose<jScalar> effector;
    std::array<jScalar, dof> joint_positions{};
    std::array<jScalar, dof> joint_velocities{};
    std::array<jScalar, dof> joint_accelerations{};
    std::array<Pose<jScalar>, dof> joint_poses; // {joint1->joint2, joint2->joint3, joint3->joint4, ... , joint?->end}
    std::array<DualQuat<jScalar>, dof> joint_pose_derivatives; // {joint1->joint2, joint2->joint3, joint3->joint4, ... , joint?->end}
    std::array<std::array<jScalar, dof>, 4> joint_limits;
};

template<typename jScalar, std::size_t dof, typename>
class SerialManipulator {
protected:
    SerialManipulatorConfig<jScalar> _cfg;
    std::array<std::unique_ptr<Joint<jScalar, void>>, dof> _pjoints;
    SerialManipulatorData<jScalar, dof> _data;

    void _construct(const std::array<std::array<jScalar, dof>, 5>& dhparams, 
                        const std::array<std::array<jScalar, dof>, 4>& limits, 
                        const std::array<jScalar, dof>& joint_positions)
    {
        // Initialize joints
        for (int i=0; i<dof; ++i){
            if (dhparams.back()[i] == 0){
                const std::array<jScalar, 4> dharray {dhparams[0][i], dhparams[1][i], dhparams[2][i], dhparams[3][i]};
                _pjoints[i].reset( static_cast<Joint<jScalar, void>*>(new RevoluteJoint<jScalar, void>(dharray)) );
            } else if (dhparams.back()[i] == 1){
                const std::array<jScalar, 4> dharray {dhparams[0][i], dhparams[1][i], dhparams[2][i], dhparams[3][i]};
                _pjoints[i].reset( static_cast<Joint<jScalar, void>*>(new PrismaticJoint<jScalar, void>(dharray)) );
            } else {
                throw std::runtime_error("Error: SerialManipulator Constructor arg0 dhparams[4][" + std::to_string(i) + "] = " + std::to_string(dhparams[4][i]) + ", this value should be 0 or 1.(0: Revolute joint, 1: Prismatic joint)\n");
            }
        }

        _data.joint_limits = limits;
        _data.joint_positions = joint_positions;
        _update_kinematics();

        std::cout << "Constructed a " + std::to_string(joint_positions.size()) + "-DoF timr::SerialManipulator.\n" ;
    }
public:
    SerialManipulator() = delete;
    ~SerialManipulator() = default;
    SerialManipulator(const SerialManipulator& other) = default;
    SerialManipulator(SerialManipulator&& other) = default;
    SerialManipulator& operator=(const SerialManipulator& other) = default;
    SerialManipulator& operator=(SerialManipulator&& other) = default;

    explicit SerialManipulator(const std::string& json_path, const std::array<jScalar, dof>& joint_positions)
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

        // Accessing JSON data
        try {
            std::array<std::array<jScalar, dof>, 5> dh_params {
                data["DH_params"]["theta"],
                data["DH_params"]["d"],
                data["DH_params"]["a"],
                data["DH_params"]["alpha"],
                data["DH_params"]["joint_types"]
            };

            std::array<std::array<jScalar, dof>, 4> limits {
                data["joint_limits"]["min_joint_positions"],
                data["joint_limits"]["max_joint_positions"],
                data["joint_limits"]["min_joint_velocities"],
                data["joint_limits"]["max_joint_velocities"]
            };

            for (int i=0; i<6; ++i) {{
                dh_params[0][i] = dh_params[0][i] / 180. * M_PI;
                dh_params[3][i] = dh_params[3][i] / 180. * M_PI;
                limits[0][i] = limits[0][i] / 180. * M_PI;
                limits[1][i] = limits[1][i] / 180. * M_PI;
                limits[2][i] = limits[2][i] / 180. * M_PI;
                limits[3][i] = limits[3][i] / 180. * M_PI;
            }}

            SerialManipulatorConfig<jScalar> cfg;
            cfg.translation_priority = data["solver_config"]["translation_priority"];
            cfg.error_gain = data["solver_config"]["error_gain"];
            cfg.joint_damping = data["solver_config"]["joint_damping"];
            cfg.sampling_time_sec = data["solver_config"]["sampling_time_sec"];

            _construct(dh_params, limits, joint_positions);
            _cfg = cfg;
        } catch (json::exception& e) {
            std::cerr << "Error accessing JSON data: " << e.what() << std::endl;
        }
    }

    explicit SerialManipulator(const std::array<std::array<jScalar, dof>, 5>& dhparams, const std::array<std::array<jScalar, dof>, 4>& limits, const std::array<jScalar, dof>& joint_positions)
    {
        _construct(dhparams, limits, joint_positions);
    }

    void update(const Pose<jScalar>& desired_pose) {
        USING_NAMESPACE_QPOASES;
        const Rotation<jScalar>& r_end = _data.joint_poses.back().rotation();
        const Translation<jScalar>& t_end = _data.joint_poses.back().translation();
        const Rotation<jScalar>& rd = desired_pose.rotation();
        const Translation<jScalar>& td = desired_pose.translation();

        std::array<Quat<jScalar>, dof> r_rd_derivatives;
        std::array<Quat<jScalar>, dof> t_derivatives;
        for (int i=0; i<dof; ++i) {
            r_rd_derivatives[i] = _data.joint_pose_derivatives[i].real().conj() * rd;
            t_derivatives[i] = 2 * (_data.joint_pose_derivatives[i].dual() * _data.joint_poses.back().real().conj() 
                                    + _data.joint_poses.back().dual() * _data.joint_pose_derivatives[i].real().conj() );
        }

        constexpr std::size_t dof2 = dof * dof;
        // Initialize a zero array
        std::array<double, dof2> H {};
        for (int i=0; i<dof; ++i) {
            for (int j=0; j<dof; ++j) {
                const double Ht = t_derivatives[i].dot( t_derivatives[j] );
                const double Hr = r_rd_derivatives[i].dot( r_rd_derivatives[j] );
                H[i*dof+j] = _cfg.translation_priority * Ht + (1-_cfg.translation_priority) * Hr;
                if (i==j)
                    H[i*dof+j] += _cfg.joint_damping;
                
            }
        }

        const Quat<jScalar> t_err = t_end - td;
        const Quat<jScalar> r_rd_err = __closest_invariant_rotation_error(r_end, rd);
        // Initialize a zero array
        std::array<double, dof> g {};
        for (int i=0; i<dof; ++i) {
            const double ct = _cfg.error_gain * t_err.dot(t_derivatives[i]);
            const double cr = _cfg.error_gain * r_rd_err.dot(r_rd_derivatives[i]);
            g[i] = _cfg.translation_priority * ct + (1-_cfg.translation_priority) * cr;
        }

        // Initialize a zero array
        std::array<double, dof2> constraints {};
        for (int i=0; i<dof; ++i) {
            for (int j=0; j<dof; ++j) {
                if (i==j)
                    constraints[i*dof+j] = 1;
            }
        }

        const std::array<jScalar, dof>& min_joint_positions = _data.joint_limits[0];
        const std::array<jScalar, dof>& max_joint_positions = _data.joint_limits[1];
        const std::array<jScalar, dof>& min_joint_velocities = _data.joint_limits[2];
        const std::array<jScalar, dof>& max_joint_velocities = _data.joint_limits[3];
        std::array<double, dof> lower_constraint_bound;
        std::array<double, dof> upper_constraint_bound;
        std::array<double, dof> lower_bound;
        std::array<double, dof> upper_bound;
        for (int i=0; i<dof; ++i) {
            lower_constraint_bound[i] = min_joint_positions[i] - _data.joint_positions[i];
            upper_constraint_bound[i] = max_joint_positions[i] - _data.joint_positions[i];
            lower_bound[i] = min_joint_velocities[i];
            upper_bound[i] = max_joint_velocities[i];
        }

        const double* H_raw = H.data();
        const double* g_raw = g.data();
        const double* A_raw = constraints.data();
        const double* lb_A_raw = lower_constraint_bound.data();
        const double* ub_A_raw = upper_constraint_bound.data();
        const double* lb_raw = lower_bound.data();
        const double* ub_raw = upper_bound.data();

        static SQProblem qp(dof, dof);
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

        double xOpt[dof];
        qp.getPrimalSolution(xOpt);
        // update joint positions
        for (int i=0; i<dof; ++i) {
            _data.joint_accelerations[i] = (xOpt[i] - _data.joint_velocities[i]) / _cfg.sampling_time_sec;
            _data.joint_velocities[i] = xOpt[i];
            _data.joint_positions[i] += xOpt[i] * _cfg.sampling_time_sec;
        }
        _update_kinematics();
    }

    template<typename Scalar>
    inline void set_base(const Pose<Scalar>& base) noexcept { _data.base = base; }
    template<typename Scalar>
    inline void set_effector(const Pose<Scalar>& effector) noexcept { _data.effector = effector; }
    template<typename Scalar>
    inline void set_config(const SerialManipulatorConfig<Scalar>& config) noexcept { _cfg = config; }
    // query
    inline std::array<jScalar, dof> joint_positions() const noexcept {return _data.joint_positions;}
    inline Pose<jScalar> end_pose() const noexcept {return _data.joint_poses.back();}
    inline std::size_t DoF() const noexcept {return dof;}
    inline SerialManipulatorConfig<jScalar>& config() noexcept {return _cfg;}
    inline const SerialManipulatorConfig<jScalar>& config() const noexcept {return _cfg;}
    inline const SerialManipulatorData<jScalar, dof>& data() const noexcept {return _data;}

private:
    void _update_kinematics() {
        _data.joint_poses.front() = _data.base * _pjoints.front()->fkm(_data.joint_positions.front());
        for (int i=1; i<dof-1; ++i) {
            _data.joint_poses[i] = _data.joint_poses[i-1] * _pjoints[i]->fkm(_data.joint_positions[i]);
        }
        _data.joint_poses.back() = _data.joint_poses[dof-2] * _pjoints.back()->fkm(_data.joint_positions.back()) * _data.effector;

        _data.joint_pose_derivatives.front() = _data.base * _pjoints.front()->derivative(_data.joint_positions.front()) 
                                                * _data.joint_poses.front().conj() * _data.joint_poses.back();
        for (int i=1; i<dof-1; ++i){
            _data.joint_pose_derivatives[i] = _data.joint_poses[i-1] * _pjoints[i]->derivative(_data.joint_positions[i]) 
                                                * _data.joint_poses[i].conj() * _data.joint_poses.back();
        }
        _data.joint_pose_derivatives.back() = _data.joint_poses[dof-2] * _pjoints.back()->derivative(_data.joint_positions.back()) * _data.effector;
    }
};

}
