#include "pid_controller_dp/pid_controller_utils.hpp"
#include <algorithm>
#include <vortex/utils/math.hpp>
#include <vortex/utils/types.hpp>
#include "pid_controller_dp/pid_controller_conversions.hpp"
#include "pid_controller_dp/typedefs.hpp"

types::Matrix3d calculate_R_quat(const types::Eta& eta) {
    return eta.as_rotation_matrix();
}

types::Matrix3d calculate_T_quat(const types::Eta& eta) {
    // Full T is 4x3; rows 1-3 map body angular velocity to d/dt[qx, qy, qz].
    return eta.as_transformation_matrix().bottomRows<3>();
}

types::Matrix6d calculate_J_sudo_inv(const types::Eta& eta) {
    types::Matrix3d R = calculate_R_quat(eta);
    types::Matrix6d J_inv = types::Matrix6d::Zero();
    J_inv.topLeftCorner<3, 3>() = R.transpose();
    J_inv.bottomRightCorner<3, 3>() = types::Matrix3d::Identity();
    return J_inv;
}

types::Eta error_eta(const types::Eta& eta, const types::Eta& eta_d) {
    types::Eta error = eta - eta_d;
    // Enforce shortest path: q and -q represent the same rotation, but only
    // qw >= 0 gives the correct sign for the vector part used as error signal.
    if (error.qw < 0.0) {
        error.qw = -error.qw;
        error.qx = -error.qx;
        error.qy = -error.qy;
        error.qz = -error.qz;
    }
    return error;
}

Eigen::VectorXd clamp_values(const Eigen::VectorXd& values,
                             double min_val,
                             double max_val) {
    return vortex::utils::math::clamp_values(values, min_val, max_val);
}

types::Vector6d anti_windup(const double dt,
                            const types::Vector6d& error_body,
                            const types::Vector6d& integral) {
    return vortex::utils::math::anti_windup(dt, error_body, integral, -50.0,
                                            50.0);
}
