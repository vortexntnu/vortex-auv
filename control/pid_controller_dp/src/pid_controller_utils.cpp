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
    types::Matrix3d T = calculate_T_quat(eta);

    types::Matrix6d J = types::Matrix6d::Zero();
    J.topLeftCorner<3, 3>() = R;
    J.bottomRightCorner<3, 3>() = T;

    return J.inverse();
}

types::Eta error_eta(const types::Eta& eta, const types::Eta& eta_d) {
    return eta - eta_d;
}

Eigen::VectorXd clamp_values(const Eigen::VectorXd& values,
                             double min_val,
                             double max_val) {
    return vortex::utils::math::clamp_values(values, min_val, max_val);
}

types::Vector6d anti_windup(const double dt,
                            const types::Eta& error,
                            const types::Vector6d& integral) {
    types::Vector6d error_6;
    error_6 << error.x, error.y, error.z, error.qx, error.qy, error.qz;
    return vortex::utils::math::anti_windup(dt, error_6, integral, -80.0, 80.0);
}
