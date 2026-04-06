#include "dp_adapt_backs_controller_quat/dp_adapt_backs_controller_utils.hpp"
#include <spdlog/spdlog.h>
#include <cmath>
#include <vortex/utils/math.hpp>
#include <vortex/utils/types.hpp>
#include "dp_adapt_backs_controller_quat/typedefs.hpp"

namespace vortex::control {

Eigen::Matrix3d calculate_R_dot(const vortex::utils::types::Pose& pose,
                                const vortex::utils::types::Twist& twist) {
    return pose.as_rotation_matrix() *
           vortex::utils::math::get_skew_symmetric_matrix(
               twist.to_vector().tail(3));
}

Eigen::Matrix3d calculate_Q_e(const Eigen::Vector3d& eps_e, const double qw_e) {
    return qw_e * Eigen::Matrix3d::Identity() +
           vortex::utils::math::get_skew_symmetric_matrix(eps_e);
}

Eigen::Matrix6d calculate_L(const Eigen::Matrix3d& R,
                            const Eigen::Matrix3d& Q_e) {
    Eigen::Matrix6d L = Eigen::Matrix6d::Zero();
    L.topLeftCorner<3, 3>() = R;
    L.bottomRightCorner<3, 3>() = Q_e;
    return L;
}

Eigen::Matrix6d calculate_L_inv(const Eigen::Matrix6d& L,
                                const double tolerance) {
    if (std::abs(L.determinant()) < tolerance) {
        spdlog::error("L is singular");
        return L.completeOrthogonalDecomposition().pseudoInverse();
    }
    return L.inverse();
}

Eigen::Matrix3d calculate_Q_e_dot(const Eigen::Vector3d& eps_e,
                                  const Eigen::Matrix3d& Q_e,
                                  const Eigen::Vector3d& omega) {
    return (-0.5 * eps_e.dot(omega)) * Eigen::Matrix3d::Identity() +
           vortex::utils::math::get_skew_symmetric_matrix(0.5 * Q_e * omega);
}

Eigen::Matrix6d calculate_L_dot(const Eigen::Matrix3d& R_dot,
                                const Eigen::Matrix3d& Q_e_dot) {
    Eigen::Matrix6d L_dot = Eigen::Matrix6d::Zero();
    L_dot.topLeftCorner<3, 3>() = R_dot;
    L_dot.bottomRightCorner<3, 3>() = Q_e_dot;
    return L_dot;
}

Eigen::Matrix6d calculate_coriolis(const double mass,
                                   const Eigen::Vector3d& r_b_bg,
                                   const vortex::utils::types::Twist& twist,
                                   const Eigen::Matrix3d& inertia_matrix_body) {
    using vortex::utils::math::get_skew_symmetric_matrix;
    const Eigen::Vector3d linear_speed = twist.to_vector().head(3);
    const Eigen::Vector3d angular_speed = twist.to_vector().tail(3);
    Eigen::Matrix6d C;
    C.topLeftCorner<3, 3>() =
        mass * vortex::utils::math::get_skew_symmetric_matrix(linear_speed);
    C.topRightCorner<3, 3>() = -mass *
                               get_skew_symmetric_matrix(angular_speed) *
                               get_skew_symmetric_matrix(r_b_bg);
    C.bottomLeftCorner<3, 3>() = mass *
                                 get_skew_symmetric_matrix(angular_speed) *
                                 get_skew_symmetric_matrix(r_b_bg);
    C.bottomRightCorner<3, 3>() =
        get_skew_symmetric_matrix(inertia_matrix_body * angular_speed);

    return C;
}

Eigen::Matrix6x12d calculate_Y_v(const vortex::utils::types::Twist& twist) {
    Eigen::Matrix6x12d Y_v;
    Y_v.setZero();

    Y_v(0, 0) = twist.u;
    Y_v(0, 1) = twist.u * std::abs(twist.u);

    Y_v(1, 2) = twist.v;
    Y_v(1, 3) = twist.v * std::abs(twist.v);

    Y_v(2, 4) = twist.w;
    Y_v(2, 5) = twist.w * std::abs(twist.w);

    Y_v(3, 6) = twist.p;
    Y_v(3, 7) = twist.p * std::abs(twist.p);

    Y_v(4, 8) = twist.q;
    Y_v(4, 9) = twist.q * std::abs(twist.q);

    Y_v(5, 10) = twist.r;
    Y_v(5, 11) = twist.r * std::abs(twist.r);

    return Y_v;
}

}  // namespace vortex::control
