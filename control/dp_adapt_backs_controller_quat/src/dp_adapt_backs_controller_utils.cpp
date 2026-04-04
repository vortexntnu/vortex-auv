#include "dp_adapt_backs_controller_quat/dp_adapt_backs_controller_utils.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <spdlog/spdlog.h>
#include <cmath>
#include <vortex/utils/math.hpp>
#include <vortex/utils/types.hpp>
#include "dp_adapt_backs_controller_quat/typedefs.hpp"

namespace vortex::control {

Eigen::Matrix6d calculate_L_inv(const vortex::utils::types::Pose& pose) {
    Eigen::Matrix6d L = Eigen::Matrix6d::Zero();
    L.topLeftCorner<3, 3>() = pose.as_rotation_matrix();
    L.bottomRightCorner<3, 3>() = pose.as_transformation_matrix().bottomRows<3>();

    constexpr double tolerance = 1e-8;

    if (std::abs(L.determinant()) < tolerance) {
        spdlog::error("L is singular");

        // Moore-Penrose pseudoinverse in case of near singular matrix, better
        // result for smaller singular values
        return L.completeOrthogonalDecomposition().pseudoInverse();
    }

    return L.inverse();
}

Eigen::Matrix3d calculate_R_dot(const vortex::utils::types::Pose& pose,
                                const vortex::utils::types::Twist& twist) {
    return pose.as_rotation_matrix() *
           vortex::utils::math::get_skew_symmetric_matrix(
               twist.to_vector().tail(3));
}

Eigen::Matrix3d calculate_Q_dot(const vortex::utils::types::Pose& pose,
                                const vortex::utils::types::Twist& twist) {
    Eigen::Vector3d omega = twist.to_vector().tail(3);
    Eigen::Matrix3d Q_tilde = pose.as_transformation_matrix().bottomRows<3>();
    Eigen::Vector3d eps = pose.ori_quaternion().vec();
    Eigen::Matrix3d eta_dot_term = eps.dot(omega) * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d eps_dot_term =
        vortex::utils::math::get_skew_symmetric_matrix(Q_tilde * omega);
    return 0.5 * (eps_dot_term - eta_dot_term);
}

Eigen::Matrix6d calculate_L_dot(const vortex::utils::types::Pose& pose,
                                const vortex::utils::types::Twist& twist) {
    Eigen::Matrix3d R_dot = calculate_R_dot(pose, twist);
    Eigen::Matrix3d Q_dot = calculate_Q_dot(pose, twist);

    Eigen::Matrix6d L_dot = Eigen::Matrix6d::Zero();
    L_dot.topLeftCorner<3, 3>() = R_dot;
    L_dot.bottomRightCorner<3, 3>() = Q_dot;

    return L_dot;
}

Eigen::Matrix6d calculate_coriolis(const double mass,
                                   const Eigen::Vector3d& r_b_bg,
                                   const vortex::utils::types::Twist& twist,
                                   const Eigen::Matrix3d& inertia_matrix_body) {
    using vortex::utils::math::get_skew_symmetric_matrix;
    Eigen::Vector3d linear_speed = twist.to_vector().head(3);
    Eigen::Vector3d angular_speed = twist.to_vector().tail(3);
    Eigen::Matrix6d C;
    C.topLeftCorner<3, 3>() =
        mass * vortex::utils::math::get_skew_symmetric_matrix(linear_speed);
    C.topRightCorner<3, 3>() = -mass *
                               get_skew_symmetric_matrix(angular_speed) *
                               get_skew_symmetric_matrix(r_b_bg);
    C.bottomLeftCorner<3, 3>() = mass *
                                 get_skew_symmetric_matrix(angular_speed) *
                                 get_skew_symmetric_matrix(r_b_bg);
    ;
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
