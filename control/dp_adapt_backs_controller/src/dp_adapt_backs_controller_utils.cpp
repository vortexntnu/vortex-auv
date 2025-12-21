#include "dp_adapt_backs_controller/dp_adapt_backs_controller_utils.hpp"
#include <spdlog/spdlog.h>
#include <vortex/utils/math.hpp>
#include <vortex/utils/types.hpp>
#include "dp_adapt_backs_controller/typedefs.hpp"

namespace vortex::control {

Eigen::Matrix6d calculate_J_inv(const vortex::utils::types::PoseEuler& pose) {
    Eigen::Matrix6d J = pose.as_j_matrix();

    constexpr double tolerance = 1e-8;

    if (std::abs(J.determinant()) < tolerance) {
        spdlog::error("J(pose) is singular");

        // Moore-Penrose pseudoinverse in case of near singular matrix, better
        // result for smaller singular values
        return J.completeOrthogonalDecomposition().pseudoInverse();
    }

    return J.inverse();
}

Eigen::Matrix3d calculate_R_dot(const vortex::utils::types::PoseEuler& pose,
                                const vortex::utils::types::Twist& twist) {
    return pose.as_rotation_matrix() *
           vortex::utils::math::get_skew_symmetric_matrix(
               twist.to_vector().tail(3));
}

Eigen::Matrix3d calculate_T_dot(const vortex::utils::types::PoseEuler& pose,
                                const vortex::utils::types::Twist& twist) {
    double cos_phi{std::cos(pose.roll)};
    double sin_phi{std::sin(pose.roll)};
    double cos_theta{std::cos(pose.pitch)};
    double sin_theta{std::sin(pose.pitch)};
    double tan_theta{sin_theta / cos_theta};
    double inv_cos2{1.0 / (cos_theta * cos_theta)};

    Eigen::Vector6d pose_dot = pose.as_j_matrix() * twist.to_vector();

    double phi_dot{pose_dot(3)};
    double theta_dot{pose_dot(4)};

    Eigen::Matrix3d dt_dphi;
    dt_dphi << 0.0, cos_phi * tan_theta * phi_dot,
        -sin_phi * tan_theta * phi_dot, 0.0, -sin_phi * phi_dot,
        -cos_phi * phi_dot, 0.0, (cos_phi * phi_dot) / cos_theta,
        (-sin_phi * phi_dot) / cos_theta;

    Eigen::Matrix3d dt_dtheta;
    dt_dtheta << 0.0, sin_phi * inv_cos2 * theta_dot,
        cos_phi * inv_cos2 * theta_dot, 0.0, 0.0, 0.0, 0.0,
        (sin_phi * sin_theta) * inv_cos2 * theta_dot,
        (cos_phi * sin_theta) * inv_cos2 * theta_dot;

    return dt_dphi + dt_dtheta;
}

Eigen::Matrix6d calculate_J_dot(const vortex::utils::types::PoseEuler& pose,
                                const vortex::utils::types::Twist& twist) {
    Eigen::Matrix3d R_dot = calculate_R_dot(pose, twist);
    Eigen::Matrix3d T_dot = calculate_T_dot(pose, twist);

    Eigen::Matrix6d J_dot = Eigen::Matrix6d::Zero();
    J_dot.topLeftCorner<3, 3>() = R_dot;
    J_dot.bottomRightCorner<3, 3>() = T_dot;

    return J_dot;
}

Eigen::Matrix6d calculate_coriolis(const double mass,
                                   const Eigen::Vector3d& r_b_bg,
                                   const vortex::utils::types::Twist& twist,
                                   const Eigen::Matrix3d& I_b) {
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
        get_skew_symmetric_matrix(I_b * angular_speed);

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
