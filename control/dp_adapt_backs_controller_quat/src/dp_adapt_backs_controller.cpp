#include "dp_adapt_backs_controller_quat/dp_adapt_backs_controller.hpp"
#include <eigen3/Eigen/Dense>
#include <spdlog/spdlog.h>
#include <vortex/utils/math.hpp>
#include <vortex/utils/types.hpp>
#include "dp_adapt_backs_controller_quat/dp_adapt_backs_controller_utils.hpp"
#include "dp_adapt_backs_controller_quat/typedefs.hpp"

namespace vortex::control {

using vortex::utils::types::Pose;
using vortex::utils::types::Twist;

DPAdaptBacksController::DPAdaptBacksController(
    const DPAdaptParams& dp_adapt_params)
    : K1_(dp_adapt_params.K1.asDiagonal().toDenseMatrix()),
      K2_(dp_adapt_params.K2.asDiagonal().toDenseMatrix()),
      r_b_bg_(dp_adapt_params.r_b_bg),
      adapt_gain_(dp_adapt_params.adapt_param.asDiagonal().toDenseMatrix()),
      d_gain_(dp_adapt_params.d_gain.asDiagonal().toDenseMatrix()),
      adapt_param_(Eigen::Vector12d::Zero()),
      d_est_(Eigen::Vector6d::Zero()),
      inertia_matrix_body_(
          dp_adapt_params.inertia_matrix_body.asDiagonal().toDenseMatrix()),
      mass_intertia_matrix_(dp_adapt_params.mass_intertia_matrix),
      m_(dp_adapt_params.mass),
      dt_(0.01) {}

Eigen::Vector6d DPAdaptBacksController::calculate_tau(const Pose& pose,
                                                      const Pose& pose_d,
                                                      const Twist& twist) {
    Eigen::Vector3d pos_error = pose.pos_vector() - pose_d.pos_vector();

    // Error quaternion q_e = q_d^{-1} * q (rotation from desired to current).
    // z_1_ori = 2*eps_e, so d/dt(z_1_ori) = (qw_e*I + S(eps_e)) * omega.
    // This requires building L with the error quaternion, not q_current,
    // otherwise the Lyapunov cross-terms don't cancel and orientation diverges.
    Eigen::Quaterniond q_e =
        pose_d.ori_quaternion().conjugate() * pose.ori_quaternion();
    if (q_e.w() < 0.0) q_e.coeffs() = -q_e.coeffs();
    const Eigen::Vector3d eps_e = q_e.vec();
    const double qw_e = q_e.w();
    const Eigen::Vector3d quat_error = 2.0 * eps_e;

    Eigen::Vector6d z_1;
    z_1 << pos_error, quat_error;

    // L: [R(q_current), 0; 0, qw_e*I + S(eps_e)]
    const Eigen::Matrix3d R = pose.as_rotation_matrix();
    const Eigen::Matrix3d Q_e =
        qw_e * Eigen::Matrix3d::Identity() +
        vortex::utils::math::get_skew_symmetric_matrix(eps_e);
    Eigen::Matrix6d L = Eigen::Matrix6d::Zero();
    L.topLeftCorner<3, 3>() = R;
    L.bottomRightCorner<3, 3>() = Q_e;

    // L_inv with singularity guard
    Eigen::Matrix6d L_inv;
    if (std::abs(L.determinant()) < 1e-8) {
        spdlog::error("L is singular");
        L_inv = L.completeOrthogonalDecomposition().pseudoInverse();
    } else {
        L_inv = L.inverse();
    }

    // L_dot: R_dot = R*S(omega), Q_e_dot via quaternion kinematics on q_e
    // qw_e_dot = -0.5 * eps_e^T * omega
    // eps_e_dot = 0.5 * Q_e * omega  (standard quat kinematics)
    const Eigen::Vector3d omega = twist.to_vector().tail<3>();
    const Eigen::Matrix3d Q_e_dot =
        (-0.5 * eps_e.dot(omega)) * Eigen::Matrix3d::Identity() +
        vortex::utils::math::get_skew_symmetric_matrix(0.5 * Q_e * omega);
    Eigen::Matrix6d L_dot = Eigen::Matrix6d::Zero();
    L_dot.topLeftCorner<3, 3>() = calculate_R_dot(pose, twist);
    L_dot.bottomRightCorner<3, 3>() = Q_e_dot;

    Eigen::Matrix6d C =
        calculate_coriolis(m_, r_b_bg_, twist, inertia_matrix_body_);
    Eigen::Vector6d alpha = -L_inv * K1_ * z_1;
    Eigen::Vector6d z_2 = twist.to_vector() - alpha;
    Eigen::Vector6d alpha_dot =
        ((L_inv * L_dot * L_inv) * K1_ * z_1) -
        (L_inv * K1_ * L * twist.to_vector());
    Eigen::Matrix6x12d Y_v = calculate_Y_v(twist);
    Eigen::Vector12d adapt_param_dot = adapt_gain_ * Y_v.transpose() * z_2;
    Eigen::Vector6d d_est_dot = d_gain_ * z_2;
    Eigen::Vector6d F_est = Y_v * adapt_param_;
    Eigen::Vector6d tau =
        (mass_intertia_matrix_ * alpha_dot) + (C * twist.to_vector()) -
        (L.transpose() * z_1) - (K2_ * z_2) - F_est - d_est_;

    // TODO: look at better ways to clamp tau w.r.t new thrusters and allocator
    tau = tau.cwiseMax(-100.0).cwiseMin(100.0);
    adapt_param_ += adapt_param_dot * dt_;
    d_est_ += d_est_dot * dt_;
    adapt_param_ = adapt_param_.cwiseMax(-10.0).cwiseMin(10.0);
    d_est_ = d_est_.cwiseMax(-10.0).cwiseMin(10.0);

    return tau;
}

void DPAdaptBacksController::reset_adap_param() {
    adapt_param_.setZero();
}

void DPAdaptBacksController::reset_d_est() {
    d_est_.setZero();
}

void DPAdaptBacksController::set_time_step(const double dt) {
    dt_ = dt;
}

}  // namespace vortex::control
