#include "dp_adapt_backs_controller_quat/dp_adapt_backs_controller.hpp"
#include <eigen3/Eigen/Dense>
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
      tau_max_(dp_adapt_params.tau_max),
      m_(dp_adapt_params.mass),
      time_step_s_(dp_adapt_params.time_step_s),
      singularity_tolerance_(dp_adapt_params.singularity_tolerance),
      adapt_param_max_(dp_adapt_params.adapt_param_max),
      d_est_max_(dp_adapt_params.d_est_max) {}

Eigen::Vector6d DPAdaptBacksController::calculate_tau(const Pose& pose,
                                                      const Pose& pose_d,
                                                      const Twist& twist) {
    Eigen::Vector3d pos_error = pose.pos_vector() - pose_d.pos_vector();

    // Error quaternion q_e = q_d^{-1} * q (rotation from desired to current).
    // z_1_ori = 2*eps_e, so d/dt(z_1_ori) = (qw_e*I + S(eps_e)) * omega.
    // This requires building L with the error quaternion, not q_current,
    // otherwise the Lyapunov cross-terms don't cancel and orientation diverges.
    const Eigen::Quaterniond q_e = vortex::utils::math::error_quaternion(
        pose_d.ori_quaternion(), pose.ori_quaternion());
    const Eigen::Vector3d eps_e = q_e.vec();
    const double qw_e = q_e.w();
    const Eigen::Vector3d quat_error = 2.0 * eps_e;

    Eigen::Vector6d z_1;
    z_1 << pos_error, quat_error;

    const Eigen::Matrix3d R = pose.as_rotation_matrix();
    const Eigen::Matrix3d Q_e = calculate_Q_e(eps_e, qw_e);
    const Eigen::Matrix6d L = calculate_L(R, Q_e);
    const Eigen::Matrix6d L_inv = calculate_L_inv(L, singularity_tolerance_);

    const Eigen::Vector3d omega = twist.to_vector().tail<3>();
    const Eigen::Matrix3d Q_e_dot = calculate_Q_e_dot(eps_e, Q_e, omega);
    const Eigen::Matrix6d L_dot =
        calculate_L_dot(calculate_R_dot(pose, twist), Q_e_dot);

    Eigen::Matrix6d C =
        calculate_coriolis(m_, r_b_bg_, twist, inertia_matrix_body_);
    Eigen::Vector6d alpha = -L_inv * K1_ * z_1;
    Eigen::Vector6d z_2 = twist.to_vector() - alpha;
    Eigen::Vector6d alpha_dot = ((L_inv * L_dot * L_inv) * K1_ * z_1) -
                                (L_inv * K1_ * L * twist.to_vector());
    Eigen::Matrix6x12d Y_v = calculate_Y_v(twist);
    Eigen::Vector12d adapt_param_dot = adapt_gain_ * Y_v.transpose() * z_2;
    Eigen::Vector6d d_est_dot = d_gain_ * z_2;
    Eigen::Vector6d F_est = Y_v * adapt_param_;
    Eigen::Vector6d tau = (mass_intertia_matrix_ * alpha_dot) +
                          (C * twist.to_vector()) - (L.transpose() * z_1) -
                          (K2_ * z_2) - F_est - d_est_;

    tau = tau.cwiseMax(-tau_max_).cwiseMin(tau_max_);
    adapt_param_ += adapt_param_dot * time_step_s_;
    d_est_ += d_est_dot * time_step_s_;
    adapt_param_ =
        adapt_param_.cwiseMax(-adapt_param_max_).cwiseMin(adapt_param_max_);
    d_est_ = d_est_.cwiseMax(-d_est_max_).cwiseMin(d_est_max_);

    return tau;
}

void DPAdaptBacksController::reset_adap_param() {
    adapt_param_.setZero();
}

void DPAdaptBacksController::reset_d_est() {
    d_est_.setZero();
}

void DPAdaptBacksController::set_time_step(const double time_step_s) {
    time_step_s_ = time_step_s;
}

}  // namespace vortex::control
