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
      m_(dp_adapt_params.mass),
      dt_(0.01) {}

Eigen::Vector6d DPAdaptBacksController::calculate_tau(const Pose& pose,
                                                      const Pose& pose_d,
                                                      const Twist& twist) {
    // TODO: implement error state calculation. Maybe look at hybrid
    // switching between pure RPY and error state when error is big
    Eigen::Vector3d pos_error = pose.pos_vector() - pose_d.pos_vector();
    Eigen::Vector3d quat_error = vortex::utils::math::quaternion_error(
        pose_d.ori_quaternion(), pose.ori_quaternion());
    Eigen::Vector6d z_1;
    z_1 << pos_error, quat_error;

    Eigen::Matrix6d L = Eigen::Matrix6d::Zero();
    L.topLeftCorner<3, 3>() = pose.as_rotation_matrix();
    L.bottomRightCorner<3, 3>() = pose.as_transformation_matrix().bottomRows<3>();

    Eigen::Matrix6d C =
        calculate_coriolis(m_, r_b_bg_, twist, inertia_matrix_body_);
    Eigen::Matrix6d L_inv = calculate_L_inv(pose);
    Eigen::Matrix6d L_dot = calculate_L_dot(pose, twist);
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
