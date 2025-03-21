#include "dp_adapt_backs_controller/dp_adapt_backs_controller.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "dp_adapt_backs_controller/dp_adapt_backs_controller_utils.hpp"
#include "dp_adapt_backs_controller/typedefs.hpp"

DPAdaptBacksController::DPAdaptBacksController(
    const dp_types::DPAdaptParams adap_params)
    : K1_(adap_params.K1.asDiagonal().toDenseMatrix()),
      K2_(adap_params.K2.asDiagonal().toDenseMatrix()),
      r_b_bg_(adap_params.r_b_bg),
      adapt_gain_(adap_params.adap_param.asDiagonal().toDenseMatrix()),
      d_gain_(adap_params.d_gain.asDiagonal().toDenseMatrix()),
      adap_param_(dp_types::Vector12d::Zero()),
      d_est_(dp_types::Vector6d::Zero()),
      I_b_(adap_params.I_b.asDiagonal().toDenseMatrix()),
      M_(adap_params.mass_matrix),
      m_(adap_params.m),
      dt_(0.01) {}

dp_types::Vector6d DPAdaptBacksController::calculate_tau(
    const dp_types::Eta& eta,
    const dp_types::Eta& eta_d,
    const dp_types::Nu& nu) {
    dp_types::Eta error = error_eta(eta, eta_d);

    dp_types::Matrix6d C = calculate_C(m_, r_b_bg_, nu, I_b_);

    dp_types::Matrix6d J = calculate_J(eta);

    dp_types::Matrix6d J_inv = calculate_J_inv(eta);

    dp_types::Matrix6d J_dot = calculate_J_dot(eta, nu);

    dp_types::Vector6d alpha = -J_inv * K1_ * error.as_vector();

    dp_types::Vector6d z_1 = error.as_vector();

    dp_types::Vector6d z_2 = nu.as_vector() - alpha;

    dp_types::Vector6d alpha_dot = ((J_inv * J_dot * J_inv) * K1_ * z_1) -
                                   (J_inv * K1_ * J * nu.as_vector());

    dp_types::Matrix6x12d Y_v = calculate_Y_v(nu);

    dp_types::Vector12d adapt_param_dot = adapt_gain_ * Y_v.transpose() * z_2;

    dp_types::Vector6d d_est_dot = d_gain_ * z_2;

    dp_types::Vector6d F_est = Y_v * adap_param_;

    dp_types::Vector6d tau = (M_ * alpha_dot) + (C * nu.as_vector()) -
                             (J.transpose() * z_1) - (K2_ * z_2) - F_est -
                             d_est_;

    tau = tau.cwiseMax(-80.0).cwiseMin(80.0);

    adap_param_ = adap_param_ + adapt_param_dot * dt_;

    d_est_ = d_est_ + d_est_dot * dt_;

    adap_param_ = adap_param_.cwiseMax(-10.0).cwiseMin(10.0);

    d_est_ = d_est_.cwiseMax(-10.0).cwiseMin(10.0);

    return tau;
}

void DPAdaptBacksController::reset_adap_param() {
    adap_param_ = dp_types::Vector12d::Zero();
}

void DPAdaptBacksController::reset_d_est() {
    d_est_ = dp_types::Vector6d::Zero();
}

void DPAdaptBacksController::set_timeStep(double dt) {
    this->dt_ = dt;
}
