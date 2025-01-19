#include "dp_adapt_backs_controller/dp_adapt_backs_controller.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "dp_adapt_backs_controller/dp_adapt_backs_controller_utils.hpp"
#include "dp_adapt_backs_controller/typedefs.hpp"

DPAdaptBacksController::DPAdaptBacksController()
    : K1_(dp_types::Matrix6d::Identity()),
      K2_(dp_types::Matrix6d::Identity()),
      r_b_bg_(dp_types::Vector3d::Zero()),
      adapt_gain_(dp_types::Matrix12d::Identity()),
      d_gain_(dp_types::Matrix6d::Zero()),
      adap_param_(dp_types::Vector12d::Zero()),
      d_est_(dp_types::Vector6d::Zero()),
      dt_(0.01) {}

dp_types::Vector6d DPAdaptBacksController::calculate_tau(
    const dp_types::Eta& eta,
    const dp_types::Eta& eta_d,
    const dp_types::Nu& nu) {
    dp_types::Eta error = error_eta(eta, eta_d);

    std::cout << "K1_: " << K1_ << std::endl;
    std::cout << "K2_: " << K2_ << std::endl;
    std::cout << "r_b_bg_: " << r_b_bg_ << std::endl;
    std::cout << "adapt_gain_: " << adapt_gain_ << std::endl;
    std::cout << "d_gain_: " << d_gain_ << std::endl;

    return error.as_vector();
}

void DPAdaptBacksController::setK1(const dp_types::Vector6d& K1) {
    this->K1_ = K1.asDiagonal().toDenseMatrix();
}

void DPAdaptBacksController::setK2(const dp_types::Vector6d& K2) {
    this->K2_ = K2.asDiagonal().toDenseMatrix();
}

void DPAdaptBacksController::setrbg(const dp_types::Vector3d& r_b_bg) {
    this->r_b_bg_ = r_b_bg;
}

void DPAdaptBacksController::setAdapParam(
    const dp_types::Vector12d& adapt_gain) {
    this->adapt_gain_ = adapt_gain.asDiagonal().toDenseMatrix();
}

void DPAdaptBacksController::setDGain(const dp_types::Vector6d& d_gain) {
    this->d_gain_ = d_gain.asDiagonal().toDenseMatrix();
}

void DPAdaptBacksController::setTimeStep(double dt) {
    this->dt_ = dt;
}
