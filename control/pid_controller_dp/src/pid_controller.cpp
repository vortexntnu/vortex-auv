#include "pid_controller_dp/pid_controller.hpp"
#include <iostream>
#include "pid_controller_dp/pid_controller_utils.hpp"

PIDController::PIDController()
    : Kp_(Eigen::Matrix6d::Identity()),
      Ki_(Eigen::Matrix6d::Zero()),
      Kd_(Eigen::Matrix6d::Zero()),
      integral_(Eigen::Vector7d::Zero()),
      dt_(0.01) {}

Eigen::Vector6d PIDController::calculate_tau(const Eigen::Vector7d& eta,
                                             const Eigen::Vector7d& eta_d,
                                             const Eigen::Vector6d& nu,
                                             const Eigen::Vector7d& eta_dot_d) {
    Eigen::Vector7d error = error_eta(eta, eta_d);

    Eigen::Matrix6x7d J_inv = calculate_J_sudo_inv(eta);

    Eigen::Vector6d nu_d = J_inv * eta_dot_d;

    Eigen::Vector6d error_nu = nu - nu_d;

    Eigen::Vector6d P = Kp_ * J_inv * error;

    Eigen::Vector6d I = Ki_ * J_inv * integral_;

    Eigen::Vector6d D = Kd_ * error_nu;

    Eigen::Vector6d tau = -limit_input((P + I + D));

    integral_ = anti_windup(dt, error, integral_);

    return tau;
}

void PIDController::setKp(const Eigen::Matrix6d& Kp) {
    this->Kp_ = Kp;
}

void PIDController::setKi(const Eigen::Matrix6d& Ki) {
    this->Ki_ = Ki;
}

void PIDController::setKd(const Eigen::Matrix6d& Kd) {
    this->Kd_ = Kd;
}

void PIDController::setTimeStep(double dt) {
    this->dt_ = dt_;
}
