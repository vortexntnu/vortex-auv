#include "pid_controller_dp/pid_controller.hpp"
#include "pid_controller_dp/pid_controller_utils.hpp"

PIDController::PIDController()
    : Kp_(types::Matrix6d::Identity()),
      Ki_(types::Matrix6d::Zero()),
      Kd_(types::Matrix6d::Zero()),
      integral_(types::Vector7d::Zero()),
      dt_(0.01) {}

types::Vector6d PIDController::calculate_tau(const types::Eta& eta,
                                             const types::Eta& eta_d,
                                             const types::Nu& nu,
                                             const types::Eta& eta_dot_d) {
    types::Eta error = error_eta(eta, eta_d);

    types::Matrix6x7d J_inv = calculate_J_sudo_inv(error);

    types::Vector6d nu_d = J_inv * eta_dot_d.as_vector();

    types::Vector6d error_nu = nu.as_vector() - nu_d;

    types::Vector6d P = Kp_ * J_inv * error.as_vector();

    types::Vector6d I = Ki_ * J_inv * integral_;

    types::Vector6d D = Kd_ * error_nu;

    types::Vector6d tau = -clamp_values((P + I + D), -80.0, 80.0);

    integral_ = anti_windup(dt_, error, integral_);

    return tau;
}

void PIDController::set_kp(const types::Matrix6d& Kp) {
    this->Kp_ = Kp;
}

void PIDController::set_ki(const types::Matrix6d& Ki) {
    this->Ki_ = Ki;
}

void PIDController::set_kd(const types::Matrix6d& Kd) {
    this->Kd_ = Kd;
}

void PIDController::set_time_step(double dt) {
    this->dt_ = dt;
}
