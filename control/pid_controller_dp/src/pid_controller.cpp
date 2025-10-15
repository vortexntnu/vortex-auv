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
    types::Eta error = error_eta(eta, eta_d); // calculate eta error

    // debug
    eta_error_debug = error;

    types::Matrix6x7d J_inv = calculate_J_sudo_inv(error); // calculate J pseudo inverse
    J_inv_debug = J_inv;
    
    types::Vector6d nu_d = J_inv * eta_dot_d.as_vector(); // calculate velocity
    nu_d_debug = nu_d;

    types::Vector6d error_nu = nu.as_vector() - nu_d; // calculate vel error
    error_nu_debug = error_nu;

    types::Vector6d P = Kp_ * J_inv * error.as_vector(); /// P term
    P_debug = P;
    Kp_debug = Kp_;

    types::Vector6d I = Ki_ * J_inv * integral_; // I term
    I_debug = I;
    Ki_debug = Ki_;
    
    types::Vector6d D = Kd_ * error_nu; // D term
    D_debug = D;
    Kd_debug = Kd_;
    types::Vector6d tau = -clamp_values((P + I + D), -80.0, 80.0);
    // types::Vector6d tau = -clamp_values((P), -80.0, 80.0);


    //debug: tau = 0
    // types::Vector6d tau = types::Vector6d::Zero();

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
