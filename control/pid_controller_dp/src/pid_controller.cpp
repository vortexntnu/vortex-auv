#include "pid_controller_dp/pid_controller.hpp"
#include "pid_controller_dp/pid_controller_utils.hpp"
#include <iostream>

PIDController::PIDController()
    : Kp_(Eigen::Matrix6d::Identity()),   
      Ki_(Eigen::Matrix6d::Zero()),        
      Kd_(Eigen::Matrix6d::Zero()),             
      integral_(Eigen::Vector6d::Zero()), 
      dt(0.01)                     
{}

Eigen::Vector6d PIDController::calculate_tau(const Eigen::Vector6d &eta, const Eigen::Vector6d &eta_d, const Eigen::Vector6d &nu, const Eigen::Vector6d &eta_dot_d) {
    Eigen::Vector6d error = apply_ssa(eta - eta_d);

    Eigen::Matrix6d J = calculate_J(eta);

    Eigen::Matrix6d J_inv = J.inverse();

    Eigen::Vector6d nu_d = J_inv * eta_dot_d;

    Eigen::Vector6d error_nu = nu - nu_d;

    Eigen::Vector6d tau = -(Kp_ * J_inv * error + Ki_ * integral_ + Kd_ * error_nu);

    integral_ = anti_windup(dt, error, integral_);

    return tau;
}

void PIDController::setKp(const Eigen::Matrix6d &Kp) {
    this->Kp_ = Kp;
    std::cout << "Kp: " << std::endl << Kp_ << std::endl;
}

void PIDController::setKi(const Eigen::Matrix6d &Ki) {
    this->Ki_ = Ki;
    std::cout << "Ki: " << std::endl << Ki_ << std::endl;
}

void PIDController::setKd(const Eigen::Matrix6d &Kd) {
    this->Kd_ = Kd;
    std::cout << "Kd: " << std::endl << Kd_ << std::endl;
}

void PIDController::setTimeStep(double dt) {
    this->dt = dt;
}