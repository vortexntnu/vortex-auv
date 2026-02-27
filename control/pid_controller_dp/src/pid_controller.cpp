#include "pid_controller_dp/pid_controller.hpp"
#include "pid_controller_dp/pid_controller_utils.hpp"

void print_eta(const types::Eta& eta) {
    // spdlog::info("Eta values:");
    auto pos = eta.pos_vector();
    auto ori = eta.ori_quaternion();
    spdlog::info("Position - North: {}, East: {}, Down: {}", pos[0], pos[1],
                 pos[2]);
    spdlog::info("Orientation - w: {}, x: {}, y: {}, z: {}", ori.w(), ori.x(),
                 ori.y(), ori.z());
}

void print_nu(const types::Nu& nu) {
    spdlog::info("Nu values:");
    auto v = nu.to_vector();
    spdlog::info("Linear Speed - u: {}, v: {}, w: {}", v(0), v(1), v(2));
    spdlog::info("Angular Speed - p: {}, q: {}, r: {}", v(3), v(4), v(5));
}

void print_vect_6d(const types::Vector6d& vec) {
    spdlog::info("Vector6d values:");
    for (int i = 0; i < 6; ++i) {
        spdlog::info("Element[{}]: {}", i, vec[i]);
    }
}

void print_J_transformation(const types::J_transformation& J) {
    spdlog::info("J_transformation:");

    spdlog::info("R (3x3) elements:");
    for (int i = 0; i < J.R.rows(); ++i) {
        for (int j = 0; j < J.R.cols(); ++j) {
            spdlog::info("R[{},{}] = {}", i, j, J.R(i, j));
        }
    }

    spdlog::info("T (4x3) elements:");
    for (int i = 0; i < J.T.rows(); ++i) {
        for (int j = 0; j < J.T.cols(); ++j) {
            spdlog::info("T[{},{}] = {}", i, j, J.T(i, j));
        }
    }

    spdlog::info("Combined Matrix (7x6) elements:");
    auto M = J.as_matrix();
    for (int i = 0; i < M.rows(); ++i) {
        for (int j = 0; j < M.cols(); ++j) {
            spdlog::info("M[{},{}] = {}", i, j, M(i, j));
        }
    }
}

void print_Jinv_transformation(const types::Matrix6x7d& J_inv) {
    spdlog::info("J_pseudo_inverse (6x7):");
    for (int i = 0; i < J_inv.rows(); ++i) {
        std::string row;
        row.reserve(128);
        row += "[";
        for (int j = 0; j < J_inv.cols(); ++j) {
            row += std::to_string(J_inv(i, j));
            if (j < J_inv.cols() - 1)
                row += ", ";
        }
        row += "]";
        spdlog::info("{}", row);
    }
}

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
    types::Eta error = error_eta(eta, eta_d);  // calculate eta error

    // set quaternion scalar part w = 0 (only use vector part of quaternion for
    // error)
    error.qw = 0.0;

    auto eta_dot_d_copy = eta_dot_d;
    eta_dot_d_copy.qw = 0.0;  // set w = 0 for desired eta_dot

    types::Matrix6x7d J_inv =
        calculate_J_sudo_inv(eta);  // calculate J pseudo inverse

    types::Vector6d nu_d =
        J_inv * eta_dot_d_copy.to_vector();  // calculate velocity

    types::Vector6d error_nu = nu.to_vector() - nu_d;  // calculate vel error

    types::Vector6d P = Kp_ * J_inv * error.to_vector();  // P term

    types::Vector6d I = Ki_ * J_inv * integral_;  // I term

    types::Vector6d D = Kd_ * error_nu;  // D term

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

types::Matrix6d PIDController::get_kp() {
    return this->Kp_;
}
types::Matrix6d PIDController::get_ki() {
    return this->Ki_;
}
types::Matrix6d PIDController::get_kd() {
    return this->Kd_;
}