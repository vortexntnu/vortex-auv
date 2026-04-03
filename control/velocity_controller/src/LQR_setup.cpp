#include "velocity_controller/lib/LQR_setup.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/msg/detail/wrench_stamped__struct.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/logger.hpp>
// #include <sstream>
#include <std_msgs/msg/string.hpp>
#include <vector>
// #include "rclcpp/rclcpp.hpp"
#include <casadi/casadi.hpp>
// #include "velocity_controller/PID_setup.hpp"
#include "ct/optcon/lqr/LQR.hpp"
#include "velocity_controller/utilities.hpp"
// #include "vortex/utils/math.hpp"

LQRController::LQRController(LQR_params params) {
    params_=params;
    inertia_matrix_inv.setZero();
    if (params_.interval <= 0){
        valid = false;
        return;
    }
    else if (params_.Q.size() != 8) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "The Q matrix has the wrong amount of elements");
        valid = false;
        return;
    }
    else if (params_.R.size() != 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "The R matrix has the wrong amount of elements");
        valid = false;
        return;
    }
    else if (params_.inertia_matrix.size() != 36) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "The M matrix has the wrong amount of elements");
        valid = false;
        return;
        
    }
    else if (params_.max_force < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "The max_force need to be >0");
        valid = false;
        return;
    }
    Q.diagonal() = Eigen::Map<Eigen::VectorXd>(params_.Q.data(), params_.Q.size());
    R.diagonal() = Eigen::Map<Eigen::VectorXd>(params_.R.data(), params_.R.size());
    Ixx = params_.inertia_matrix.at(6 * 3 + 3);
    Iyy = params_.inertia_matrix.at(4 * 6 + 4);
    Izz = params_.inertia_matrix.at(5 * 6 + 5);
    mass = params_.inertia_matrix.at(0);

    Eigen::Matrix<double, 6, 6> inertia_matrix =
        Eigen::Map<const Eigen::Matrix<double, 6, 6>>(params_.inertia_matrix.data(), 6,
                                                      6);
    D = Eigen::Map<const Eigen::Matrix<double, 6, 6>>(params_.D_low.data(), 6, 6);
    inertia_matrix_inv = inertia_matrix.inverse();

    Eigen::Matrix<double, 6, 3> B_t =
        inertia_matrix_inv * (Eigen::Matrix<double, 6, 3>() << 1, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1)
                                 .finished();
    Eigen::Matrix<double, 9, 3> B_m = Eigen::Matrix<double, 9, 3>::Zero();
    B_m.block<6, 3>(0, 0) = B_t;
    std::vector<std::vector<int>> swaplines{{1, 7}, {2, 8}, {3, 4}, {4, 5}};
    for (int64_t i = 0; i < swaplines.size(); i++) {
        B_m.row(swaplines[i][0]).swap(B_m.row(swaplines[i][1]));
    }
    B.block<5, 3>(0, 0) = B_m.block<5, 3>(0, 0);
    reset_controller();
    valid = true;
}

std::tuple<double, double> LQRController::saturate(double value,
                                                   bool windup,
                                                   double limit) {
    if (abs(value) > limit) {
        windup = true;
        value = limit * (value / abs(value));
    } else {
        windup = false;
    }
    return {windup, value};
}

double LQRController::anti_windup(double error,
                                  double integral_sum,
                                  bool windup) {
    if (!windup) {
        integral_sum += error * params_.interval;
    }
    return integral_sum;
}

Eigen::Matrix<double, 8, 8> LQRController::linearize(State s) {
    Eigen::Matrix<double, 6, 6> D_ = Eigen::Matrix<double, 6, 6>::Zero();

    D_ = -inertia_matrix_inv * D;  // Assuming linear dampening for now

    Eigen::Matrix<double, 6, 6> C = coriolis(s);

    D_ -= inertia_matrix_inv * C;  // To avoid unnecessary allocation

    Eigen::Matrix<double, 3, 3> T = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 9, 9> A;
    A.block<6, 6>(0, 0) = D_;
    A.block<3, 3>(0, 6) = A.block<3, 3>(6, 0) = A.block<3, 3>(6, 6) =
        Eigen::Matrix3d::Zero();
    A.block<3, 3>(6, 3) = T;
    std::vector<std::vector<int>> swaplines{{1, 7}, {2, 8}, {3, 4}, {4, 5}};
    for (int64_t i = 0; i < swaplines.size(); i++) {
        A.row(swaplines[i][0]).swap(A.row(swaplines[i][1]));
        A.col(swaplines[i][0]).swap(A.col(swaplines[i][1]));
    }

    Eigen::Matrix<double, 8, 8> ret;
    ret.setZero();
    ret.block<5, 5>(0, 0) = A.block<5, 5>(0, 0);
    ret.block<3, 3>(5, 0) = Eigen::Matrix3d::Identity();

    return ret;
}
Eigen::Vector<double, 8> LQRController::update_error(const State& error_state,
                                                     const State& state) {
    double surge_error = error_state.surge;
    double pitch_error = error_state.pitch;
    double yaw_error = error_state.yaw;
    integral_error_surge =
        anti_windup(surge_error, integral_error_surge, surge_windup);
    integral_error_pitch =
        anti_windup(pitch_error, integral_error_pitch, pitch_windup);
    integral_error_yaw = anti_windup(yaw_error, integral_error_yaw, yaw_windup);

    Eigen::Vector<double, 8> state_error = {
        surge_error,          pitch_error,       yaw_error,
        -state.pitch_rate,    -state.yaw_rate,   integral_error_surge,
        integral_error_pitch, integral_error_yaw};
    return state_error;
}
Eigen::Vector<double, 3> LQRController::saturate_input(
    Eigen::Vector<double, 3> u) {
    double force_x, torque_y, torque_z;
    std::tie(surge_windup, force_x) = saturate(u[0], surge_windup, params_.max_force);
    std::tie(pitch_windup, torque_y) = saturate(u[1], pitch_windup, params_.max_force);
    std::tie(yaw_windup, torque_z) = saturate(u[2], yaw_windup, params_.max_force);
    return {force_x, torque_y, torque_z};
}
geometry_msgs::msg::WrenchStamped LQRController::calculate_thrust(State state, State error_state) {
    ct::optcon::LQR<8, 3> lqr;
    Eigen::Matrix<double, 3, 8> K_l;
    bool INFO = lqr.compute(Q, R, linearize(state), B, K_l, true, false);
    if (INFO == 0)
        valid=false;
    Eigen::Matrix<double, 8, 1> state_error = update_error(error_state, state);
    //TODO:(henrimha) fix how i return the value here
    Eigen::Vector<double,3>u = saturate_input((K_l * state_error));
    geometry_msgs::msg::WrenchStamped wrench;
    wrench.wrench.force.x = u[0];
    wrench.wrench.torque.y = u[1];
    wrench.wrench.torque.z = u[2];
    return wrench;
}
void LQRController::reset_controller(int nr) {
    if (nr == 0 || nr == 1) {
        integral_error_surge = 0.0;
        surge_windup = false;
    }
    if (nr == 0 || nr == 2) {
        integral_error_pitch = 0.0;
        pitch_windup = false;
    }
    if (nr == 0 || nr == 3) {
        integral_error_yaw = 0.0;
        yaw_windup = false;
    }

    return;
}

// TODO(henrimha): double check the matrices here
Eigen::Matrix<double, 6, 6> LQRController::coriolis(const State& s) {
    double u = s.surge;
    double v = s.sway;
    double w = s.heave;
    double p = s.roll_rate;
    double q = s.pitch_rate;
    double r = s.yaw_rate;
    Eigen::Matrix<double, 6, 6> C = Eigen::Matrix<double, 6, 6>::Zero();

    // Top-right block (translational-rotational coupling)
    C(0, 4) = mass * w;
    C(0, 5) = -mass * v;
    C(1, 3) = -mass * w;
    C(1, 5) = mass * u;
    C(2, 3) = mass * v;
    C(2, 4) = -mass * u;

    // Bottom-left block (rotational-translational coupling)
    C(3, 1) = mass * w;
    C(3, 2) = -mass * v;
    C(4, 0) = -mass * w;
    C(4, 2) = mass * u;
    C(5, 0) = mass * v;
    C(5, 1) = -mass * u;

    // Bottom-right block (rotational-rotational coupling)
    C(3, 4) = Izz * r;
    C(3, 5) = -Iyy * q;
    C(4, 3) = -Izz * r;
    C(4, 5) = Ixx * p;
    C(5, 3) = Iyy * q;
    C(5, 4) = -Ixx * p;

    return C;
}
