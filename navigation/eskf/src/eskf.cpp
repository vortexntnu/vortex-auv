#include "eskf/eskf.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <functional>
#include <iterator>
#include <unsupported/Eigen/MatrixFunctions>
#include "eskf/eskf_utils.hpp"
#include "eskf/typedefs.hpp"

ESKF::ESKF(const eskf_params& params) : 
    Q_(params.Q) {}

std::pair<Eigen::Matrix18d, Eigen::Matrix18d> ESKF::van_loan_discretization(
    const Eigen::Matrix18d& A_c,
    const Eigen::Matrix18x12d& G_c,
    const double dt) {
    Eigen::Matrix18d GQG_T = G_c * Q_ * G_c.transpose();
    Eigen::Matrix36d vanLoanMat = Eigen::Matrix36d::Zero();

    vanLoanMat.topLeftCorner<18, 18>() = -A_c;
    vanLoanMat.topRightCorner<18, 18>() = GQG_T;
    vanLoanMat.bottomRightCorner<18, 18>() = A_c.transpose();

    Eigen::Matrix36d vanLoanExp = (vanLoanMat * dt).exp();

    Eigen::Matrix18d V1 = vanLoanExp.bottomRightCorner<18, 18>().transpose();
    Eigen::Matrix18d V2 = vanLoanExp.topRightCorner<18, 18>();

    Eigen::Matrix18d A_d = V1;
    Eigen::Matrix18d GQG_d = A_d * V2;

    return {A_d, GQG_d};
}

Eigen::Matrix4x3d ESKF::calculate_Q_delta(const state_quat& nom_state) {
    Eigen::Matrix4x3d Q_delta_theta = Eigen::Matrix4x3d::Zero();
    double qw = nom_state.quat.w();
    double qx = nom_state.quat.x();
    double qy = nom_state.quat.y();
    double qz = nom_state.quat.z();

    Q_delta_theta << -qx, -qy, -qz, qw, -qz, qy, qz, qw, -qx, -qy, qx, qw;

    Q_delta_theta *= 0.5;
    return Q_delta_theta;
}
Eigen::Matrix3x19d ESKF::calculate_Hx(const state_quat& nom_state) {
    Eigen::Matrix3x19d Hx = Eigen::Matrix3x19d::Zero();

    Eigen::Quaterniond q = nom_state.quat.normalized();
    Eigen::Matrix3d R_bn = q.toRotationMatrix();
    
    Eigen::Vector3d v_n = nom_state.vel;

    Hx.block<3, 3>(0, 3) = R_bn.transpose();

    Eigen::Matrix<double, 3, 4> dR_dq;
    double qw = q.w();
    double qx = q.x();
    double qy = q.y();
    double qz = q.z();

    dR_dq.col(0) = 2 * Eigen::Vector3d(
        qw * v_n.x() + qz * v_n.y() - qy * v_n.z(),
       -qz * v_n.x() + qw * v_n.y() + qx * v_n.z(),
        qy * v_n.x() - qx * v_n.y() + qw * v_n.z()
    );

    dR_dq.col(1) = 2 * Eigen::Vector3d(
        qx * v_n.x() + qy * v_n.y() + qz * v_n.z(),
        qy * v_n.x() - qx * v_n.y() - qw * v_n.z(),
        qz * v_n.x() + qw * v_n.y() - qx * v_n.z()
    );

    dR_dq.col(2) = 2 * Eigen::Vector3d(
       -qy * v_n.x() + qx * v_n.y() + qw * v_n.z(),
        qx * v_n.x() + qy * v_n.y() + qz * v_n.z(),
       -qw * v_n.x() + qz * v_n.y() - qy * v_n.z()
    );

    dR_dq.col(3) = 2 * Eigen::Vector3d(
       -qz * v_n.x() - qw * v_n.y() + qx * v_n.z(),
        qw * v_n.x() - qz * v_n.y() + qy * v_n.z(),
        qx * v_n.x() + qy * v_n.y() + qz * v_n.z()
    );

    Hx.block<3, 4>(0, 6) = dR_dq;

    return Hx;
}

Eigen::Matrix3x18d ESKF::calculate_H(const state_quat& nom_state) {
    Eigen::Matrix19x18d X_delta = Eigen::Matrix19x18d::Zero();
    X_delta.block<6, 6>(0, 0) = Eigen::Matrix6d::Identity();
    X_delta.block<4, 3>(6, 6) = calculate_Q_delta(nom_state);
    X_delta.block<9, 9>(10, 9) = Eigen::Matrix9d::Identity();

    Eigen::Matrix3x18d H = calculate_Hx(nom_state) * X_delta;
    return H;
}

Eigen::Matrix3x1d ESKF::calculate_h(const state_quat& nom_state) {
    Eigen::Matrix3x1d h;
    Eigen::Matrix3d R_bn = nom_state.quat.normalized().toRotationMatrix().transpose();

    h = R_bn * nom_state.vel;

    return h;
}

state_quat ESKF::nominal_state_discrete(const state_quat& nom_state,
                                        const imu_measurement& imu_meas,
                                        const double dt) {
    Eigen::Vector3d acc = nom_state.get_R() * (imu_meas.accel - nom_state.accel_bias) + nom_state.gravity;
    Eigen::Vector3d gyro = (imu_meas.gyro - nom_state.gyro_bias) * dt;

    state_quat next_nom_state;

    next_nom_state.pos = nom_state.pos + nom_state.vel * dt + 0.5 * sq(dt) * acc;
    next_nom_state.vel = nom_state.vel + dt * acc;
    next_nom_state.quat = (nom_state.quat * vector3d_to_quaternion(gyro));
    next_nom_state.quat.normalize();
    next_nom_state.gyro_bias = nom_state.gyro_bias;
    next_nom_state.accel_bias = nom_state.accel_bias;
    next_nom_state.gravity = nom_state.gravity;

    return next_nom_state;
}

state_euler ESKF::error_state_prediction(const state_euler& error_state,
                                         const state_quat& nom_state,
                                         const imu_measurement& imu_meas,
                                         const double dt) {
    Eigen::Matrix3d R = nom_state.get_R();
    Eigen::Vector3d acc = (imu_meas.accel - nom_state.accel_bias);
    Eigen::Vector3d gyro = (imu_meas.gyro - nom_state.gyro_bias);

    Eigen::Matrix18d A_c = Eigen::Matrix18d::Zero();
    A_c.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    A_c.block<3, 3>(3, 6) = -R * skew(acc);
    A_c.block<3, 3>(6, 6) = -skew(gyro);
    A_c.block<3, 3>(3, 9) = -R;
    A_c.block<3, 3>(9, 9) = -Eigen::Matrix3d::Identity();
    A_c.block<3, 3>(12, 12) = -Eigen::Matrix3d::Identity();
    A_c.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity();
    A_c.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity();

    Eigen::Matrix18x12d G_c = Eigen::Matrix18x12d::Zero();
    G_c.block<3, 3>(3, 0) = -R;
    G_c.block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity();
    G_c.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
    G_c.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();

    auto [A_d, GQG_d] = van_loan_discretization(A_c, G_c, dt);

    state_euler next_error_state;
    next_error_state.covariance =
        A_d * error_state.covariance * A_d.transpose() + GQG_d;

    return next_error_state;
}

state_euler ESKF::measurement_update(const state_quat& nom_state,
                                     const state_euler& error_state,
                                     const dvl_measurement& dvl_meas) {
    state_euler new_error_state;

    Eigen::Matrix3x18d H = calculate_H(nom_state);
    Eigen::Matrix18d P = error_state.covariance;
    Eigen::Matrix3d R = dvl_meas.cov;

    Eigen::Matrix3d S = H * P * H.transpose() + R;
    Eigen::Matrix18x3d K = P * H.transpose() * S.inverse();
    Eigen::Vector3d innovation = dvl_meas.vel - calculate_h(nom_state);
    new_error_state.set_from_vector(K * innovation);

    Eigen::Matrix18d I_KH = Eigen::Matrix18d::Identity() - K * H;
    new_error_state.covariance =
        I_KH * P * I_KH.transpose() +
        K * R * K.transpose();  // Used joseph form for more stable calculations

    return new_error_state;
}

std::pair<state_quat, state_euler> ESKF::injection_and_reset(
    const state_quat& nom_state,
    const state_euler& error_state) {
    state_quat next_nom_state;

    next_nom_state.pos = nom_state.pos + error_state.pos;
    next_nom_state.vel = nom_state.vel + error_state.vel;
    next_nom_state.quat = nom_state.quat * vector3d_to_quaternion(error_state.euler);
    next_nom_state.quat.normalize();
    next_nom_state.gyro_bias = nom_state.gyro_bias + error_state.gyro_bias;
    next_nom_state.accel_bias = nom_state.accel_bias + error_state.accel_bias;
    next_nom_state.gravity = nom_state.gravity + error_state.gravity;

    state_euler new_error_state;

    Eigen::Matrix18d G = Eigen::Matrix18d::Identity();

    new_error_state.covariance = G * error_state.covariance * G.transpose();

    return {next_nom_state, new_error_state};
}

std::pair<state_quat, state_euler> ESKF::imu_update(
    const state_quat& nom_state,
    const state_euler& error_state,
    const imu_measurement& imu_meas,
    const double dt) {
    state_quat next_nom_state = nominal_state_discrete(nom_state, imu_meas, dt);
    state_euler next_error_state = error_state_prediction(error_state, next_nom_state, imu_meas, dt);

    return {next_nom_state, next_error_state};
}

std::pair<state_quat, state_euler> ESKF::dvl_update(
    const state_quat& nom_state,
    const state_euler& error_state,
    const dvl_measurement& dvl_meas) {
    state_euler new_error_state = measurement_update(nom_state, error_state, dvl_meas);
    auto [updated_nom_state, updated_error_state] = injection_and_reset(nom_state, new_error_state);

    return {updated_nom_state, updated_error_state};
}
