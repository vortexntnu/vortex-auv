#include "eskf/eskf.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <functional>
#include <iterator>
#include <unsupported/Eigen/MatrixFunctions>
#include "eskf/eskf_utils.hpp"
#include "eskf/typedefs.hpp"
#include "iostream"

ESKF::ESKF(const eskf_params& params) : Q_(params.Q) {}

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

Eigen::Matrix4x3d ESKF::calculate_q_delta() {
    Eigen::Matrix4x3d q_delta_theta = Eigen::Matrix4x3d::Zero();
    double qw = current_nom_state_.quat.w();
    double qx = current_nom_state_.quat.x();
    double qy = current_nom_state_.quat.y();
    double qz = current_nom_state_.quat.z();

    q_delta_theta << -qx, -qy, -qz, qw, -qz, qy, qz, qw, -qx, -qy, qx, qw;

    q_delta_theta *= 0.5;
    return q_delta_theta;
}
Eigen::Matrix3x19d ESKF::calculate_hx() {
    Eigen::Matrix3x19d Hx = Eigen::Matrix3x19d::Zero();

    Eigen::Quaterniond q = current_nom_state_.quat.normalized();
    Eigen::Matrix3d R_bn = q.toRotationMatrix();

    Eigen::Vector3d v_n = current_nom_state_.vel;

    Hx.block<3, 3>(0, 3) = R_bn.transpose();

    double qw = q.w();
    double qx = q.x();
    double qy = q.y();
    double qz = q.z();

    Eigen::Matrix3d I3   = Eigen::Matrix3d::Identity();

    Eigen::Vector3d eps(qx, qy, qz);

    Eigen::Matrix3d dR_deta =  2*qw * I3 - 2*skew(eps);

    Eigen::Vector3d e1_vec(1,0,0), e2_vec(0,1,0), e3_vec(0,0,1);

    Eigen::Matrix3d dR_dqx = -2*qx*I3
                        + 2*(e1_vec*eps.transpose() + eps*e1_vec.transpose())
                        - 2*qw*skew(e1_vec);

    Eigen::Matrix3d dR_dqy = -2*qy*I3
                        + 2*(e2_vec*eps.transpose() + eps*e2_vec.transpose())
                        - 2*qw*skew(e2_vec);

    Eigen::Matrix3d dR_dqz = -2*qz*I3
                        + 2*(e3_vec*eps.transpose() + eps*e3_vec.transpose())
                        - 2*qw*skew(e3_vec);

    Eigen::Matrix<double,3,4> dR_dq;
    dR_dq.col(0) = dR_deta * v_n;  
    dR_dq.col(1) = dR_dqx  * v_n;   
    dR_dq.col(2) = dR_dqy  * v_n;   
    dR_dq.col(3) = dR_dqz  * v_n;   

    Hx.block<3, 4>(0, 6) = dR_dq;

    return Hx;
}

Eigen::Matrix3x18d ESKF::calculate_h_jacobian() {
    Eigen::Matrix19x18d x_delta = Eigen::Matrix19x18d::Zero();
    x_delta.block<6, 6>(0, 0) = Eigen::Matrix6d::Identity();
    x_delta.block<4, 3>(6, 6) = calculate_q_delta();
    x_delta.block<9, 9>(10, 9) = Eigen::Matrix9d::Identity();

    Eigen::Matrix3x18d H = calculate_hx() * x_delta;
    return H;
}

Eigen::Matrix3x1d ESKF::calculate_h() {
    Eigen::Matrix3x1d h;
    Eigen::Matrix3d R_bn =
        current_nom_state_.quat.normalized().toRotationMatrix().transpose();

    h = R_bn * current_nom_state_.vel;

    return h;
}

void ESKF::nominal_state_discrete(const imu_measurement& imu_meas,
                                  const double dt) {
    Eigen::Vector3d acc = current_nom_state_.quat.normalized().toRotationMatrix() * (imu_meas.accel - current_nom_state_.accel_bias) + current_nom_state_.gravity;
    Eigen::Vector3d gyro = (imu_meas.gyro - current_nom_state_.gyro_bias) * dt;

    current_nom_state_.pos = current_nom_state_.pos + current_nom_state_.vel * dt + 0.5 * sq(dt) * acc;
    current_nom_state_.vel = current_nom_state_.vel + dt * acc;

    current_nom_state_.quat = (current_nom_state_.quat * vector3d_to_quaternion(gyro));
    current_nom_state_.quat.normalize();

    current_nom_state_.gyro_bias = current_nom_state_.gyro_bias;
    current_nom_state_.accel_bias = current_nom_state_.accel_bias;
    current_nom_state_.gravity = current_nom_state_.gravity;
}

void ESKF::error_state_prediction(const imu_measurement& imu_meas,
                                  const double dt) {
    Eigen::Matrix3d R = current_nom_state_.quat.normalized().toRotationMatrix();
    Eigen::Vector3d acc = (imu_meas.accel - current_nom_state_.accel_bias);
    Eigen::Vector3d gyro = (imu_meas.gyro - current_nom_state_.gyro_bias);

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

    Eigen::Matrix18d A_d, GQG_d;
    std::tie(A_d, GQG_d) = van_loan_discretization(A_c, G_c, dt);

    state_euler next_error_state;
    current_error_state_.covariance =
        A_d * current_error_state_.covariance * A_d.transpose() + GQG_d;
}

void ESKF::NIS(const Eigen::Vector3d& innovation, const Eigen::Matrix3d& S) {
    Eigen::Matrix3d S_inv = S.inverse();
    NIS_ = innovation.transpose() * S_inv * innovation;
}

void ESKF::NEES() {

    Eigen::Vector18d error_state = current_nom_state_.nees_error(ground_truth_);
    
    // Use SVD-based pseudo-inverse for better numerical stability
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(current_error_state_.covariance, 
                                          Eigen::ComputeThinU | Eigen::ComputeThinV);
    const double epsilon = 1e-10; // Threshold for singular values
    Eigen::VectorXd singular_values = svd.singularValues();
    Eigen::VectorXd singular_values_inv(singular_values.size());
    
    for (int i = 0; i < singular_values.size(); ++i) {
        if (singular_values(i) > epsilon) {
            singular_values_inv(i) = 1.0 / singular_values(i);
        } else {
            singular_values_inv(i) = 0.0;
        }
    }
    
    Eigen::MatrixXd cov_inv = svd.matrixV() * singular_values_inv.asDiagonal() * svd.matrixU().transpose();
    
    NEES_ = error_state.transpose() * cov_inv * error_state;
}

void ESKF::measurement_update(const dvl_measurement& dvl_meas) {
    Eigen::Matrix3x18d H = calculate_h_jacobian();
    Eigen::Matrix18d P = current_error_state_.covariance;
    Eigen::Matrix3d R = dvl_meas.cov;

    Eigen::Matrix3d S = H * P * H.transpose() + R;
    Eigen::Matrix18x3d K = P * H.transpose() * S.inverse();
    Eigen::Vector3d innovation = dvl_meas.vel - calculate_h();
    NIS(innovation, S);
    current_error_state_.set_from_vector(K * innovation);

    Eigen::Matrix18d I_KH = Eigen::Matrix18d::Identity() - K * H;
    current_error_state_.covariance =
        I_KH * P * I_KH.transpose() +
        K * R * K.transpose();  // Used joseph form for more stable calculations
    
    NEES();
}

void ESKF::injection_and_reset() {
    current_nom_state_.pos = current_nom_state_.pos + current_error_state_.pos;
    current_nom_state_.vel = current_nom_state_.vel + current_error_state_.vel;
    current_nom_state_.quat = current_nom_state_.quat * vector3d_to_quaternion(current_error_state_.euler);
    current_nom_state_.quat.normalize();
    current_nom_state_.gyro_bias = current_nom_state_.gyro_bias + current_error_state_.gyro_bias;
    current_nom_state_.accel_bias = current_nom_state_.accel_bias + current_error_state_.accel_bias;
    current_nom_state_.gravity = current_nom_state_.gravity + current_error_state_.gravity;

    Eigen::Matrix18d G = Eigen::Matrix18d::Identity();

    current_error_state_.covariance =
        G * current_error_state_.covariance * G.transpose();
    current_error_state_.set_from_vector(Eigen::Vector18d::Zero());
}

std::pair<state_quat, state_euler> ESKF::imu_update(
    const imu_measurement& imu_meas,
    const double dt) {
    nominal_state_discrete(imu_meas, dt);
    error_state_prediction(imu_meas, dt);

    return {current_nom_state_, current_error_state_};
}

std::pair<state_quat, state_euler> ESKF::dvl_update(
    const dvl_measurement& dvl_meas) {
    measurement_update(dvl_meas);
    injection_and_reset();

    return {current_nom_state_, current_error_state_};
}
