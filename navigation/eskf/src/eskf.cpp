#include "eskf/eskf.hpp"
#include <spdlog/spdlog.h>
#include <functional>
#include <iterator>
#include <unsupported/Eigen/MatrixFunctions>
#include <vortex/utils/math.hpp>
#include "eskf/typedefs.hpp"

double compute_nis(const Eigen::Vector3d& innovation,
                   const Eigen::Matrix3d& S) {
    Eigen::Matrix3d S_inv = S.inverse();
    return innovation.transpose() * S_inv * innovation;
}

ESKF::ESKF(const EskfParams& params) : Q_(params.Q) {
    current_error_state_.covariance = params.P;
}

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

Eigen::Matrix3x19d ESKF::calculate_hx() {
    Eigen::Matrix3x19d Hx = Eigen::Matrix3x19d::Zero();

    Eigen::Quaterniond q = current_nom_state_.quat.normalized();
    Eigen::Matrix3d R_bn = q.toRotationMatrix();

    Eigen::Vector3d v_n = current_nom_state_.vel;

    // Correct derivative w.r.t velocity (nominal state: v_n)
    Hx.block<3, 3>(0, 3) = R_bn.transpose();

    // Derivative w.r.t quaternion (nominal state: q)
    // Compute partial derivative w.r.t quaternion directly:
    double qw = q.w();
    Eigen::Vector3d q_vec(q.x(), q.y(), q.z());
    Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 3, 4> dhdq;
    dhdq.col(0) = 2 * (qw * v_n + q_vec.cross(v_n));
    dhdq.block<3, 3>(0, 1) =
        2 * (q_vec.dot(v_n) * I3 + q_vec * v_n.transpose() -
             v_n * q_vec.transpose() -
             qw * vortex::utils::math::get_skew_symmetric_matrix(v_n));

    // Assign quaternion derivative (3x4 block at columns 6:9)
    Hx.block<3, 4>(0, 6) = dhdq;

    return Hx;
}

Eigen::Matrix3x18d ESKF::calculate_h_jacobian() {
    Eigen::Matrix19x18d x_delta = Eigen::Matrix19x18d::Zero();
    x_delta.block<6, 6>(0, 0) = Eigen::Matrix6d::Identity();
    x_delta.block<4, 3>(6, 6) =
        vortex::utils::math::get_transformation_matrix_attitude_quat(
            current_nom_state_.quat);
    x_delta.block<9, 9>(10, 9) = Eigen::Matrix9d::Identity();

    Eigen::Matrix3x18d H = calculate_hx() * x_delta;
    return H;
}

Eigen::Vector3d ESKF::calculate_h() {
    Eigen::Vector3d h;
    Eigen::Matrix3d R_bn =
        current_nom_state_.quat.normalized().toRotationMatrix().transpose();

    h = R_bn * current_nom_state_.vel;
    // 0.027293, 0.028089, 0.028089, 0.00255253, 0.00270035, 0.00280294,
    return h;
}

void ESKF::nominal_state_discrete(const ImuMeasurement& imu_meas,
                                  const double dt) {
    Eigen::Vector3d acc =
        current_nom_state_.quat.normalized().toRotationMatrix() *
            (imu_meas.accel - current_nom_state_.accel_bias) +
        current_nom_state_.gravity;
    Eigen::Vector3d gyro = (imu_meas.gyro - current_nom_state_.gyro_bias) * dt;

    current_nom_state_.pos = current_nom_state_.pos +
                             current_nom_state_.vel * dt + 0.5 * dt * dt * acc;
    current_nom_state_.vel = current_nom_state_.vel + dt * acc;

    current_nom_state_.quat =
        (current_nom_state_.quat *
         vortex::utils::math::eigen_vector3d_to_quaternion(gyro));
    current_nom_state_.quat.normalize();

    current_nom_state_.gyro_bias = current_nom_state_.gyro_bias;
    current_nom_state_.accel_bias = current_nom_state_.accel_bias;
    current_nom_state_.gravity = current_nom_state_.gravity;
}

void ESKF::error_state_prediction(const ImuMeasurement& imu_meas,
                                  const double dt) {
    Eigen::Matrix3d R = current_nom_state_.quat.normalized().toRotationMatrix();
    Eigen::Vector3d acc = (imu_meas.accel - current_nom_state_.accel_bias);
    Eigen::Vector3d gyro = (imu_meas.gyro - current_nom_state_.gyro_bias);

    Eigen::Matrix18d A_c = Eigen::Matrix18d::Zero();
    A_c.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    A_c.block<3, 3>(3, 6) =
        -R * vortex::utils::math::get_skew_symmetric_matrix(acc);
    A_c.block<3, 3>(6, 6) =
        -vortex::utils::math::get_skew_symmetric_matrix(gyro);
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

    StateEuler next_error_state;
    current_error_state_.covariance =
        A_d * current_error_state_.covariance * A_d.transpose() + GQG_d;
}

void ESKF::measurement_update(const DvlMeasurement& dvl_meas) {
    Eigen::Matrix3x18d H = calculate_h_jacobian();
    Eigen::Matrix18d P = current_error_state_.covariance;
    Eigen::Matrix3d R = dvl_meas.cov;

    Eigen::Matrix3d S = H * P * H.transpose() + R;
    Eigen::Matrix18x3d K = P * H.transpose() * S.inverse();
    Eigen::Vector3d innovation = dvl_meas.vel - calculate_h();

    nis_ = compute_nis(innovation, S);
    current_error_state_.set_from_vector(K * innovation);

    Eigen::Matrix18d I_KH = Eigen::Matrix18d::Identity() - K * H;
    current_error_state_.covariance =
        I_KH * P * I_KH.transpose() +
        K * R * K.transpose();  // Used joseph form for more stable calculations
}

void ESKF::injection_and_reset() {
    current_nom_state_.pos = current_nom_state_.pos + current_error_state_.pos;
    current_nom_state_.vel = current_nom_state_.vel + current_error_state_.vel;
    current_nom_state_.quat = current_nom_state_.quat *
                              vortex::utils::math::eigen_vector3d_to_quaternion(
                                  current_error_state_.euler);
    current_nom_state_.quat.normalize();
    current_nom_state_.gyro_bias =
        current_nom_state_.gyro_bias + current_error_state_.gyro_bias;
    current_nom_state_.accel_bias =
        current_nom_state_.accel_bias + current_error_state_.accel_bias;
    current_nom_state_.gravity =
        current_nom_state_.gravity + current_error_state_.gravity;

    Eigen::Matrix18d G = Eigen::Matrix18d::Identity();

    current_error_state_.covariance =
        G * current_error_state_.covariance * G.transpose();
    current_error_state_.set_from_vector(Eigen::Vector18d::Zero());
}

void ESKF::imu_update(const ImuMeasurement& imu_meas, const double dt) {
    nominal_state_discrete(imu_meas, dt);
    error_state_prediction(imu_meas, dt);
}

void ESKF::dvl_update(const DvlMeasurement& dvl_meas) {
    // for testing the visual odom
    return;
    
    // measurement_update(dvl_meas);
    // injection_and_reset();
}

void ESKF::visualEgomotion_update(const VisualMeasurement& visual_meas) {
    if (last_vo_stamp_sec_ > 0.0) {
        const double gap = visual_meas.stamp_sec - last_vo_stamp_sec_;
        if (gap > vo_reset_gap_sec_) {
            have_vo_anchor_ = false;
        }
    }
    last_vo_stamp_sec_ = visual_meas.stamp_sec;

    if (!have_vo_anchor_) {
        q_nav_vo_ = (current_nom_state_.quat * visual_meas.quat.inverse()).normalized();
        p_nav_vo_ = current_nom_state_.pos - q_nav_vo_ * visual_meas.pos;
        have_vo_anchor_ = true;
        return;
    }

    const Eigen::Vector3d p_meas_nav = p_nav_vo_ + q_nav_vo_ * visual_meas.pos;
    const Eigen::Quaterniond q_meas_nav = (q_nav_vo_ * visual_meas.quat).normalized();

    Eigen::Vector3d pos_err_vec = p_meas_nav - current_nom_state_.pos;
    double error_mag = pos_err_vec.norm();

    spdlog::info("ESKF: Visual Update. Error: {:.3f}m", error_mag);

    if (error_mag > 0.2) {
        spdlog::warn("VO rejected: error_mag={:.3f}m", error_mag);
        return;
    }

    Eigen::Matrix<double, 6, 1> y;
    y.head<3>() = pos_err_vec;

    Eigen::Quaterniond q_err = current_nom_state_.quat.inverse() * q_meas_nav;
    q_err.normalize();
    y.segment<3>(3) = (q_err.w() < 0) ? -2.0 * q_err.vec() : 2.0 * q_err.vec();

    Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

    Eigen::Matrix18d P = current_error_state_.covariance;
    Eigen::Matrix<double, 6, 6> R = visual_meas.R;
    Eigen::Matrix<double, 6, 6> S = H * P * H.transpose() + R;
    Eigen::Matrix<double, 18, 6> K = P * H.transpose() * S.inverse();

    current_error_state_.set_from_vector(K * y);

    Eigen::Matrix18d I_KH = Eigen::Matrix18d::Identity() - K * H;
    current_error_state_.covariance =
        I_KH * P * I_KH.transpose() + K * R * K.transpose();

    injection_and_reset();
}
