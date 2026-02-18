#include "eskf/eskf.hpp"
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
    // Initialize Covariance
    current_error_state_.covariance = params.P;

    // Initialize Nominal Quaternion to Identity
    current_nom_state_.quat = Eigen::Quaterniond::Identity();
}

std::pair<Eigen::Matrix15d, Eigen::Matrix15d> ESKF::van_loan_discretization(
    const Eigen::Matrix15d& A_c,
    const Eigen::Matrix15x12d& G_c,
    const double dt) {
    Eigen::Matrix15d GQG_T = G_c * Q_ * G_c.transpose();
    Eigen::Matrix30d vanLoanMat = Eigen::Matrix30d::Zero();

    vanLoanMat.topLeftCorner<15, 15>() = -A_c;
    vanLoanMat.topRightCorner<15, 15>() = GQG_T;
    vanLoanMat.bottomRightCorner<15, 15>() = A_c.transpose();

    Eigen::Matrix30d vanLoanExp = (vanLoanMat * dt).exp();

    Eigen::Matrix15d V1 = vanLoanExp.bottomRightCorner<15, 15>().transpose();
    Eigen::Matrix15d V2 = vanLoanExp.topRightCorner<15, 15>();

    Eigen::Matrix15d A_d = V1;
    Eigen::Matrix15d GQG_d = A_d * V2;

    return {A_d, GQG_d};
}

Eigen::Matrix3x16d calculate_hx(const StateQuat& current_nom_state_) {
    Eigen::Matrix3x16d Hx = Eigen::Matrix3x16d::Zero();

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

Eigen::Matrix3x15d calculate_h_jacobian(const StateQuat& current_nom_state_) {
    Eigen::Matrix16x15d x_delta = Eigen::Matrix16x15d::Zero();
    x_delta.block<6, 6>(0, 0) = Eigen::Matrix6d::Identity();
    x_delta.block<4, 3>(6, 6) =
        vortex::utils::math::get_transformation_matrix_attitude_quat(
            current_nom_state_.quat);
    x_delta.block<6, 6>(10, 9) = Eigen::Matrix6d::Identity();

    Eigen::Matrix3x15d H = calculate_hx(current_nom_state_) * x_delta;
    return H;
}

Eigen::Vector3d calculate_h(const StateQuat& current_nom_state_) {
    Eigen::Vector3d h;
    Eigen::Matrix3d R_bn =
        current_nom_state_.quat.normalized().toRotationMatrix().transpose();

    h = R_bn * current_nom_state_.vel;
    return h;
}

void ESKF::nominal_state_discrete(const ImuMeasurement& imu_meas,
                                  const double dt) {
    Eigen::Vector3d acc =
        current_nom_state_.quat.normalized().toRotationMatrix() *
            (imu_meas.accel - current_nom_state_.accel_bias) +
        this->g_;
    Eigen::Vector3d gyro = (imu_meas.gyro - current_nom_state_.gyro_bias) * dt;

    current_nom_state_.pos = current_nom_state_.pos +
                             current_nom_state_.vel * dt + 0.5 * dt * dt * acc;
    current_nom_state_.vel = current_nom_state_.vel + dt * acc;

    current_nom_state_.quat =
        (current_nom_state_.quat *
         vortex::utils::math::eigen_vector3d_to_quaternion(gyro));
    current_nom_state_.quat.normalize();

    current_nom_state_.accel_bias =
        current_nom_state_.accel_bias * std::exp(accm_bias_p_ * dt);
    current_nom_state_.gyro_bias =
        current_nom_state_.gyro_bias * std::exp(gyro_bias_p_ * dt);
}

void ESKF::error_state_prediction(const ImuMeasurement& imu_meas,
                                  const double dt) {
    Eigen::Matrix3d R = current_nom_state_.quat.normalized().toRotationMatrix();
    Eigen::Vector3d acc = (imu_meas.accel - current_nom_state_.accel_bias);
    Eigen::Vector3d gyro = (imu_meas.gyro - current_nom_state_.gyro_bias);

    Eigen::Matrix15d A_c = Eigen::Matrix15d::Zero();
    A_c.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    A_c.block<3, 3>(3, 6) =
        -R * vortex::utils::math::get_skew_symmetric_matrix(acc);
    A_c.block<3, 3>(6, 6) =
        -vortex::utils::math::get_skew_symmetric_matrix(gyro);
    A_c.block<3, 3>(3, 9) = -R;
    A_c.block<3, 3>(9, 9) = -Eigen::Matrix3d::Identity();
    A_c.block<3, 3>(12, 12) = -Eigen::Matrix3d::Identity();
    A_c.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity();

    Eigen::Matrix15x12d G_c = Eigen::Matrix15x12d::Zero();
    G_c.block<3, 3>(3, 0) = -R;
    G_c.block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity();
    G_c.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
    G_c.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();

    Eigen::Matrix15d A_d, GQG_d;
    std::tie(A_d, GQG_d) = van_loan_discretization(A_c, G_c, dt);

    StateEuler next_error_state;
    current_error_state_.covariance =
        A_d * current_error_state_.covariance * A_d.transpose() + GQG_d;
}

void ESKF::injection_and_reset() {
    // injection
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

    // reset
    current_error_state_.set_from_vector(Eigen::Vector15d::Zero());
}

void ESKF::imu_update(const ImuMeasurement& imu_meas, const double dt) {
    nominal_state_discrete(imu_meas, dt);
    error_state_prediction(imu_meas, dt);
}

void ESKF::dvl_update(const SensorDVL& dvl_meas) {
    measurement_update(dvl_meas);
    injection_and_reset();
}

// DVL sensor model implementations

Eigen::VectorXd SensorDVL::innovation(const StateQuat& state) const {
    Eigen::Vector3d innovation = this->measurement - calculate_h(state);
    return innovation;
}

Eigen::MatrixXd SensorDVL::jacobian(const StateQuat& state) const {
    Eigen::Matrix3x15d H = calculate_h_jacobian(state);
    return H;
}

Eigen::MatrixXd SensorDVL::noise_covariance() const {
    return this->measurement_noise;
}
