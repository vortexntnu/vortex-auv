#include <iostream>
#include <random>
#include <stdexcept>
#include "eskf/eskf.hpp"
#include "eskf/lie.hpp"
#include "eskf/output.hpp"
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;
using M15 = Eigen::Matrix15d;
void check(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}
V3 logq(Eigen::Quaterniond q) {
    q.normalize();
    if (q.w() < 0)
        q.coeffs() *= -1;
    const double n = q.vec().norm();
    return n < 1e-14 ? V3::Zero().eval()
                     : (q.vec() * (2 * std::atan2(n, q.w()) / n)).eval();
}
void derivatives() {
    std::mt19937 rng(42);
    std::normal_distribution<double> normal;
    constexpr double eps = 1e-6;
    for (int trial = 0; trial < 40; ++trial) {
        NominalState state;
        state.quat = Eigen::Quaterniond(normal(rng), normal(rng), normal(rng),
                                        normal(rng))
                         .normalized();
        state.vel = V3(normal(rng), normal(rng), normal(rng));
        SensorDepth depth{2, .1, V3(-.035, -.115, -.095)};
        Eigen::Matrix3x15d numeric = Eigen::Matrix3x15d::Zero();
        Eigen::Matrix<double, 1, 15> depth_numeric =
            Eigen::Matrix<double, 1, 15>::Zero();
        for (int axis = 0; axis < 3; ++axis) {
            auto plus = state, minus = state;
            plus.vel[axis] += eps;
            minus.vel[axis] -= eps;
            numeric.col(3 + axis) =
                (calculate_h(plus) - calculate_h(minus)) / (2 * eps);
            plus = state;
            minus = state;
            plus.quat *= eskf_math::exp(eps * V3::Unit(axis));
            minus.quat *= eskf_math::exp(-eps * V3::Unit(axis));
            numeric.col(6 + axis) =
                (calculate_h(plus) - calculate_h(minus)) / (2 * eps);
            depth_numeric(6 + axis) =
                -(depth.innovation(plus)(0) - depth.innovation(minus)(0)) /
                (2 * eps);
        }
        depth_numeric(2) = 1;
        check((calculate_h_jacobian(state) - numeric).norm() < 1e-8,
              "DVL local Jacobian");
        check((depth.jacobian(state) - depth_numeric).norm() < 1e-8,
              "Depth lever-arm Jacobian");
    }
}
void prediction() {
    EskfParams params;
    params.P.setZero();
    params.Q.setZero();
    params.g_.setZero();
    for (int index : {9, 12}) {
        params.P.setZero();
        params.P(index, index) = 1;
        ESKF filter(params);
        check(filter.imu_update({}, .01), "Prediction accepted");
        const auto P = filter.get_error_state().covariance;
        check(std::abs(P(index, index) - 1) < 1e-12,
              "No fictitious bias decay");
        check(
            std::abs(P(index == 9 ? 6 : 3, index == 9 ? 6 : 3) - 1e-4) < 1e-12,
            "Correct bias coupling");
        check(std::abs(P(index == 9 ? 3 : 6, index == 9 ? 3 : 6)) < 1e-12,
              "No swapped bias coupling");
    }
    params.P = M15::Identity();
    NominalState initial;
    initial.vel.x() = 1;
    ESKF filter(params, initial);
    for (double dt : {0., -.01, .2, std::numeric_limits<double>::quiet_NaN()})
        check(!filter.imu_update({}, dt), "Invalid dt rejected");
    check(filter.get_nominal_state().pos.isZero(), "Rejected dt is atomic");
    ImuMeasurement invalid;
    invalid.gyro.x() = std::numeric_limits<double>::infinity();
    check(!filter.imu_update(invalid, .01), "Nonfinite IMU rejected");
    EskfParams earth;
    ESKF stationary(earth);
    for (int i = 0; i < 125; ++i)
        check(stationary.imu_update({-earth.g_, V3::Zero()}, .008),
              "Stationary propagation");
    check(stationary.get_nominal_state().pos.norm() < 1e-12,
          "Specific force cancels gravity");
    const auto P = stationary.get_error_state().covariance;
    check(P.isApprox(P.transpose(), 1e-12) && P.ldlt().isPositive(),
          "Covariance symmetric PSD");
    // Gyro-bias random walk belongs to the gyro-bias block.
    params.P.setZero();
    params.Q.setZero();
    params.Q(6, 6) = 2;
    ESKF noise(params);
    noise.imu_update({}, .01);
    check(std::abs(noise.get_error_state().covariance(9, 9) - .02) < 1e-12,
          "Process noise ordering");
    check(noise.get_error_state().covariance(12, 12) == 0,
          "Bias noise separation");
}
void corrections() {
    EskfParams params;
    ESKF filter(params);
    const auto before = filter.get_error_state().covariance;
    check(!filter.depth_update({10000, .01}), "Depth outlier rejected");
    check(filter.get_nis_depth() > params.depth_nis_threshold,
          "Rejected NIS retained");
    check(filter.get_nominal_state().pos.isZero() &&
              filter.get_error_state().covariance == before,
          "Outlier atomicity");
    for (double variance : {-1., 0., std::numeric_limits<double>::infinity()})
        check(!filter.depth_update({1, variance}), "Invalid variance rejected");
    SensorDVL bad;
    bad.measurement_noise(0, 1) = .1;
    check(!filter.dvl_update(bad), "Asymmetric noise rejected");
    // Independent finite-difference reset chart, including cross-covariances.
    params.P = M15::Identity();
    params.P(0, 6) = params.P(6, 0) = .2;
    params.P(6, 6) = 2;
    params.P(7, 7) = 3;
    NominalState initial;
    initial.vel = V3(1, 2, 3);
    ESKF reset(params, initial);
    SensorDVL measurement{calculate_h(initial) + V3(.2, -.3, .1),
                          M3::Identity() * .1};
    const auto H = measurement.jacobian(initial);
    const M3 S = H * params.P * H.transpose() + measurement.measurement_noise;
    const Eigen::Matrix<double, 15, 3> K =
        S.llt().solve((params.P * H.transpose()).transpose()).transpose();
    const Eigen::Vector15d delta = K * measurement.innovation(initial);
    const M15 A = M15::Identity() - K * H;
    const M15 posterior = A * params.P * A.transpose() +
                          K * measurement.measurement_noise * K.transpose();
    M15 J = M15::Identity();
    const V3 rotation = delta.segment<3>(6);
    for (int axis = 0; axis < 3; ++axis) {
        const auto left = eskf_math::exp(-rotation);
        J.block<3, 1>(6, 6 + axis) =
            (logq(left * eskf_math::exp(rotation + 1e-6 * V3::Unit(axis))) -
             logq(left * eskf_math::exp(rotation - 1e-6 * V3::Unit(axis)))) /
            2e-6;
    }
    check(reset.dvl_update(measurement), "DVL correction accepted");
    check((reset.get_error_state().covariance - J * posterior * J.transpose())
                  .norm() < 1e-8,
          "Full reset covariance transport");
    check(reset.get_error_state().as_vector().isZero(), "Error mean reset");
    check(std::abs(reset.get_nominal_state().quat.norm() - 1) < 1e-12,
          "Quaternion normalization");
}
void outputs() {
    NominalState state;
    state.quat =
        Eigen::Quaterniond(Eigen::AngleAxisd(std::acos(-1) / 2, V3::UnitZ()));
    M15 P = M15::Zero();
    P.diagonal().segment<3>(3) = V3(1, 4, 9);
    const auto twist =
        eskf_output::twist_covariance(state, P, M3::Identity() * .2);
    check((twist.diagonal().head<3>() - V3(4, 1, 9)).norm() < 1e-12,
          "Body velocity covariance rotation");
    check(std::abs(twist(3, 3) - .2) < 1e-12,
          "Gyro sample covariance exported");
    P = M15::Identity();
    P(0, 6) = P(6, 0) = .2;
    const auto pose = eskf_output::pose_covariance(state, P);
    check(std::abs(pose(0, 4) - .2) < 1e-12,
          "Pose cross covariance transported");
}
int main() {
    try {
        derivatives();
        prediction();
        corrections();
        outputs();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    std::cout << "ESKF derivative, prediction, correction, rejection and frame "
                 "regressions passed\n";
}
