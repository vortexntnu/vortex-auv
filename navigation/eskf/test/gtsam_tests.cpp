#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/ImuFactor.h>
#include <iostream>
#include <random>
#include <stdexcept>
#include "eskf/eskf.hpp"
#include "eskf/gtsam_factors.hpp"
#include "eskf/gtsam_navigation.hpp"
void check(bool pass, const char* message) {
    if (!pass)
        throw std::runtime_error(message);
}
EskfParams parameters() {
    EskfParams p;
    p.P = Eigen::Matrix15d::Identity() * .1;
    p.Q = Eigen::Matrix12d::Identity() * 1e-5;
    p.g_ = Eigen::Vector3d(0, 0, 9.81);
    return p;
}
void factor_derivatives() {
    using namespace gtsam;
    std::mt19937 random(98);
    std::normal_distribution<double> normal;
    for (int i = 0; i < 20; ++i) {
        Pose3 pose(Rot3::Expmap(
                       Vector3(normal(random), normal(random), normal(random))),
                   Vector3(1, 2, 3));
        Vector3 velocity(.5, -.2, .1);
        eskf_gtsam::DvlFactor dvl(
            0, 1, SensorDVL{Vector3(.1, .2, .3), Matrix3::Identity()});
        Matrix Hpose, Hvelocity;
        dvl.evaluateError(pose, velocity, Hpose, Hvelocity);
        const auto numeric_pose = numericalDerivative11<Vector, Pose3>(
            [&](const Pose3& value) -> Vector {
                return dvl.evaluateError(value, velocity);
            },
            pose);
        const auto numeric_velocity = numericalDerivative11<Vector, Vector3>(
            [&](const Vector3& value) -> Vector {
                return dvl.evaluateError(pose, value);
            },
            velocity);
        check((Hpose - numeric_pose).norm() < 1e-8, "DVL pose Jacobian");
        check((Hvelocity - numeric_velocity).norm() < 1e-8,
              "DVL velocity Jacobian");
        eskf_gtsam::DepthFactor depth(
            0, SensorDepth{2, .01, Vector3(-.035, -.115, -.095)});
        Matrix Hdepth;
        depth.evaluateError(pose, Hdepth);
        const auto numeric_depth = numericalDerivative11<Vector, Pose3>(
            [&](const Pose3& value) -> Vector {
                return depth.evaluateError(value);
            },
            pose);
        check((Hdepth - numeric_depth).norm() < 1e-8, "Depth pose Jacobian");
    }
}
void preintegration_reference() {
    using namespace gtsam;
    auto params = boost::make_shared<PreintegrationParams>(Vector3(0, 0, 9.81));
    params->accelerometerCovariance = Matrix3::Identity() * 1e-4;
    params->gyroscopeCovariance = Matrix3::Identity() * 1e-5;
    params->integrationCovariance = Matrix3::Identity() * 1e-8;
    const imuBias::ConstantBias bias(Vector3(.01, -.02, .03),
                                     Vector3(.001, .002, -.003));
    PreintegratedImuMeasurements pim(params, bias);
    NavState initial(Rot3::RzRyRx(.2, -.1, .3), Vector3(1, 2, 3),
                     Vector3(.1, -.2, .3));
    Eigen::Quaterniond q = initial.attitude().toQuaternion();
    Vector3 position = initial.position(), velocity = initial.v();
    const double dt = .008;
    const Vector3 omega(.3, -.1, .2), force(.2, -.3, -9.81);
    for (int i = 0; i < 25; ++i) {
        pim.integrateMeasurement(force + bias.accelerometer(),
                                 omega + bias.gyroscope(), dt);
        const Vector3 acceleration = q * force + params->n_gravity;
        position += velocity * dt + .5 * acceleration * dt * dt;
        velocity += acceleration * dt;
        q = (q * Eigen::Quaterniond(
                     Eigen::AngleAxisd(omega.norm() * dt, omega.normalized())))
                .normalized();
    }
    const auto predicted = pim.predict(initial, bias);
    check((predicted.position() - position).norm() < 1e-10,
          "Preintegrated position matches independent integration");
    check((predicted.v() - velocity).norm() < 1e-10,
          "Preintegrated velocity matches independent integration");
    check(predicted.attitude().toQuaternion().angularDistance(q) < 1e-10,
          "Preintegrated attitude matches independent integration");
    const imuBias::ConstantBias revised(
        bias.accelerometer() + Vector3::Constant(1e-5),
        bias.gyroscope() + Vector3::Constant(1e-5));
    PreintegratedImuMeasurements reintegrated(params, revised);
    for (int i = 0; i < 25; ++i)
        reintegrated.integrateMeasurement(force + bias.accelerometer(),
                                          omega + bias.gyroscope(), dt);
    const auto corrected = pim.predict(initial, revised);
    const auto reference = reintegrated.predict(initial, revised);
    check(corrected.localCoordinates(reference).norm() < 1e-8,
          "Bias Jacobian correction agrees with reintegration");
    check(pim.preintMeasCov().llt().info() == Eigen::Success,
          "Preintegration covariance positive definite");
}
void navigation() {
    auto p = parameters();
    GtsamNavigationParams options;
    options.lag_seconds = .8;
    options.keyframe_interval = .1;
    NominalState initial;
    initial.vel.x() = .4;
    GtsamNavigation filter(p, options, initial);
    const ImuMeasurement rest{-p.g_, Eigen::Vector3d::Zero()};
    check(!filter.imu_update(rest, -.01), "Negative dt rejected");
    check(!filter.depth_update({10000, .01}), "Depth outlier rejected");
    for (int i = 0; i < 300; ++i) {
        check(filter.imu_update(rest, .008), "Smoother propagation");
        if (i % 25 == 24) {
            check(filter.dvl_update({Eigen::Vector3d(.4, 0, 0),
                                     Eigen::Matrix3d::Identity() * .01}),
                  "DVL factor accepted");
            check(filter.depth_update({0, .01}), "Depth factor accepted");
        }
    }
    check(filter.healthy(), "Smoother healthy after multiple marginalizations");
    check(std::abs(filter.get_nominal_state().pos.x() - .96) < 1e-5,
          "Known constant velocity trajectory");
    check(filter.get_nominal_state().vel.isApprox(initial.vel, 1e-5),
          "Velocity preserved");
    check(filter.keyframe_count() > 20 && filter.active_state_count() < 20,
          "Fixed-lag window marginalizes old states");
    const auto covariance = filter.get_error_state().covariance;
    check(covariance.allFinite() &&
              covariance.isApprox(covariance.transpose(), 1e-10) &&
              covariance.llt().info() == Eigen::Success,
          "Navigation covariance valid");
    check(covariance.block<3, 3>(0, 3).norm() > 1e-8,
          "Export preserves cross-covariance");
    const auto before = filter.get_nominal_state().as_vector();
    check(!filter.depth_update({10000, .01}),
          "Post-marginalization outlier rejection");
    check((filter.get_nominal_state().as_vector() - before).isZero(),
          "Rejected factor is atomic");
    // Aiding changes a deliberately biased initial velocity estimate.
    NominalState offset;
    offset.vel.x() = .5;
    GtsamNavigation aided(p, options, offset);
    for (int i = 0; i < 25; ++i)
        check(aided.imu_update(rest, .008), "Aided propagation");
    const auto earlier_velocity = aided.keyframe_state(1)->vel.x();
    check(aided.dvl_update(
              {Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() * .01}),
          "DVL correction");
    check(aided.keyframe_state(1)->vel.x() < earlier_velocity - .2,
          "Later aiding revises an earlier retained state");
    check(!filter.keyframe_state(0),
          "Marginalized states are no longer retained");
    check(std::abs(aided.get_nominal_state().vel.x()) < .1,
          "Graph correction affects velocity");
}
int main() {
    try {
        factor_derivatives();
        preintegration_reference();
        navigation();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    std::cout << "GTSAM factor Jacobians, direct integration, bias correction, "
                 "navigation and fixed-lag regressions passed\n";
}
