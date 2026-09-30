#include "gtsam_navigation/dvl_factor.hpp"
#include "gtsam_navigation/estimator.hpp"

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <Eigen/Eigenvalues>
#include <random>

namespace gtsam_navigation {
namespace {
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

Config test_config() {
    Config config;
    config.initialization_duration = 0.2;
    config.reorder_delay = 0.02;
    config.lag = 1.0;
    // Existing Nautilus geometry composed as IMU_T_DVL.
    config.imu_p_dvl =
        gtsam::Pose3(gtsam::Rot3::Rz(3.14159), {-0.030, 0.004, 0.1564});
    return config;
}

ImuSample stationary(double time) {
    return {time, {0, 0, -9.81}, {0, 0, 0}};
}

void feed_stationary(Estimator& estimator, int first, int last) {
    for (int i = first; i <= last; ++i) {
        const double time = i * 0.01;
        if (i % 10 == 0) {
            estimator.add_dvl({time, {0, 0, 0}, gtsam::I_3x3 * 2.5e-5});
        }
        ASSERT_TRUE(estimator.add_imu(stationary(time)))
            << estimator.status().detail;
    }
}

TEST(DvlFactor, JacobiansIncludeRotationAndGyroBiasLeverArm) {
    const auto sensor =
        gtsam::Pose3(gtsam::Rot3::RzRyRx(0.3, -0.2, 1.7), {0.4, 0.1, -0.2});
    DvlFactor factor(X(0), V(0), B(0), {0.1, 0.2, -0.3}, {0.4, -0.2, 0.1},
                     sensor, gtsam::noiseModel::Isotropic::Sigma(3, 0.01));
    const auto pose =
        gtsam::Pose3(gtsam::Rot3::RzRyRx(-0.1, 0.2, 0.6), {1, 2, 3});
    const gtsam::Vector3 velocity(0.5, -0.3, 0.2);
    const gtsam::imuBias::ConstantBias bias({0.01, 0.02, -0.01},
                                            {0.03, -0.02, 0.01});
    gtsam::Matrix hp, hv, hb;
    factor.evaluateError(pose, velocity, bias, hp, hv, hb);
    const std::function<gtsam::Vector(const gtsam::Pose3&,
                                      const gtsam::Vector3&,
                                      const gtsam::imuBias::ConstantBias&)>
        error = [&factor](const auto& p, const auto& v, const auto& b) {
            return factor.evaluateError(p, v, b);
        };
    EXPECT_TRUE(hp.isApprox(
        gtsam::numericalDerivative31(error, pose, velocity, bias), 1e-6));
    EXPECT_TRUE(hv.isApprox(
        gtsam::numericalDerivative32(error, pose, velocity, bias), 1e-6));
    EXPECT_TRUE(hb.isApprox(
        gtsam::numericalDerivative33(error, pose, velocity, bias), 1e-6));
}

TEST(Initialization, StationaryTiltBiasAndMovingWindow) {
    auto config = test_config();
    Estimator estimator(config);
    for (int i = 0; i < 50; ++i) {
        auto sample = stationary(i * 0.01);
        sample.angular_velocity.z() = 0.3;
        estimator.add_imu(sample);
    }
    EXPECT_FALSE(estimator.status().initialized);
    const auto rotation = gtsam::Rot3::RzRyRx(0.2, -0.1, 0.0);
    const gtsam::Vector3 bias(2e-5, -1e-5, 3e-5);
    for (int i = 50; i <= 120; ++i) {
        estimator.add_imu(
            {i * 0.01, rotation.unrotate(gtsam::Vector3(0, 0, -9.81)), bias});
    }
    ASSERT_TRUE(estimator.status().initialized);
    const auto result = estimator.latest();
    ASSERT_TRUE(result);
    EXPECT_LT(
        gtsam::Rot3::Logmap(rotation.between(result->pose.rotation())).norm(),
        1e-8);
    EXPECT_LT(result->velocity.norm(), 1e-8);
    EXPECT_LT((result->bias.gyroscope() - bias).norm(), 1e-10);
    EXPECT_NEAR(result->pose.rotation().toQuaternion().norm(), 1.0, 1e-12);
}

TEST(Estimator, StationaryAndBoundedHistory) {
    Estimator estimator(test_config());
    feed_stationary(estimator, 0, 2000);
    const auto slots = estimator.status().factor_slots;
    feed_stationary(estimator, 2001, 12000);
    ASSERT_FALSE(estimator.status().fault) << estimator.status().detail;
    ASSERT_TRUE(estimator.latest());
    EXPECT_LT(estimator.latest()->pose.translation().norm(), 1e-7);
    EXPECT_LT(estimator.latest()->velocity.norm(), 1e-7);
    EXPECT_LE(estimator.status().active_states, 25u);
    EXPECT_LE(estimator.status().factor_slots, slots + 20);
    EXPECT_LE(estimator.status().buffered_samples, 10u);
}

TEST(Estimator, InvalidDataAndGapFailClosed) {
    Estimator estimator(test_config());
    feed_stationary(estimator, 0, 100);
    EXPECT_FALSE(estimator.add_imu(stationary(0.5)));
    EXPECT_FALSE(estimator.add_imu(stationary(1.0)));
    auto bad = stationary(1.01);
    bad.acceleration.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(estimator.add_imu(bad));
    EXPECT_FALSE(estimator.add_dvl({1.02, {0, 0, 0}, gtsam::Matrix3::Zero()}));
    EXPECT_FALSE(estimator.add_dvl({1.02, {0, 0, 0}, -gtsam::I_3x3}));
    estimator.add_imu(stationary(2.0));
    estimator.add_imu(stationary(2.1));
    EXPECT_TRUE(estimator.status().fault);
    EXPECT_FALSE(estimator.latest());
}

TEST(Estimator, ReorderingAndExactDvlBoundaries) {
    auto config = test_config();
    config.reorder_delay = 0.05;
    Estimator ordered(config), reordered(config);
    for (int group = 0; group < 200; ++group) {
        const double start = group * 0.03;
        if (group % 3 == 0) {
            const DvlSample dvl{
                start + 0.005, {0, 0, 0}, gtsam::I_3x3 * 2.5e-5};
            ordered.add_dvl(dvl);
            reordered.add_dvl(dvl);
        }
        for (int offset : {0, 1, 2}) {
            ordered.add_imu(stationary(start + offset * 0.01));
        }
        for (int offset : {2, 0, 1}) {
            reordered.add_imu(stationary(start + offset * 0.01));
        }
    }
    ASSERT_TRUE(ordered.latest());
    ASSERT_TRUE(reordered.latest());
    EXPECT_TRUE(ordered.latest()->pose.equals(reordered.latest()->pose, 1e-9));
    EXPECT_TRUE(ordered.latest()->covariance.isApprox(
        reordered.latest()->covariance, 1e-8));
    EXPECT_EQ(reordered.status().rejected_imu, 0u);
}

TEST(Estimator, AccelerationTurningAndDvlDropoutRecovery) {
    auto config = test_config();
    Estimator estimator(config);
    feed_stationary(estimator, 0, 100);
    gtsam::Pose3 pose;
    gtsam::Vector3 velocity = gtsam::Vector3::Zero();
    const double dt = 0.01;
    // The previous sample at t=1 was stationary. From t=1.01, integrate exactly
    // the same piecewise constant specific force used by the physical fixture.
    ImuSample previous = stationary(1.0);
    for (int i = 101; i <= 1000; ++i) {
        const gtsam::Vector3 a_world = pose.rotation() * previous.acceleration +
                                       gtsam::Vector3(0, 0, 9.81);
        pose = gtsam::Pose3(
            pose.rotation() *
                gtsam::Rot3::Expmap(previous.angular_velocity * dt),
            pose.translation() + velocity * dt + 0.5 * a_world * dt * dt);
        velocity += a_world * dt;
        const double time = i * dt;
        const gtsam::Vector3 acceleration(i < 300 ? 0.02 : 0.0, 0, 0);
        const gtsam::Vector3 omega(0, 0, 0.07 * std::sin(time));
        const ImuSample sample{
            time,
            pose.rotation().unrotate(acceleration - gtsam::Vector3(0, 0, 9.81)),
            omega};
        if (i % 20 == 0 && !(time > 4 && time < 6)) {
            // Factor uses the left endpoint gyro over this interval.
