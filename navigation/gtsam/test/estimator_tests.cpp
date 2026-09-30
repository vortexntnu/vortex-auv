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
            const auto measured = config.imu_p_dvl.rotation().unrotate(
                pose.rotation().unrotate(velocity) +
                previous.angular_velocity.cross(
                    config.imu_p_dvl.translation()));
            estimator.add_dvl({time, measured, gtsam::I_3x3 * 2.5e-5});
        }
        ASSERT_TRUE(estimator.add_imu(sample)) << estimator.status().detail;
        previous = sample;
    }
    const auto result = estimator.latest();
    ASSERT_TRUE(result);
    EXPECT_LT((result->pose.translation() - pose.translation()).norm(), 2e-3);
    EXPECT_LT((result->velocity - velocity).norm(), 2e-3);
    EXPECT_LT(
        gtsam::Rot3::Logmap(result->pose.rotation().between(pose.rotation()))
            .norm(),
        2e-3);
    EXPECT_GT(estimator.status().last_dvl_time, 9.0);
    EXPECT_TRUE(
        result->covariance.isApprox(result->covariance.transpose(), 1e-9));
    EXPECT_GT(Eigen::SelfAdjointEigenSolver<Matrix15>(result->covariance)
                  .eigenvalues()
                  .minCoeff(),
              -1e-12);
}

TEST(Estimator, DvlOutlierRejectedWithoutCorruptingState) {
    Estimator estimator(test_config());
    feed_stationary(estimator, 0, 100);
    const auto rejected = estimator.status().rejected_dvl;
    estimator.add_dvl({1.055, {100, 0, 0}, gtsam::I_3x3 * 2.5e-5});
    feed_stationary(estimator, 101, 150);
    EXPECT_GT(estimator.status().rejected_dvl, rejected);
    ASSERT_TRUE(estimator.latest());
    EXPECT_LT(estimator.latest()->velocity.norm(), 1e-7);
}

TEST(Estimator, SeededStim300ProfilesWithInternalBiasAndDropout) {
    for (double accel_density : {0.07 / 60.0, 0.21 / 60.0}) {
        auto config = test_config();
        config.accel_noise_density = accel_density;
        config.initialization_duration = 2.0;
        config.lag = 5.0;
        Estimator estimator(config);
        std::mt19937 rng(42);
        std::normal_distribution<double> gaussian(0, 1);
        const auto noise = [&rng, &gaussian](double sigma) -> gtsam::Vector3 {
            return gtsam::Vector3(gaussian(rng), gaussian(rng), gaussian(rng)) *
                   sigma;
        };
        gtsam::Vector3 accel_bias(0.003, -0.002, 0.001);
        gtsam::Vector3 gyro_bias(2e-5, -1e-5, 1e-5);
        gtsam::Pose3 pose;
        gtsam::Vector3 velocity = gtsam::Vector3::Zero();
        auto previous = stationary(0);
        const double dt = 0.01;
        double squared_position = 0, squared_velocity = 0;
        int estimates = 0;
        for (int i = 0; i <= 3000; ++i) {
            if (i > 0) {
                const gtsam::Vector3 acceleration =
                    pose.rotation() * previous.acceleration +
                    gtsam::Vector3(0, 0, 9.81);
                pose = gtsam::Pose3(
                    pose.rotation() *
                        gtsam::Rot3::Expmap(previous.angular_velocity * dt),
                    pose.translation() + velocity * dt +
                        0.5 * acceleration * dt * dt);
                velocity += acceleration * dt;
            }
            const double time = i * dt;
            const gtsam::Vector3 acceleration(i > 500 && i < 1000 ? 0.02 : 0, 0,
                                              0);
            const gtsam::Vector3 omega =
                i > 500 ? gtsam::Vector3(0.01 * std::sin(time),
                                         0.01 * std::cos(time), 0.04)
                        : gtsam::Vector3::Zero();
            ImuSample ideal{time,
                            pose.rotation().unrotate(
                                acceleration - gtsam::Vector3(0, 0, 9.81)),
                            omega};
            if (i % 20 == 0 && !(time >= 15 && time < 17)) {
                const auto measurement = config.imu_p_dvl.rotation().unrotate(
                    pose.rotation().unrotate(velocity) +
                    previous.angular_velocity.cross(
                        config.imu_p_dvl.translation()));
                estimator.add_dvl(
                    {time, measurement + noise(0.005), gtsam::I_3x3 * 2.5e-5});
            }
            accel_bias += noise(config.accel_bias_random_walk * std::sqrt(dt));
            gyro_bias += noise(config.gyro_bias_random_walk * std::sqrt(dt));
            auto measured = ideal;
            measured.acceleration +=
                accel_bias + noise(accel_density / std::sqrt(dt));
            measured.angular_velocity +=
                gyro_bias + noise(config.gyro_noise_density / std::sqrt(dt));
            ASSERT_TRUE(estimator.add_imu(measured))
                << estimator.status().detail;
            if (const auto estimate = estimator.latest()) {
                squared_position +=
                    (estimate->pose.translation() - pose.translation())
                        .squaredNorm();
                squared_velocity +=
                    (estimate->velocity - velocity).squaredNorm();
                ++estimates;
            }
            previous = ideal;
        }
        ASSERT_GT(estimates, 2000);
        const auto estimate = estimator.latest();
        ASSERT_TRUE(estimate);
        const double position_rmse = std::sqrt(squared_position / estimates);
        const double velocity_rmse = std::sqrt(squared_velocity / estimates);
        const double attitude_error =
            gtsam::Rot3::Logmap(
                estimate->pose.rotation().between(pose.rotation()))
                .norm();
        std::cout << "STIM300 density=" << accel_density
                  << ": position RMSE=" << position_rmse
                  << " m, velocity RMSE=" << velocity_rmse
                  << " m/s, final attitude error=" << attitude_error
                  << " rad, accel bias error="
                  << (estimate->bias.accelerometer() - accel_bias).norm()
                  << " m/s^2, gyro bias error="
                  << (estimate->bias.gyroscope() - gyro_bias).norm()
                  << " rad/s\n";
        EXPECT_LT(position_rmse, 0.2);
        EXPECT_LT(velocity_rmse, 0.04);
        EXPECT_LT(attitude_error, 0.03);
        EXPECT_GT(estimator.status().last_dvl_time, 29.0);
    }
}

TEST(Covariance, GrowsWithoutDvlAndShrinksAfterRecovery) {
    Estimator estimator(test_config());
    feed_stationary(estimator, 0, 200);
    const double before =
        estimator.latest()->covariance.block<3, 3>(6, 6).trace();
    for (int i = 201; i <= 500; ++i) {
        estimator.add_imu(stationary(i * 0.01));
    }
    const double dropout =
        estimator.latest()->covariance.block<3, 3>(6, 6).trace();
    feed_stationary(estimator, 501, 700);
    const double recovered =
        estimator.latest()->covariance.block<3, 3>(6, 6).trace();
    EXPECT_GT(dropout, before);
    EXPECT_LT(recovered, dropout);
}

TEST(Covariance, RosMappingsMatchNumericalJacobians) {
    Estimate estimate;
    estimate.pose =
        gtsam::Pose3(gtsam::Rot3::RzRyRx(0.3, -0.2, 0.4), {1, 2, 3});
    estimate.velocity = gtsam::Vector3(0.4, -0.1, 0.2);
    estimate.covariance = Matrix15::Identity() * 0.01;
    estimate.covariance(0, 12) = estimate.covariance(12, 0) = 0.002;
    const auto config = test_config();
    const auto [pose_cov, twist_cov] = odometry_covariances(estimate, config);
    const std::function<gtsam::Vector6(const Eigen::Matrix<double, 15, 1>&)>
        pose_fn = [&estimate](const auto& dx) {
            const auto pose = estimate.pose.retract(dx.template head<6>());
            gtsam::Vector6 result;
            result << pose.translation(),
                gtsam::Rot3::Logmap(pose.rotation() *
                                    estimate.pose.rotation().inverse());
            return result;
        };
    const std::function<gtsam::Vector6(const Eigen::Matrix<double, 15, 1>&)>
        twist_fn = [&estimate](const auto& dx) {
            const auto pose = estimate.pose.retract(dx.template head<6>());
            gtsam::Vector6 result;
            result << pose.rotation().unrotate(estimate.velocity +
                                               dx.template segment<3>(6)),
                -dx.template tail<3>();
            return result;
        };
    const Eigen::Matrix<double, 15, 1> zero =
        Eigen::Matrix<double, 15, 1>::Zero();
    const auto jp = gtsam::numericalDerivative11(pose_fn, zero);
    const auto jt = gtsam::numericalDerivative11(twist_fn, zero);
    auto expected_twist = (jt * estimate.covariance * jt.transpose()).eval();
    expected_twist.bottomRightCorner<3, 3>().diagonal().array() +=
        std::pow(config.gyro_noise_density, 2) / estimate.gyro_sample_dt;
    EXPECT_TRUE(
        pose_cov.isApprox(jp * estimate.covariance * jp.transpose(), 1e-6));
    EXPECT_TRUE(twist_cov.isApprox(expected_twist, 1e-6));
}

TEST(Smoother, MarginalizationMatchesShortBatchReference) {
    gtsam::ISAM2Params settings;
    settings.findUnusedFactorSlots = true;
    settings.relinearizeSkip = 1;
    settings.setRelinearizeThreshold(0.001);
    gtsam::IncrementalFixedLagSmoother smoother(0.3, settings);
    gtsam::NonlinearFactorGraph batch;
    gtsam::Values all_values;
    // A linear Gaussian velocity chain isolates information preservation from
    // nonlinear relinearization effects, and has an exact batch reference.
    for (int i = 0; i < 30; ++i) {
        gtsam::NonlinearFactorGraph factors;
        if (i == 0) {
            factors.add(gtsam::PriorFactor<gtsam::Vector3>(
                V(i), {0, 0, 0}, gtsam::noiseModel::Isotropic::Sigma(3, 0.1)));
        } else {
            factors.add(gtsam::BetweenFactor<gtsam::Vector3>(
                V(i - 1), V(i), {0.01, 0, 0},
                gtsam::noiseModel::Isotropic::Sigma(3, 0.02)));
        }
        factors.add(gtsam::PriorFactor<gtsam::Vector3>(
            V(i), {i * 0.01 + 0.003 * std::sin(i), 0, 0},
            gtsam::noiseModel::Isotropic::Sigma(3, 0.05)));
        const gtsam::Vector3 guess(i * 0.01, 0, 0);
        gtsam::Values values;
        values.insert(V(i), guess);
        all_values.insert(V(i), guess);
        batch.push_back(factors);
        smoother.update(factors, values, {{V(i), i * 0.1}});
    }
    const auto solution =
        gtsam::LevenbergMarquardtOptimizer(batch, all_values).optimize();
    EXPECT_TRUE(smoother.calculateEstimate<gtsam::Vector3>(V(29)).isApprox(
        solution.at<gtsam::Vector3>(V(29)), 1e-8));
    EXPECT_LE(smoother.timestamps().size(), 5u);
}
}  // namespace
}  // namespace gtsam_navigation
