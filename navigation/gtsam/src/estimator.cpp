#include "gtsam_navigation/estimator.hpp"
#include "gtsam_navigation/dvl_factor.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <Eigen/Cholesky>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace gtsam_navigation {
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;
static_assert(std::is_same_v<gtsam::DefaultPreintegrationType,
                             gtsam::TangentPreintegration>,
              "Build GTSAM with tangent preintegration ON and LieGroup OFF");

namespace {
gtsam::ISAM2Params isam_parameters() {
    gtsam::ISAM2Params params;
    params.findUnusedFactorSlots = true;
    params.relinearizeSkip = 1;
    params.setRelinearizeThreshold(0.01);
    params.setFactorization("QR");
    return params;
}

bool positive_covariance(const gtsam::Matrix3& matrix) {
    return matrix.allFinite() && matrix.isApprox(matrix.transpose(), 1e-8) &&
           Eigen::LLT<gtsam::Matrix3>(matrix).info() == Eigen::Success;
}
}  // namespace

Estimator::Estimator(const Config& config)
    : config_(config), smoother_(config.lag, isam_parameters()) {
    for (double value :
         {config.gravity, config.accel_noise_density, config.gyro_noise_density,
          config.accel_bias_random_walk, config.gyro_bias_random_walk,
          config.integration_sigma, config.lag, config.keyframe_interval,
          config.max_imu_gap, config.initialization_duration,
          config.stationary_accel_std, config.stationary_gyro_std,
          config.stationary_gyro_norm, config.stationary_gravity_tolerance,
          config.dvl_gate_squared, config.minimum_dvl_interval}) {
        if (!std::isfinite(value) || value <= 0) {
            throw std::invalid_argument(
                "Estimator parameters must be finite and positive");
        }
    }
    if (!std::isfinite(config.reorder_delay) || config.reorder_delay < 0 ||
        config.lag < 2 * config.keyframe_interval ||
        config.max_buffer_samples < 10 ||
        !config.imu_p_dvl.matrix().allFinite()) {
        throw std::invalid_argument(
            "Invalid lag, buffer size, reorder delay or sensor pose");
    }
    params_ = gtsam::PreintegrationParams::MakeSharedD(config.gravity);
    params_->setBodyPSensor(gtsam::Pose3());
    params_->setAccelerometerCovariance(
        gtsam::I_3x3 * std::pow(config.accel_noise_density, 2));
    params_->setGyroscopeCovariance(gtsam::I_3x3 *
                                    std::pow(config.gyro_noise_density, 2));
    params_->setIntegrationCovariance(gtsam::I_3x3 *
                                      std::pow(config.integration_sigma, 2));
    pim_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(params_);
}

void Estimator::fail(const std::string& reason) {
    status_.fault = true;
    status_.detail = reason;
    imu_buffer_.clear();
    dvl_buffer_.clear();
    status_.buffered_samples = 0;
}

bool Estimator::add_imu(const ImuSample& sample) {
    if (status_.fault) {
        return false;
    }
    if (!std::isfinite(sample.time) || sample.time < 0 ||
        !sample.acceleration.allFinite() ||
        !sample.angular_velocity.allFinite() ||
        (previous_imu_ && sample.time <= previous_imu_->time) ||
        imu_buffer_.count(sample.time)) {
        ++status_.rejected_imu;
        return false;
    }
